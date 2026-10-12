#we will use this file to create manual our manual inputs rather
# than using nissancan.py, but we will use the same idea

import os

from opendbc.can import CANPacker
from opendbc.car.crc import CRC8J1850, mk_crc8_fun

# TODO: add this checksum to the CANPacker
# NOTE - this is Nissan's checksum inside the data bytes, not the CAN
# protocol CRC. The CAN hardware does that one separately.
#
# CHANGED - init_crc/xor_out match opendbc's nissancan.py exactly. This is
# NOT textbook CRC-8/SAE-J1850 (that one is init 0xFF / xor 0xFF). Nissan's
# variant uses init 0x00. Do not "fix" this to the standard values; openpilot
# drives real Leafs with these and the EPS accepts them.
nissan_checksum = mk_crc8_fun(CRC8J1850, init_crc=0x00, xor_out=0xFF)

# ADDED - from opendbc nissan values.py, angle limits for the Leaf
MAX_ANGLE = 600.0

# ADDED - bus numbers taken from our own Cabana captures, not guessed.
#   0x27A (TURN_SIGNALS)  captured on bus 2, 47.5 Hz
#   0x3AF (HAZARD_SWITCH) captured on bus 0, 10.1 Hz
#   0x60D (DOORS_LIGHTS)  captured on bus 0, 10.3 Hz
# A flipped comma harness swaps buses 0 and 2, so confirm car_harness_status
# in CANInterface.print_health() before trusting these.
TURN_SIGNAL_BUS = 2
HAZARD_SWITCH_BUS = 0
LKAS_BUS = 0            # unverified - 0x169 does not appear in our captures

# 0x3AF pulse shape measured from 110 real button presses:
# 0.131 s min, 0.250 s median, 0.560 s max, at a 10 Hz message rate.
HAZARD_PULSE_S = 0.25
HAZARD_RATE_HZ = 10.0

# ADDED - our capstone DBC, resolved next to this file so it does not matter
# what directory you run the scripts from. This file contains 0x27A and 0x3AF,
# which opendbc's stock nissan_leaf_2018_generated.dbc does NOT have.
CAPSTONE_DBC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "nissan_leaf_2025_capstone.dbc")


#This is the main class for organizing CAN messages
class CarController_Manual():

  def __init__(self, dbc_name=None):

    # CHANGED - default to our own DBC instead of leaving the packer None.
    # Passing "nissan_leaf_2018_generated" loads opendbc's stock Leaf DBC,
    # which has no HAZARD_SWITCH and no TURN_SIGNALS_27A, so blinkers and
    # hazards raised KeyError before this.
    if dbc_name is None:
      dbc_name = CAPSTONE_DBC

    # ADDED - opendbc's DBC() tries os.path.exists(name) first and otherwise
    # falls back to a short-name lookup in its own dbc folder, so a missing
    # file here surfaces as a bare "DBC not found". Check it ourselves and say
    # what is actually wrong, including what IS in the folder. Windows hides
    # known extensions, so a rename in Explorer often lands as ".dbc.txt".
    if os.path.sep in dbc_name or os.path.altsep and os.path.altsep in dbc_name:
      if not os.path.exists(dbc_name):
        folder = os.path.dirname(dbc_name)
        try:
          found = sorted(os.listdir(folder))
        except OSError:
          found = []
        raise FileNotFoundError(
          f"DBC not found: {dbc_name}\n"
          f"Files in {folder}:\n  " + "\n  ".join(found or ["<none>"]) + "\n"
          "Expected 'nissan_leaf_2025_capstone.dbc'. If you see the DBC under "
          "another name, rename it (git mv keeps the history). If you see a "
          "trailing '.txt', Explorer appended the extension instead of "
          "replacing it."
        )

    self.packer = CANPacker(dbc_name)

    # CHANGED - one counter per message name, not one counter for the whole
    # bus. Every CAN ID carries its own independent rolling counter; sharing
    # a single counter made the LKAS sequence skip whenever a blinker frame
    # went out in between, and the receiving ECU rejects out-of-sequence
    # frames.
    self.counters = {}

  # ADDED - next counter value for one message, 0..15 then wraps
  def _next_counter(self, message_name):
    value = self.counters.get(message_name, 0)
    self.counters[message_name] = (value + 1) % 0x10
    return value

  # ADDED - builds a message with the counter and checksum
  def make_checksummed_msg(self, message_name, bus, values,
                           counter_field="COUNTER", checksum_field="CHECKSUM"):
    # Packs twice. The checksum covers the packed bytes, so we have to
    # pack once to get them, then pack again with the checksum filled in.
    #
    # CRC-8 variant, checksum lives in the last byte and covers everything
    # before it.

    # copy so we don't change the caller's dict
    values = dict(values)
    values[counter_field] = self._next_counter(message_name)
    values[checksum_field] = 0

    # pack 1 - checksum still zero
    first_pass = self.packer.make_can_msg(message_name, bus, values)[1]

    # CHANGED - was hardcoded [:7]. Derived from the frame length now, so this
    # stays correct if we point it at a message that is not 8 bytes.
    values[checksum_field] = nissan_checksum(first_pass[:len(first_pass) - 1])

    # pack 2 - real checksum
    return self.packer.make_can_msg(message_name, bus, values)

  # ADDED (Saul) - checksum for 0x27A, plain byte sum not CRC8/J1850,
  # so make_checksummed_msg does not work for this message.
  #
  # CHANGED - was sum(data[1:7]). Our DBC comment documents the algorithm as
  # (Byte1 + ... + Byte6 + 0x7C) & 0xFF, which is 1-indexed, so in Python that
  # is data[0:6]. The old slice skipped byte 0 and included the checksum byte.
  # It gave the right answer only because byte 0 is currently always 0x00 and
  # the checksum byte is 0 on the first pass. The moment we fill in real
  # byte-0 content captured from the car, the old version silently diverges.
  def sum_checksum_27a(self, data):
    return (sum(data[0:6]) + 0x7C) & 0xFF

  # ADDED (Saul) - turn signals and hazards on 0x27A, which carries the
  # actual lamp state
  def create_turn_signal_control(self, left, right, bus=TURN_SIGNAL_BUS):

    message_name = "TURN_SIGNALS_27A"

    values = {
      # ADDED - byte 2 is 0x4D in all 50432 captured frames. It was missing
      # from the DBC, so every frame we packed had 0x00 here and did not look
      # like anything the car sends.
      "SET_0x4D": 0x4D,
      "LEFT_BLINKER_27A": 1 if left else 0,
      "RIGHT_BLINKER_27A": 1 if right else 0,
      "COUNTER_27A": self._next_counter(message_name),
      "CHECKSUM_27A": 0,
    }

    # pack 1 - checksum still zero
    first_pass = self.packer.make_can_msg(message_name, bus, values)[1]
    values["CHECKSUM_27A"] = self.sum_checksum_27a(first_pass)

    # pack 2 - real checksum
    return self.packer.make_can_msg(message_name, bus, values)

  # ADDED - LKAS steering message, ID 0x169
  # Fields come from the 2018 Leaf DBC. Still has to be confirmed on our 2025.
  # The packer handles the raw encoding, so DESIRED_ANGLE is in degrees.
  def create_steering_control(self, angle_deg, lka_active, max_torque=1.0):

    # refuse anything past the limit instead of letting it wrap around
    if abs(angle_deg) > MAX_ANGLE:
      raise ValueError(f"Angle {angle_deg} outside +/-{MAX_ANGLE} degrees")

    values = {
      "DESIRED_ANGLE": angle_deg,
      "SET_0x80_2": 0x80,     # constant, openpilot always sends this
      "SET_0x80": 0x80,       # constant
      # CHANGED - openpilot sends `lkas_max_torque if steer_on else 0`.
      # We were leaving 1.0 Nm in the frame with LKA_ACTIVE clear.
      "MAX_TORQUE": max_torque if lka_active else 0.0,
      "LKA_ACTIVE": 1 if lka_active else 0,
    }

    # LKAS is on bus 0 and carries both a counter and a checksum
    return self.make_checksummed_msg("LKAS", LKAS_BUS, values)

  # Hazard lights. 0x3AF is the COMMAND side of the hazard function - the
  # capture shows all 35 sustained hazard periods in 0x27A beginning 20-30 ms
  # after a pulse here. It is a momentary press and the same pulse serves ON
  # and OFF, so this toggles rather than sets. No counter, no checksum, and
  # bytes 1-7 are constant 0x00.
  def create_hazard_control(self, enabled, bus=HAZARD_SWITCH_BUS):
    return self.packer.make_can_msg(
      "HAZARD_SWITCH", bus, {"HAZARD_BUTTON": 1 if enabled else 0}
    )

  # ADDED (steering) - LKAS_BUS is a placeholder. 0x169 is absent from every
  # capture we have, so we have not confirmed the camera transmits it on this
  # car, let alone on which bus. Capture it before trusting create_steering_control.
