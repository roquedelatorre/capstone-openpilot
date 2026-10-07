#we will use this file to create manual our manual inputs rather
# than using nissancan.py, but we will use the same idea

#from opendbc.car.nissan.values import CAR
from opendbc.can import CANPacker
from opendbc.car.crc import CRC8J1850, mk_crc8_fun
from opendbc.car.nissan.values import CAR
#from opendbc.car.interfaces import CarControllerBase
#from opendbc.car.nissan import nissancan
#from opendbc.car.nissan.values import CAR, CarControllerParams
#from opendbc.car.common.filter_simple import FirstOrderFilter

# TODO: add this checksum to the CANPacker
# NOTE - this is Nissan's checksum inside the data bytes, not the CAN
# protocol CRC. The CAN hardware does that one separately.
nissan_checksum = mk_crc8_fun(CRC8J1850, init_crc=0x00, xor_out=0xFF)

# ADDED - from opendbc nissan values.py, angle limits for the Leaf
MAX_ANGLE = 600.0

#This is the main class for organizing CAN messages
class CarController_Manual():

  def __init__(self, dbc_name=None):    #Sends blank dbc until we send one

    #CANPacker will be initialized here when provided DBC name
    self.packer = None

    #Sends DBC into pre-made CAN packer
    if dbc_name is not None:
      self.packer = CANPacker(dbc_name)

    # ADDED - counter 0 to 15, has to go up every frame or the car rejects it
    self.counter = 0

     # can_sends = []
     # return can_sends

  # ADDED - packer check, used by every builder below
  def _require_packer(self):
    if self.packer is None:
      raise RuntimeError("CANPacker not initialized")

  # ADDED - builds a message with the counter and checksum
  def make_checksummed_msg(self, message_name, bus, values,
                           counter_field="COUNTER", checksum_field="CHECKSUM"):
    # Packs twice. The checksum covers the packed bytes, so we have to
    # pack once to get them, then pack again with the checksum filled in.
    self._require_packer()

    # copy so we don't change the caller's dict
    values = dict(values)
    values[counter_field] = self.counter
    values[checksum_field] = 0

    # pack 1 - checksum still zero
    first_msg = self.packer.make_can_msg(message_name, bus, values)
    first_pass = first_msg[1]

    # checksum covers bytes 0-6, goes in byte 7
    values[checksum_field] = nissan_checksum(first_pass[:7])

    # pack 2 - real checksum
    msg = self.packer.make_can_msg(message_name, bus, values)

    # bump counter only after the frame is built
    self.counter = (self.counter + 1) % 0x10

    return msg

  # ADDED (Saul) - checksum for 634, plain byte sum not CRC8/J1850,
  # so make_checksummed_msg does not work for this message
  def sum_checksum_27a(self, data):
    return (sum(data[1:7]) + 0x7C) & 0xFF

  # ADDED (Saul) - turn signals and hazards on 634, which carries the
  # actual lamp state
  def create_turn_signal_control(self, left, right):

    self._require_packer()

    values = {
      "LEFT_BLINKER_27A": 1 if left else 0,
      "RIGHT_BLINKER_27A": 1 if right else 0,
      "COUNTER_27A": self.counter,
      "CHECKSUM_27A": 0,
    }

    # pack 1 - checksum still zero
    first_msg = self.packer.make_can_msg("TURN_SIGNALS_27A", 0, values)
    values["CHECKSUM_27A"] = self.sum_checksum_27a(first_msg[1])

    # pack 2 - real checksum
    msg = self.packer.make_can_msg("TURN_SIGNALS_27A", 0, values)

    self.counter = (self.counter + 1) % 0x10

    return msg

  # ADDED - LKAS steering message, ID 0x169
  # Fields come from the 2018 Leaf DBC. Still has to be confirmed on our 2025.
  # The packer handles the raw encoding, so DESIRED_ANGLE is in degrees.
  def create_steering_control(self, angle_deg, lka_active, max_torque=1.0):

    self._require_packer()

    # refuse anything past the limit instead of letting it wrap around
    if abs(angle_deg) > MAX_ANGLE:
      raise ValueError(f"Angle {angle_deg} outside +/-{MAX_ANGLE} degrees")

    values = {
      "DESIRED_ANGLE": angle_deg,
      "SET_0x80_2": 0x80,     # constant, openpilot always sends this
      "SET_0x80": 0x80,       # constant
      "MAX_TORQUE": max_torque,
      "LKA_ACTIVE": 1 if lka_active else 0,
    }

    # LKAS is on bus 0 and carries both a counter and a checksum
    return self.make_checksummed_msg("LKAS", 0, values)

  # Hazard lights, only testing hazards for now
  def create_hazard_control(self, enabled):

      #Makes sure CANPacker is given DBC value
      self._require_packer()   # CHANGED - was an inline if

      #Need these values from Cabana
      message_name = "HAZARD_SWITCH"
      bus = 0

      #From cabana
      # TODO: put the hazard signal here and set it from `enabled`
      values = {"HAZARD_BUTTON": 1 if enabled else 0}

      #Adding some safety checks
      if message_name is None or bus is None:
        raise NotImplementedError(
          "Hazard CAN message not identified"
        )


      #Now validated, create actual signals
      # NOTE - if this message has COUNTER and CHECKSUM, use
      # make_checksummed_msg instead

      return self.packer.make_can_msg(message_name, bus, values)
