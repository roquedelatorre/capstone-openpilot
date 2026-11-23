"""Acceleration with the camera isolated AND its messages forwarded by us.

OUTLINE / SCAFFOLD. This is the acceleration sibling of steer_forward.py and
deliberately keeps its exact shape: self-contained encoder, a self-test against
a REAL captured frame, a passthrough-first validation mode, the forward-and-
substitute loop that reuses the camera's counter, and an atexit relay failsafe.

It will NOT run until the TODOs below are filled from a Cabana capture:
  - which message actually carries the ACC acceleration command, and its bus
  - that message's byte layout (where the accel value lives)
  - its counter location (if any)
  - its checksum algorithm (if any)
  - a real reference frame to self-test the encoder against

Everything marked "TODO CONFIRM" is a guess from nissan_leaf_2025_capstone.dbc,
not something we have seen the car send.

READ THIS BEFORE YOU PLAN A TEST
--------------------------------
Acceleration is NOT the same kind of problem steering was, for two reasons that
the steering work already surfaced:

  1. openpilot's Nissan port is LATERAL ONLY. It steers on 0x169 and leaves
     longitudinal to stock ProPILOT. So unlike steering, there is no
     openpilot-blessed "send this to accelerate" message to copy. The command
     we substitute here is our own hypothesis, not something openpilot ships.

  2. "A Leaf will not engage cruise in park" (steer_forward.py, nissan-mode
     note). ProPILOT/ACC is what moves the car longitudinally, and it will not
     engage stationary. A chocked-in-park bench demo that moved the WHEEL
     cannot move the DRIVETRAIN the same way - there is nothing for an accel
     command to act on with the car in park and the ACC disengaged.

     => Decide what "success" means before you capture anything. The honest
        near-term deliverable is almost certainly NOT "the car accelerated in
        the garage." It is one of:
          (a) prove the forwarder holds ACC state up with the relay open
              (passthrough mode, analogous to the steering passthrough), or
          (b) show the ACC SET_SPEED / state responds to a substituted command
              on a dyno / closed course with a driver, ACC actually engaged,
              brake-ready. That is a road test, governed by the stock system's
              own limits, NOT a garage bring-up.

TWO MECHANISMS - pick one when you capture
------------------------------------------
  (A) Direct accel substitution (scaffolded here). If the camera/ADAS sends a
      longitudinal-accel command on bus 2 (candidate: 0x292 ACCEL, ROUGH_ACCEL
      in m/s^2), we intercept it and substitute our own value, exactly like
      0x169. Needs the message to actually BE a command the car acts on.

  (B) Cruise-button nudge (fallback, not scaffolded). Spoof RES/SET on
      0x239 CRUISE_THROTTLE to walk the stock ProPILOT set-speed up/down while
      ACC is engaged. More like the hazard-button pulse than like steering.
      Choose this if (A) turns out to be a report, not a command.

MODES - run passthrough FIRST (same philosophy as steering)
-----------------------------------------------------------
  --mode passthrough  (default) forward everything bus 2 -> bus 0 unchanged,
                      including the camera's own accel command. Send nothing of
                      our own. Success = ACC stays available/engaged and the
                      forward rate matches the camera's native rate. THIS IS
                      THE VALIDATION STEP - if it fails, no accel result means
                      anything.
  --mode accel        forward everything except the accel command, substitute
                      ours, reusing the camera's counter.

    sudo systemctl stop comma
    python accel_forward.py --mode passthrough --seconds 15
    python accel_forward.py --mode accel --target 0.5 --seconds 10   # road only
"""

import argparse
import atexit
import time

from manual_can_interface import CANInterface

# ------------------------------------------------------------------ addresses
# TODO CONFIRM every address and bus here against a relay-OPEN Cabana capture.
# The bus-0 fingerprint in the port notes was taken relay-CLOSED (panda
# bridging), so it does NOT tell us the true origin side - same open question
# that applies to 0x169/0x30f. Capture with the relay open to settle it.

# The command we substitute. Leading candidate from the DBC: 0x292 ACCEL.
# UNCONFIRMED as a command (vs a report), and the bus is a guess.
ACCEL_ADDR = 0x292            # TODO confirm id (dec 658)
ACCEL_CAM_BUS = 2             # TODO confirm the ADAS transmits it on bus 2

# Read-back signals that tell us whether anything actually happened.
CRUISE_STATE_ADDR = 0x30F     # CRUISE_STATE.CRUISE_ENABLED  (bus 2)  dec 783
PROPILOT_HUD_ADDR = 0x2B1     # PROPILOT_HUD.SET_SPEED        (bus 2)  dec 689
WHEEL_FRONT_ADDR = 0x284      # WHEEL_SPEEDS_FRONT            (bus 0)  dec 644
ACCEL_FB_ADDR = 0x292         # ROUGH_ACCEL reported accel    (bus ?)  dec 658

CAM_BUS = 2
CAR_BUS = 0

# From nissan_leaf_2025_capstone.dbc:
#   BO_ 658 ACCEL:  SG_ ROUGH_ACCEL : 7|16@0+ (0.0005,-16.5) [-16.5|16.27] "m/s^2"
#   BO_ 644 WHEEL_SPEEDS_FRONT: 0.005 KPH per bit
#   BO_ 689 PROPILOT_HUD: SG_ SET_SPEED : 39|8@0+ (1,0)
ACCEL_SCALE = 0.0005
ACCEL_OFFSET = -16.5
WHEEL_SCALE = 0.005           # KPH per bit

# A sane ceiling for a first command, well inside the DBC range. Tighten once
# you know what the stock system itself commands in normal driving.
MAX_ACCEL_MS2 = 1.0           # TODO set from captured stock-ACC values

# What the camera's native accel-command rate is. steering's camera 0x169 ran
# ~120 Hz; fill this in from the capture so the forwarder can be judged.
EXPECTED_CAM_HZ = None        # TODO measure


# ------------------------------------------------------------------ checksum
# Steering used CRC-8/SAE-J1850 (poly 0x1D, register seeded 0xFF, final XOR
# 0xFF). We do NOT yet know what, if anything, the accel command uses. Capture
# the message, hold the payload constant, and watch which byte tracks the rest
# (checksum) and which increments mod-N (counter).
#
# Keep the J1850 helper here so it is ready if the accel message turns out to
# use the same family as 0x169/0x185 - but do not assume it does.

def _gen_crc8_table(poly):
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return table


_CRC8J1850 = _gen_crc8_table(0x1D)


def nissan_checksum(data):
    """opendbc's mk_crc8_fun(CRC8J1850, init_crc=0x00, xor_out=0xFF)."""
    crc = 0xFF
    for b in data:
        crc = _CRC8J1850[crc ^ b]
    return crc ^ 0xFF


def accel_checksum(data_without_checksum):
    """TODO: the real algorithm for the accel command.

    Replace the body once captured. Options to test against the data:
      - CRC-8/J1850 over the first N bytes (reuse nissan_checksum)
      - additive sum + constant, like 0x27A: (sum(bytes) + K) & 0xFF
      - no checksum at all (some Nissan command msgs have none, e.g. 0x3AF)
    Until then this returns 0 so the encoder is syntactically complete but
    self_test() below will refuse to run.
    """
    raise NotImplementedError("accel command checksum not captured yet")


# ------------------------------------------------------------------ encode/decode
# TODO CONFIRM the byte layout. The skeleton below assumes a signed 16-bit
# big-endian accel starting at byte 0 using the ROUGH_ACCEL scale/offset, an
# active bit and a 4-bit counter somewhere, and a trailing checksum byte - all
# PLACEHOLDERS copied from the shape of the steering frame, NOT measured.

def encode_accel(accel_ms2, active, counter):
    """Build the accel command. PLACEHOLDER layout - verify every field.

    Mirrors encode_lkas: pack the value, set an active bit, drop the counter
    in, then checksum the bytes before it.
    """
    raw = int(round((accel_ms2 - ACCEL_OFFSET) / ACCEL_SCALE))
    raw = max(0, min(raw, (1 << 16) - 1))

    data = bytearray(8)
    data[0] = (raw >> 8) & 0xFF          # TODO confirm start bit / endianness
    data[1] = raw & 0xFF                 # TODO confirm
    # data[?] = active bit               # TODO where is the engage/active bit?
    # data[?] = counter & 0x0F           # TODO where is the counter, how wide?
    # data[7] = accel_checksum(bytes(data[:7]))   # TODO once checksum known
    return bytes(data)


def decode_accel_fb(data):
    """Reported acceleration from 0x292 ROUGH_ACCEL (m/s^2)."""
    if len(data) < 2:
        return None
    raw = (data[0] << 8) | data[1]       # TODO confirm against DBC start bit 7|16@0+
    return ACCEL_OFFSET + raw * ACCEL_SCALE


def decode_wheel_speed(data):
    """Front wheel speed in KPH (average of FL/FR). TODO confirm byte order."""
    if len(data) < 8:
        return None
    fr = (data[0] << 8) | data[1]
    fl = (data[2] << 8) | data[3]
    return (fr + fl) * 0.5 * WHEEL_SCALE


def decode_cruise_enabled(data):
    """CRUISE_STATE.CRUISE_ENABLED, bit 3. TODO confirm."""
    if len(data) < 1:
        return None
    return bool((data[0] >> 3) & 0x01)


def decode_set_speed(data):
    """PROPILOT_HUD.SET_SPEED, byte 4 (bit 39|8@0+). TODO confirm."""
    if len(data) < 5:
        return None
    return data[4]


# ------------------------------------------------------------------ self test
# Same contract as steering: refuse to command until the encoder can reproduce
# a frame we actually captured from this car. Drop a real one in here.
REFERENCE_FRAME = None          # TODO bytes.fromhex("....") from a real capture
REFERENCE_ACCEL = None          # its decoded accel (m/s^2)
REFERENCE_ACTIVE = None
REFERENCE_COUNTER = None


def self_test():
    if REFERENCE_FRAME is None:
        print("self-test: NO REFERENCE FRAME SET.")
        print("   Capture one real accel-command frame from the car, put its")
        print("   bytes in REFERENCE_FRAME and its decoded fields below, and")
        print("   fill in encode_accel + accel_checksum. The script will not")
        print("   command until the encoder reproduces that frame exactly -")
        print("   same contract as steer_forward.py.")
        return False
    try:
        built = encode_accel(REFERENCE_ACCEL, REFERENCE_ACTIVE, REFERENCE_COUNTER)
    except NotImplementedError as exc:
        print(f"self-test: encoder not finished - {exc}")
        return False
    ok = built == REFERENCE_FRAME
    print("encoder self-test against a real frame from this car:")
    print(f"   car sent : {' '.join(f'{b:02x}' for b in REFERENCE_FRAME)}")
    print(f"   we build : {' '.join(f'{b:02x}' for b in built)}")
    print(f"   -> {'MATCH' if ok else 'MISMATCH'}")
    if not ok:
        print("Refusing to run. Encoder cannot reproduce a captured frame.")
    return ok


# ------------------------------------------------------------------ panda io
# Identical to steer_forward.py - the bus/flag masking and batch sender are not
# steering-specific.

def unpack(msg):
    """panda reports the bus with flag bits in the top nibble - mask them."""
    if len(msg) == 3:
        addr, data, bus = msg[0], msg[1], msg[2]
    else:
        addr, data, bus = msg[0], msg[2], msg[3]
    return addr, bytes(data), bus & 0x0F


class Sender:
    """Batch sender that works out which panda API this build exposes."""

    def __init__(self, panda):
        self.panda = panda
        self.many = None
        self.errors = 0

    def send(self, frames):
        if not frames:
            return 0
        if self.many is not False:
            try:
                self.panda.can_send_many([(a, d, b) for a, d, b in frames])
                self.many = True
                return len(frames)
            except TypeError:
                self.many = False
            except Exception:
                self.many = False
        sent = 0
        for addr, data, bus in frames:
            try:
                self.panda.can_send(addr, data, bus)
                sent += 1
            except Exception:
                self.errors += 1
        return sent


# ------------------------------------------------------------------ main
def main():
    p = argparse.ArgumentParser(
        description="forward the camera's messages and substitute our accel command")
    p.add_argument("--mode", choices=("passthrough", "accel"),
                   default="passthrough",
                   help="passthrough forwards everything unchanged and sends "
                        "nothing of our own - run this FIRST (default). accel "
                        "substitutes our accel command")
    p.add_argument("--target", type=float, default=0.0, metavar="MS2",
                   help="accel mode: target acceleration in m/s^2 (default 0)")
    p.add_argument("--rate", type=float, default=0.5, metavar="MS2_S",
                   help="accel mode: ramp rate toward target, m/s^2 per second "
                        "(default 0.5 - command no jerk)")
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--listen", type=float, default=4.0, metavar="SEC",
                   help="baseline listen with the relay closed (default 4)")
    args = p.parse_args()

    # Guard rails - tighten once real numbers are known.
    if abs(args.target) > MAX_ACCEL_MS2:
        p.error(f"keep --target within +/-{MAX_ACCEL_MS2} m/s^2 for now")
    if args.seconds > 30:
        p.error("30 s is plenty")

    print()
    if not self_test():
        return                      # refuses until the TODOs are filled
    print()

    can = CANInterface(simulation=False, allow_output=True)
    panda = can.panda
    sender = Sender(panda)

    closed = {"done": False}

    def close_once():
        if closed["done"]:
            return
        closed["done"] = True
        try:
            can.set_relay(False)
            print("failsafe: relay closed on exit")
        except Exception as exc:
            print(f"!! FAILSAFE COULD NOT CLOSE THE RELAY: {exc}")
            print("   Camera may still be isolated. Ignition cycle the car.")

    atexit.register(close_once)

    # ---- baseline, relay closed ------------------------------------------
    print(f"baseline - relay CLOSED, listening {args.listen:.1f} s")
    can.set_relay(False)
    time.sleep(0.6)

    cruise_on = None
    set_speed = None
    meas_accel = None
    end = time.monotonic() + args.listen
    while time.monotonic() < end:
        for msg in panda.can_recv():
            addr, data, _bus = unpack(msg)
            if addr == CRUISE_STATE_ADDR:
                cruise_on = decode_cruise_enabled(data)
            elif addr == PROPILOT_HUD_ADDR:
                set_speed = decode_set_speed(data)
            elif addr == ACCEL_FB_ADDR:
                meas_accel = decode_accel_fb(data)

    print(f"   ACC engaged : {cruise_on}")
    print(f"   set speed   : {set_speed}")
    print(f"   accel (fb)  : {meas_accel}")
    if not cruise_on:
        print("   NOTE: ACC is not engaged. A substituted accel command has")
        print("   nothing to act on - this run can only validate forwarding,")
        print("   not motion. (A Leaf will not engage cruise in park.)")

    print()
    print(f"mode: {args.mode}")
    if args.mode == "passthrough":
        print("   forwarding bus 2 -> bus 0 unchanged. WE SEND NOTHING OF OURS.")
        print("   Success = ACC stays available and the forward rate matches")
        print("   the camera's native accel-command rate.")
    else:
        print(f"   forwarding bus 2 -> bus 0, substituting 0x{ACCEL_ADDR:03x}")
        print(f"   target {args.target:+.2f} m/s^2, ramp {args.rate:.2f} m/s^2/s")
        print("   our frame reuses the camera's counter")
        print("   *** ROAD / DYNO ONLY, ACC ENGAGED, DRIVER BRAKE-READY ***")
    print(f"   holding {args.seconds:.1f} s")
    print()
    print("CONFIRM THE TEST ENVIRONMENT IS SAFE FOR LONGITUDINAL MOTION.")
    if input("type GO to continue: ").strip() != "GO":
        return

    print()
    print("opening relay - camera isolated, we are now its only route to the car")
    can.set_relay(True)
    time.sleep(0.4)

    # ---- the forwarding loop ---------------------------------------------
    # TODO CONFIRM: how the accel command carries its counter. steering reused
    # the camera's 0x169 counter verbatim; do the same once you know where it
    # lives in this message.
    fwd_counts = {}
    subbed = 0
    cam_cmd = 0
    current = 0.0
    last_ramp = time.monotonic()
    last_report = time.monotonic()
    t0 = time.monotonic()
    end = t0 + args.seconds

    try:
        while time.monotonic() < end:
            now = time.monotonic()

            if args.mode == "accel":
                step = args.rate * (now - last_ramp)
                last_ramp = now
                if current < args.target:
                    current = min(args.target, current + step)
                elif current > args.target:
                    current = max(args.target, current - step)

            out = []
            for msg in panda.can_recv():
                addr, data, bus = unpack(msg)

                # read-back for the result, regardless of bus
                if addr == ACCEL_FB_ADDR:
                    meas_accel = decode_accel_fb(data)
                elif addr == CRUISE_STATE_ADDR:
                    cruise_on = decode_cruise_enabled(data)
                elif addr == PROPILOT_HUD_ADDR:
                    set_speed = decode_set_speed(data)

                if bus != CAM_BUS:
                    continue

                if addr == ACCEL_ADDR:
                    cam_cmd += 1
                    if args.mode == "accel":
                        # TODO reuse the camera's counter from `data`
                        counter = 0
                        out.append((ACCEL_ADDR,
                                    encode_accel(current, True, counter),
                                    CAR_BUS))
                        subbed += 1
                        continue
                    # passthrough: forward the camera's own command untouched

                out.append((addr, data, CAR_BUS))
                fwd_counts[addr] = fwd_counts.get(addr, 0) + 1

            sender.send(out)

            if now - last_report >= 1.0:
                last_report = now
                print(f"  t+{now - t0:5.1f}s  fwd {sum(fwd_counts.values()):6d}"
                      f"  sub {subbed:5d}  cam_cmd {cam_cmd:5d}"
                      f"  ACC {cruise_on}  set {set_speed}  accel {meas_accel}")

    except KeyboardInterrupt:
        print("\ninterrupted")

    finally:
        close_once()
        elapsed = max(1e-6, time.monotonic() - t0)

        print()
        print("=== result ===")
        print(f"elapsed {elapsed:.1f} s")
        print(f"camera accel cmd seen: {cam_cmd} ({cam_cmd / elapsed:.1f} Hz)")
        if EXPECTED_CAM_HZ:
            print(f"   expected ~{EXPECTED_CAM_HZ:.0f} Hz - well under means we"
                  f" are dropping frames")
        print(f"frames forwarded     : {sum(fwd_counts.values())} "
              f"({sum(fwd_counts.values()) / elapsed:.0f} /s)")
        if args.mode == "accel":
            print(f"accel cmd substituted: {subbed} ({subbed / elapsed:.1f} Hz)")
        if sender.errors:
            print(f"send errors          : {sender.errors}")
        print(f"send path            : "
              f"{'can_send_many' if sender.many else 'can_send one at a time'}")

        print()
        print("forwarded by address:")
        for addr, n in sorted(fwd_counts.items()):
            print(f"   0x{addr:03x}  {n:6d}  {n / elapsed:6.1f} Hz")

        print()
        print(f"final  ACC engaged {cruise_on}  set speed {set_speed}  "
              f"accel fb {meas_accel}")

        print()
        can.print_summary()


if __name__ == "__main__":
    main()