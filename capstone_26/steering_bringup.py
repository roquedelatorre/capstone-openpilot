# ADDED (Saul) - first steering test, with the two things that make it a
# real experiment instead of another null result.
#
# 1. Opens the harness intercept relay. Our bus scan showed bus 0 and bus 2
#    carrying identical traffic, which means the relay is closed and the
#    camera's own 0x169 reaches the EPS at ~94 Hz. Sending ours alongside it
#    gives the EPS two conflicting sources on the same ID. Opening the relay
#    isolates the camera so ours is the only LKAS on the car side.
#
# 2. Seeds the ramp from the MEASURED angle. SteeringHold starts at 0.0; if
#    the wheel is actually at -155 deg, the first frame claims a 155 deg
#    error and the EPS rejects it. openpilot always starts from measured.
#
# It also reads LKAS_STATUS back from 0x185 the whole time:
#     5 = INACTIVE   EPS is not engaged
#     6 = ACTIVE     EPS accepted the command
#     9 = FAULT      EPS rejected it - script stops immediately
#    14 = seen on our 2025 while the relay was open and the chassis was
#         faulted. Not in the 2018 enum.
#
# Commands RELATIVE to wherever the wheel is now.
#
#   python steering_bringup.py --watch
#   python steering_bringup.py --delta 5 --seconds 5
#   python steering_bringup.py --delta 5 --seconds 5 --no-relay --torque 2.0
#
# Car in park. Wheels chocked. Nobody with hands on the wheel.

import argparse
import functools
import time

from manual_can_interface import CANInterface
from manual_nissancan import CarController_Manual, LKAS_BUS
from manual_tx_loop import PeriodicTransmitter, SteeringHold

LKAS_RATE_HZ = 100.0

LKAS_STATUS_NAMES = {5: "INACTIVE", 6: "ACTIVE", 9: "FAULT"}


# 0x185 STEER_TORQUE_SENSOR
#   STEER_ANGLE   23|18@0+  scale -0.01  offset 1310
#   LKAS_STATUS   55|4@0+
def decode_185(data):
    if len(data) < 8:
        return None
    raw = (data[2] << 10) | (data[3] << 2) | (data[4] >> 6)
    angle = 1310 - (raw * 0.01)
    status = (data[6] >> 4) & 0x0F
    return angle, status


def unpack(msg):
    """panda returns (addr, data, bus) or (addr, busTime, data, src).
    The bus byte carries flag bits in the top nibble, so mask them off -
    that is why the scan reported buses 128 / 130 / 192 / 194."""
    if len(msg) == 3:
        addr, data, bus = msg[0], msg[1], msg[2]
    else:
        addr, data, bus = msg[0], msg[2], msg[3]
    return addr, data, bus & 0x0F


def read_angle(panda, timeout_s=2.0):
    """Listen until we get a clean 0x185 and return (angle, status)."""
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        for msg in panda.can_recv():
            addr, data, _ = unpack(msg)
            if addr == 0x185:
                result = decode_185(data)
                if result is not None:
                    return result
    return None


def main():
    p = argparse.ArgumentParser(description="0x169 steering bring-up")
    p.add_argument("--delta", type=float, default=5.0, metavar="DEG",
                   help="degrees to command RELATIVE to current wheel angle "
                        "(default 5)")
    p.add_argument("--seconds", type=float, default=5.0, metavar="SEC",
                   help="how long to hold the command (default 5)")
    # ADDED - the only software knob we have for a stationary wheel.
    # opendbc uses 1.0 for the 2018 Leaf; our DBC scale allows up to 2.55.
    p.add_argument("--torque", type=float, default=1.0, metavar="NM",
                   help="MAX_TORQUE in the LKAS frame (default 1.0, max 2.5)")
    p.add_argument("--watch", action="store_true",
                   help="read 0x185 only. No transmit, no relay.")
    p.add_argument("--no-relay", action="store_true",
                   help="do NOT open the intercept relay. The camera keeps "
                        "transmitting 0x169 alongside ours.")
    p.add_argument("--dbc", default=None)
    args = p.parse_args()

    if abs(args.delta) > 30:
        p.error("keep the first tests small - 30 deg is plenty")
    if args.seconds > 30:
        p.error("30 s is plenty")
    # ADDED
    if not 0.0 <= args.torque <= 2.5:
        p.error("MAX_TORQUE must be 0 to 2.5 (DBC scale tops out at 2.55)")

    # --watch never transmits
    can = CANInterface(simulation=False, allow_output=not args.watch)
    panda = can.panda

    # ---- read where the wheel actually is -------------------------------
    print()
    print("reading 0x185 ...")
    reading = read_angle(panda)
    if reading is None:
        print("no 0x185 received. Is the car awake? Is openpilot holding")
        print("the panda? Nothing was transmitted.")
        return

    start_angle, status = reading
    print(f"wheel is at {start_angle:+.2f} deg")
    print(f"LKAS_STATUS = {status} "
          f"({LKAS_STATUS_NAMES.get(status, 'unknown')})")

    if args.watch:
        print()
        print("watching. Ctrl+C to stop. Turn the wheel by hand and check")
        print("the angle tracks it.")
        try:
            while True:
                reading = read_angle(panda, timeout_s=1.0)
                if reading:
                    angle, status = reading
                    print(f"  {angle:+8.2f} deg   status {status} "
                          f"({LKAS_STATUS_NAMES.get(status, '?')})")
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\nstopped. Nothing was transmitted.")
        return

    target = start_angle + args.delta

    print()
    print(f"about to stream 0x169 on bus {LKAS_BUS} at {LKAS_RATE_HZ:.0f} Hz")
    print(f"  from {start_angle:+.2f} deg  ->  {target:+.2f} deg")
    print(f"  MAX_TORQUE {args.torque:.2f} Nm")          # ADDED
    print(f"  holding {args.seconds:.1f} s, then releasing")
    if not args.no_relay:
        print("  intercept relay will OPEN (camera isolated, ProPILOT off)")
    print()
    print("CAR IN PARK. WHEELS CHOCKED. HANDS OFF THE WHEEL.")
    if input("type GO to continue: ").strip() != "GO":
        raise SystemExit("aborted")

    nissan = CarController_Manual(args.dbc)

    # ADDED - SteeringHold.build() calls create_steering_control without a
    # torque argument, so it would get the 1.0 default. Bind ours instead of
    # duplicating the ramp logic here.
    nissan.create_steering_control = functools.partial(
        nissan.create_steering_control, max_torque=args.torque)

    steering = SteeringHold(nissan, rate_hz=LKAS_RATE_HZ, timeout_s=0)

    # THE SEED. Without this the first frame commands an angle the wheel is
    # nowhere near, and the EPS rejects the whole stream.
    steering.current = start_angle
    steering.target = start_angle

    tx = PeriodicTransmitter(can, base_rate_hz=LKAS_RATE_HZ)

    relay_opened = False
    try:
        if not args.no_relay:
            can.set_relay(True)
            relay_opened = True
            time.sleep(0.2)

        # stream at the current angle first so the EPS sees a steady,
        # plausible stream before anything moves
        tx.set("LKAS", steering.build, rate_hz=LKAS_RATE_HZ)
        tx.start()
        time.sleep(0.5)

        steering.engage(target)
        print(f"engaged, ramping at {steering.MAX_RATE_DEG_S:.0f} deg/s")
        print()

        end = time.monotonic() + args.seconds
        faulted = False

        while time.monotonic() < end:
            reading = read_angle(panda, timeout_s=0.5)
            if reading:
                angle, status = reading
                name = LKAS_STATUS_NAMES.get(status, "?")
                print(f"  commanded {steering.current:+8.2f}   "
                      f"measured {angle:+8.2f}   status {status} ({name})")

                if status == 9:
                    print()
                    print("!! EPS reported FAULT. Stopping.")
                    faulted = True
                    break
            time.sleep(0.2)

        if not faulted:
            print()
            print("hold finished.")

    except KeyboardInterrupt:
        print("\ninterrupted")

    finally:
        # always release, always stop, always close the relay
        steering.release()
        time.sleep(0.3)
        tx.stop()
        if relay_opened:
            can.set_relay(False)
        print()
        tx.notices.print_all()
        tx.print_stats()
        print()
        can.print_summary()
        print()
        print("distinct frames (these are what to hard-code on the ESP32):")
        can.export_c()


if __name__ == "__main__":
    main()