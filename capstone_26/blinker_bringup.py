# ADDED - first-hardware-contact test. Blinkers only, nothing else.
#
# This is deliberately the smallest thing that proves the whole chain works:
# DBC -> packer -> counter -> checksum -> panda -> bus -> a lamp you can see
# from outside the car. 0x27A is the right message to start on because our
# group confirmed it end-to-end by capture, and because a blinker is
# observable and harmless.
#
# It does NOT touch steering and does NOT open the harness relay.
#
# Run on the comma device (openpilot must be stopped first - pandad holds the
# panda otherwise):
#
#   python capstone_26/blinker_bringup.py --check        # read-only, no TX
#   python capstone_26/blinker_bringup.py --left 3       # left blinker, 3 s
#
# Start with --check. If the harness or safety mode is wrong, nothing you
# transmit will reach the bus and you want to know that before you wonder
# about your DBC.

import argparse
import time

from manual_can_interface import CANInterface
from manual_nissancan import (CarController_Manual, HAZARD_RATE_HZ,
                              HAZARD_SWITCH_BUS)
from manual_tx_loop import PeriodicTransmitter, BlinkerHold

TURN_SIGNAL_RATE_HZ = 50.0


def main():
    p = argparse.ArgumentParser(description="0x27A blinker bring-up")
    p.add_argument("--check", action="store_true",
                   help="connect read-only (SAFETY_SILENT), print health, exit")
    p.add_argument("--left", type=float, metavar="SEC", default=0.0,
                   help="hold the left blinker for N seconds")
    p.add_argument("--right", type=float, metavar="SEC", default=0.0,
                   help="hold the right blinker for N seconds")
    p.add_argument("--hazard", type=float, metavar="SEC", default=0.0,
                   help="hold both 0x27A bits for N seconds (reports state; "
                        "per our capture this probably will NOT light anything)")
    p.add_argument("--press", action="store_true",
                   help="pulse 0x3AF like a real hazard button press. This is "
                        "the command path: every hazard period in our capture "
                        "began 20-30 ms after a pulse here.")
    p.add_argument("--press-width", type=float, default=None, metavar="SEC",
                   help="pulse width (default 0.25 s, the measured median)")
    p.add_argument("--simulate", action="store_true",
                   help="do not touch hardware, just print frames")
    p.add_argument("--dbc", default=None)
    args = p.parse_args()

    duration = max(args.left, args.right, args.hazard)
    if not args.check and not args.press and duration <= 0:
        p.error("pick one of --check / --press / --left SEC / --right SEC / "
                "--hazard SEC")
    if duration > 30:
        p.error("30 s is plenty for a lamp test")

    # --check never transmits: SAFETY_SILENT, so even a bug cannot put a
    # frame on the bus.
    can = CANInterface(simulation=args.simulate,
                       allow_output=not args.check)

    if args.check:
        print()
        print("read-only check complete. Nothing was transmitted.")
        print("If harness says NOT CONNECTED or the safety mode did not take,")
        print("fix that before trying --left.")
        return

    # --press goes through Vehicle_Controls because the pulse needs the
    # transmitter, and that is where the timing lives.
    if args.press:
        from vehicle_control import Vehicle_Controls
        car = Vehicle_Controls(dbc_name=args.dbc, simulation=args.simulate)
        width = args.press_width or 0.25
        print()
        print(f"about to pulse 0x3AF (hazard button) for {width:.3f} s "
              f"at {HAZARD_RATE_HZ:.0f} Hz on bus {HAZARD_SWITCH_BUS}.")
        print("this emulates one press, so expect hazards to TOGGLE.")
        print("watch the lamps and the dash indicator.")
        if input("type GO to continue: ").strip() != "GO":
            raise SystemExit("aborted")
        with car:
            car.hazard_pulse(duration_s=width)
        print()
        print("pulse sent.")
        car.notices.print_all()
        car.can.print_summary()
        print()
        car.can.export_c()
        return

    nissan = CarController_Manual(args.dbc)
    blinkers = BlinkerHold(nissan)
    tx = PeriodicTransmitter(can, base_rate_hz=TURN_SIGNAL_RATE_HZ)

    if args.hazard:
        left, right, label = True, True, "HAZARD (both bits)"
    elif args.left:
        left, right, label = True, False, "LEFT"
    else:
        left, right, label = False, True, "RIGHT"

    print()
    print(f"about to stream 0x27A at {TURN_SIGNAL_RATE_HZ:.0f} Hz: {label}")
    print(f"for {duration:.1f} s, then stop and clear.")
    print("watch the actual lamp, not the terminal.")
    if input("type GO to continue: ").strip() != "GO":
        raise SystemExit("aborted")

    blinkers.set(left, right)
    tx.set("TURN_SIGNALS_27A", blinkers.build, rate_hz=TURN_SIGNAL_RATE_HZ)
    tx.start()

    try:
        end = time.monotonic() + duration
        while time.monotonic() < end:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        # clear the lamp state, keep streaming briefly so the last frame the
        # car sees is "off", then stop
        blinkers.set(False, False)
        time.sleep(0.3)
        tx.stop()
        print()
        print("stopped.")
        tx.notices.print_all()
        tx.print_stats()
        print()
        can.print_summary()
        print()
        print("distinct frames (these are what to hard-code on the ESP32):")
        can.export_c()


if __name__ == "__main__":
    main()
