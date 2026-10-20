# Simple script to write to the car and test functions
#
# CHANGED - the stack now runs a background transmitter, so this script has
# to start and stop it. Steering and blinkers latch and keep streaming until
# you clear them; they are no longer one frame per keypress.

import argparse

from vehicle_control import Vehicle_Controls


def build_parser():
  p = argparse.ArgumentParser(description="Manual Leaf CAN control")
  # CHANGED - simulation was hardcoded True, so there was no way to reach
  # hardware without editing the file.
  p.add_argument("--hardware", action="store_true",
                 help="talk to a real panda instead of simulating")
  p.add_argument("--dbc", default=None,
                 help="DBC path (defaults to nissan_leaf_2025_capstone.dbc)")
  p.add_argument("--verbose-tx", action="store_true",
                 help="print every frame the 100 Hz loop sends (very noisy, "
                      "and it will fight the menu for the terminal)")
  p.add_argument("--hold-timeout", type=float, default=None, metavar="SEC",
                 help="seconds a steering hold stays engaged before LKA_ACTIVE "
                      "is cleared (default 60, 0 disables the deadman)")
  return p


def steering_menu(car):
  print("\nSteering Control")
  print("1. Set steering angle (engages LKA and holds it)")
  print("2. Steering OFF (keep streaming, LKA clear)")
  print("3. Stop sending 0x169 entirely")
  print("4. Refresh hold (push the deadman out, same angle)")

  choice = input("Enter command: ").strip()

  if choice == "1":
    try:
      angle = float(input("Angle in degrees: "))
    except ValueError:
      print("Invalid angle")
      return
    car.steer(angle)
    print(f"engaged, ramping to {angle:+.1f} deg")
    print(car.steering.status())

  elif choice == "4":
    if car.steer_refresh():
      print("hold refreshed: " + car.steering.status())
    else:
      print("nothing to refresh (not engaged, or deadman disabled)")

  elif choice == "2":
    car.steer_off()
    print("LKA_ACTIVE cleared, still streaming 0x169")

  elif choice == "3":
    car.steer_stop_sending()
    print("stopped sending 0x169")

  else:
    print("Command not found")


def hazard_menu(car):
  print("\nHazard Control")
  print("1. Press Hazards   (0x3AF button pulse)")
  print("2. Release Hazards (0x3AF button pulse)")
  print("3. Hazards ON  (via 0x27A, both blinker bits)")
  print("4. Hazards OFF (via 0x27A)")

  choice = input("Enter command: ").strip()

  if choice == "1" or choice == "2":
    # CHANGED (Saul) - these sent one frame each. Our capture shows a real
    # press is 2-3 frames at 10 Hz, and the same pulse serves ON and OFF,
    # so both options now send a proper pulse and the hazards toggle.
    width = car.hazard_pulse()
    print(f"pulsed 0x3AF for {width:.3f} s - hazards should TOGGLE")

  elif choice == "3":
    car.hazards_27a_ON()

  elif choice == "4":
    car.hazards_27a_OFF()

  else:
    print("Command not found")


def blinker_menu(car):
  print("\nBlinker Control")
  print("1. Left")
  print("2. Right")
  print("3. Off")

  choice = input("Enter command: ").strip()

  if choice == "1":
    car.blinker_left()
  elif choice == "2":
    car.blinker_right()
  elif choice == "3":
    car.blinkers_off()
  else:
    print("Command not found")


def not_yet(car, label, fn):
  print(f"\n{label}")
  try:
    fn()
  except NotImplementedError as exc:
    print(exc)


def main_test(car):
  while True:
    # ADDED - drain anything the tx thread wanted to say. Doing it here, just
    # before we take over the terminal, is what keeps background notices from
    # landing in the middle of the operator's input.
    car.print_notices()

    print("\nManual Vehicle Control")

    print("1. Hazards")
    print("2. AC")
    print("3. Steering")
    print("4. Windshield Wiper")
    print("5. Blinkers")
    print("6. Show sent messages (tail)")
    print("7. Export frames as C")
    print("8. Status / transmitter stats")
    print("9. Frame count by ID")
    print("0. Exit")

    command = input("Enter command: ").strip()

    if command == "1":
      hazard_menu(car)

    elif command == "2":
      not_yet(car, "AC Control", car.AC_ON)

    elif command == "3":
      steering_menu(car)

    elif command == "4":
      not_yet(car, "Windshield Wiper Control", car.wipers_ON)

    elif command == "5":
      blinker_menu(car)

    elif command == "6":
      car.can.print_log(last=40)

    elif command == "7":
      car.can.export_c()

    elif command == "8":
      car.print_status()

    elif command == "9":
      car.can.print_summary()

    elif command == "0":
      print("Exiting manual vehicle control.")
      break

    else:
      print("Command not found")


if __name__ == "__main__":
  args = build_parser().parse_args()

  car = Vehicle_Controls(
    dbc_name=args.dbc,
    simulation=not args.hardware,
    verbose_tx=args.verbose_tx,
    hold_timeout_s=args.hold_timeout,
  )

  if args.hold_timeout == 0:
    print("WARNING: steering deadman disabled. A hold will stay engaged until "
          "you clear it or kill the process.")

  if args.hardware:
    print("\n*** HARDWARE MODE - panda will be set to SAFETY_ALLOUTPUT ***")
    print("*** Car in park, wheels chocked, nobody near the wheel. ***")
    if input("type GO to continue: ").strip() != "GO":
      raise SystemExit("aborted")

  # ADDED - context manager so the transmitter is always released, including
  # on Ctrl-C. Leaving a 100 Hz steering stream running is exactly what you
  # do not want on exit.
  with car:
    try:
      main_test(car)
    except KeyboardInterrupt:
      print("\ninterrupted, releasing controls")