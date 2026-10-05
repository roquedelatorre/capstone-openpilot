#Simple script to write to the car and test functions
from vehicle_control import Vehicle_Controls

car = Vehicle_Controls()

def main_test():

  while True:
    print("Manual Vehicle Control")
    print("1. Hazards ON")
    print("2. Hazards OFF")
    print("3. Steering angle")     # ADDED
    print("4. Steering OFF")       # ADDED
    print("5. Show sent messages") # ADDED
    print("6. Exit")               # ADDED - no way to quit before

    command = input("Enter command: ").strip()

    if command == "1":
      car.hazards_ON()    # FIXED - was hazards_ON(True), takes no arguments
    elif command == "2":
      car.hazards_OFF()   # FIXED - same

    # ADDED - type an angle and watch the frame it builds
    elif command == "3":
      try:
        angle = float(input("Angle in degrees: "))
        car.steer(angle)
      except ValueError as error:
        print(error)

    elif command == "4":    # ADDED
      car.steer_off()

    elif command == "5":    # ADDED
      car.can.print_log()

    elif command == "6":    # ADDED
      break

    else:                 # FIXED - was else("Command not found"), not valid Python
      print("Command not found")

if __name__ == "__main__":    # FIXED - missing colon, file would not run
  main_test()
