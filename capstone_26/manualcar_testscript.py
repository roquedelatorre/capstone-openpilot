# Simple script to write to the car and test functions
from vehicle_control import Vehicle_Controls

car = Vehicle_Controls(simulation=True)


def main_test():

  while True:
    #Main menu for car commands
    print("\nManual Vehicle Control")

    print("1. Hazards")
    print("2. AC")
    print("3. Steering")
    print("4. Windshield Wiper")
    print("5. Show sent messages")
    print("6. Exit")

    #Command will be sent here
    command = input("Enter command: ").strip()


    # Hazards sub-menu
    if command == "1":

      print("\nHazard Control")
      print("1. Hazards ON")
      print("2. Hazards OFF")

      hazard_command = input("Enter command: ").strip()

      if hazard_command == "1":
        car.hazards_ON()

      elif hazard_command == "2":
        car.hazards_OFF()

      else:
        print("Command not found")


    # AC sub-menu
    elif command == "2":

      print("\nAC Control")
      print("1. AC ON")
      print("2. AC OFF")

      ac_command = input("Enter command: ").strip()

      # Add later:
      # if ac_command == "1":
      #   car.ac_ON()
      # elif ac_command == "2":
      #   car.ac_OFF()


    # Steering sub-meny
    elif command == "3":

      print("\nSteering Control")
      print("1. Set steering angle")
      print("2. Steering OFF")

      steering_command = input("Enter command: ").strip()

      if steering_command == "1":
        try:
          angle = float(input("Angle in degrees: "))
          car.steer(angle)

        except ValueError:
          print("Invalid angle")

      elif steering_command == "2":
        car.steer_off()

      else:
        print("Command not found")


    # Wipers sub-meny
    elif command == "4":

      print("\nWindshield Wiper Control")
      print("1. Wipers ON")
      print("2. Wipers OFF")

      wiper_command = input("Enter command: ").strip()

      # Add later:
      # if wiper_command == "1":
      #   car.wipers_ON()
      # elif wiper_command == "2":
      #   car.wipers_OFF()


    # Car-long menu
    elif command == "5":
      car.can.print_log()


    # Exit
    elif command == "6":
      print("Exiting manual vehicle control.")
      break


    else:
      print("Command not found")


if __name__ == "__main__":
  main_test()