#Simple script to write to the car and test functions
from vehicle_control import Vehicle_Controls

car = Vehicle_Controls(simulation=True)

def main_test():

  while True:
    print("\nManual Vehicle Control")
    print("1. Hazards ON")
    print("2. Hazards OFF")
    print("3. Exit")

    command = input("Enter command: ")

    if command == "1":
      car.hazards_ON()
    elif command == "2":
      car.hazards_OFF()
    elif command == "3":
      break

    else :
      print("Command not found")

if __name__ == "__main__":
  main_test()
