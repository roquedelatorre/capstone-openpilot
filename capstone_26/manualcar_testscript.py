#Simple script to write to the car and test functions
from vehicle_control import Vehicle_Controls

car = Vehicle_Controls()

def main_test():

  while True:
    print("Manual Vehicle Control")
    print("1. Hazards ON")
    print("2. Hazards OFF")

    command = input("Enter command: ")

    if command == "1":
      car.hazards_ON(True)
    elif command == "2":
      car.hazards_OFF(True)

    else("Command not found")

if __name__ == "__main__"
  main_test()
