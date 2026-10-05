# Used to send car controls manually

# Class to send manual commands using the terminal
# Helps organize all car commands to control Leaf

from manual_nissancan import CarController_Manual
from manual_can_interface import CANInterface

class Vehicle_Controls:
    def __init__(self, dbc_name=None, simulation=True):
        self.simulation = simulation

        #builds Nissan CAN messages
        self.nissan = CarController_Manual(dbc_name)
        #Sends CAN to Nissan
        self.can = CANInterface(simulation=simulation)

    def send_car_command(self, command, value):
        if self.simulation:
            print(f"SIMULATION: {command} = {value}")
            return

        #error function incase command does not exist
        raise NotImplementedError(
            "CAN command not created"
        )

    #list of commands we want to control
    def hazards_ON(self):
        self.send_car_command("hazards", "ON")

    def hazards_OFF(self):
        self.send_car_command("hazards", "OFF")

    def AC_ON(self):
        self.send_car_command("AC", "ON")

    def AC_OFF(self):
        self.send_car_command("AC", "OFF")

    def set_temp(self, temperature):
        if not 16 <= temperature <= 30:
            raise ValueError("Temperature outside range")

        self.send_car_command("temperature", temperature)


# Main program
if __name__ == "__main__":
    car = Vehicle_Controls(simulation=True)

    #Currently runs the commands but does not output CAN message to the car
    while True:
        command = input("Enter command: ").strip().lower()

        if command == "hazards_on":
            car.hazards_ON()

        elif command == "hazards_off":
            car.hazards_OFF()

        elif command == "ac_on":
            car.AC_ON()

        elif command == "ac_off":
            car.AC_OFF()

        #Allows user to unput a certain temperature to car
        elif command == "set_temp":
            try:
                temperature = float(
                    input("Enter temperature: ")
                )
                car.set_temp(temperature)
            except ValueError as error:
                print(error)

        elif command == "exit":
            break

        else:
            print("Unknown command")
