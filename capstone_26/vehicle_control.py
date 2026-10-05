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
        # CHANGED - this used to just print, so self.nissan and self.can were
        # never used. Now it builds the message then sends it.
        if command == "hazards":
            message = self.nissan.create_hazard_control(value == "ON")

        # ADDED - steering goes through the LKAS builder
        elif command == "steer":
            message = self.nissan.create_steering_control(value, lka_active=True)

        elif command == "steer_off":
            # same message with LKA off, angle held at 0
            message = self.nissan.create_steering_control(0.0, lka_active=False)

        else:
            #error function incase command does not exist
            raise NotImplementedError(
                f"CAN command not created: {command}"
            )

        self.can.send(message)

    #list of commands we want to control
    def hazards_ON(self):
        self.send_car_command("hazards", "ON")

    def hazards_OFF(self):
        self.send_car_command("hazards", "OFF")

    # ADDED - steering, range checked the same way set_temp is
    def steer(self, angle_deg):
        if not -600 <= angle_deg <= 600:
            raise ValueError("Steering angle outside range")

        self.send_car_command("steer", angle_deg)

    def steer_off(self):
        self.send_car_command("steer_off", None)

    def AC_ON(self):
        self.send_car_command("AC", "ON")

    def AC_OFF(self):
        self.send_car_command("AC", "OFF")

    def set_temp(self, temperature):
        if not 16 <= temperature <= 30:
            raise ValueError("Temperature outside range")

        self.send_car_command("temperature", temperature)


# REMOVED - the input loop that was here. The test script already has one
# and they used different command names.
