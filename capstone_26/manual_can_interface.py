#This script actually sends the CAN messages
#pulls the messages from manual_nissancan
class CANInterface:

    def __init__(self, simulation=True):
        self.simulation = simulation

    def send(self, message):
        if message is None:
            raise ValueError("Failed to send empty CAN")

        if self.simulation:
            print("Simulation CAN message: ")
            print(message)
            return
