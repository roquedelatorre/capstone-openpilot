#This script actually sends the CAN messages
#pulls the messages from manual_nissancan

from panda import Panda
class CANInterface:

    def __init__(self, simulation=True):
        self.simulation = simulation

        self.panda = None  #only connect when actually using Panda

        if not self.simulation:
            self.panda = Panda()

        # ADDED - keeps every frame we tried to send so we can check
        # the bytes after a test run instead of scrolling the terminal
        self.sent_log = []

    # ADDED - prints a frame the way Cabana shows it, hex ID and hex bytes.
    # Raw print gives a tuple of ints, which is hard to compare against Cabana.
    def format_msg(self, message):
        addr, data, bus = message
        hex_bytes = " ".join(f"{b:02X}" for b in data)
        return (
            f"bus {bus}  "
            f"ID 0x{addr:03X}  "
            f"[{len(data)}]  "
            f"{hex_bytes}"
        )

    def send(self, message):
        if message is None:
            raise ValueError("Failed to send empty CAN")

        self.sent_log.append(message)   # ADDED


        #In simulation mode
        if self.simulation:
            print("Simulation CAN message: ")
            print(self.format_msg(message))   # CHANGED - was print(message)
            return

        # ADDED - real hardware path
        addr, dat, bus = message

        print("Sending CAN message")
        print(self.format_msg(message))

        self.panda.can_send(addr, dat, bus)

    # ADDED - dump everything sent this run
    def print_log(self):
        if not self.sent_log:
            print("No messages sent")
            return
        for i, message in enumerate(self.sent_log):

            print(f"{i}: {self.format_msg(message)}")

            print(f"{i}: {self.format_msg(message)}")

    # ADDED (Saul) - export sent frames as C arrays for the ESP32
    # Python is just the prototype. This gives us the exact bytes to
    # hard-code on the board instead of retyping them by hand.
    def export_c(self):
        if not self.sent_log:
            print("No messages sent")
            return

        for i, message in enumerate(self.sent_log):
            addr, data, bus = message
            byte_list = ", ".join(f"0x{b:02X}" for b in data)
            print(f"// frame {i}  ID 0x{addr:03X}  bus {bus}")
            print(f"uint8_t frame{i}[{len(data)}] = {{{byte_list}}};")
