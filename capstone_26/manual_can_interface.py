#This script actually sends the CAN messages
#pulls the messages from manual_nissancan

import threading


class CANInterface:

    def __init__(self, simulation=True, allow_output=True):
        self.simulation = simulation

        self.panda = None  #only connect when actually using Panda

        # ADDED - keeps every frame we tried to send so we can check
        # the bytes after a test run instead of scrolling the terminal
        self.sent_log = []
        self.log_limit = 20000     # ADDED - don't grow forever under the 100 Hz loop
        self.lock = threading.Lock()   # ADDED - tx thread and menu both touch this

        if not self.simulation:
            # import here so simulation mode works on a machine with no panda
            from panda import Panda
            # CHANGED - Panda.SAFETY_ALLOUTPUT does not exist in the panda
            # version this repo pins. set_safety_mode takes a capnp enum.
            from opendbc.car.structs import CarParams

            # On a comma device the panda is internal and reached over SPI;
            # Panda() tries USB first and falls back to SPI automatically, so
            # this same call works on a laptop and on the device. But pandad
            # holds the handle while openpilot is running - stop openpilot
            # first or this raises.
            self.panda = Panda()

            # ADDED - THE OTHER ONE THAT WOULD HAVE BITTEN US.
            # The panda expects a heartbeat from openpilot. With no heartbeat
            # it raises HEARTBEAT_LOST and stops transmitting after a few
            # seconds. We are not openpilot, so turn the check off. Note that
            # sending any heartbeat re-enables it.
            self.panda.set_heartbeat_disabled()

            # ADDED - THIS IS THE ONE THAT BIT US.
            # Panda boots in SAFETY_SILENT. In that mode can_send() returns
            # normally but the panda drops every frame on the floor, so the
            # whole stack looked like it was working and nothing reached the
            # bus. ALLOUTPUT disables the safety model entirely and lets us
            # transmit arbitrary IDs, which is what we need for bring-up.
            #
            # ALLOUTPUT means there is nothing between this script and the
            # vehicle bus. Wheels chocked, car in park, know what you are
            # sending before you start the tx loop.
            mode = (CarParams.SafetyModel.allOutput if allow_output
                    else CarParams.SafetyModel.silent)
            self.panda.set_safety_mode(mode)

            self.print_health(expect_mode=int(mode))

    # ADDED - read the panda back instead of assuming the writes landed.
    # Run this before anything else on the car.
    HARNESS_STATUS = {0: "NOT CONNECTED", 1: "normal", 2: "FLIPPED"}

    def print_health(self, expect_mode=None):
        if self.panda is None:
            print("simulation mode, no panda")
            return

        h = self.panda.health()
        serial = self.panda.get_serial()[0]
        harness = h.get("car_harness_status")

        print(f"panda serial      : {serial}")
        print(f"safety mode       : {h.get('safety_mode')}"
              + (f" (expected {expect_mode})" if expect_mode is not None else ""))
        print(f"harness           : {self.HARNESS_STATUS.get(harness, harness)}")
        print(f"heartbeat lost    : {h.get('heartbeat_lost')}")
        print(f"ignition (line)   : {h.get('ignition_line')}")
        print(f"voltage           : {h.get('voltage')} mV")
        print(f"rx invalid checks : {h.get('safety_rx_checks_invalid')}")
        print(f"faults            : {h.get('fault_status')}")

        # the checks that actually decide whether a transmit can work
        if expect_mode is not None and h.get("safety_mode") != expect_mode:
            print("!! safety mode did not take. Is pandad still running?")
        if harness == 0:
            print("!! harness not detected. With no harness the panda cannot")
            print("   map buses to the car; nothing you send will go anywhere.")
        if h.get("heartbeat_lost"):
            print("!! heartbeat was lost at some point. set_heartbeat_disabled()")
            print("   should prevent this; check nothing else is driving the panda.")

    # ADDED - open the harness relay to isolate the ADAS camera from the bus.
    #
    # NOT called automatically, and not needed for blinkers. It matters for
    # steering: on a stock car the camera transmits 0x169 continuously, so
    # your frames and its frames both reach the EPS and you learn nothing.
    # Opening the relay takes the camera off the car-side bus.
    #
    # This disables ProPILOT while it is open. Know that before you flip it.
    def set_relay(self, intercept):
        if self.panda is None:
            print(f"simulation mode, relay {'open' if intercept else 'closed'} noted")
            return
        self.panda.force_relay_drive(bool(intercept), False)
        print(f"harness intercept relay -> {'OPEN (camera isolated)' if intercept else 'closed (stock)'}")

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

    def send(self, message, verbose=True):
        if message is None:
            raise ValueError("Failed to send empty CAN")

        with self.lock:
            self.sent_log.append(message)
            if len(self.sent_log) > self.log_limit:
                del self.sent_log[:len(self.sent_log) - self.log_limit]

        if verbose:
            prefix = "Simulation CAN message:" if self.simulation else "Sending CAN message:"
            print(f"{prefix} {self.format_msg(message)}")

        if self.simulation:
            return

        addr, dat, bus = message
        self.panda.can_send(addr, dat, bus)

    # ADDED - dump everything sent this run
    def print_log(self, last=None):
        with self.lock:
            log = list(self.sent_log)

        if not log:
            print("No messages sent")
            return

        # ADDED - the 100 Hz loop makes this huge, so default to a tail
        if last is not None:
            log = log[-last:]

        for i, message in enumerate(log):
            # CHANGED - this printed every frame twice
            print(f"{i}: {self.format_msg(message)}")

    # ADDED - how many of each ID we sent. Much more useful than the raw dump
    # once the tx loop is running, and it is the quick way to confirm the loop
    # is actually hitting its rate.
    def print_summary(self):
        with self.lock:
            log = list(self.sent_log)

        if not log:
            print("No messages sent")
            return

        counts = {}
        for addr, _, _ in log:
            counts[addr] = counts.get(addr, 0) + 1

        print(f"{len(log)} frames total")
        for addr in sorted(counts):
            print(f"  ID 0x{addr:03X}: {counts[addr]} frames")

    # ADDED (Saul) - export sent frames as C arrays for the ESP32
    # Python is just the prototype. This gives us the exact bytes to
    # hard-code on the board instead of retyping them by hand.
    #
    # CHANGED - de-duplicates. Under the 100 Hz loop the raw log is thousands
    # of near-identical frames, and we only want the distinct ones.
    def export_c(self, unique=True):
        with self.lock:
            log = list(self.sent_log)

        if not log:
            print("No messages sent")
            return

        if unique:
            seen = set()
            deduped = []
            for message in log:
                addr, data, bus = message
                key = (addr, bytes(data), bus)
                if key not in seen:
                    seen.add(key)
                    deduped.append(message)
            log = deduped

        for i, message in enumerate(log):
            addr, data, bus = message
            byte_list = ", ".join(f"0x{b:02X}" for b in data)
            print(f"// frame {i}  ID 0x{addr:03X}  bus {bus}")
            print(f"uint8_t frame{i}[{len(data)}] = {{{byte_list}}};")
