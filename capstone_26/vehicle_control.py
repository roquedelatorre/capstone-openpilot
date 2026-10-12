# Used to send car controls manually

# Class to send manual commands using the terminal
# Helps organize all car commands to control Leaf

import time

from manual_nissancan import (CarController_Manual, MAX_ANGLE,
                              HAZARD_PULSE_S, HAZARD_RATE_HZ)
from manual_can_interface import CANInterface
from manual_tx_loop import (PeriodicTransmitter, SteeringHold, BlinkerHold,
                            NoticeQueue)

# Rates openpilot / our captures use. 0x169 is ~100 Hz, 0x27A is ~50 Hz.
LKAS_RATE_HZ = 100.0
TURN_SIGNAL_RATE_HZ = 50.0


class Vehicle_Controls:
    def __init__(self, dbc_name=None, simulation=True, verbose_tx=False,
                 hold_timeout_s=None):
        self.simulation = simulation

        #builds Nissan CAN messages
        self.nissan = CarController_Manual(dbc_name)
        #Sends CAN to Nissan
        self.can = CANInterface(simulation=simulation)

        # ADDED - the tx thread must not write to stdout while the menu is in
        # input(), so everything it wants to say goes here and the caller
        # prints it when the terminal is free.
        self.notices = NoticeQueue()

        # ADDED - anything that holds a state has to be streamed, not sent
        # once. See manual_tx_loop.py for why.
        self.tx = PeriodicTransmitter(self.can, base_rate_hz=LKAS_RATE_HZ,
                                      verbose=verbose_tx, notices=self.notices)
        self.steering = SteeringHold(self.nissan, rate_hz=LKAS_RATE_HZ,
                                     timeout_s=hold_timeout_s,
                                     notices=self.notices)
        self.blinkers = BlinkerHold(self.nissan)

    # ADDED - call this whenever the terminal is idle and safe to write to
    def print_notices(self):
        self.notices.print_all()

    # ---------------- lifecycle ----------------

    def start(self):
        self.tx.start()

    def stop(self):
        """Release everything, then stop transmitting. Always call this."""
        self.steering.release()
        self.blinkers.set(False, False)
        self.tx.stop()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()
        return False

    # ---------------- steering (streamed) ----------------

    def steer(self, angle_deg):
        if not -MAX_ANGLE <= angle_deg <= MAX_ANGLE:
            raise ValueError(f"Steering angle outside +/-{MAX_ANGLE} degrees")

        self.steering.engage(angle_deg)
        # register once; the transmitter rebuilds the frame every tick so the
        # counter and checksum stay valid
        self.tx.set("LKAS", self.steering.build, rate_hz=LKAS_RATE_HZ)

    def steer_off(self):
        # keep streaming with LKA_ACTIVE clear so the EPS sees a clean
        # handover instead of the stream just vanishing
        self.steering.release()
        self.tx.set("LKAS", self.steering.build, rate_hz=LKAS_RATE_HZ)

    def steer_stop_sending(self):
        self.steering.release()
        self.tx.clear("LKAS")

    # ADDED - push the deadman out without re-typing the angle
    def steer_refresh(self):
        return self.steering.refresh()

    # ---------------- blinkers (streamed) ----------------

    def blinker_left(self):
        self._set_blinkers(True, False)

    def blinker_right(self):
        self._set_blinkers(False, True)

    def blinkers_off(self):
        self._set_blinkers(False, False)

    # ADDED (Saul) - both blinker bits on at once is how the car shows
    # hazards, second way to try hazards if the 0x3AF button does nothing
    def hazards_27a_ON(self):
        self._set_blinkers(True, True)

    def hazards_27a_OFF(self):
        self._set_blinkers(False, False)

    def _set_blinkers(self, left, right):
        self.blinkers.set(left, right)
        self.tx.set("TURN_SIGNALS_27A", self.blinkers.build,
                    rate_hz=TURN_SIGNAL_RATE_HZ)

    # ---------------- hazards (one-shot, it is a button) ----------------

    # CHANGED - a single frame is not a button press. Real presses in our
    # capture are 0.131-0.560 s wide (median 0.250 s) on a message that runs
    # at 10 Hz, so a press is 2-3 consecutive frames with bit 7 set. One
    # frame among the BCM's own stream of zeros is very unlikely to register.
    def hazard_pulse(self, duration_s=None, rate_hz=None):
        """Emulate one hazard button press, then return to released."""
        duration_s = HAZARD_PULSE_S if duration_s is None else duration_s
        rate_hz = HAZARD_RATE_HZ if rate_hz is None else rate_hz

        pressed = [True]

        def build():
            return self.nissan.create_hazard_control(pressed[0])

        self.tx.set("HAZARD_SWITCH", build, rate_hz=rate_hz)
        if not self.tx.running:
            self.tx.start()

        time.sleep(duration_s)
        pressed[0] = False
        # hold the released state briefly so the last frames the BCM sees
        # from us are "not pressed"
        time.sleep(max(0.1, 2.0 / rate_hz))
        self.tx.clear("HAZARD_SWITCH")

        return duration_s

    # kept for compatibility, but prefer hazard_pulse
    def hazards_press(self):
        self.can.send(self.nissan.create_hazard_control(True))

    def hazards_release(self):
        self.can.send(self.nissan.create_hazard_control(False))

    # ---------------- not yet reverse engineered ----------------

    # CHANGED - these used to dispatch command strings that send_car_command
    # had no branch for, so they raised from deep inside the dispatcher. Now
    # they say what is actually missing.

    def AC_ON(self):
        raise NotImplementedError(
            "A/C message not identified yet. Capture the climate bus in Cabana "
            "while pressing A/C, add the message to nissan_leaf_2025_capstone.dbc, "
            "then add a builder in manual_nissancan.py."
        )

    AC_OFF = AC_ON

    def set_temp(self, temperature):
        if not 16 <= temperature <= 30:
            raise ValueError("Temperature outside range")
        raise NotImplementedError("Climate setpoint message not identified yet.")

    def wipers_ON(self):
        raise NotImplementedError("Wiper message not identified yet.")

    wipers_OFF = wipers_ON

    # ---------------- status ----------------

    def print_status(self):
        print(f"mode:     {'SIMULATION' if self.simulation else 'HARDWARE'}")
        print(f"steering: {self.steering.status()}")
        print(f"blinkers: {self.blinkers.status()}")
        self.tx.print_stats()
