# ADDED - background CAN transmitter.
#
# Why this file exists:
#
# A single frame does nothing. openpilot queues LKAS (0x169) on every
# iteration of its control loop - roughly 100 Hz, continuously, for as long
# as lateral control is engaged. The EPS expects an unbroken stream with a
# valid incrementing counter; one frame from a menu keypress gets discarded
# and the wheel never moves. 0x27A behaves the same way at ~50 Hz.
#
# So the menu cannot send directly for anything that holds a state. It
# registers a builder here, and this thread keeps packing and sending it at
# the right rate until the menu clears it.

import collections
import threading
import time


class NoticeQueue:
    """Messages from the tx thread, drained by whoever owns the terminal.

    ADDED - the transmitter used to print directly. A background thread
    writing to stdout while the menu sits in input() corrupts the line the
    operator is typing: the notice lands mid-prompt and the keystrokes before
    it get merged into the next read. So the thread posts here instead and
    the menu prints them at a moment of its own choosing.
    """

    def __init__(self, maxlen=200):
        self.items = collections.deque(maxlen=maxlen)
        self.lock = threading.Lock()

    def post(self, text):
        with self.lock:
            self.items.append(text)

    def drain(self):
        with self.lock:
            out = list(self.items)
            self.items.clear()
        return out

    def print_all(self, prefix="[tx] "):
        for line in self.drain():
            print(f"{prefix}{line}")


class PeriodicTransmitter:
    """Sends registered message builders on a fixed tick.

    Each entry supplies a zero-argument callable that returns a
    (addr, data, bus) tuple. It is called fresh on every send so the
    counter and checksum are recomputed per frame, which is required.
    """

    def __init__(self, can, base_rate_hz=100.0, verbose=False, notices=None):
        self.can = can
        self.base_dt = 1.0 / base_rate_hz
        self.verbose = verbose
        self.notices = notices if notices is not None else NoticeQueue()

        self.entries = {}          # name -> dict(builder, divider, tick)
        self.lock = threading.RLock()

        self.thread = None
        self.running = False

        # stats so we can confirm the loop is actually keeping up
        self.ticks = 0
        self.overruns = 0
        self.started_at = None

    # ---------------- registration ----------------

    def set(self, name, builder, rate_hz=100.0):
        """Register (or replace) a periodically-sent message."""
        divider = max(1, round((1.0 / rate_hz) / self.base_dt))
        with self.lock:
            self.entries[name] = {"builder": builder, "divider": divider, "tick": 0}

    def clear(self, name):
        with self.lock:
            self.entries.pop(name, None)

    def clear_all(self):
        with self.lock:
            self.entries.clear()

    def active(self):
        with self.lock:
            return sorted(self.entries)

    # ---------------- thread control ----------------

    def start(self):
        if self.running:
            return
        self.running = True
        self.started_at = time.monotonic()
        self.ticks = 0
        self.overruns = 0
        # daemon so a stuck loop can never keep the interpreter alive
        self.thread = threading.Thread(target=self._run, name="can-tx", daemon=True)
        self.thread.start()

    def stop(self):
        """Stop transmitting. Clears entries first so nothing is left latched."""
        self.clear_all()
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None

    def _run(self):
        next_tick = time.monotonic()

        while self.running:
            with self.lock:
                due = []
                for entry in self.entries.values():
                    if entry["tick"] % entry["divider"] == 0:
                        due.append(entry["builder"])
                    entry["tick"] += 1

            for builder in due:
                try:
                    message = builder()
                except Exception as exc:
                    # never let one bad builder kill the whole transmitter
                    # CHANGED - posted, not printed. See NoticeQueue.
                    self.notices.post(f"builder error: {type(exc).__name__}: {exc}")
                    continue
                if message is not None:
                    self.can.send(message, verbose=self.verbose)

            self.ticks += 1

            next_tick += self.base_dt
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                # we fell behind; resync instead of accumulating debt
                self.overruns += 1
                next_tick = time.monotonic()

    # ---------------- diagnostics ----------------

    def print_stats(self):
        if self.started_at is None:
            print("transmitter never started")
            return
        elapsed = time.monotonic() - self.started_at
        rate = self.ticks / elapsed if elapsed > 0 else 0.0
        print(f"running: {self.running}")
        print(f"elapsed: {elapsed:.1f} s")
        print(f"ticks:   {self.ticks}  ({rate:.1f} Hz actual, "
              f"{1.0 / self.base_dt:.0f} Hz target)")
        print(f"overruns: {self.overruns}")
        print(f"active:   {', '.join(self.active()) or 'none'}")


class SteeringHold:
    """Holds a steering angle and ramps toward it instead of jumping.

    Two reasons this is not just a stored float:

    1. The EPS rejects implausible angle steps. Commanding 0 -> 90 degrees in
       one 10 ms frame is not something a human wrist can do, and openpilot
       rate-limits for exactly this reason.
    2. A full-rate step command on a real rack is violent.

    MAX_RATE_DEG_S is deliberately conservative for stationary garage
    testing. Tune it against what you actually measure on 0x002
    (STEER_ANGLE_RATE) when a person turns the wheel.
    """

    MAX_RATE_DEG_S = 100.0

    # CHANGED - was 10 s, which is not enough time to walk the menus. The
    # deadman is here to stop an abandoned terminal from holding the rack
    # driven indefinitely, not to police how fast you can type. 0 disables it.
    DEFAULT_TIMEOUT_S = 60.0

    def __init__(self, controller, rate_hz=100.0, timeout_s=None, notices=None):
        self.controller = controller
        self.step = self.MAX_RATE_DEG_S / rate_hz
        self.timeout_s = self.DEFAULT_TIMEOUT_S if timeout_s is None else timeout_s
        self.notices = notices if notices is not None else NoticeQueue()

        self.lock = threading.Lock()
        self.target = 0.0
        self.current = 0.0
        self.active = False
        self.expires_at = None

    def engage(self, angle_deg):
        with self.lock:
            self.target = float(angle_deg)
            self.active = True
            self.expires_at = (time.monotonic() + self.timeout_s
                               if self.timeout_s else None)

    # ADDED - push the deadman out without changing the commanded angle, so
    # the operator can refresh a hold from the menu instead of re-typing it.
    def refresh(self):
        with self.lock:
            if self.active and self.timeout_s:
                self.expires_at = time.monotonic() + self.timeout_s
                return True
            return False

    def release(self):
        """Drop LKA_ACTIVE. Angle is held at the last commanded value in the
        frame, but with the active bit clear and MAX_TORQUE zeroed the rack
        is not being driven."""
        with self.lock:
            self.active = False
            self.target = self.current
            self.expires_at = None

    def build(self):
        with self.lock:
            # ADDED - deadman. If we engaged and nobody re-engaged within the
            # window, stop driving the rack rather than holding an angle
            # forever because a terminal got closed.
            if (self.active and self.expires_at is not None
                    and time.monotonic() > self.expires_at):
                # CHANGED - posted, not printed. See NoticeQueue.
                self.notices.post(
                    f"steering hold timed out after {self.timeout_s:.0f}s, "
                    "LKA_ACTIVE cleared (still streaming 0x169)"
                )
                self.active = False
                self.target = self.current
                self.expires_at = None

            # ramp toward target
            delta = self.target - self.current
            if abs(delta) <= self.step:
                self.current = self.target
            else:
                self.current += self.step if delta > 0 else -self.step

            angle = self.current
            active = self.active

        return self.controller.create_steering_control(angle, lka_active=active)

    def status(self):
        # CHANGED - "timeout" read like it described the steering motion. It
        # is a countdown to the deadman and has nothing to do with degrees.
        # The ramp rate is the number that governs how fast the angle moves.
        with self.lock:
            remaining = ("disabled" if self.expires_at is None
                         else f"{max(0.0, self.expires_at - time.monotonic()):.1f}s left")
            at_target = abs(self.target - self.current) < 1e-6
            eta = "" if at_target else (
                f" (~{abs(self.target - self.current) / self.MAX_RATE_DEG_S:.1f}s to go)"
            )
            return (f"LKA {'ACTIVE' if self.active else 'off'}  |  "
                    f"angle now {self.current:+.2f} deg  ->  "
                    f"commanded {self.target:+.2f} deg{eta}  |  "
                    f"ramp {self.MAX_RATE_DEG_S:.0f} deg/s  |  "
                    f"deadman {remaining}")


class BlinkerHold:
    """0x27A is a lamp-state message at ~50 Hz, not a button press, so it has
    to be streamed the same way steering is."""

    def __init__(self, controller):
        self.controller = controller
        self.lock = threading.Lock()
        self.left = False
        self.right = False

    def set(self, left, right):
        with self.lock:
            self.left = bool(left)
            self.right = bool(right)

    def build(self):
        with self.lock:
            left, right = self.left, self.right
        return self.controller.create_turn_signal_control(left, right)

    def status(self):
        with self.lock:
            if self.left and self.right:
                return "hazard (both bits)"
            if self.left:
                return "left"
            if self.right:
                return "right"
            return "off"
