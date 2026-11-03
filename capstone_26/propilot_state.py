"""Is ProPILOT actually engaged? Read-only, transmits nothing.

WHY THIS EXISTS
---------------
Every steering test so far ran with the panda in allOutput, which stops the
PANDA from blocking our frames. It does not remove any precondition the EPS
itself has.

openpilot's design is: the driver engages ProPILOT -> the car reports cruise
engaged -> only then does openpilot send LKAS. We skipped the first two steps
and went straight to the third. If the EPS also requires the car's own ADAS to
be switched on, that would explain exactly what we see: the frame is accepted
and acknowledged (LKAS_STATUS 5 -> 6) and no torque is ever applied.

Nobody has checked whether ProPILOT was on during any of our tests. This
checks it.

WHAT IT WATCHES
---------------
  CRUISE_AVAILABLE  0x239 byte 2 bit 1   ProPILOT main switch is on
  PROPILOT_BUTTON   0x239 byte 3 bit 0   the button, pressed right now
  CRUISE_ENABLED    0x30F byte 0 bit 3   cruise actually engaged
  LKAS_ENABLED      0x4CB byte 6 bit 3   the car's own lane keeping is on
  LKAS_STATUS       0x185 byte 6 hi nib  what the EPS reports

Every reading carries the time it arrived. A value that stopped updating is
shown as STALE rather than reported as current - a decoded value can never
establish liveness, only an arrival time can. That mistake cost the Mazda
project weeks and it would cost us the same.

HOW TO USE IT
-------------
    sudo systemctl stop comma
    python propilot_state.py

Car on, in park. Then, while it runs:

  1. watch the resting state for ~10 s and write it down
  2. press the ProPILOT button on the steering wheel
  3. watch which flags change

What you are looking for is whether CRUISE_AVAILABLE and CRUISE_ENABLED were
0 the whole time during our steering tests. If they were, we have been
commanding a system the driver never switched on.
"""

import time

from opendbc.car.structs import CarParams
from panda import Panda

STALE_AFTER_S = 1.0

# addr -> (label, decoder)
WATCH = {
    0x239: "CRUISE_THROTTLE",
    0x30F: "CRUISE_STATE",
    0x4CB: "LKAS_SETTINGS",
    0x185: "STEER_TORQUE_SENSOR",
}

LKAS_STATUS_NAMES = {5: "INACTIVE", 6: "ACTIVE", 9: "FAULT", 14: "not-a-legal-value"}


def unpack(msg):
    """panda reports the bus with flag bits in the top nibble - mask them."""
    if len(msg) == 3:
        addr, data, bus = msg[0], msg[1], msg[2]
    else:
        addr, data, bus = msg[0], msg[2], msg[3]
    return addr, bytes(data), bus & 0x0F


class Field:
    """One signal plus the time it last arrived."""

    def __init__(self, name, note):
        self.name = name
        self.note = note
        self.value = None
        self.at = None

    def update(self, value, now):
        changed = value != self.value
        self.value = value
        self.at = now
        return changed

    def render(self, now):
        if self.at is None:
            return f"{self.name:<18} --        never seen"
        age = now - self.at
        stale = "  STALE" if age > STALE_AFTER_S else ""
        shown = self.value
        if self.name == "LKAS_STATUS":
            shown = f"{self.value} ({LKAS_STATUS_NAMES.get(self.value, '?')})"
        return f"{self.name:<18} {str(shown):<9} {self.note}{stale}"


def main():
    panda = Panda()
    panda.set_heartbeat_disabled()
    # SILENT. This script never transmits.
    panda.set_safety_mode(CarParams.SafetyModel.silent)

    fields = {
        "CRUISE_AVAILABLE": Field("CRUISE_AVAILABLE", "ProPILOT main switch"),
        "PROPILOT_BUTTON": Field("PROPILOT_BUTTON", "button pressed now"),
        "CRUISE_ENABLED": Field("CRUISE_ENABLED", "cruise engaged"),
        "LKAS_ENABLED": Field("LKAS_ENABLED", "car's own lane keeping"),
        "LKAS_STATUS": Field("LKAS_STATUS", "what the EPS reports"),
    }

    print()
    print("Read-only. Transmitting nothing.")
    print("Press the ProPILOT button while this runs and watch what changes.")
    print("Ctrl-C to stop.")
    print()

    last_print = 0.0
    seen_any = False

    try:
        while True:
            now = time.monotonic()
            changed = False

            for msg in panda.can_recv():
                addr, data, _bus = unpack(msg)
                if addr not in WATCH or len(data) < 1:
                    continue
                seen_any = True

                if addr == 0x239 and len(data) >= 4:
                    changed |= fields["CRUISE_AVAILABLE"].update((data[2] >> 1) & 1, now)
                    changed |= fields["PROPILOT_BUTTON"].update(data[3] & 1, now)
                elif addr == 0x30F:
                    changed |= fields["CRUISE_ENABLED"].update((data[0] >> 3) & 1, now)
                elif addr == 0x4CB and len(data) >= 7:
                    changed |= fields["LKAS_ENABLED"].update((data[6] >> 3) & 1, now)
                elif addr == 0x185 and len(data) >= 7:
                    changed |= fields["LKAS_STATUS"].update((data[6] >> 4) & 0x0F, now)

            # print on any change, and once a second regardless so staleness shows
            if changed or (now - last_print) >= 1.0:
                last_print = now
                print(f"--- t={now:7.1f} {'CHANGED' if changed else ''}")
                for f in fields.values():
                    print("   " + f.render(now))

            time.sleep(0.02)

    except KeyboardInterrupt:
        print()
        if not seen_any:
            print("Saw none of the watched messages. Is the car awake?")
            print("Is openpilot holding the panda? (sudo systemctl stop comma)")
        else:
            print("stopped. Nothing was transmitted.")
        panda.set_safety_mode(CarParams.SafetyModel.silent)


if __name__ == "__main__":
    main()