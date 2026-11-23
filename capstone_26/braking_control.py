"""Offline braking-request prototype for the Nissan Leaf capstone.

This is NOT a vehicle brake controller. No Panda, CAN, relay or car-control
modules are imported or accessed. The DBC 'BRAKE_PRESSURE' field is an
uncalibrated raw field; its transmission/actuation meaning is unverified.

Example:
    python braking_control.py --pressure 120 --seconds 4
"""

import argparse
import time
from dataclasses import dataclass

RAW_MIN = 0
RAW_MAX = 511  # Signal encoding range from capstone DBC, NOT a safe actuation limit
SIM_RATE_HZ = 10


@dataclass(frozen=True)
class BrakeRequest:
    brake_pressure: int
    brake_active: bool
    simulation: bool = True

    def as_signals(self) -> dict:
        """Return signal names for inspection, NOT a transmissible CAN frame."""
        return {
            "message": "PROPILOT_BRAKE",
            "BRAKE_PRESSURE": self.brake_pressure,
            "BRAKE_ACTIVE": int(self.brake_active),
            "SIMULATION_ONLY": self.simulation,
        }


class BrakeControllerSimulation:
    def __init__(self):
        self.request = BrakeRequest(0, False)

    def engage(self, raw_pressure: int) -> BrakeRequest:
        if not isinstance(raw_pressure, int) or not RAW_MIN <= raw_pressure <= RAW_MAX:
            raise ValueError(f"Raw pressure must be an integer from {RAW_MIN} to {RAW_MAX}")
        self.request = BrakeRequest(raw_pressure, raw_pressure > 0)
        return self.request

    def release(self) -> BrakeRequest:
        self.request = BrakeRequest(0, False)
        return self.request

    def build(self) -> dict:
        return self.request.as_signals()


def main() -> None:
    parser = argparse.ArgumentParser(description="OFFLINE simulated braking request (no CAN output)")
    parser.add_argument("--pressure", type=int, default=120,
                        help="raw simulated field value, not physical brake pressure (default: 120)")
    parser.add_argument("--seconds", type=float, default=4.0,
                        help="duration of simulated hold, 0 to 30 seconds (default: 4)")
    args = parser.parse_args()
    if not RAW_MIN <= args.pressure <= RAW_MAX:
        parser.error(f"pressure must be in [{RAW_MIN}, {RAW_MAX}]")
    if not 0 <= args.seconds <= 30:
        parser.error("seconds must be between 0 and 30")

    brake = BrakeControllerSimulation()
    print("SIMULATION ONLY — no CAN transmission, relay changes or vehicle control")
    print("Raw signal magnitude is uncalibrated; NOT a braking force or pressure unit")
    print("Phase 1: RELEASED", brake.build())

    try:
        brake.engage(args.pressure)
        print("Phase 2: REQUESTED", brake.build())
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            remaining = end - time.monotonic()
            time.sleep(min(1.0 / SIM_RATE_HZ, max(remaining, 0.0)))
    except KeyboardInterrupt:
        print("\nSimulation interrupted")
    finally:
        brake.release()
        print("Phase 3: RELEASED", brake.build())
        print("Simulation complete. No CAN frames were produced.")


if __name__ == "__main__":
    main()
