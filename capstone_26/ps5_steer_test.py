"""
PS5 DualSense -> OFFLINE steering command generator.

Mirrors the G29 teleop pipeline without any network or vehicle I/O:

  PS5 left stick X  ->  CMD;<seq>;<steer>;0.0000;0.0000;<square>   (same
                        packet format as g29_to_server_G29_FINAL.py -- built and
                        printed/logged, NEVER sent)
                    ->  FabriceBoard_G29_UDP.ino steering math, simulated:
                        deadzone -> target deg -> slew -> feed-forward counts ->
                        steer1/steer2 DAC volts

The PID trim is NOT simulated (it needs live CAN angle feedback), so the
volts shown are the feed-forward part only -- what the board would command
with zero steering error.

STEERING ONLY: throttle and brake are always 0 in the packet -- pedals are
operated by a person in the vehicle.

Controls (indices vary by OS/driver; the script prints button ids so you can
remap with the flags below):
  left stick X  steering
  Square        simulated engage/disengage latch (--square-button)

Usage:
  python ps5_steer_offline.py
  python ps5_steer_offline.py --csv steer_log.csv
  python ps5_steer_offline.py --start-engaged    # skip the Square press
Ctrl+C to stop.
"""

from __future__ import annotations

import argparse
import csv
import os
import time
from dataclasses import dataclass

# ---- constants copied from the packaged sources --------------------------
# g29_to_server_G29_FINAL.py
SENDER_HZ = 60
# FabriceBoard_G29_UDP.ino
CTRL_HZ = 100
STEER_MAX_DEG = 90.0
STICK_DEADZONE = 0.05
SETPOINT_SLEW_DPT = 6.0      # deg per control tick
FF_CNT_PER_DEG = 3.0
OUT_SAT_FULL = 420.0         # counts
OUT_SLEW_CNT = 40.0          # counts per control tick
STEER_CENTER = 2.47          # volts
VFS = 4.096                  # volts full scale


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def trigger_01(raw: float) -> float:
    """Trigger axis -1 (released) .. +1 (pressed) -> 0..1."""
    return clamp((raw + 1.0) / 2.0, 0.0, 1.0)


def serialize_cmd(seq: int, steering: float, throttle: float, brake: float,
                  square: bool) -> str:
    """Byte-for-byte the same text as the G29 sender's serialize_cmd()."""
    return (f"CMD;{seq};{steering:.4f};{throttle:.4f};{brake:.4f};"
            f"{1 if square else 0}")


@dataclass
class SteerSim:
    """Feed-forward half of the FabriceBoard steering loop, one tick at a time."""
    target_slewed: float = 0.0
    last_out: float = 0.0

    def reset(self) -> None:
        self.target_slewed = 0.0
        self.last_out = 0.0

    def tick(self, stick: float) -> dict:
        s = 0.0 if abs(stick) < STICK_DEADZONE else stick
        target = s * STEER_MAX_DEG
        ds = target - self.target_slewed
        if ds > SETPOINT_SLEW_DPT:
            self.target_slewed += SETPOINT_SLEW_DPT
        elif ds < -SETPOINT_SLEW_DPT:
            self.target_slewed -= SETPOINT_SLEW_DPT
        else:
            self.target_slewed = target

        ff = clamp(FF_CNT_PER_DEG * self.target_slewed, -OUT_SAT_FULL, OUT_SAT_FULL)
        d = ff - self.last_out
        if d > OUT_SLEW_CNT:
            ff = self.last_out + OUT_SLEW_CNT
        elif d < -OUT_SLEW_CNT:
            ff = self.last_out - OUT_SLEW_CNT
        self.last_out = ff

        uv = ff * (VFS / 4096.0)
        return {
            "target_deg": target,
            "slewed_deg": self.target_slewed,
            "ff_counts": ff,
            "steer1_v": STEER_CENTER + uv,
            "steer2_v": STEER_CENTER - uv,
        }


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="PS5 -> offline steering command")
    p.add_argument("--index", type=int, default=0, help="controller index")
    p.add_argument("--steer-axis", type=int, default=0)
    p.add_argument("--square-button", type=int, default=2,
                   help="Square id (often 2 on Windows, 3 on Linux)")
    p.add_argument("--csv", help="log every control tick to this CSV file")
    p.add_argument("--print-hz", type=float, default=5.0)
    p.add_argument("--show-axes", action="store_true", help="also print raw axes")
    p.add_argument("--start-engaged", action="store_true",
                   help="start the simulation ENGAGED (no Square press needed)")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
    import pygame

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() <= args.index:
        print("No PS5 controller found. Plug in USB or pair Bluetooth, then retry.")
        pygame.quit()
        return 1
    js = pygame.joystick.Joystick(args.index)
    if hasattr(js, "get_init") and not js.get_init():
        js.init()
    n_axes = js.get_numaxes()
    n_buttons = js.get_numbuttons()
    buttons_prev = [False] * n_buttons

    if not 0 <= args.steer_axis < n_axes:
        print(f"--steer-axis {args.steer_axis} out of range ({n_axes} axes).")
        pygame.quit()
        return 1

    print(f"Controller: {js.get_name()}  ({n_axes} axes, {js.get_numbuttons()} buttons)")
    print(f"steer=axis {args.steer_axis}  square=button {args.square_button}  "
          f"(STEERING ONLY: throttle/brake fixed at 0)")
    print("OFFLINE: nothing is sent to the relay, ESP32 or vehicle.")
    print("Press Square to toggle simulated ENGAGE. Ctrl+C to stop.\n")

    writer = None
    csv_file = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow(["t_s", "seq", "engaged", "stick", "throttle", "brake",
                         "target_deg", "slewed_deg", "ff_counts",
                         "steer1_v", "steer2_v", "cmd"])

    sim = SteerSim()
    engaged = args.start_engaged
    if engaged:
        print("[SIM] starting ENGAGED (--start-engaged)")
    seq = 0
    period = 1.0 / CTRL_HZ
    t0 = time.monotonic()
    last_print = 0.0
    last_cmd_t = 0.0
    cmd = ""

    try:
        while True:
            started = time.monotonic()
            for ev in pygame.event.get():
                if ev.type == pygame.JOYDEVICEREMOVED:
                    print("!! Controller disconnected -- the real board would "
                          "disengage after 400 ms of no CMD.")
                    return 2

            # Poll every button and detect press edges directly (does not rely
            # on JOYBUTTONDOWN events, which Windows may not deliver here).
            buttons_now = [bool(js.get_button(b)) for b in range(n_buttons)]
            for b, (now_down, was_down) in enumerate(zip(buttons_now, buttons_prev)):
                if now_down and not was_down:
                    if b == args.square_button:
                        engaged = not engaged
                        if not engaged:
                            sim.reset()
                        print(f"[SQUARE] id={b} -> simulated "
                              f"{'ENGAGE' if engaged else 'DISENGAGE'}", flush=True)
                    else:
                        print(f"[BUTTON] id={b} pressed", flush=True)
            buttons_prev = buttons_now

            stick = clamp(js.get_axis(args.steer_axis), -1.0, 1.0)
            throttle = brake = 0.0          # steering only; pedals are manual
            square = buttons_now[args.square_button] \
                if args.square_button < n_buttons else False

            now = started - t0
            # The sender builds CMD at 60 Hz; the board's control loop runs at 100 Hz.
            if now - last_cmd_t >= 1.0 / SENDER_HZ:
                last_cmd_t = now
                cmd = serialize_cmd(seq, stick, throttle, brake, square)
                seq = (seq + 1) & 0xFFFFFFFF

            out = sim.tick(stick) if engaged else {
                "target_deg": 0.0, "slewed_deg": 0.0, "ff_counts": 0.0,
                "steer1_v": STEER_CENTER, "steer2_v": STEER_CENTER}

            if writer:
                writer.writerow([f"{now:.3f}", seq, int(engaged), f"{stick:.4f}",
                                 f"{throttle:.4f}", f"{brake:.4f}",
                                 f"{out['target_deg']:.2f}", f"{out['slewed_deg']:.2f}",
                                 f"{out['ff_counts']:.1f}", f"{out['steer1_v']:.4f}",
                                 f"{out['steer2_v']:.4f}", cmd])

            if now - last_print >= 1.0 / args.print_hz:
                last_print = now
                state = "DBW " if engaged else "SAFE"
                print(f"[{state}] {cmd:<40} tgt {out['target_deg']:+6.1f} "
                      f"slew {out['slewed_deg']:+6.1f} deg  ff {out['ff_counts']:+6.1f} cnt  "
                      f"s1 {out['steer1_v']:.3f}V s2 {out['steer2_v']:.3f}V")
                if args.show_axes:
                    print("    " + " ".join(f"a{i}={js.get_axis(i):+.2f}"
                                            for i in range(n_axes)))

            sleep = period - (time.monotonic() - started)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if csv_file:
            csv_file.close()
            print(f"Log saved to {args.csv}")
        pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
