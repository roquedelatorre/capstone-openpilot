"""
PS5 DualSense left-joystick test (read-only, sends NOTHING over the network).

Shows the left stick X/Y live with bars, tracks min/max seen, and prints any
button press by its raw pygame id so you can find L3 (left stick click).

Axis numbering differs by OS/driver (SDL HIDAPI vs Linux hid-playstation),
so the defaults are axis 0 = X, axis 1 = Y; use --show-all to watch every
axis and confirm which ones move when you push the left stick.

Usage:
  python ps5_left_stick_test.py
  python ps5_left_stick_test.py --show-all
  python ps5_left_stick_test.py --x-axis 0 --y-axis 1 --deadzone 0.05
Ctrl+C to quit. At exit it prints the min/max/center range observed.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
import pygame  # noqa: E402


def bar(value: float, width: int = 21) -> str:
    """Render -1..+1 as a centered bar, e.g. [-----|##--------]."""
    value = max(-1.0, min(1.0, value))
    half = width // 2
    pos = int(round(value * half))
    cells = ["-"] * width
    cells[half] = "|"
    lo, hi = sorted((half, half + pos))
    for i in range(lo, hi + 1):
        if i != half:
            cells[i] = "#"
    return "[" + "".join(cells) + "]"


def apply_deadzone(v: float, dz: float) -> float:
    return 0.0 if abs(v) < dz else v


def pick_controller(index: int | None) -> pygame.joystick.JoystickType | None:
    count = pygame.joystick.get_count()
    if count == 0:
        return None
    print(f"Found {count} controller(s):")
    chosen = None
    for i in range(count):
        js = pygame.joystick.Joystick(i)
        name = js.get_name()
        print(f"  [{i}] {name}  axes={js.get_numaxes()} buttons={js.get_numbuttons()}")
        is_ps5 = any(k in name.lower() for k in ("dualsense", "ps5", "wireless controller"))
        if index is None and chosen is None and is_ps5:
            chosen = i
    if index is not None:
        chosen = index
    if chosen is None:
        chosen = 0
        print("  (no name matched DualSense/PS5 -- using controller 0; pass --index to choose)")
    js = pygame.joystick.Joystick(chosen)
    js.init()
    return js


def main() -> int:
    p = argparse.ArgumentParser(description="PS5 left joystick tester")
    p.add_argument("--index", type=int, default=None, help="controller index (default: first PS5 found)")
    p.add_argument("--x-axis", type=int, default=0)
    p.add_argument("--y-axis", type=int, default=1)
    p.add_argument("--deadzone", type=float, default=0.05)
    p.add_argument("--show-all", action="store_true", help="also print every raw axis value")
    p.add_argument("--hz", type=float, default=20.0, help="display refresh rate")
    args = p.parse_args()

    pygame.init()
    pygame.joystick.init()

    js = pick_controller(args.index)
    if js is None:
        print("No controller found. Check USB/Bluetooth pairing, then retry.")
        pygame.quit()
        return 1

    n_axes = js.get_numaxes()
    for name, idx in (("x-axis", args.x_axis), ("y-axis", args.y_axis)):
        if not 0 <= idx < n_axes:
            print(f"--{name} {idx} is out of range (controller has {n_axes} axes).")
            pygame.quit()
            return 1

    print(f"\nUsing: {js.get_name()}")
    print(f"Left stick: X=axis {args.x_axis}, Y=axis {args.y_axis}, deadzone {args.deadzone}")
    print("Note: pygame Y is usually NEGATIVE when pushed UP.")
    print("Move the stick in full circles, click it (L3), then Ctrl+C.\n")

    mins = [1.0, 1.0]
    maxs = [-1.0, -1.0]
    rest = None
    period = 1.0 / args.hz
    single_line = not args.show_all and sys.stdout.isatty()

    try:
        while True:
            started = time.monotonic()
            for ev in pygame.event.get():
                if ev.type == pygame.JOYBUTTONDOWN:
                    print(f"\n[BUTTON DOWN] id={ev.button}  (L3/left-stick click is often 7 or 11)")
                elif ev.type == pygame.JOYBUTTONUP:
                    print(f"\n[BUTTON UP]   id={ev.button}")
                elif ev.type == pygame.JOYDEVICEREMOVED:
                    print("\n!! Controller disconnected.")
                    return 2

            x = js.get_axis(args.x_axis)
            y = js.get_axis(args.y_axis)
            if rest is None:
                rest = (x, y)
            for i, v in enumerate((x, y)):
                mins[i] = min(mins[i], v)
                maxs[i] = max(maxs[i], v)

            xd = apply_deadzone(x, args.deadzone)
            yd = apply_deadzone(y, args.deadzone)
            mag = min(1.0, (xd * xd + yd * yd) ** 0.5)
            line = (f"X {x:+.3f} {bar(xd)}   Y {y:+.3f} {bar(yd)}   "
                    f"mag {mag:.2f}  {'IN DEADZONE' if mag == 0 else '           '}")

            if args.show_all:
                raw = " ".join(f"a{i}={js.get_axis(i):+.2f}" for i in range(n_axes))
                print(f"{line}\n    {raw}")
            elif single_line:
                print("\r" + line, end="", flush=True)
            else:
                print(line)

            sleep = period - (time.monotonic() - started)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n\n=== Summary ===")
        if rest is not None:
            print(f"Rest (first reading): X {rest[0]:+.3f}  Y {rest[1]:+.3f}  "
                  f"(drift > {args.deadzone} means raise --deadzone)")
            print(f"X range seen: {mins[0]:+.3f} .. {maxs[0]:+.3f}")
            print(f"Y range seen: {mins[1]:+.3f} .. {maxs[1]:+.3f}")
            print("A healthy stick reaches about -1.0 and +1.0 on both axes.")
        pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())