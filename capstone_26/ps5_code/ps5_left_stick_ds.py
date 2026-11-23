"""PS5 DualSense left-stick test using the dualsense-controller library.
Read-only: prints left stick X/Y and L3 presses. Press PS or Ctrl+C to quit.
Requires hidapi.dll in this folder (Windows).
"""
import time

from dualsense_controller import DualSenseController

if not DualSenseController.enumerate_devices():
    raise SystemExit("No DualSense found - check USB/Bluetooth.")

controller = DualSenseController()
running = True


def on_left_stick(stick):
    print(f"LX {stick.x:+.3f}  LY {stick.y:+.3f}", flush=True)


def on_l3_down():
    print("[L3 pressed]", flush=True)


def stop():
    global running
    running = False


controller.left_stick.on_change(on_left_stick)
controller.btn_l3.on_down(on_l3_down)
controller.btn_ps.on_down(stop)  # press PS button to quit

controller.activate()
print("Move the left stick. Press PS (or Ctrl+C) to quit.")
try:
    while running:
        time.sleep(0.05)
except KeyboardInterrupt:
    pass
finally:
    controller.deactivate()
    print("Stopped.")