"""PS5 left stick -> absolute steering angle, live.

Built on steer_position_manualoverride.py (the final draft). Same frame path,
same encoder and self-test, same driver override, same release behaviour -
all imported from that file, not copied. The only new part is where the
target comes from: the left stick, every loop, instead of a fixed list.

    target = -stick * --max-angle        (positive is LEFT, as in the draft)

    stick centred      ->   0 deg (straight ahead)
    stick full right   ->  -max-angle  (right)
    stick full left    ->  +max-angle  (left)

The wheel ramps toward the target at --rate, so a stick flick cannot jerk it.

HOW IT LETS GO
--------------
Any of these releases the wheel for the rest of the run, exactly the way the
draft handles a driver override: keep streaming 0x169 INACTIVE, tracking the
wheel, for one second (a clean handover - the EPS never sees its camera go
quiet), then stop and close the relay.

  * driver torque > --override Nm, averaged over 12 samples (openpilot's)
  * controller unplugged, read error, or no fresh reading for 0.25 s
  * Square / Circle / Cross pressed (pygame ids 0-2) - a handheld stop button
  * --seconds runs out, or Ctrl+C

Driver override cannot be turned off in this script: someone is always in the
car with a joystick demo.

USAGE
-----
    python steer_joystick.py --dry-run                 # no panda: angles + bytes
    sudo systemctl stop comma
    python steer_joystick.py --max-angle 15 --seconds 20   # first live run
    python steer_joystick.py --max-angle 45

Controller on the laptop instead (the comma device does not see USB
controllers): run ps5_stick_sender.py on the laptop and add
    --udp-from <laptop IP>
here. Packets stopping for 0.25 s releases the wheel, same as a lost
controller.
NOTE: at GO the wheel ramps from wherever it is to 0 deg (stick centred).
"""

import argparse
import atexit
import os
import socket
import time
from collections import deque

try:
    from steer_position_manualoverride import (
        CAM_BUS, CAR_BUS, EPS_ADDR, LKAS_ADDR, LKAS_STATUS_NAMES,
        STEER_THRESHOLD, TORQUE_SAMPLES, Sender, decode_185, driver_torque,
        encode_lkas, lkas_counter, self_test, unpack)
except ImportError:
    from steer_position import (  # same file under its shorter name
        CAM_BUS, CAR_BUS, EPS_ADDR, LKAS_ADDR, LKAS_STATUS_NAMES,
        STEER_THRESHOLD, TORQUE_SAMPLES, Sender, decode_185, driver_torque,
        encode_lkas, lkas_counter, self_test, unpack)

STALE_S = 0.25              # no good controller read for this long -> release
STOP_BUTTONS = (0, 1, 2)    # Cross / Circle / Square on Windows SDL; any = stop
MAX_ANGLE_CAP = 180.0
MAX_SECONDS = 120.0


# ---------------------------------------------------------------- controller

class Stick:
    """Left-stick X via pygame. Reports loss instead of serving stale values."""

    def __init__(self, axis, deadzone, invert):
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
        import pygame
        self.pg = pygame
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("no PS5 controller found")
        self.js = pygame.joystick.Joystick(0)
        if hasattr(self.js, "get_init") and not self.js.get_init():
            self.js.init()
        if not 0 <= axis < self.js.get_numaxes():
            raise RuntimeError(f"axis {axis} out of range")
        self.axis, self.deadzone = axis, deadzone
        self.sign = -1.0 if invert else 1.0
        self.n_buttons = self.js.get_numbuttons()
        self.lost = None
        self.last_ok = time.monotonic()
        self.name = self.js.get_name()

    def read(self):
        """(stick -1..1 deadzoned, stop_pressed), or None once lost."""
        if self.lost:
            return None
        try:
            for ev in self.pg.event.get():
                if ev.type == self.pg.JOYDEVICEREMOVED:
                    self.lost = "controller disconnected"
                    return None
            v = float(self.js.get_axis(self.axis))
            stop = any(self.js.get_button(b) for b in STOP_BUTTONS
                       if b < self.n_buttons)
        except Exception as exc:
            self.lost = f"controller read failed ({exc})"
            return None
        self.last_ok = time.monotonic()
        v = max(-1.0, min(1.0, v))
        return (0.0 if abs(v) < self.deadzone else v * self.sign), stop

    def stale(self):
        return time.monotonic() - self.last_ok > STALE_S

    def close(self):
        try:
            self.pg.quit()
        except Exception:
            pass


class UdpStick:
    """Left-stick X from ps5_stick_sender.py on the laptop, over UDP.

    Only packets from --udp-from are accepted, and only with a sequence number
    newer than the last one. No fresh packet for STALE_S -> read() returns
    None and the caller releases the wheel. A packet with stop=1 is a stop.
    """

    def __init__(self, sender_ip, port, deadzone, invert):
        self.sender_ip = sender_ip
        self.deadzone = deadzone
        self.sign = -1.0 if invert else 1.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.last_seq = -1
        self.value, self.stop = 0.0, False
        self.last_ok = 0.0          # no packet yet
        self.lost = None
        self.ignored = 0
        self.name = f"UDP from {sender_ip}:{port} (ps5_stick_sender.py)"

    def _drain(self):
        while True:
            try:
                data, (ip, _port) = self.sock.recvfrom(256)
            except BlockingIOError:
                return
            except OSError:
                return
            if ip != self.sender_ip:
                self.ignored += 1
                continue
            try:
                tag, seq, stick, stop = data.decode().split(";")
                seq, stick, stop = int(seq), float(stick), stop == "1"
            except ValueError:
                self.ignored += 1
                continue
            if tag != "PS5" or seq <= self.last_seq or not -1.0 <= stick <= 1.0:
                self.ignored += 1
                continue
            self.last_seq = seq
            self.value, self.stop = stick, stop
            self.last_ok = time.monotonic()

    def read(self):
        """(stick deadzoned, stop) from a fresh packet, or None if stale."""
        self._drain()
        if self.stale():
            if self.last_ok:
                self.lost = "no data from the laptop for >0.25 s"
            return None
        v = self.value
        return (0.0 if abs(v) < self.deadzone else v * self.sign), self.stop

    def stale(self):
        return time.monotonic() - self.last_ok > STALE_S

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


class TcpStick(UdpStick):
    """Same packets as UdpStick, over TCP from an 'ssh -L' tunnel.

    Listens on 127.0.0.1 only, so the sole way in is through SSH - nothing on
    the Wi-Fi can reach it. One connection at a time; it closing (laptop
    script stopped, SSH dropped) goes stale and releases the wheel.
    """

    def __init__(self, port, deadzone, invert):
        self.deadzone = deadzone
        self.sign = -1.0 if invert else 1.0
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", port))
        self.srv.listen(1)
        self.srv.setblocking(False)
        self.conn = None
        self.buf = b""
        self.last_seq = -1
        self.value, self.stop = 0.0, False
        self.last_ok = 0.0
        self.lost = None
        self.ignored = 0
        self.name = f"SSH tunnel on 127.0.0.1:{port} (ps5_stick_sender.py --ssh-tunnel)"

    def _drain(self):
        if self.conn is None:
            try:
                self.conn, _ = self.srv.accept()
                self.conn.setblocking(False)
                self.buf = b""
            except BlockingIOError:
                return
        while True:
            try:
                chunk = self.conn.recv(4096)
            except BlockingIOError:
                break
            except OSError:
                chunk = b""
            if not chunk:                       # laptop side closed
                self.conn.close()
                self.conn = None
                break
            self.buf += chunk
        *lines, self.buf = self.buf.split(b"\n")
        for line in lines:
            self._take(line.strip())

    def _take(self, data):
        try:
            tag, seq, stick, stop = data.decode().split(";")
            seq, stick, stop = int(seq), float(stick), stop == "1"
        except ValueError:
            self.ignored += 1
            return
        if tag != "PS5" or seq <= self.last_seq or not -1.0 <= stick <= 1.0:
            self.ignored += 1
            return
        self.last_seq = seq
        self.value, self.stop = stick, stop
        self.last_ok = time.monotonic()

    def close(self):
        for sck in (self.conn, self.srv):
            try:
                if sck:
                    sck.close()
            except Exception:
                pass


def stick_to_angle(stick, max_angle):
    """Stick right (+) -> negative = RIGHT, matching the draft's convention."""
    return -stick * max_angle


def ramp(current, target, rate, dt):
    step = rate * dt
    if current < target:
        return min(target, current + step)
    if current > target:
        return max(target, current - step)
    return current


def wait_for_centre(stick, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        r = stick.read()
        if r is not None and r[0] == 0.0 and not r[1]:
            return True
        time.sleep(0.02)
    return False


# ---------------------------------------------------------------- dry run

def dry_run(args, stick):
    print("DRY RUN - no panda, no CAN, nothing leaves this machine.")
    print("Pretending the wheel starts at +0.00 deg. Square/Circle/Cross or "
          "Ctrl+C stops.\n")
    cmd, counter = 0.0, 0
    t0 = last = last_print = time.monotonic()
    try:
        while time.monotonic() - t0 < args.seconds:
            now = time.monotonic()
            r = stick.read()
            if r is None or stick.stale():
                print(f"!! {stick.lost or 'controller stale'} -> would release")
                return
            v, stop = r
            if stop:
                print("stop button -> would release")
                return
            target = stick_to_angle(v, args.max_angle)
            cmd = ramp(cmd, target, args.rate, now - last)
            last = now
            frame = encode_lkas(cmd, args.torque, True, counter)
            counter = (counter + 1) & 0x0F
            if now - last_print >= 0.2:
                last_print = now
                side = "LEFT " if cmd > 0.05 else "RIGHT" if cmd < -0.05 else "     "
                print(f"stick {v:+.3f}  target {target:+7.2f}  cmd {cmd:+7.2f} deg {side}"
                      f"  0x169 {' '.join(f'{b:02x}' for b in frame)}")
            time.sleep(1 / 120)
    except KeyboardInterrupt:
        print("\nstopped")


# ---------------------------------------------------------------- main

def main():
    p = argparse.ArgumentParser(description="PS5 left stick -> absolute steering")
    p.add_argument("--max-angle", type=float, default=30.0, metavar="DEG",
                   help=f"full stick = this many deg left/right of centre "
                        f"(default 30, max {MAX_ANGLE_CAP:.0f})")
    p.add_argument("--torque", type=float, default=1.0, metavar="NM")
    p.add_argument("--rate", type=float, default=100.0, metavar="DEG_S")
    p.add_argument("--seconds", type=float, default=30.0,
                   help=f"run length (max {MAX_SECONDS:.0f})")
    p.add_argument("--listen", type=float, default=4.0, metavar="SEC")
    p.add_argument("--override", type=float, default=STEER_THRESHOLD, metavar="NM",
                   help=f"driver torque that takes over, averaged over "
                        f"{TORQUE_SAMPLES} samples (default {STEER_THRESHOLD})")
    p.add_argument("--steer-axis", type=int, default=0)
    p.add_argument("--deadzone", type=float, default=0.05)
    p.add_argument("--invert", action="store_true",
                   help="flip direction if the wheel turns opposite to the stick")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--udp-from", metavar="LAPTOP_IP",
                   help="take the stick from ps5_stick_sender.py on this IP "
                        "instead of a controller plugged into this machine")
    p.add_argument("--udp-port", type=int, default=5005)
    p.add_argument("--tcp-port", type=int, metavar="PORT",
                   help="take the stick through an SSH tunnel on this local "
                        "port (ps5_stick_sender.py --ssh-tunnel), e.g. 5006")
    args = p.parse_args()

    if not 0 < args.max_angle <= MAX_ANGLE_CAP:
        p.error(f"--max-angle must be >0 and <= {MAX_ANGLE_CAP:.0f}")
    if not 0 < args.seconds <= MAX_SECONDS:
        p.error(f"--seconds must be >0 and <= {MAX_SECONDS:.0f}")
    if not 0.0 <= args.torque <= 2.5:
        p.error("--torque must be 0 to 2.5")
    if args.override <= 0:
        p.error("--override must be > 0")

    print()
    if not self_test():
        return
    print()

    try:
        if args.tcp_port:
            stick = TcpStick(args.tcp_port, args.deadzone, args.invert)
        elif args.udp_from:
            stick = UdpStick(args.udp_from, args.udp_port, args.deadzone,
                             args.invert)
        else:
            stick = Stick(args.steer_axis, args.deadzone, args.invert)
    except (RuntimeError, OSError) as exc:
        print(f"controller: {exc}")
        return
    atexit.register(stick.close)
    print(f"controller: {stick.name}, full stick = "
          f"+/-{args.max_angle:.0f} deg{' (inverted)' if args.invert else ''}")
    print("centre the left stick (and release all buttons)...")
    remote = bool(args.udp_from or args.tcp_port)
    if not wait_for_centre(stick, 60.0 if remote else 10.0):
        print("stick not centred, or no controller data - aborting")
        return

    if args.dry_run:
        dry_run(args, stick)
        return

    from manual_can_interface import CANInterface
    can = CANInterface(simulation=False, allow_output=True)
    panda = can.panda
    sender = Sender(panda)

    closed = {"done": False}

    def close_once():
        if closed["done"]:
            return
        closed["done"] = True
        try:
            can.set_relay(False)
            print("failsafe: relay closed on exit")
        except Exception as exc:
            print(f"!! FAILSAFE COULD NOT CLOSE THE RELAY: {exc}")
            print(" Camera may still be isolated. Ignition cycle the car.")

    atexit.register(close_once)

    # ---- baseline, relay closed (same as the draft)
    print(f"baseline - relay CLOSED, listening {args.listen:.1f} s")
    can.set_relay(False)
    time.sleep(0.6)

    start_angle = status = None
    rest_torques = []
    end = time.monotonic() + args.listen
    while time.monotonic() < end:
        for m in panda.can_recv():
            addr, data, _bus = unpack(m)
            if addr == EPS_ADDR:
                d = decode_185(data)
                if d is not None:
                    start_angle, status = d
                t = driver_torque(data)
                if t is not None:
                    rest_torques.append(t)

    if start_angle is None:
        print(" no 0x185 - is the car awake? Is pandad stopped?")
        return
    if not rest_torques:
        print(" !! no torque readings - driver override cannot work. Stopping.")
        return

    print(f" wheel {start_angle:+.2f} deg, LKAS_STATUS {status} "
          f"({LKAS_STATUS_NAMES.get(status, '?')})")
    torque_zero = sum(rest_torques) / len(rest_torques)
    spread = max(rest_torques) - min(rest_torques)
    print(f" driver torque at rest {torque_zero:+.2f} Nm (spread {spread:.2f}) "
          f"- using that as zero")
    if spread > args.override:
        print(f" !! resting noise is wider than the {args.override:.2f} Nm "
              f"threshold - it will trip instantly. Stopping.")
        return
    if status == 14:
        print(" (14 = EPS still recovering from an earlier isolation; wait a"
              " couple of minutes or the run will be hard to read)")

    print()
    print("plan - JOYSTICK, absolute angle, positive is LEFT:")
    print(f" stick centre 0 deg, full left +{args.max_angle:.0f}, "
          f"full right -{args.max_angle:.0f}")
    print(f" at GO the wheel ramps {start_angle:+.2f} -> +0.00 "
          f"({abs(start_angle) / args.rate:.2f} s)")
    print(f" MAX_TORQUE {args.torque:.2f}, ramp {args.rate:.0f} deg/s, "
          f"{args.seconds:.0f} s max")
    print(f" driver override ON at {args.override:.2f} Nm averaged over "
          f"{TORQUE_SAMPLES} samples")
    print(" Square / Circle / Cross, or losing the controller, also lets go")
    print()
    print("CAR IN PARK. WHEELS CHOCKED.")
    if input("type GO to continue: ").strip() != "GO":
        return
    if not wait_for_centre(stick, 3.0):
        print("stick not centred at GO - aborting")
        return

    print()
    print("opening relay - camera isolated, we are its only route to the car")
    can.set_relay(True)
    time.sleep(0.4)

    commanded = measured = start_angle
    target = 0.0
    stick_v = 0.0
    seen_statuses = {}
    cam_169 = fwd = 0
    torque_window = deque([0.0] * TORQUE_SAMPLES, TORQUE_SAMPLES)
    peak_torque = 0.0
    released = None            # dict once we have let go
    stop_at = None
    end_reason = "time limit"

    t0 = last_tick = last_report = time.monotonic()

    def release(reason, now, avg=0.0):
        nonlocal released
        if released is None:
            released = {"at": now - t0, "reason": reason, "torque": avg,
                        "angle": measured, "target": target}
            print()
            print(f" *** {reason} - releasing the wheel at {measured:+.2f} deg ***")

    try:
        while True:
            now = time.monotonic()
            dt = now - last_tick
            last_tick = now

            if now - t0 >= args.seconds and released is None:
                release("time limit", now)

            # controller -> target
            if released is None:
                r = stick.read()
                if r is None or stick.stale():
                    release(stick.lost or "controller stale", now)
                elif r[1]:
                    release("stop button", now)
                else:
                    stick_v = r[0]
                    target = stick_to_angle(stick_v, args.max_angle)
                    commanded = ramp(commanded, target, args.rate, dt)

            # after release: one second of inactive frames, then stop
            if released is not None:
                if stop_at is None:
                    stop_at = now + 1.0
                elif now >= stop_at:
                    end_reason = released["reason"]
                    break

            out = []
            for m in panda.can_recv():
                addr, data, bus = unpack(m)

                if addr == EPS_ADDR:
                    d = decode_185(data)
                    if d is not None:
                        measured, st = d
                        seen_statuses[st] = seen_statuses.get(st, 0) + 1
                    t = driver_torque(data)
                    if t is not None:
                        torque_window.append(t - torque_zero)
                        avg = abs(sum(torque_window) / TORQUE_SAMPLES)
                        peak_torque = max(peak_torque, avg)
                        if released is None and avg > args.override:
                            release(f"DRIVER OVERRIDE at {avg:.2f} Nm", now, avg)

                if bus != CAM_BUS:
                    continue

                if addr == LKAS_ADDR:
                    cam_169 += 1
                    if released is not None:
                        out.append((LKAS_ADDR,
                                    encode_lkas(measured, 0.0, False,
                                                lkas_counter(data)),
                                    CAR_BUS))
                    else:
                        out.append((LKAS_ADDR,
                                    encode_lkas(commanded, args.torque, True,
                                                lkas_counter(data)),
                                    CAR_BUS))
                    continue

                out.append((addr, data, CAR_BUS))
                fwd += 1

            sender.send(out)

            if now - last_report >= 0.5:
                last_report = now
                st = max(seen_statuses, key=seen_statuses.get) if seen_statuses else None
                avg_now = abs(sum(torque_window) / TORQUE_SAMPLES)
                tag = "RELEASED" if released is not None else "steer"
                print(f" t+{now - t0:5.1f}s {tag:<8} stick {stick_v:+.2f} "
                      f"cmd {commanded:+7.2f} wheel {measured:+7.2f} "
                      f"err {commanded - measured:+6.2f} driver {avg_now:4.2f} Nm "
                      f"status {st} ({LKAS_STATUS_NAMES.get(st, '?')})")

    except KeyboardInterrupt:
        end_reason = "interrupted"
        print("\ninterrupted")

    finally:
        close_once()
        elapsed = max(1e-6, time.monotonic() - t0)
        print()
        print("=== result ===")
        print(f"ended by         : {end_reason}")
        print(f"elapsed {elapsed:.1f} s  camera 0x169 {cam_169} "
              f"({cam_169 / elapsed:.1f} Hz of ~120)  forwarded {fwd}")
        if sender.errors:
            print(f"send errors: {sender.errors}")
        print("LKAS_STATUS seen:")
        for st, n in sorted(seen_statuses.items()):
            print(f" {st:>2} ({LKAS_STATUS_NAMES.get(st, '?')}): {n}")
        print(f"highest driver torque: {peak_torque:.2f} Nm "
              f"(override at {args.override:.2f})")
        if released is not None:
            print(f"released at t+{released['at']:.1f}s, wheel "
                  f"{released['angle']:+.2f} deg, heading for "
                  f"{released['target']:+.2f} - inactive frames sent after")
        print()
        can.print_summary()


if __name__ == "__main__":
    main()
