"""PS5 left stick -> UDP -> comma device (steer_joystick.py --udp-from).

Runs on the LAPTOP the controller is plugged into. Sends at 60 Hz:

    PS5;<seq>;<stick>;<stop>

  stick  left-stick X, -1..1, raw (deadzone/invert are applied on the comma)
  stop   1 = let go now (stop button, Ctrl+C, controller lost)

The comma side releases the wheel if packets stop for more than 0.25 s, so
this script simply going silent is also a stop.

    python ps5_stick_sender.py --to 192.168.1.50        (plain UDP)

Through SSH instead (works when a firewall blocks UDP). In one window:
    ssh -N -L 5006:127.0.0.1:5006 comma@192.168.1.92
then in another:
    python ps5_stick_sender.py --ssh-tunnel
and on the comma: steer_joystick.py --tcp-port 5006
"""

import argparse
import os
import socket
import time

DEFAULT_PORT = 5005
HZ = 60
STOP_BUTTONS = (0, 1, 2)          # Cross / Circle / Square on Windows SDL
STOP_REPEATS = 10


def packet(seq, stick, stop):
    return f"PS5;{seq};{stick:.4f};{1 if stop else 0}".encode()


def main():
    p = argparse.ArgumentParser(description="send PS5 left stick to the comma")
    p.add_argument("--to", help="comma device IP (plain UDP mode)")
    p.add_argument("--ssh-tunnel", action="store_true",
                   help="send over TCP to 127.0.0.1:--tcp-port, i.e. through "
                        "an 'ssh -L' tunnel to the comma")
    p.add_argument("--tcp-port", type=int, default=5006)
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--axis", type=int, default=0)
    args = p.parse_args()
    if not args.ssh_tunnel and not args.to:
        p.error("give --to <comma IP>, or --ssh-tunnel")

    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
    import pygame
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No PS5 controller found.")
        return 1
    js = pygame.joystick.Joystick(0)
    if hasattr(js, "get_init") and not js.get_init():
        js.init()
    n_buttons = js.get_numbuttons()

    if args.ssh_tunnel:
        target = ("127.0.0.1", args.tcp_port)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.connect(target)
        except OSError as exc:
            print(f"Could not reach the tunnel on 127.0.0.1:{args.tcp_port} ({exc}).")
            print("Is 'ssh -N -L 5006:127.0.0.1:5006 comma@<ip>' running in another "
                  "window, and steer_joystick.py --tcp-port 5006 on the comma?")
            return 1
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        def tx(data):
            sock.sendall(data + b"\n")
    else:
        target = (args.to, args.port)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        def tx(data):
            sock.sendto(data, target)
    seq = 0

    def send_stop(reason):
        nonlocal seq
        print(f"\nSTOP ({reason}) - telling the comma to let go")
        for _ in range(STOP_REPEATS):
            try:
                tx(packet(seq, 0.0, True))
            except OSError:
                return
            seq += 1
            time.sleep(0.01)

    where = (f"SSH tunnel 127.0.0.1:{args.tcp_port}" if args.ssh_tunnel
             else f"{args.to}:{args.port} (UDP)")
    print(f"{js.get_name()} -> {where} at {HZ} Hz")
    print("Square/Circle/Cross or Ctrl+C = stop. Unplugging the controller "
          "also stops.\n")

    last_print = 0.0
    try:
        while True:
            t = time.monotonic()
            for ev in pygame.event.get():
                if ev.type == pygame.JOYDEVICEREMOVED:
                    send_stop("controller disconnected")
                    return 2
            try:
                stick = max(-1.0, min(1.0, float(js.get_axis(args.axis))))
                stop = any(js.get_button(b) for b in STOP_BUTTONS if b < n_buttons)
            except pygame.error as exc:
                send_stop(f"controller read failed: {exc}")
                return 2
            if stop:
                send_stop("stop button")
                return 0

            try:
                tx(packet(seq, stick, False))
            except OSError as exc:
                print(f"\nconnection to the comma lost ({exc}) - it will let go")
                return 2
            seq += 1

            if t - last_print >= 0.2:
                last_print = t
                print(f"seq {seq:6d}  stick {stick:+.3f}")
            time.sleep(max(0.0, 1 / HZ - (time.monotonic() - t)))
    except KeyboardInterrupt:
        send_stop("Ctrl+C")
    finally:
        sock.close()
        pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
