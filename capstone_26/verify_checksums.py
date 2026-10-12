# ADDED - recover a message's checksum algorithm from captured frames.
#
# Why: our DBC comment on 0x169 documents the checksum as CRC-8/SAE-J1850
# with "initial value 0xFF", but manual_nissancan.py uses init_crc=0x00
# (copied from opendbc, which drives real 2018-23 Leafs). Both cannot be
# right for our 2025. If the wrong variant goes on the wire, the EPS drops
# every frame without complaint and it looks like the steering command is
# being ignored for some much more interesting reason.
#
# Capture real frames in Cabana or SavvyCAN, paste them below, run this.
# It brute-forces the parameter space and reports every combination that
# reproduces the checksum byte on EVERY frame you gave it.
#
# Usage:
#   python verify_checksums.py
#
# Paste at least 4-5 frames per ID. One frame will match many parameter
# sets by coincidence; five will usually leave exactly one.

# ---------------------------------------------------------------------------
# PASTE CAPTURED FRAMES HERE (hex, space or no separator, full data field)
# ---------------------------------------------------------------------------

# 0x169 LKAS - 8 bytes, checksum believed to be the last byte
LKAS_FRAMES = [
    # "7F EE 00 80 64 80 10 97",
]

# 0x27A TURN_SIGNALS - 7 bytes, checksum believed to be byte 7 (index 6)
TURN_SIGNAL_FRAMES = [
    # "00 00 00 01 00 07 84",
]


# ---------------------------------------------------------------------------

def parse(frame):
    return bytes.fromhex(frame.replace(" ", "").replace("0x", ""))


def crc8(data, poly, init, xor_out, reflect=False):
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            if reflect:
                crc = ((crc >> 1) ^ poly) & 0xFF if crc & 0x01 else (crc >> 1) & 0xFF
            else:
                crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ xor_out


# Polynomials worth trying. 0x1D is SAE-J1850, 0x2F is AUTOSAR/H2F,
# 0x07 is CRC-8/SMBUS, 0x31 and 0x9B show up on other makes.
POLYS = (0x1D, 0x2F, 0x07, 0x31, 0x9B)


def search_crc(frames, label):
    print(f"\n=== {label}: CRC-8 search over {len(frames)} frame(s) ===")
    if not frames:
        print("  no frames pasted, skipping")
        return

    sizes = {len(f) for f in frames}
    if len(sizes) > 1:
        print(f"  WARNING: mixed frame lengths {sizes}")

    hits = []
    n = len(frames[0])

    # which byte holds the checksum, and which bytes it covers
    for chk_idx in (n - 1, 0):
        for start in range(0, 2):
            for end in (chk_idx, n):
                if end <= start:
                    continue
                covered = (start, end)
                if chk_idx in range(start, end) and end - start > 1:
                    # covering the checksum byte itself only makes sense if
                    # it is zeroed first; skip, we test that separately below
                    continue
                for poly in POLYS:
                    for reflect in (False, True):
                        for init in range(256):
                            for xor_out in range(256):
                                if all(
                                    crc8(f[covered[0]:covered[1]], poly, init,
                                         xor_out, reflect) == f[chk_idx]
                                    for f in frames
                                ):
                                    hits.append((poly, init, xor_out, reflect,
                                                 covered, chk_idx))

    if not hits:
        print("  no CRC-8 parameterization matches. The checksum may be")
        print("  additive, cover a different byte range, or include the CAN ID.")
        print("  Try search_additive() below, or paste more frames.")
        return

    print(f"  {len(hits)} match(es):")
    for poly, init, xor_out, reflect, covered, chk_idx in hits[:20]:
        print(f"    poly=0x{poly:02X} init=0x{init:02X} xor_out=0x{xor_out:02X} "
              f"reflect={reflect} covers=data[{covered[0]}:{covered[1]}] "
              f"checksum_byte={chk_idx}")
    if len(hits) > 20:
        print(f"    ... and {len(hits) - 20} more - paste more frames to narrow it")

    # Is our code's variant among them?
    ours = (0x1D, 0x00, 0xFF, False, (0, n - 1), n - 1)
    doc = (0x1D, 0xFF, 0xFF, False, (0, n - 1), n - 1)
    print()
    print(f"  code's variant  (init 0x00, xor 0xFF, data[0:{n-1}]): "
          f"{'MATCHES' if ours in hits else 'does NOT match'}")
    print(f"  DBC comment     (init 0xFF, xor 0xFF, data[0:{n-1}]): "
          f"{'MATCHES' if doc in hits else 'does NOT match'}")


def search_additive(frames, label):
    """Plain byte-sum checksums, as documented for 0x27A."""
    print(f"\n=== {label}: additive search over {len(frames)} frame(s) ===")
    if not frames:
        print("  no frames pasted, skipping")
        return

    n = len(frames[0])
    hits = []

    for chk_idx in (n - 1, 0):
        for start in range(0, 3):
            for end in range(start + 1, n + 1):
                if chk_idx in range(start, end):
                    continue
                for const in range(256):
                    for sign in (1, -1):
                        if all(
                            (sign * sum(f[start:end]) + const) & 0xFF == f[chk_idx]
                            for f in frames
                        ):
                            hits.append((start, end, const, sign, chk_idx))

    if not hits:
        print("  no additive checksum matches")
        return

    print(f"  {len(hits)} match(es):")
    for start, end, const, sign, chk_idx in hits[:20]:
        op = "sum" if sign == 1 else "-sum"
        print(f"    ({op}(data[{start}:{end}]) + 0x{const:02X}) & 0xFF "
              f"-> byte {chk_idx}")
    if len(hits) > 20:
        print(f"    ... and {len(hits) - 20} more")

    ours = (0, 6, 0x7C, 1, 6)
    print()
    print(f"  code's variant (sum(data[0:6]) + 0x7C -> byte 6): "
          f"{'MATCHES' if ours in hits else 'does NOT match'}")


if __name__ == "__main__":
    lkas = [parse(f) for f in LKAS_FRAMES]
    turn = [parse(f) for f in TURN_SIGNAL_FRAMES]

    if not lkas and not turn:
        print(__doc__ or "")
        print("Paste captured frames into LKAS_FRAMES / TURN_SIGNAL_FRAMES first.")
        raise SystemExit(1)

    if lkas:
        search_crc(lkas, "0x169 LKAS")
    if turn:
        search_additive(turn, "0x27A TURN_SIGNALS")
        search_crc(turn, "0x27A TURN_SIGNALS")
