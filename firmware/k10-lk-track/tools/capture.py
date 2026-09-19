#!/usr/bin/env python3
"""Capture one frame from the K10 over USB-CDC and write it as a PNG.

    tools/capture.py                 # detector luma (120x160) -> gray.png
    tools/capture.py --raw           # raw RGB565 (240x320)    -> frame.png
    tools/capture.py -p /dev/cu.usbmodem1101 -o out.png

Sends 'd' (or 'D') and reads the base64 block the firmware answers with.
Needs pyserial; Pillow is optional (without it a .pgm/.ppm is written, which
Preview and every image tool opens fine).
"""

import argparse
import base64
import glob
import re
import sys
import time

# The pipeline task emits the dump while loop() keeps printing its 1 Hz stats
# line, so the two interleave on the wire. Payload lines are full 64-char base64
# groups and nothing else is, so match on that rather than trying to coordinate
# the two tasks.
B64_LINE = re.compile(r"^[A-Za-z0-9+/]{4,}={0,2}$")

try:
    import serial
except ImportError:
    sys.exit("pyserial missing — pip install pyserial")


def find_port():
    for pat in ("/dev/cu.usbmodem*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    sys.exit("no serial port found — pass -p")


def capture(port, cmd, timeout):
    # The K10 reboots on DTR; keep it asserted the way the monitor does.
    with serial.Serial(port, 115200, timeout=0.2) as ser:
        time.sleep(0.3)
        ser.reset_input_buffer()
        ser.write(cmd.encode())
        ser.flush()

        deadline = time.time() + timeout
        head, payload = None, []
        while time.time() < deadline:
            line = ser.readline().decode("ascii", "replace").strip()
            if not line:
                continue
            if line.startswith("--BEGIN"):
                head = line.split()          # --BEGIN TAG W H LEN
                payload = []
            elif line.startswith("--END"):
                if head:
                    return head, base64.b64decode("".join(payload))
            elif head is not None and B64_LINE.match(line):
                payload.append(line)
            # anything else (the 1 Hz stats line) is passed over
    sys.exit("timed out waiting for a frame — is the board running and not "
             "already held open by `pio device monitor`?")


def write_image(path, w, h, pixels, rgb):
    try:
        from PIL import Image
        mode = "RGB" if rgb else "L"
        Image.frombytes(mode, (w, h), pixels).save(path)
        return path
    except ImportError:
        path = path.rsplit(".", 1)[0] + (".ppm" if rgb else ".pgm")
        magic = b"P6" if rgb else b"P5"
        with open(path, "wb") as f:
            f.write(b"%s\n%d %d\n255\n" % (magic, w, h))
            f.write(pixels)
        return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", default=None)
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--raw", action="store_true", help="raw RGB565 frame")
    ap.add_argument("-t", "--timeout", type=float, default=30.0)
    a = ap.parse_args()

    port = a.port or find_port()
    head, data = capture(port, "D" if a.raw else "d", a.timeout)
    tag, w, h, n = head[1], int(head[2]), int(head[3]), int(head[4])
    if len(data) != n:
        print("warning: expected %d bytes, got %d" % (n, len(data)), file=sys.stderr)

    if tag == "RGB565":
        out = a.out or "frame.png"
        px = bytearray(w * h * 3)
        for i in range(w * h):
            v = (data[2 * i] << 8) | data[2 * i + 1]  # camera bytes are swapped vs CPU uint16_t
            r5, g6, b5 = (v >> 11) & 0x1F, (v >> 5) & 0x3F, v & 0x1F
            px[3 * i]     = (r5 << 3) | (r5 >> 2)
            px[3 * i + 1] = (g6 << 2) | (g6 >> 4)
            px[3 * i + 2] = (b5 << 3) | (b5 >> 2)
        path = write_image(out, w, h, bytes(px), True)
    else:
        out = a.out or "gray.png"
        path = write_image(out, w, h, data, False)

    print("wrote %s (%s %dx%d, %d bytes) from %s" % (path, tag, w, h, len(data), port))


if __name__ == "__main__":
    main()
