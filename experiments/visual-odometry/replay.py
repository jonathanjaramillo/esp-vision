#!/usr/bin/env python3
"""Replay a UDP session recorded by `recv_server.py --record FILE`.

Lets VO tuning (RANSAC thresholds, inlier gates, parallax gate, BA window
size) iterate against a fixed dataset offline instead of requiring live
hardware for every change -- likely the actual bottleneck once tuning
starts, given how much of VO tuning is threshold-chasing.

Record a session (from this directory, with the board streaming):
    python3 recv_server.py --record session1.bin

Replay it back into a (separately running) recv_server.py:
    python3 replay.py session1.bin                # real-time pacing
    python3 replay.py session1.bin --speed 4       # 4x faster
    python3 replay.py session1.bin --speed 0       # as fast as possible

Record file format (must match recv_server.py's writer -- see its
`--record` handling): a flat sequence of
    <d wall_dt><H dest_port><H payload_len><payload_len bytes>
little-endian records, one per received UDP datagram of any kind (ORBF,
ACCL, or TRCK), in receive order. `wall_dt` is seconds since the recording
started, used to reproduce the original packet timing on replay.
"""

import argparse
import socket
import struct
import sys
import time

REC_HDR_FMT = "<dHH"
REC_HDR_LEN = struct.calcsize(REC_HDR_FMT)


def iter_records(path):
    with open(path, "rb") as f:
        while True:
            hdr = f.read(REC_HDR_LEN)
            if len(hdr) < REC_HDR_LEN:
                return
            wall_dt, port, length = struct.unpack(REC_HDR_FMT, hdr)
            data = f.read(length)
            if len(data) < length:
                return
            yield wall_dt, port, data


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("path")
    ap.add_argument("--host", default="127.0.0.1",
                     help="where the listening recv_server.py is")
    ap.add_argument("--speed", type=float, default=1.0,
                     help="playback speed multiplier (0 = as fast as possible)")
    a = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t0 = time.time()
    n = 0
    for wall_dt, port, data in iter_records(a.path):
        if a.speed > 0:
            delay = (t0 + wall_dt / a.speed) - time.time()
            if delay > 0:
                time.sleep(delay)
        sock.sendto(data, (a.host, port))
        n += 1
    print("replayed %d packets from %s" % (n, a.path), file=sys.stderr)


if __name__ == "__main__":
    main()
