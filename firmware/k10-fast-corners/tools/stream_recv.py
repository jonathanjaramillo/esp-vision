#!/usr/bin/env python3
"""Receive the K10's UDP broadcast streams: ORB features + accelerometer.

    tools/stream_recv.py                     # print live stats for both streams
    tools/stream_recv.py --orb-port 5005 --accel-port 5006
    tools/stream_recv.py --once              # decode and print exactly one ORB packet, then exit

No K10 IP needs to be configured here: the firmware broadcasts on the local
subnet (see main.cpp's wifi_stream_send_orb/wifi_stream_send_accel), so this
just needs to be running on a machine on the same WiFi network ("Fallyn").

Wire format (must match the `#pragma pack(push,1)` structs in main.cpp
exactly — see PLAN.md "Streaming to a host for visual odometry"):

    orb_pkt_hdr_t    (18 B): magic[4]="ORBF"  seq:u32  t_us:u32
                             frame_w:u16  frame_h:u16  n:u16
    orb_pkt_corner_t (38 B, repeated n times): x:i16  y:i16
                             angle_mrad:i16  desc[32]:u8
        -- x, y are in the DETECTION frame (frame_w x frame_h, half the
           camera's) -- multiply by 2 for full-res camera/display coords.
        -- angle_mrad is radians * 1000, range (-3142, 3142].

    accel_pkt_t (18 B): magic[4]="ACCL"  seq:u32  t_us:u32
                        ax:i16  ay:i16  az:i16

This module's decode_orb_packet()/decode_accel_packet() are the reusable
piece for a future visual-odometry consumer -- import them directly instead
of re-parsing stdout.
"""

import argparse
import selectors
import socket
import struct
import sys
import time

ORB_HDR_FMT = "<4sIIHHH"
ORB_HDR_LEN = struct.calcsize(ORB_HDR_FMT)          # 18
ORB_CORNER_FMT = "<hhh32s"
ORB_CORNER_LEN = struct.calcsize(ORB_CORNER_FMT)    # 38
ACCEL_FMT = "<4sIIhhh"
ACCEL_LEN = struct.calcsize(ACCEL_FMT)              # 18


class OrbFrame:
    __slots__ = ("seq", "t_us", "frame_w", "frame_h", "corners")

    def __init__(self, seq, t_us, frame_w, frame_h, corners):
        self.seq = seq
        self.t_us = t_us
        self.frame_w = frame_w
        self.frame_h = frame_h
        self.corners = corners  # list of (x, y, angle_rad, desc: bytes[32])

    def __repr__(self):
        return "OrbFrame(seq=%d, t_us=%d, %dx%d, %d corners)" % (
            self.seq, self.t_us, self.frame_w, self.frame_h, len(self.corners))


class AccelSample:
    __slots__ = ("seq", "t_us", "ax", "ay", "az")

    def __init__(self, seq, t_us, ax, ay, az):
        self.seq, self.t_us, self.ax, self.ay, self.az = seq, t_us, ax, ay, az

    def __repr__(self):
        return "AccelSample(seq=%d, t_us=%d, ax=%d, ay=%d, az=%d)" % (
            self.seq, self.t_us, self.ax, self.ay, self.az)


def decode_orb_packet(data):
    """Decode one UDP payload into an OrbFrame, or None if it's not one
    (wrong magic / truncated -- UDP can drop or mangle packets)."""
    if len(data) < ORB_HDR_LEN:
        return None
    magic, seq, t_us, frame_w, frame_h, n = struct.unpack_from(ORB_HDR_FMT, data, 0)
    if magic != b"ORBF":
        return None
    need = ORB_HDR_LEN + n * ORB_CORNER_LEN
    if len(data) < need:
        return None
    corners = []
    off = ORB_HDR_LEN
    for _ in range(n):
        x, y, angle_mrad, desc = struct.unpack_from(ORB_CORNER_FMT, data, off)
        corners.append((x, y, angle_mrad / 1000.0, desc))
        off += ORB_CORNER_LEN
    return OrbFrame(seq, t_us, frame_w, frame_h, corners)


def decode_accel_packet(data):
    if len(data) < ACCEL_LEN:
        return None
    magic, seq, t_us, ax, ay, az = struct.unpack(ACCEL_FMT, data[:ACCEL_LEN])
    if magic != b"ACCL":
        return None
    return AccelSample(seq, t_us, ax, ay, az)


def make_socket(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", port))
    s.setblocking(False)
    return s


def main():
    # Line-buffer stdout even when piped/backgrounded/redirected to a file --
    # otherwise Python fully-buffers non-tty stdout and a killed/backgrounded
    # process can look like it received nothing when it didn't, right up
    # until the buffer flushes. Bit us once diagnosing this exact tool.
    sys.stdout.reconfigure(line_buffering=True)

    ap = argparse.ArgumentParser()
    ap.add_argument("--orb-port", type=int, default=5005)
    ap.add_argument("--accel-port", type=int, default=5006)
    ap.add_argument("--once", action="store_true",
                    help="decode and print exactly one ORB packet, then exit")
    ap.add_argument("--stats-period", type=float, default=1.0)
    a = ap.parse_args()

    sel = selectors.DefaultSelector()
    orb_sock = make_socket(a.orb_port)
    accel_sock = make_socket(a.accel_port)
    sel.register(orb_sock, selectors.EVENT_READ, "orb")
    sel.register(accel_sock, selectors.EVENT_READ, "accel")

    print("listening: ORB udp/%d, accel udp/%d (Ctrl-C to stop)" %
          (a.orb_port, a.accel_port), file=sys.stderr)

    orb_count = orb_corner_total = accel_count = 0
    last_accel = None
    last_report = time.time()

    try:
        while True:
            for key, _ in sel.select(timeout=1.0):
                sock = key.fileobj  # a socket.socket, registered below
                data, _addr = sock.recvfrom(4096)
                if key.data == "orb":
                    frame = decode_orb_packet(data)
                    if frame is None:
                        continue
                    orb_count += 1
                    orb_corner_total += len(frame.corners)
                    if a.once:
                        print(frame)
                        for i, (x, y, ang, desc) in enumerate(frame.corners):
                            print("  #%2d (%3d,%3d) angle=%+.2f rad  desc=%s"
                                  % (i, x, y, ang, desc.hex()))
                        return
                else:
                    sample = decode_accel_packet(data)
                    if sample is None:
                        continue
                    accel_count += 1
                    last_accel = sample

            now = time.time()
            if now - last_report >= a.stats_period:
                dt = now - last_report
                avg_corners = orb_corner_total / orb_count if orb_count else 0.0
                print("orb: %5.1f pkt/s (%.1f corners/pkt avg)   "
                      "accel: %5.1f pkt/s   last=%s"
                      % (orb_count / dt, avg_corners, accel_count / dt, last_accel))
                orb_count = orb_corner_total = accel_count = 0
                last_report = now
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
