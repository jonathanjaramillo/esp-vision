#!/usr/bin/env python3
"""Host-side receiver for the K10 visual-odometry experiments.

Listens on UDP for whichever firmware is currently streaming and serves live
stats over a tiny web server (default http://localhost:8080):

    udp/5005  "ORBF"  ORB features      (firmware/k10-fast-corners)
    udp/5006  "ACCL"  accelerometer     (both firmwares)
    udp/5007  "TRCK"  LK tracks         (firmware/k10-lk-track)

The firmware unicasts to STREAM_HOST_IP (this laptop's address on the K10's
WiFi, baked in at firmware build time -- see README.md's Runbook), so run this
on that laptop while it's on the same WiFi as the board.

Reports, per stream: packets/sec ("messages"), frames/sec (one feature/track
packet == one video frame), detections or tracks per packet (avg + last),
and the latest/raw accelerometer samples with |a| magnitude.

    python3 recv_server.py                 # listen 5005/5006/5007, serve :8080
    python3 recv_server.py --http-port 9000 --verbose

Decoders are standalone copies kept in sync with the `#pragma pack(push,1)`
structs in lib/k10stream/k10stream.h -- if you change the wire format there,
change the formats below to match (see experiments/visual-odometry/README.md).

TRCK format (18 B header + n * 10 B track records):
    trck_pkt_hdr_t : magic[4]="TRCK"  seq:u32  t_us:u32  frame_w:u16  frame_h:u16  n:u16
    trck_pkt_track_t: x_q4:i16  y_q4:i16  id:u16  age:u16  score:u16
        -- x_q4/y_q4 are level-0 (half-res, pyramid base) pixel coords times
           16 (Q4 fixed-point) so LK's sub-pixel precision survives the wire;
           multiply by 2 / 16 for full-res camera coords.
"""

import argparse
import json
import math
import selectors
import socket
import struct
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# --------------------------------------------------------------------------
# Wire formats (must match lib/k10stream/k10stream.h exactly)
# --------------------------------------------------------------------------

ORB_HDR_FMT = "<4sIIHHH"
ORB_HDR_LEN = struct.calcsize(ORB_HDR_FMT)          # 18
ORB_CORNER_FMT = "<hhh32s"
ORB_CORNER_LEN = struct.calcsize(ORB_CORNER_FMT)    # 38

TRCK_HDR_FMT = "<4sIIHHH"
TRCK_HDR_LEN = struct.calcsize(TRCK_HDR_FMT)        # 18
TRCK_TRACK_FMT = "<hhHHH"
TRCK_TRACK_LEN = struct.calcsize(TRCK_TRACK_FMT)    # 10

ACCEL_FMT = "<4sIIhhh"
ACCEL_LEN = struct.calcsize(ACCEL_FMT)              # 18


def decode_orb_packet(data):
    """-> dict(seq, t_us, frame_w, frame_h, points=[(x, y, angle_rad, desc)]) or None."""
    if len(data) < ORB_HDR_LEN:
        return None
    magic, seq, t_us, w, h, n = struct.unpack_from(ORB_HDR_FMT, data, 0)
    if magic != b"ORBF" or len(data) < ORB_HDR_LEN + n * ORB_CORNER_LEN:
        return None
    pts = []
    off = ORB_HDR_LEN
    for _ in range(n):
        x, y, ang_mrad, desc = struct.unpack_from(ORB_CORNER_FMT, data, off)
        pts.append((x, y, ang_mrad / 1000.0, desc))
        off += ORB_CORNER_LEN
    return {"seq": seq, "t_us": t_us, "frame_w": w, "frame_h": h, "points": pts}


def decode_tracks_packet(data):
    """-> dict(seq, t_us, frame_w, frame_h, tracks=[(x, y, id, age, score)]) or None.

    x/y are floats (level-0 pixels; multiply by 2 for full-res camera px)."""
    if len(data) < TRCK_HDR_LEN:
        return None
    magic, seq, t_us, w, h, n = struct.unpack_from(TRCK_HDR_FMT, data, 0)
    if magic != b"TRCK" or len(data) < TRCK_HDR_LEN + n * TRCK_TRACK_LEN:
        return None
    tracks = []
    off = TRCK_HDR_LEN
    for _ in range(n):
        xq, yq, tid, age, score = struct.unpack_from(TRCK_TRACK_FMT, data, off)
        tracks.append((xq / 16.0, yq / 16.0, tid, age, score))
        off += TRCK_TRACK_LEN
    return {"seq": seq, "t_us": t_us, "frame_w": w, "frame_h": h, "tracks": tracks}


def decode_accel_packet(data):
    """-> dict(seq, t_us, ax, ay, az) or None."""
    if len(data) < ACCEL_LEN:
        return None
    magic, seq, t_us, ax, ay, az = struct.unpack(ACCEL_FMT, data[:ACCEL_LEN])
    if magic != b"ACCL":
        return None
    return {"seq": seq, "t_us": t_us, "ax": ax, "ay": ay, "az": az}


# --------------------------------------------------------------------------
# Stats bookkeeping
# --------------------------------------------------------------------------

class StreamStats:
    """Rolling packet/point counters for one UDP stream.

    A "frame" is one feature/track packet (the firmware sends exactly one per
    processed video frame); accel packets are samples, not frames."""

    def __init__(self, name, is_frame_stream=True):
        self.name = name
        self.is_frame_stream = is_frame_stream
        self.total_packets = 0
        self.total_points = 0
        self.last_count = 0
        self.last_wall = None       # time.time() of last packet
        self.first_seen = None
        self._win_packets = 0
        self._win_points = 0
        self._rate = 0.0            # packets/sec over the last window
        self._pt_rate = 0.0         # points/sec over the last window
        self.last_frame = None      # last decoded dict (feature streams)

    def on_packet(self, n_points=0):
        now = time.time()
        if self.first_seen is None:
            self.first_seen = now
        self.total_packets += 1
        self.total_points += n_points
        self.last_count = n_points
        self.last_wall = now
        self._win_packets += 1
        self._win_points += n_points

    def roll(self, window_s):
        """Called by the receive loop every ~stats-period; updates rates."""
        self._rate = self._win_packets / window_s if window_s > 0 else 0.0
        self._pt_rate = self._win_points / window_s if window_s > 0 else 0.0
        self._win_packets = 0
        self._win_points = 0

    def snapshot(self):
        return {
            "packets_total": self.total_packets,
            "packets_per_sec": round(self._rate, 1),
            "points_total": self.total_points,
            "points_per_sec": round(self._pt_rate, 1),
            "frames_per_sec": round(self._rate, 1) if self.is_frame_stream else None,
            "last_count": self.last_count,
            "avg_count": round(self.total_points / self.total_packets, 1)
                         if self.total_packets else 0.0,
            "last_age_s": None if self.last_wall is None else round(time.time() - self.last_wall, 2),
            "seen": self.first_seen is not None,
        }


# --------------------------------------------------------------------------
# Receive loop (runs in a daemon thread; feeds the shared `stats` object)
# --------------------------------------------------------------------------

class Receiver:
    def __init__(self, orb_port, accel_port, trck_port, stats_period, verbose):
        self.socks = {}
        self.stats = {
            "orb": StreamStats("ORB features"),
            "tracks": StreamStats("LK tracks"),
            "accel": StreamStats("Accelerometer", is_frame_stream=False),
        }
        self.accel_last = None
        self.accel_window = deque(maxlen=2000)  # (t_wall, ax, ay, az, t_us)
        self.stats_period = stats_period
        self.verbose = verbose
        self.started = time.time()
        self.bad_packets = 0
        for name, port in (("orb", orb_port), ("accel", accel_port),
                           ("tracks", trck_port)):
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("", port))
            s.setblocking(False)
            self.socks[name] = s

    def run(self):
        sel = selectors.DefaultSelector()
        for name, sock in self.socks.items():
            sel.register(sock, selectors.EVENT_READ, name)
        last_roll = time.time()
        while True:
            for key, _ in sel.select(timeout=1.0):
                data, _addr = key.fileobj.recvfrom(4096)
                kind = key.data
                if kind == "orb":
                    f = decode_orb_packet(data)
                    if f is None:
                        self.bad_packets += 1
                        continue
                    st = self.stats["orb"]
                    st.on_packet(len(f["points"]))
                    st.last_frame = f
                    if self.verbose:
                        print("ORBF seq=%d %dx%d n=%d" %
                              (f["seq"], f["frame_w"], f["frame_h"], len(f["points"])))
                elif kind == "tracks":
                    f = decode_tracks_packet(data)
                    if f is None:
                        self.bad_packets += 1
                        continue
                    st = self.stats["tracks"]
                    st.on_packet(len(f["tracks"]))
                    st.last_frame = f
                    if self.verbose:
                        print("TRCK seq=%d %dx%d n=%d" %
                              (f["seq"], f["frame_w"], f["frame_h"], len(f["tracks"])))
                else:
                    a = decode_accel_packet(data)
                    if a is None:
                        self.bad_packets += 1
                        continue
                    self.stats["accel"].on_packet(1)
                    self.accel_last = a
                    self.accel_window.append(
                        (time.time(), a["ax"], a["ay"], a["az"], a["t_us"]))

            now = time.time()
            dt = now - last_roll
            if dt >= self.stats_period:
                for st in self.stats.values():
                    st.roll(dt)
                last_roll = now

    def accel_summary(self):
        if not self.accel_window:
            return None
        _t, ax, ay, az, t_us = self.accel_window[-1]
        return {
            "last": {"ax": ax, "ay": ay, "az": az, "t_us": t_us,
                     "mag": round(math.sqrt(ax * ax + ay * ay + az * az), 1)},
            "window_n": len(self.accel_window),
            "window_mean": {
                k: round(sum(s[i] for s in self.accel_window) / len(self.accel_window), 1)
                for i, k in ((1, "ax"), (2, "ay"), (3, "az"))},
        }


# --------------------------------------------------------------------------
# Web server
# --------------------------------------------------------------------------

PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>K10 VO receiver</title>
<style>
 body{font-family:ui-monospace,Menlo,monospace;background:#111;color:#ddd;
      margin:2rem;max-width:60rem}
 h1{font-size:1.2rem} h2{font-size:1rem;margin-top:1.5rem;color:#8cf}
 table{border-collapse:collapse;margin:.4rem 0}
 td,th{padding:.15rem .8rem .15rem 0;text-align:right}
 td:first-child,th:first-child{text-align:left;color:#999}
 .live{color:#5f5} .idle{color:#f95}
 #accel canvas{background:#181818;margin-top:.4rem}
</style></head><body>
<h1>UNIHIKER K10 &mdash; visual odometry receiver</h1>
<div id="uptime"></div>
<h2>ORB features <span id="orb-live" class="idle"></span></h2>
<div id="orb"></div>
<h2>LK tracks <span id="tracks-live" class="idle"></span></h2>
<div id="tracks"></div>
<h2>Accelerometer <span id="accel-live" class="idle"></span></h2>
<div id="accel"></div>
<canvas id="plot" width="900" height="160"></canvas>
<script>
function row(k,v){return "<tr><td>"+k+"</td><td>"+v+"</td></tr>"}
function table(s){
  let h="<table>";
  h+=row("messages (packets)", s.packets_total.toLocaleString());
  h+=row("messages/sec", s.packets_per_sec.toFixed(1));
  if(s.frames_per_sec!==null) h+=row("frames/sec", s.frames_per_sec.toFixed(1));
  h+=row(s.name+" last packet", s.last_count);
  h+=row(s.name+" avg/packet", s.avg_count.toFixed(1));
  h+=row(s.name+"/sec", s.points_per_sec.toFixed(1));
  h+=row("last seen", s.last_age_s===null?"never":s.last_age_s.toFixed(1)+"s ago");
  return h+"</table>";
}
async function tick(){
  try{
    const r=await fetch("/stats.json"); const d=await r.json();
    document.getElementById("uptime").textContent =
      "listening since "+new Date(d.started*1000).toLocaleTimeString()+
      " · uptime "+d.uptime.toFixed(0)+"s · bad pkts "+d.bad_packets;
    for(const k of ["orb","tracks"]){
      const s=d[k]; s.name = k==="orb"?"features":"tracks";
      document.getElementById(k).innerHTML=table(s);
      const live=document.getElementById(k+"-live");
      live.textContent=s.packets_per_sec>0?"● streaming":"(silent)";
      live.className=s.packets_per_sec>0?"live":"idle";
    }
    const a=d.accel;
    document.getElementById("accel-live").textContent=
      a.packets_per_sec>0?"● streaming":"(silent)";
    document.getElementById("accel-live").className=
      a.packets_per_sec>0?"live":"idle";
    let h="<table>"+row("messages/sec",a.packets_per_sec.toFixed(1));
    if(d.accel_last){
      const L=d.accel_last;
      h+=row("ax / ay / az", L.ax+" / "+L.ay+" / "+L.az);
      h+=row("|a|", L.mag);
      h+=row("window mean", d.accel_summary.window_mean.ax+" / "+
             d.accel_summary.window_mean.ay+" / "+d.accel_summary.window_mean.az+
             "  (n="+d.accel_summary.window_n+")");
    }
    document.getElementById("accel").innerHTML=h+"</table>";
    if(d.accel_series) plot(d.accel_series);
  }catch(e){/* server not up yet */}
}
function plot(series){
  const c=document.getElementById("plot"),g=c.getContext("2d");
  g.clearRect(0,0,c.width,c.height);
  const n=series.ax.length; if(n<2) return;
  const lo=Math.min(...series.ax,...series.ay,...series.az);
  const hi=Math.max(...series.ax,...series.ay,...series.az);
  const span=Math.max(hi-lo,1);
  ["ax","ay","az"].forEach((k,i)=>{
    g.strokeStyle=["#5f5","#f95","#8cf"][i]; g.beginPath();
    series[k].forEach((v,j)=>{
      const x=j/(n-1)*c.width, y=c.height-4-(v-lo)/span*(c.height-8);
      j?g.lineTo(x,y):g.moveTo(x,y);
    });
    g.stroke();
  });
}
tick(); setInterval(tick,1000);
</script></body></html>"""


def make_handler(recv):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):  # quiet
            pass

        def _send(self, body, ctype):
            data = body.encode()
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send(PAGE, "text/html; charset=utf-8")
            elif self.path.startswith("/stats.json"):
                now = time.time()
                out = {
                    "started": recv.started,
                    "uptime": now - recv.started,
                    "bad_packets": recv.bad_packets,
                }
                for k in ("orb", "tracks", "accel"):
                    out[k] = recv.stats[k].snapshot()
                out["accel_last"] = None
                out["accel_summary"] = None
                out["accel_series"] = None
                if recv.accel_last is not None:
                    out["accel_last"] = {
                        **{k: recv.accel_last[k] for k in ("ax", "ay", "az", "t_us")},
                        "mag": round(math.sqrt(sum(recv.accel_last[c] ** 2
                                                  for c in ("ax", "ay", "az"))), 1)}
                    out["accel_summary"] = recv.accel_summary()
                    tail = list(recv.accel_window)[-300:]
                    out["accel_series"] = {
                        "ax": [s[1] for s in tail],
                        "ay": [s[2] for s in tail],
                        "az": [s[3] for s in tail]}
                self._send(json.dumps(out), "application/json")
            else:
                self.send_error(404)
    return Handler


def main():
    sys.stdout.reconfigure(line_buffering=True)
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--orb-port", type=int, default=5005)
    ap.add_argument("--accel-port", type=int, default=5006)
    ap.add_argument("--tracks-port", type=int, default=5007)
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--stats-period", type=float, default=1.0,
                    help="rate window for packets/sec and frames/sec")
    ap.add_argument("--verbose", action="store_true",
                    help="print one line per decoded feature/track packet")
    a = ap.parse_args()

    recv = Receiver(a.orb_port, a.accel_port, a.tracks_port,
                    a.stats_period, a.verbose)
    t = threading.Thread(target=recv.run, daemon=True)
    t.start()

    httpd = ThreadingHTTPServer(("0.0.0.0", a.http_port), make_handler(recv))
    print("receiver listening: ORBF udp/%d  ACCL udp/%d  TRCK udp/%d"
          % (a.orb_port, a.accel_port, a.tracks_port), file=sys.stderr)
    print("stats at http://localhost:%d/  (Ctrl-C to stop)" % a.http_port,
          file=sys.stderr)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
