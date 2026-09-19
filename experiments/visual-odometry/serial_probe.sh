#!/usr/bin/env bash
# One-shot camera health probe: hard-reset the K10 via RTS, capture the
# boot + first ~16s of serial (esp_camera DMA logs, cam_hal warnings, the
# firmware's 1 Hz stats line), and print a verdict.
#
#   ./serial_probe.py [port] [seconds]
#
# Why this exists: whether the GC2145 DVP path is alive is decided by log
# lines that only appear once at boot ("cam config ok", node counts) and by
# whether "Failed to get the frame on time!" spam appears after — neither is
# visible from the UDP side, and `pio device monitor` can't be scripted.
set -euo pipefail
python3 - "$@" <<'EOF'
import serial, time, sys
port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem101"
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 16
ser = serial.Serial(port, 115200, timeout=1)
ser.dtr = False; ser.rts = True; time.sleep(0.1); ser.rts = False
time.sleep(0.3); ser.reset_input_buffer()
end, buf = time.time() + secs, b""
while time.time() < end:
    d = ser.read(512)
    if d: buf += d
txt = buf.decode(errors="replace")
lines = txt.splitlines()
key = [l for l in lines if any(k in l for k in
       ("cam config", "dma", "DMA", "buffer_size", "Allocating", "gc2145",
        "Detected", "CAMERA_INIT", "Failed to get", "wifi up", "ready.",
        "fps=", "Guru", "abort", "abort()", "Task"))]
print("\n".join(key[:40]))
starve = sum(1 for l in lines if "Failed to get the frame on time" in l)
stats = [l for l in lines if l.startswith("fps=")]
up = sum(1 for l in stats if not l.split("fps=")[1].split(" ")[0] == "0")
print("---- verdict: %d starvation warnings | %d/%d stats lines with fps>0 ----"
      % (starve, up, len(stats)))
EOF
