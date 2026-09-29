"""Thread that drains decoded ORBF frames from a queue into vo.MonocularVO
and exposes the latest trajectory/state to the HTTP server.

No sockets here -- recv_server.py owns the UDP receive loop and pushes
decoded ORBF frame dicts (from decode_orb_packet) onto the queue this reads
from. Kept separate from vo.py so the VO math stays independently testable
with no threading involved (see the offline unit-test / replay.py workflow
in README.md).
"""

import threading

import vo


class VOWorker:
    def __init__(self, intrinsics, frame_queue, ba_window=vo.BA_WINDOW):
        self.vo = vo.MonocularVO(intrinsics, ba_window=ba_window)
        self.queue = frame_queue
        self._lock = threading.Lock()
        self._latest = None

    def run(self):
        while True:
            pkt = self.queue.get()
            if pkt is None:   # sentinel: stop
                return
            kp, desc = vo.frame_from_packet(pkt)
            rec = self.vo.process(kp, desc)
            with self._lock:
                self._latest = rec

    def start(self):
        t = threading.Thread(target=self.run, daemon=True)
        t.start()
        return t

    def trajectory_snapshot(self, n=500):
        """Called from the HTTP handler thread -- reads MonocularVO state
        the worker thread is concurrently appending to. Safe under the GIL
        (list slicing/appends don't tear), no lock needed on self.vo.trajectory
        itself; self._latest is guarded for clarity even though single
        attribute reads/writes are already atomic."""
        with self._lock:
            latest = self._latest
        points = list(self.vo.trajectory[-n:])
        return {"points": points, "latest": latest, "map_size": len(self.vo.map_ids)}
