"""Pure monocular visual-odometry logic for the K10 ORB feature stream (v1).

No sockets here -- feed decoded ORBF frames in (via `frame_from_packet`),
get pose/trajectory records out (via `MonocularVO.process`). See
recv_server.py (owns the UDP receive loop) and vo_worker.py (the thread that
drains frames from a queue into this module).

v1 scope and accepted limitations (see /Users/jonathan/.claude/plans/
ok-i-want-to-fancy-lake.md for the full writeup):
  - Pure monocular VO, no IMU fusion -- the K10 has no gyroscope (SC7A20H is
    accel-only), so real inertial preintegration isn't possible. The
    accelerometer stream is not consulted anywhere in this module.
  - Monocular scale is permanently ambiguous: fixed arbitrarily at bootstrap
    (the first frame pair's translation is unit-norm) and never tied to an
    absolute reference. Treat trajectory coordinates as "relative to the
    first frame pair", not meters -- the dashboard labels it as such.
  - No persistent map or loop closure -- each re-bootstrap after tracking
    loss starts a fresh, disconnected trajectory `segment`, not a
    relocalization against the old map.
"""

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares

# --------------------------------------------------------------------------
# Tunables
# --------------------------------------------------------------------------

MIN_MATCHES_BOOTSTRAP = 12   # matched pairs required before attempting E-mat
MIN_MATCHES_TRACK = 8        # matched pairs required before attempting PnP
MIN_INLIERS = 12             # RANSAC inliers required to accept a pose
MIN_PARALLAX_PX = 3.0        # median matched-keypoint displacement (detection-
                              # res px, i.e. the 120x160 space ORBF streams in)
                              # below which bootstrap is rejected -- guards
                              # against essential-matrix degeneracy under
                              # near-pure-rotation motion (a likely real
                              # usage pattern for a handheld board).
LOST_FRAMES_THRESHOLD = 5    # consecutive failed-track frames -> re-bootstrap
RANSAC_THRESHOLD_PX = 1.5
RANSAC_CONFIDENCE = 0.999
BA_WINDOW = 6                # sliding-window size (frames) for local BA
MAP_EXTEND_EVERY = 5         # frames between "add new landmarks" attempts
MAP_EXTEND_MIN_SIZE = 20     # ...also trigger early if map shrinks below this
MAX_MAP_POINTS = 400         # cap so the map doesn't grow unbounded
MAX_TRAJECTORY = 2000        # cap on retained trajectory records


# --------------------------------------------------------------------------
# Camera model
# --------------------------------------------------------------------------

@dataclass
class CameraIntrinsics:
    K: np.ndarray      # 3x3, already scaled to the ORBF stream's coordinate
                        # space (the 120x160 half-res detection frame -- see
                        # firmware/k10-fast-corners/src/main.cpp's DET_W/DET_H)
    dist: np.ndarray    # distortion coeffs (k1 k2 p1 p2 k3)

    @classmethod
    def load(cls, path):
        """Load a calibrate.py output JSON, scaling K from the captured
        resolution (240x320, full camera frame) down to the ORBF stream's
        actual coordinate space via the file's `stream_scale` (0.5)."""
        import json
        with open(path) as f:
            d = json.load(f)
        K = np.array(d["K"], dtype=np.float64)
        scale = d.get("stream_scale", 0.5)
        K = K.copy()
        K[0, 0] *= scale
        K[1, 1] *= scale
        K[0, 2] *= scale
        K[1, 2] *= scale
        dist = np.array(d.get("dist", [0.0] * 5), dtype=np.float64)
        return cls(K=K, dist=dist)


def frame_from_packet(pkt):
    """decode_orb_packet()'s dict -> (kp (N,2) float32, desc (N,32) uint8)."""
    pts = pkt["points"]
    n = len(pts)
    kp = np.empty((n, 2), dtype=np.float32)
    desc = np.empty((n, 32), dtype=np.uint8)
    for i, (x, y, _angle_rad, d) in enumerate(pts):
        kp[i, 0] = x
        kp[i, 1] = y
        desc[i] = np.frombuffer(d, dtype=np.uint8)
    return kp, desc


# --------------------------------------------------------------------------
# Matching / geometry primitives
# --------------------------------------------------------------------------

_matcher = cv2.BFMatcher(cv2.NORM_HAMMING)


def _match(desc_a, desc_b, ratio=0.75):
    """Lowe's-ratio-filtered brute-force Hamming matches on 256-bit ORB
    descriptors -- with only ~15-30 features/frame (see
    firmware/k10-camera-bench/RESULTS.md), brute force is trivially fast; no
    need for an approximate index. Returns (M,2) int array of (idx_a, idx_b)."""
    if len(desc_a) < 2 or len(desc_b) < 2:
        return np.empty((0, 2), dtype=int)
    knn = _matcher.knnMatch(desc_a, desc_b, k=2)
    good = []
    for pair in knn:
        if len(pair) < 2:
            continue
        m, n = pair
        if m.distance < ratio * n.distance:
            good.append((m.queryIdx, m.trainIdx))
    return np.array(good, dtype=int).reshape(-1, 2)


def _undistort(kp, intr):
    """kp (N,2) pixel coords -> undistorted pixel coords (still in K's
    projection, via P=K), so downstream code can keep using intr.K as-is."""
    pts = kp.reshape(-1, 1, 2).astype(np.float64)
    und = cv2.undistortPoints(pts, intr.K, intr.dist, P=intr.K)
    return und.reshape(-1, 2)


def _triangulate(R0, t0, pts0, R1, t1, pts1, intr):
    """Triangulate matched undistorted point pairs given two known poses
    (world->camera, X_cam = R@X_world + t -- the OpenCV/solvePnP convention).
    Returns (pts3d (N,3), valid (N,) bool -- cheirality: in front of both
    cameras)."""
    P0 = intr.K @ np.hstack([R0, t0.reshape(3, 1)])
    P1 = intr.K @ np.hstack([R1, t1.reshape(3, 1)])
    pts4d = cv2.triangulatePoints(P0, P1, pts0.T, pts1.T)
    pts3d = (pts4d[:3] / pts4d[3]).T
    z0 = (R0 @ pts3d.T + t0.reshape(3, 1))[2]
    z1 = (R1 @ pts3d.T + t1.reshape(3, 1))[2]
    valid = (z0 > 0) & (z1 > 0) & np.isfinite(pts3d).all(axis=1)
    return pts3d, valid


@dataclass
class _Bootstrap:
    R: np.ndarray        # 3x3, frame B relative to frame A (A = origin)
    t: np.ndarray         # (3,), unit-norm -- arbitrary, permanent scale
    pts3d: np.ndarray     # (N,3), triangulated landmarks, in A's frame
    desc: np.ndarray      # (N,32) uint8 -- frame B's descriptors for each point
    pts2d_B: np.ndarray   # (N,2) undistorted pixel coords in frame B


def _try_bootstrap(kp0, desc0, kp1, desc1, intr):
    """Essential-matrix + triangulation init from a single frame pair, or
    None if matches/parallax/inliers are insufficient. See MIN_PARALLAX_PX's
    docstring for why the parallax gate exists."""
    matches = _match(desc0, desc1)
    if len(matches) < MIN_MATCHES_BOOTSTRAP:
        return None
    pts0 = kp0[matches[:, 0]]
    pts1 = kp1[matches[:, 1]]
    disp = np.linalg.norm(pts1 - pts0, axis=1)
    if np.median(disp) < MIN_PARALLAX_PX:
        return None

    pts0u = _undistort(pts0, intr)
    pts1u = _undistort(pts1, intr)
    E, mask = cv2.findEssentialMat(pts0u, pts1u, intr.K, method=cv2.RANSAC,
                                    prob=RANSAC_CONFIDENCE,
                                    threshold=RANSAC_THRESHOLD_PX)
    if E is None or mask is None or E.shape != (3, 3):
        return None
    inl = mask.ravel().astype(bool)
    if inl.sum() < MIN_INLIERS:
        return None

    n_pose, R, t, pose_mask = cv2.recoverPose(E, pts0u[inl], pts1u[inl], intr.K)
    if n_pose < MIN_INLIERS:
        return None
    pose_inl = pose_mask.ravel().astype(bool)

    sel0 = pts0u[inl][pose_inl]
    sel1 = pts1u[inl][pose_inl]
    sel_desc1 = desc1[matches[inl, 1][pose_inl]]

    pts3d, valid = _triangulate(np.eye(3), np.zeros(3), sel0, R, t.ravel(), sel1, intr)
    if valid.sum() < MIN_INLIERS:
        return None

    return _Bootstrap(R=R, t=t.ravel(), pts3d=pts3d[valid], desc=sel_desc1[valid],
                       pts2d_B=sel1[valid])


def _track_pnp(map_pts, map_desc, kp, desc, intr):
    """Match the current frame against the existing 3D map and solve for
    this frame's pose via PnP -- NOT chained essential matrices, which
    would produce a trajectory with meaningless relative segment lengths
    (each recoverPose call has its own independent unit-norm translation).
    Returns (R, t, n_inliers, map_idx, kp_idx) or None."""
    if len(map_pts) < MIN_MATCHES_TRACK or len(desc) < MIN_MATCHES_TRACK:
        return None
    matches = _match(map_desc, desc)
    if len(matches) < MIN_MATCHES_TRACK:
        return None

    obj_pts = map_pts[matches[:, 0]].astype(np.float64)
    img_pts = _undistort(kp[matches[:, 1]], intr).astype(np.float64)
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        obj_pts, img_pts, intr.K, None,
        reprojectionError=RANSAC_THRESHOLD_PX, confidence=RANSAC_CONFIDENCE,
        iterationsCount=200, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or inliers is None or len(inliers) < MIN_INLIERS:
        return None

    inl = inliers.ravel()
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.ravel()
    return R, t, len(inl), matches[inl, 0], matches[inl, 1]


# --------------------------------------------------------------------------
# Sliding-window local bundle adjustment
# --------------------------------------------------------------------------

def _bundle_adjust_window(window, map_pts, map_ids, K):
    """Refine window[1:]'s poses (window[0] held fixed as the gauge
    reference) and any map points observed >=2x in the window, by
    minimizing reprojection error. Best-effort: on any failure or an
    under-constrained problem, returns the window unchanged -- tracking's
    PnP pose already stands on its own without this.

    window: list of {"R": 3x3, "t": (3,), "obs": [(point_id, x, y), ...]}
    Returns (new_window, {point_id: new_xyz} | None).
    """
    id_to_map_idx = {pid: i for i, pid in enumerate(map_ids.tolist())}

    obs_count = {}
    for f in window:
        for pid, _x, _y in f["obs"]:
            obs_count[pid] = obs_count.get(pid, 0) + 1
    opt_ids = [pid for pid, c in obs_count.items() if c >= 2 and pid in id_to_map_idx]
    if not opt_ids:
        return window, None
    pid_to_opt = {pid: i for i, pid in enumerate(opt_ids)}

    n_poses = len(window) - 1
    n_pts = len(opt_ids)
    if n_poses <= 0:
        return window, None

    obs_list = []
    for fi, f in enumerate(window):
        for pid, ux, uy in f["obs"]:
            if pid in pid_to_opt:
                obs_list.append((fi, pid_to_opt[pid], ux, uy))
    if len(obs_list) < 2 * (n_poses + n_pts):   # under-constrained, skip
        return window, None

    x0 = np.zeros(6 * n_poses + 3 * n_pts)
    for i, f in enumerate(window[1:]):
        rvec, _ = cv2.Rodrigues(f["R"])
        x0[6 * i:6 * i + 3] = rvec.ravel()
        x0[6 * i + 3:6 * i + 6] = f["t"]
    for i, pid in enumerate(opt_ids):
        x0[6 * n_poses + 3 * i: 6 * n_poses + 3 * i + 3] = map_pts[id_to_map_idx[pid]]

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    R0, t0 = window[0]["R"], window[0]["t"]

    def residuals(x):
        pts = x[6 * n_poses:].reshape(n_pts, 3)
        res = np.zeros(2 * len(obs_list))
        for k, (fi, pi, ux, uy) in enumerate(obs_list):
            if fi == 0:
                Rm, tv = R0, t0
            else:
                rvec = x[6 * (fi - 1):6 * (fi - 1) + 3]
                tv = x[6 * (fi - 1) + 3:6 * (fi - 1) + 6]
                Rm, _ = cv2.Rodrigues(rvec)
            Xc = Rm @ pts[pi] + tv
            if Xc[2] <= 1e-6:
                res[2 * k:2 * k + 2] = 1e3
                continue
            res[2 * k] = fx * Xc[0] / Xc[2] + cx - ux
            res[2 * k + 1] = fy * Xc[1] / Xc[2] + cy - uy
        return res

    try:
        result = least_squares(residuals, x0, method="trf", loss="huber",
                                f_scale=2.0, max_nfev=50)
    except Exception:
        return window, None
    x = result.x

    new_window = [window[0]]
    for i in range(n_poses):
        rvec = x[6 * i:6 * i + 3]
        tvec = x[6 * i + 3:6 * i + 6]
        Rm, _ = cv2.Rodrigues(rvec)
        f = dict(window[i + 1])
        f["R"] = Rm
        f["t"] = tvec
        new_window.append(f)
    pts = x[6 * n_poses:].reshape(n_pts, 3)
    updated = {opt_ids[i]: pts[i] for i in range(n_pts)}
    return new_window, updated


# --------------------------------------------------------------------------
# MonocularVO -- the stateful pipeline
# --------------------------------------------------------------------------

class MonocularVO:
    """Feed ORBF frames in via process(kp, desc); get a trajectory record
    back each call. `self.trajectory` accumulates every record (capped at
    MAX_TRAJECTORY). Not thread-safe -- call process() from one thread only
    (see vo_worker.py)."""

    def __init__(self, intrinsics, ba_window=BA_WINDOW):
        self.intr = intrinsics
        self.ba_window = ba_window

        self.mode = "bootstrapping"       # "bootstrapping" | "tracking"
        self.prev_frame: Optional[Tuple[np.ndarray, np.ndarray]] = None
        # (kp, desc) of the most recent frame, for bootstrap attempts
        self._last_tracked_raw: Optional[
            Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = None
        # (kp, desc, R, t) of the most recent successfully-tracked frame,
        # for map extension

        self.map_pts = np.zeros((0, 3))
        self.map_desc = np.zeros((0, 32), dtype=np.uint8)
        self.map_ids = np.zeros((0,), dtype=np.int64)
        self._next_id = 0

        self.window = []
        self.segment = 0
        self.frame_idx = 0
        self.lost_streak = 0
        self.trajectory = []

    # -- public -------------------------------------------------------

    def process(self, kp, desc):
        self.frame_idx += 1
        if self.mode == "bootstrapping":
            rec = self._bootstrap_step(kp, desc)
        else:
            rec = self._track_step(kp, desc)
        self.prev_frame = (kp, desc)
        return rec

    # -- internals ------------------------------------------------------

    def _bootstrap_step(self, kp, desc):
        if self.prev_frame is None:
            return self._emit("bootstrapping", 0)
        kp0, desc0 = self.prev_frame
        boot = _try_bootstrap(kp0, desc0, kp, desc, self.intr)
        if boot is None:
            return self._emit("bootstrapping", 0)

        self.segment += 1
        n = len(boot.pts3d)
        self.map_pts = boot.pts3d
        self.map_desc = boot.desc
        self.map_ids = np.arange(self._next_id, self._next_id + n)
        self._next_id += n

        obs = list(zip(self.map_ids.tolist(),
                        boot.pts2d_B[:, 0].tolist(), boot.pts2d_B[:, 1].tolist()))
        self.window = [
            {"R": np.eye(3), "t": np.zeros(3), "obs": []},
            {"R": boot.R, "t": boot.t, "obs": obs},
        ]
        self.mode = "tracking"
        self.lost_streak = 0
        self._last_tracked_raw = (kp, desc, boot.R, boot.t)
        return self._emit("tracking", n, boot.R, boot.t)

    def _track_step(self, kp, desc):
        res = _track_pnp(self.map_pts, self.map_desc, kp, desc, self.intr)
        if res is None:
            self.lost_streak += 1
            if self.lost_streak >= LOST_FRAMES_THRESHOLD:
                self.mode = "bootstrapping"
                self.window = []
                return self._emit("lost", 0)
            return self._emit("tracking", 0)   # coast: no new pose this frame

        self.lost_streak = 0
        R, t, n_inliers, map_idx, kp_idx = res

        obs = list(zip(self.map_ids[map_idx].tolist(),
                        kp[kp_idx, 0].tolist(), kp[kp_idx, 1].tolist()))
        self.window.append({"R": R, "t": t, "obs": obs})
        if len(self.window) > self.ba_window:
            self.window.pop(0)

        if (self.frame_idx % MAP_EXTEND_EVERY == 0
                or len(self.map_pts) < MAP_EXTEND_MIN_SIZE):
            self._extend_map(kp, desc, R, t, kp_idx)
        else:
            self._last_tracked_raw = (kp, desc, R, t)

        if len(self.window) >= 3:
            self._bundle_adjust()

        return self._emit("tracking", n_inliers, R, t)

    def _extend_map(self, kp, desc, R, t, matched_kp_idx):
        """Triangulate newly-matched (not already in the map) feature pairs
        between the last successfully-tracked frame and this one, using
        their now-known poses, to keep adding landmarks as the camera moves
        past the bootstrap area."""
        if self._last_tracked_raw is None:
            self._last_tracked_raw = (kp, desc, R, t)
            return
        kp0, desc0, R0, t0 = self._last_tracked_raw
        self._last_tracked_raw = (kp, desc, R, t)

        matches = _match(desc0, desc, ratio=0.8)
        if len(matches) < 8:
            return
        excl = set(matched_kp_idx.tolist())
        keep = [i for i, (_i0, i1) in enumerate(matches) if i1 not in excl]
        if not keep:
            return
        m = matches[keep]
        pts0 = _undistort(kp0[m[:, 0]], self.intr)
        pts1 = _undistort(kp[m[:, 1]], self.intr)
        pts3d, valid = _triangulate(R0, t0, pts0, R, t, pts1, self.intr)
        if valid.sum() == 0:
            return

        new_pts = pts3d[valid]
        new_desc = desc[m[valid, 1]]
        n = len(new_pts)
        ids = np.arange(self._next_id, self._next_id + n)
        self._next_id += n
        self.map_pts = np.vstack([self.map_pts, new_pts])
        self.map_desc = np.vstack([self.map_desc, new_desc])
        self.map_ids = np.concatenate([self.map_ids, ids])

        if len(self.map_ids) > MAX_MAP_POINTS:
            self.map_pts = self.map_pts[-MAX_MAP_POINTS:]
            self.map_desc = self.map_desc[-MAX_MAP_POINTS:]
            self.map_ids = self.map_ids[-MAX_MAP_POINTS:]

    def _bundle_adjust(self):
        try:
            new_window, updated = _bundle_adjust_window(
                self.window, self.map_pts, self.map_ids, self.intr.K)
        except Exception:
            return
        self.window = new_window
        if updated:
            id_to_idx = {pid: i for i, pid in enumerate(self.map_ids.tolist())}
            for pid, pos in updated.items():
                idx = id_to_idx.get(pid)
                if idx is not None:
                    self.map_pts[idx] = pos

    def _emit(self, state, inliers, R=None, t=None):
        if R is not None and t is not None:
            # Camera center in the current segment's world frame: C = -R^T t
            x, y, z = (-R.T @ t).tolist()
        else:
            x = y = z = None
        rec = {"frame_idx": self.frame_idx, "segment": self.segment,
               "state": state, "inliers": int(inliers), "x": x, "y": y, "z": z}
        self.trajectory.append(rec)
        if len(self.trajectory) > MAX_TRAJECTORY:
            self.trajectory.pop(0)
        return rec
