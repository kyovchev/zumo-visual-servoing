#!/usr/bin/env python3
"""
Zumo robot: drive a square path with visual servoing (overhead camera + ArUco marker + WebSocket).

Setup
  pip install opencv-contrib-python numpy websocket-client
  - Camera looks straight down on the floor, robot has a 50 mm ArUco marker (ID 5) flat on top.
  - Robot firmware: zumo_esp32.ino (WebSocket on port 81, {"cmd":"drive","left":..,"right":..}).

Run
  python zumo_square_servo.py --host 192.168.1.50 --cam 0
  python zumo_square_servo.py --no-robot            (vision test only, robot is never commanded)

How it works
  1. Marker pixel size gives the scale (mm per pixel). The marker plane is the measurement plane,
     so the camera must be roughly perpendicular to the floor.
  2. Auto-calibration: a short spin gives the turn direction, a short straight move gives the angle
     between the marker's axis and the robot's forward direction.
  3. The square (side 300 mm) starts at the robot's current pose: first edge along its heading,
     then right turns (use --ccw for left turns), back to the start.
  4. Position-based visual servoing, every camera frame:
       - TURN: rotate in place until the heading error is small, stop, settle, re-check.
       - DRIVE: chase a lookahead point on the edge (keeps the robot on the line), speed falls
         with distance to the corner, steering is proportional to the heading error.
     If the marker is lost the robot stops and resumes when it is seen again.

Keys (in the video window):  S = start,  Q or ESC = abort (robot is stopped).
"""
import argparse
import json
import math
import os
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np
import websocket  # pip install websocket-client


# ----------------------------------------------------------------------------- helpers
def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def circ_mean(angles):
    return math.atan2(np.mean(np.sin(angles)), np.mean(np.cos(angles)))


def screen_size():
    """Screen size in pixels (Windows API; falls back to Full HD elsewhere)."""
    try:
        import ctypes
        u = ctypes.windll.user32
        return u.GetSystemMetrics(0), u.GetSystemMetrics(1)
    except Exception:
        return 1920, 1080


class Abort(Exception):
    pass


@dataclass
class Pose:
    x: float        # mm
    y: float        # mm (image y axis points down)
    ang: float      # raw marker angle in the image, rad
    # robot heading in the image, rad (marker angle + calibrated offset)
    theta: float
    side: float     # marker side length in pixels


@dataclass
class Cfg:
    host: str = "zumo.local"
    port: int = 81
    cam: str = "0"
    width: int = 1280
    height: int = 720
    dict_name: str = "DICT_ARUCO_ORIGINAL"
    marker_id: int = 7
    marker_mm: float = 50.0
    side_mm: float = 300.0
    ccw: bool = False
    calib: str = ""
    no_robot: bool = False
    offset_deg: float = None
    turn_sign: int = 0
    # controller (wheel commands are Zumo units, -400..400)
    vmax: float = 200.0         # cruising speed
    vmin: float = 100.0         # speed when close to the corner
    kv: float = 1.5             # speed per mm of remaining distance
    kw: float = 170.0           # steering command per rad of heading error
    kt: float = 200.0           # in-place turn command per rad of heading error
    turn_min: float = 80.0
    turn_max: float = 140.0
    min_pwm: float = 80.0       # motors do not move below this
    max_cmd: float = 300.0
    lookahead: float = 70.0     # mm
    arrive_tol: float = 15.0    # mm
    turn_tol_deg: float = 4.0
    spin_thresh_deg: float = 50.0
    settle_s: float = 0.35
    lost_stop_s: float = 0.25
    seg_timeout: float = 40.0
    turn_timeout: float = 20.0
    cal_speed: float = 120.0
    cal_spin: float = 120.0
    cal_dist: float = 60.0
    cal_timeout: float = 4.0
    view_scale: float = 0.0     # display scale of the video window, 0 = fit the screen


# ----------------------------------------------------------------------------- robot link
class RobotLink:
    """WebSocket client for the Zumo. Telemetry is read in a background thread."""

    def __init__(self, host, port, enabled=True):
        self.enabled = enabled
        self.telemetry = {}
        self._last = (None, None)
        self._last_t = 0.0
        if not enabled:
            return
        self._open = threading.Event()
        self.ws = websocket.WebSocketApp(
            f"ws://{host}:{port}",
            on_open=lambda ws: self._open.set(),
            on_message=self._on_msg,
            on_close=lambda ws, *a: self._open.clear(),
            on_error=lambda ws, e: print("WebSocket error:", e))
        threading.Thread(target=self.ws.run_forever, daemon=True).start()
        if not self._open.wait(6):
            raise RuntimeError(f"cannot connect to ws://{host}:{port}")

    def _on_msg(self, ws, msg):
        try:
            m = json.loads(msg)
            if m.get("type") == "telemetry":
                self.telemetry = m
        except ValueError:
            pass

    def _send(self, obj):
        if self.enabled:
            try:
                self.ws.send(json.dumps(obj))
            except Exception as e:  # noqa
                print("send failed:", e)

    def drive(self, left, right):
        left, right = int(clamp(left, -400, 400)), int(clamp(right, -400, 400))
        now = time.time()
        changed = (left, right) != self._last
        if (changed and now - self._last_t >= 0.04) or now - self._last_t >= 0.15:
            self._send({"cmd": "drive", "left": left, "right": right})
            self._last, self._last_t = (left, right), now

    def stop(self):
        self._send({"cmd": "stop"})
        self._last, self._last_t = (0, 0), time.time()

    def close(self):
        self.stop()
        if self.enabled:
            self.ws.close()


# ----------------------------------------------------------------------------- camera + detector
class Camera:
    """Reads frames in a thread so the controller always works on the newest image."""

    def __init__(self, src, width, height):
        src = int(src) if str(src).isdigit() else src
        backend = cv2.CAP_DSHOW if (
            os.name == "nt" and isinstance(src, int)) else cv2.CAP_ANY
        self.cap = cv2.VideoCapture(src, backend)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open camera {src}")
        if backend == cv2.CAP_DSHOW:     # uncompressed YUY2 limits many USB cameras to a few fps at high resolution
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        print(f"camera: {int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}, "
              f"driver reports {self.cap.get(cv2.CAP_PROP_FPS):.0f} fps")
        self.fps = 0.0                   # measured frames per second delivered by the camera
        self._frame, self._n, self._seen = None, 0, 0
        self._lock = threading.Lock()
        self._run = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        count, t0 = 0, time.time()
        while self._run:
            ok, f = self.cap.read()
            if ok:
                with self._lock:
                    self._frame, self._n = f, self._n + 1
                count += 1
                now = time.time()
                if now - t0 >= 1.0:
                    self.fps, count, t0 = count / (now - t0), 0, now
            else:
                time.sleep(0.01)

    def read(self, timeout=2.0):
        t = time.time() + timeout
        while time.time() < t:
            with self._lock:
                if self._frame is not None and self._n != self._seen:
                    self._seen = self._n
                    return self._frame.copy()
            time.sleep(0.002)
        raise RuntimeError("camera gives no frames")

    def release(self):
        self._run = False
        time.sleep(0.05)
        self.cap.release()


class Vision:
    def __init__(self, dict_name, marker_id):
        self.id = marker_id
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        if hasattr(cv2.aruco, "ArucoDetector"):
            p = cv2.aruco.DetectorParameters()
            p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            det = cv2.aruco.ArucoDetector(d, p)
            self._detect = lambda g: det.detectMarkers(g)
        else:
            p = cv2.aruco.DetectorParameters_create()
            p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self._detect = lambda g: cv2.aruco.detectMarkers(
                g, d, parameters=p)

    def find_all(self, frame):
        """{id: corners (4x2, pixels)} for every marker in the image."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(gray)
        out = {}
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                out[int(i)] = c.reshape(4, 2).astype(np.float64)
        return out

    def find(self, frame):
        """Corners (4x2, pixels) of the robot marker, or None."""
        return self.find_all(frame).get(self.id)


# ----------------------------------------------------------------------------- visual servoing
class Servo:
    def __init__(self, cfg, cam, vision, robot, calib=None):
        self.cfg, self.cam, self.vision, self.robot = cfg, cam, vision, robot
        self.calib = calib
        self.offset = 0.0 if cfg.offset_deg is None else math.radians(
            cfg.offset_deg)
        self.turn_sign = cfg.turn_sign or 1
        self.scale = None                 # mm per pixel
        self.waypoints = None
        self.trail = []
        self.last_key = -1
        self.last_found = {}              # all markers of the latest frame
        self._window = False
        # optional callable(frame) for extra drawing (full-res frame)
        self.overlay = None
        # display scale (window size / camera image size)
        self.view = None
        # multiply line widths, radii, font sizes by this when drawing
        self.ui = 1.0
        # processing loop rate (frames handled per second)
        self.fps = 0.0
        self.detect_ms = 0.0              # marker detection time per frame
        self._t_prev = None
        self.last_seen = self.now()
        self.frame_size = (cfg.width, cfg.height)

    def now(self):
        return time.time()

    # ---- perception
    def step(self, info=""):
        """Grab a frame, detect the marker, draw the overlay, handle keys. Returns Pose or None."""
        frame = self.cam.read()
        tn = time.perf_counter()
        if self._t_prev is not None and tn > self._t_prev:
            inst = 1.0 / (tn - self._t_prev)
            self.fps = inst if self.fps == 0 else 0.9 * self.fps + 0.1 * inst
        self._t_prev = tn
        if self.calib:
            frame = cv2.undistort(frame, *self.calib)
        self.frame_size = (frame.shape[1], frame.shape[0])
        if self.view is None:
            w, h = self.frame_size
            if self.cfg.view_scale > 0:
                self.view = min(1.0, self.cfg.view_scale)
            else:
                sw, sh = screen_size()
                # leave room for title bar / taskbar
                self.view = min(1.0, 0.95 * sw / w, 0.9 * sh / h)
            self.ui = 1.0 / self.view
        t_det = time.perf_counter()
        self.last_found = self.vision.find_all(frame)
        self.detect_ms = 0.9 * self.detect_ms + \
            0.1 * (time.perf_counter() - t_det) * 1000
        corners = self.last_found.get(self.cfg.marker_id)
        pose = None
        if corners is not None:
            c = corners.mean(axis=0)
            side = float(
                np.mean([np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)]))
            ang = math.atan2(*(corners[1] - corners[0])[::-1])
            s = self.scale or self.cfg.marker_mm / side
            pose = Pose(c[0] * s, c[1] * s, ang, wrap(ang + self.offset), side)
            self.last_seen = self.now()
            self.trail.append((int(c[0]), int(c[1])))
            self.trail = self.trail[-1500:]
        elif self.now() - self.last_seen > self.cfg.lost_stop_s:
            self.robot.stop()
            info = "MARKER LOST - stopped"
        self._draw(frame, corners, pose, info)
        key = cv2.waitKey(1) & 0xFF
        self.last_key = key
        if key in (27, ord("q")):
            raise Abort()
        return pose

    def _draw(self, frame, corners, pose, info):
        ui = self.ui
        # thickness / radius that looks the same after downscaling
        def t(n): return max(1, int(round(n * ui)))
        if corners is not None:
            cv2.polylines(frame, [corners.astype(int)],
                          True, (0, 255, 0), t(2))
        if self.scale and self.waypoints is not None:
            pts = (np.array(self.waypoints) / self.scale).astype(int)
            cv2.polylines(frame, [pts.reshape(-1, 1, 2)],
                          False, (255, 160, 0), t(2))
            for p in pts:
                cv2.circle(frame, tuple(int(v)
                           for v in p), t(6), (0, 0, 255), -1)
        if len(self.trail) > 1:
            cv2.polylines(frame, [np.array(
                self.trail, np.int32).reshape(-1, 1, 2)], False, (0, 220, 255), t(1))
        if pose and self.scale:
            c = np.array([pose.x, pose.y]) / self.scale
            e = c + 70 * ui * \
                np.array([math.cos(pose.theta), math.sin(pose.theta)])
            cv2.arrowedLine(frame, tuple(int(v) for v in c), tuple(
                int(v) for v in e), (255, 0, 255), t(2))
        if self.overlay:
            self.overlay(frame)
        cv2.putText(frame, info, (t(10), t(28)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7 * ui, (0, 255, 255), t(2))
        stats = (f"camera {getattr(self.cam, 'fps', 0.0):.1f} fps | loop {self.fps:.1f} fps | "
                 f"detect {self.detect_ms:.0f} ms | {frame.shape[1]}x{frame.shape[0]}")
        cv2.putText(frame, stats, (t(10), t(58)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6 * ui, (0, 255, 255), t(2))
        if self.view < 1.0:
            frame = cv2.resize(frame, None, fx=self.view,
                               fy=self.view, interpolation=cv2.INTER_AREA)
        if not self._window:
            cv2.namedWindow("zumo servo", cv2.WINDOW_AUTOSIZE)
            cv2.moveWindow("zumo servo", 0, 0)
            self._window = True
        cv2.imshow("zumo servo", frame)

    def set_mouse(self, cb):
        """Register cb(event, x, y, flags, param); x, y are given in camera-image pixels, not window pixels."""
        def wrapper(ev, x, y, flags, param):
            cb(ev, int(x * self.ui), int(y * self.ui), flags, param)
        cv2.setMouseCallback("zumo servo", wrapper)

    def wait(self, secs, info="waiting"):
        t = self.now() + secs
        while self.now() < t:
            self.step(info)

    def avg_pose(self, n=8, timeout=4.0):
        poses, t = [], self.now() + timeout
        while len(poses) < n and self.now() < t:
            p = self.step("measuring")
            if p:
                poses.append(p)
        if len(poses) < max(3, n // 2):
            raise RuntimeError("marker not visible")
        return Pose(np.mean([p.x for p in poses]), np.mean([p.y for p in poses]),
                    circ_mean([p.ang for p in poses]), circ_mean(
                        [p.theta for p in poses]),
                    np.mean([p.side for p in poses]))

    # ---- actuation
    def set_wheels(self, v, u):
        """v: forward command, u: turn command (positive = turn clockwise as seen by the camera)."""
        c = self.cfg
        u *= self.turn_sign
        l, r = v + u, v - u
        m = max(abs(l), abs(r))
        if 0 < m < c.min_pwm:
            l, r = l * c.min_pwm / m, r * c.min_pwm / m
        elif m > c.max_cmd:
            l, r = l * c.max_cmd / m, r * c.max_cmd / m
        self.robot.drive(round(l), round(r))

    def spin(self, alpha):
        c = self.cfg
        mag = clamp(c.kt * abs(alpha), c.turn_min, c.turn_max)
        self.set_wheels(0.0, mag if alpha > 0 else -mag)

    # ---- calibration
    def estimate_scale(self, n=30):
        sides = []
        while len(sides) < n:
            p = self.step("measuring marker size")
            if p:
                sides.append(p.side)
        self.scale = self.cfg.marker_mm / float(np.median(sides))
        print(f"scale: {self.scale:.3f} mm/pixel")

    def calibrate_heading(self):
        c = self.cfg
        print("calibration 1/2: spin test")
        a0 = self.avg_pose().ang
        self.robot.drive(c.cal_spin, -c.cal_spin)
        self.wait(0.6, "calibrating: spin")
        self.robot.stop()
        self.wait(0.6, "calibrating: settle")
        d = wrap(self.avg_pose().ang - a0)
        if abs(d) < math.radians(5):
            raise RuntimeError(
                "spin test: the marker did not rotate (battery? wires? marker visible?)")
        if self.cfg.turn_sign == 0:
            self.turn_sign = 1 if d > 0 else -1
        print(f"turn sign: {self.turn_sign:+d}")

        print("calibration 2/2: forward test")
        p0, angs = self.avg_pose(), []
        t_end = self.now() + c.cal_timeout
        while self.now() < t_end:
            p = self.step("calibrating: forward")
            if p is None:
                continue
            angs.append(p.ang)
            if math.hypot(p.x - p0.x, p.y - p0.y) >= c.cal_dist:
                break
            self.robot.drive(c.cal_speed, c.cal_speed)
        self.robot.stop()
        self.wait(0.6, "calibrating: settle")
        p1 = self.avg_pose()
        if math.hypot(p1.x - p0.x, p1.y - p0.y) < 15:
            raise RuntimeError("forward test: the robot did not move")
        travel = math.atan2(p1.y - p0.y, p1.x - p0.x)
        self.offset = wrap(travel - circ_mean(angs))
        print(f"heading offset: {math.degrees(self.offset):.1f} deg")

    # ---- motion primitives
    def turn_to(self, target, label):
        tol = math.radians(self.cfg.turn_tol_deg)
        state, t_settle = "turn", 0.0
        t_limit = self.now() + self.cfg.turn_timeout
        while True:
            if self.now() > t_limit:
                raise RuntimeError("turn timeout")
            pose = self.step(f"{label}: turning")
            if pose is None:
                continue
            a = wrap(target - pose.theta)
            if state == "turn":
                if abs(a) < tol:
                    self.robot.stop()
                    state, t_settle = "settle", self.now() + self.cfg.settle_s
                else:
                    self.spin(a)
            elif self.now() >= t_settle:
                if abs(a) < 2 * tol:
                    return
                state = "turn"

    def drive_segment(self, A, B, label):
        c = self.cfg
        A, B = np.array(A, float), np.array(B, float)
        L = float(np.linalg.norm(B - A))
        d = (B - A) / L
        t_limit = self.now() + c.seg_timeout
        while True:
            if self.now() > t_limit:
                raise RuntimeError("segment timeout")
            pose = self.step(f"{label}: driving")
            if pose is None:
                continue
            P = np.array([pose.x, pose.y])
            along = float((P - A) @ d)
            dist = float(np.linalg.norm(B - P))
            if dist < c.arrive_tol or along >= L:
                self.robot.stop()
                self.wait(c.settle_s, f"{label}: arrived")
                return
            # carrot on the edge
            Q = A + d * clamp(along + c.lookahead, 0.0, L)
            a = wrap(math.atan2(Q[1] - P[1], Q[0] - P[0]) - pose.theta)
            if abs(a) > math.radians(c.spin_thresh_deg):
                # heading lost: re-align in place
                self.spin(a)
                continue
            v = clamp(c.kv * dist, c.vmin, c.vmax) * math.cos(a)
            self.set_wheels(v, c.kw * a)

    def prepare(self, return_to_start=True):
        """Preview, wait for S, measure the scale, calibrate, then drive back to where the robot was."""
        c = self.cfg
        while True:
            pose = self.step("S = start, Q = quit")
            if self.last_key == ord("s") and pose:
                break
        self.estimate_scale()
        if c.offset_deg is None:
            start = self.avg_pose()                      # pose before the calibration moves
            self.calibrate_heading()
            if return_to_start:
                # its heading, now that the offset is known
                theta = wrap(start.ang + self.offset)
                err = self.goto((start.x, start.y), theta, "back to start")
                self.waypoints = None
                print(f"back at the start position (error {err:.1f} mm)")
        else:
            print(
                f"using heading offset {c.offset_deg} deg, turn sign {self.turn_sign:+d}")

    def goto(self, target, theta=None, label="goto"):
        """Drive to target (mm, same frame as Pose.x/y). If theta (rad) is given, finish with that heading."""
        p = self.avg_pose(4)
        A, B = np.array([p.x, p.y]), np.array(target, float)
        self.waypoints = [tuple(A), tuple(B)]
        if np.linalg.norm(B - A) > 2 * self.cfg.arrive_tol:
            self.turn_to(math.atan2(B[1] - A[1], B[0] - A[0]), label)
            self.drive_segment(A, B, label)
        if theta is not None:
            self.turn_to(theta, label)
        pf = self.avg_pose(4)
        # final position error, mm
        return math.hypot(pf.x - B[0], pf.y - B[1])

    def scan_markers(self, n=25, min_frac=0.5):
        """Average all non-robot markers over n frames. Returns {id: (center_px (2,), side_px)}."""
        acc = {}
        for _ in range(n):
            self.step("scanning markers")
            for i, cr in self.last_found.items():
                if i != self.cfg.marker_id:
                    side = float(
                        np.mean([np.linalg.norm(cr[k] - cr[(k + 1) % 4]) for k in range(4)]))
                    acc.setdefault(i, []).append((cr.mean(axis=0), side))
        return {i: (np.mean([a for a, _ in v], axis=0), float(np.mean([b for _, b in v])))
                for i, v in acc.items() if len(v) >= min_frac * n}

    # ---- mission
    def square_path(self, p0):
        """Headings of the four edges and the five waypoints (start ... start) of the square."""
        c = self.cfg
        step_ang = -math.pi / 2 if c.ccw else math.pi / 2
        psis = [p0.theta + i * step_ang for i in range(4)]
        W = [np.array([p0.x, p0.y])]
        for psi in psis:
            W.append(W[-1] + c.side_mm *
                     np.array([math.cos(psi), math.sin(psi)]))
        # close the square exactly
        W[-1] = W[0].copy()
        return psis, W

    def fits(self, W, margin=40):
        w_px, h_px = self.frame_size
        return all(margin <= x <= w_px - margin and margin <= y <= h_px - margin
                   for x, y in (w / self.scale for w in W))

    def wait_until_square_fits(self):
        """The square does not fit: the robot is moved by hand. Shows the square for the robot's
        current pose live and returns when it fits and S is pressed."""
        print("The square does not fit in the camera image. Move the robot by hand, then press S.")
        info = "square does not fit - move the robot by hand"
        while True:
            pose = self.step(info)
            if pose is None:
                info = "marker not visible"
                continue
            _, W = self.square_path(pose)
            ok = self.fits(W)
            self.waypoints = [tuple(w) for w in W]
            info = "square fits - press S to start" if ok else "square does not fit - move the robot by hand"
            if ok and self.last_key == ord("s"):
                return

    def run(self):
        self.prepare()
        while True:
            p0 = self.avg_pose()
            psis, W = self.square_path(p0)
            if self.fits(W):
                break
            self.wait_until_square_fits()
        self.waypoints = [tuple(w) for w in W]

        for i, psi in enumerate(psis):
            self.turn_to(psi, f"edge {i + 1}/4")
            self.drive_segment(W[i], W[i + 1], f"edge {i + 1}/4")
        self.turn_to(p0.theta, "final heading")
        pf = self.avg_pose()
        err = math.hypot(pf.x - p0.x, pf.y - p0.y)
        print(f"done: end position error {err:.1f} mm, heading error "
              f"{math.degrees(wrap(pf.theta - p0.theta)):.1f} deg")
        self.wait(1.5, f"DONE - error {err:.1f} mm")


# ----------------------------------------------------------------------------- shared CLI / setup
def common_parser(desc):
    ap = argparse.ArgumentParser(description=desc)
    ap.add_argument("--host", default=Cfg.host, help="robot IP or hostname")
    ap.add_argument("--port", type=int, default=Cfg.port)
    ap.add_argument("--cam", default=Cfg.cam,
                    help="camera index or stream URL")
    ap.add_argument("--width", type=int, default=Cfg.width)
    ap.add_argument("--height", type=int, default=Cfg.height)
    ap.add_argument("--dict", dest="dict_name", default=Cfg.dict_name,
                    help="ArUco dictionary, e.g. DICT_4X4_50, DICT_5X5_100, DICT_6X6_250, DICT_ARUCO_ORIGINAL")
    ap.add_argument("--id", dest="marker_id", type=int,
                    default=Cfg.marker_id, help="ID of the marker on the robot")
    ap.add_argument("--marker-mm", type=float, default=Cfg.marker_mm,
                    help="outer side of the robot marker's black square")
    ap.add_argument("--calib", default="",
                    help="npz file with camera_matrix and dist_coeffs (optional)")
    ap.add_argument("--no-robot", action="store_true",
                    help="vision only, never command the robot")
    ap.add_argument("--offset-deg", type=float, default=None,
                    help="skip calibration: marker-to-robot heading offset")
    ap.add_argument("--turn-sign", type=int, default=0,
                    choices=[-1, 0, 1], help="skip calibration of turn sign")
    ap.add_argument("--vmax", type=float, default=Cfg.vmax)
    ap.add_argument("--vmin", type=float, default=Cfg.vmin)
    ap.add_argument("--kw", type=float, default=Cfg.kw)
    ap.add_argument("--lookahead", type=float, default=Cfg.lookahead)
    ap.add_argument("--view-scale", type=float, default=Cfg.view_scale,
                    help="display scale of the video window (0 = fit the screen automatically)")
    return ap


def build_servo(args, servo_cls=Servo):
    cfg = Cfg(**{k: v for k, v in vars(args).items()
              if k in Cfg.__dataclass_fields__})
    calib = None
    if cfg.calib:
        z = np.load(cfg.calib)
        calib = (z["camera_matrix"], z["dist_coeffs"])
    cam = Camera(cfg.cam, cfg.width, cfg.height)
    try:
        robot = RobotLink(cfg.host, cfg.port, enabled=not cfg.no_robot)
    except Exception:
        cam.release()
        raise
    if robot.enabled:
        time.sleep(0.4)
        mv = robot.telemetry.get("battery_mv")
        print("connected, battery:", f"{mv / 1000:.2f} V" if mv else "n/a")
    return servo_cls(cfg, cam, Vision(cfg.dict_name, cfg.marker_id), robot, calib)


def shutdown(servo):
    servo.robot.close()
    servo.cam.release()
    cv2.destroyAllWindows()


# ----------------------------------------------------------------------------- main
def main():
    ap = common_parser("Zumo square path by visual servoing")
    ap.add_argument("--side", dest="side_mm", type=float,
                    default=Cfg.side_mm, help="square side, mm")
    ap.add_argument("--ccw", action="store_true",
                    help="turn left instead of right")
    servo = build_servo(ap.parse_args())
    try:
        servo.run()
    except Abort:
        print("aborted")
    except RuntimeError as e:
        print("ERROR:", e)
    finally:
        shutdown(servo)


if __name__ == "__main__":
    main()
