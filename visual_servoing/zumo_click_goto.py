#!/usr/bin/env python3
"""
Zumo robot: drive to a position and orientation chosen with the mouse (visual servoing).

Needs zumo_square_servo.py in the same folder (camera, ArUco tracking, WebSocket, controllers).

  python zumo_click_goto.py --host 192.168.1.50 --cam 0

In the video window
  S                   start (after the marker is visible): scale + heading calibration, then ready
  left click          target position
  second click        orientation: the robot will face from the target toward this point
  (or press at the target and drag to the orientation point, release)
  G                   go to the clicked position only, without a final orientation
  right click / C     clear the pending target
  Space               cancel the current move (robot stops, program keeps running)
  Q / ESC             quit (robot is stopped)
"""
import math

import cv2
import numpy as np

import zumo_square_servo as zs


class Cancel(Exception):
    pass


class ClickServo(zs.Servo):
    busy = False

    def step(self, info=""):
        pose = super().step(info)
        if self.busy and self.last_key == ord(" "):
            self.robot.stop()
            raise Cancel()
        return pose


def main():
    ap = zs.common_parser(
        "Click to drive the Zumo to a position and orientation")
    servo = zs.build_servo(ap.parse_args(), ClickServo)

    tgt = {"pos": None, "head": None, "down_set": False, "cursor": None}
    # result of the last move, shown in the window
    last = {"pos": None, "label": "", "text": ""}
    MIN_PX = 12

    def far(p, x, y):
        return math.hypot(x - p[0], y - p[1]) > MIN_PX * servo.ui

    def on_mouse(ev, x, y, flags, _):
        if servo.busy:
            return
        # live end of the orientation arrow
        tgt["cursor"] = (x, y)
        if ev == cv2.EVENT_RBUTTONDOWN:
            tgt.update(pos=None, head=None, down_set=False)
        elif ev == cv2.EVENT_LBUTTONDOWN:
            if tgt["pos"] is None or tgt["head"] is not None:
                tgt.update(pos=(x, y), head=None, down_set=True)
            elif far(tgt["pos"], x, y):
                tgt["head"] = (x, y)
        elif ev == cv2.EVENT_LBUTTONUP:
            if tgt["down_set"] and tgt["pos"] and tgt["head"] is None and far(tgt["pos"], x, y):
                tgt["head"] = (x, y)                       # press-drag-release
            tgt["down_set"] = False

    def overlay(frame):
        ui = servo.ui
        def t(n): return max(1, int(round(n * ui)))
        font, green, orange = cv2.FONT_HERSHEY_SIMPLEX, (
            0, 255, 0), (0, 165, 255)
        if tgt["pos"]:
            cv2.circle(frame, tgt["pos"], t(9), (0, 0, 255), t(2))
            if tgt["head"]:
                cv2.arrowedLine(
                    frame, tgt["pos"], tgt["head"], (0, 0, 255), t(2), tipLength=0.2)
            elif tgt["cursor"] and far(tgt["pos"], *tgt["cursor"]):
                # preview while dragging / moving the mouse after the first click
                cv2.arrowedLine(
                    frame, tgt["pos"], tgt["cursor"], orange, t(2), tipLength=0.2)
            if servo.busy and servo.trail:                       # live distance to the target while moving
                rx, ry = servo.trail[-1]
                dist = math.hypot(tgt["pos"][0] - rx,
                                  tgt["pos"][1] - ry) * servo.scale
                cv2.line(frame, (rx, ry), tgt["pos"], orange, t(1))
                cv2.putText(frame, f"{dist:.0f} mm", (tgt["pos"][0] + t(14), tgt["pos"][1] - t(14)),
                            font, 0.7 * ui, orange, t(2))
                cv2.putText(frame, f"distance to target: {dist:.0f} mm", (t(10), t(88)),
                            font, 0.7 * ui, orange, t(2))
        # final error next to the last target
        elif last["pos"]:
            p = last["pos"]
            cv2.drawMarker(frame, p, green, cv2.MARKER_CROSS, t(18), t(2))
            cv2.putText(frame, last["label"], (p[0] + t(14),
                        p[1] - t(14)), font, 0.7 * ui, green, t(2))
        if last["text"] and not (servo.busy and tgt["pos"]):
            cv2.putText(frame, last["text"], (t(10), t(
                88)), font, 0.7 * ui, green, t(2))

    servo.overlay = overlay
    try:
        servo.prepare()
        servo.set_mouse(on_mouse)
        print("ready: click a target position, then click the orientation")
        while True:
            servo.step(
                "click target, click/drag orientation | G = go (any heading) | C = clear | Q = quit")
            pos, head = tgt["pos"], tgt["head"]
            pos_only = servo.last_key == ord(
                "g") and pos is not None and head is None
            if pos is not None and (head is not None or pos_only):
                theta = None if pos_only else math.atan2(
                    head[1] - pos[1], head[0] - pos[0])
                servo.busy = True
                try:
                    err = servo.goto(
                        np.array(pos) * servo.scale, theta, "goto")
                    msg = f"position error {err:.1f} mm"
                    if theta is not None:
                        herr = math.degrees(
                            zs.wrap(servo.avg_pose(4).theta - theta))
                        msg += f", heading error {herr:.1f} deg"
                    print("arrived:", msg)
                    last.update(
                        pos=pos, label=f"error {err:.1f} mm", text="last move: " + msg)
                except Cancel:
                    print("move cancelled")
                    last.update(pos=None, label="",
                                text="last move: cancelled")
                except RuntimeError as e:
                    print("ERROR:", e)
                    servo.robot.stop()
                    last.update(pos=None, label="",
                                text=f"last move failed: {e}")
                finally:
                    servo.busy = False
                    servo.waypoints = None
                    tgt.update(pos=None, head=None, down_set=False)
            elif servo.last_key == ord("c"):
                tgt.update(pos=None, head=None, down_set=False)
    except zs.Abort:
        print("quit")
    except RuntimeError as e:
        print("ERROR:", e)
    finally:
        zs.shutdown(servo)


if __name__ == "__main__":
    main()
