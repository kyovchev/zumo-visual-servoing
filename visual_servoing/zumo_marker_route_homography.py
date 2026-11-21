#!/usr/bin/env python3
"""
Zumo robot: drive to the positions of ArUco markers lying on the floor, in the order of their IDs.

Needs zumo_square_servo.py in the same folder (camera, ArUco tracking, WebSocket, controllers).

  python zumo_marker_route.py --host 192.168.1.50 --cam 0
  python zumo_marker_route.py --ids 0,1,2,3 --loop

  - Floor markers use the same dictionary as the robot's marker (--dict). The robot's own ID (--id)
    is never a target.
  - The camera may be tilted: every position is converted with the plane homography of the robot's marker
    (see zumo_square_servo.py). The target of a floor marker is the point of the robot's marker plane where
    the marker appears in the picture, so the floor markers should be about as high as the robot's marker
    (a robot marker only a little above the floor, or the targets on a board of the same height). A height
    difference shifts the target by roughly  height difference x tan(viewing angle).
  - The markers are read when the program starts and again whenever you press R, always while the
    robot stands still. Do not move them during a run. A marker that is covered by the robot cannot
    be seen: move the robot off the markers before rescanning.

Keys (video window)
  S      start: marker plane + heading calibration (once, at the beginning)
  G      go: drive through the route (again and again if you like)
  R      rescan the floor markers (after you moved them or the robot)
  Space  cancel the current run (robot stops, program keeps running)
  Q/ESC  quit (robot is stopped)
"""
import cv2
import numpy as np

import zumo_square_servo_homography as zs


class Cancel(Exception):
    pass


class RouteServo(zs.Servo):
    busy = False

    def step(self, info=""):
        pose = super().step(info)
        if self.busy and self.last_key == ord(" "):
            self.robot.stop()
            raise Cancel()
        return pose


def build_route(servo, a):
    """Scan the floor markers (robot must stand still). Returns [(id, target in mm on the marker plane, marker pixel)]."""
    found = servo.scan_markers()
    wanted = {int(x) for x in a.ids.split(",") if x.strip()} if a.ids else None
    ids = sorted(i for i in found if wanted is None or i in wanted)
    if wanted and wanted - set(ids):
        raise RuntimeError(
            f"markers not found in the image: {sorted(wanted - set(ids))}")
    if not ids:
        raise RuntimeError(
            "no floor markers found (check --dict, lighting, marker sizes)")
    return [(i, servo.px2mm([found[i][0]])[0], found[i][0]) for i in ids]


def main():
    ap = zs.common_parser(
        "Drive the Zumo through floor markers in the order of their IDs")
    ap.add_argument("--ids", default="",
                    help="comma-separated IDs to visit (default: all found)")
    ap.add_argument("--loop", action="store_true",
                    help="repeat the route until Space or Q is pressed")
    a = ap.parse_args()
    servo = zs.build_servo(a, RouteServo)

    state = {"route": [], "errs": {}}

    def overlay(frame):
        ui = servo.ui
        def t(n): return max(1, int(round(n * ui)))
        for n, (i, P, p_px) in enumerate(state["route"], 1):
            q = tuple(int(v) for v in p_px)
            label = f"{n}: id {i}"
            if i in state["errs"]:
                label += f"   error {state['errs'][i]:.0f} mm"
            cv2.circle(frame, q, t(14), (255, 255, 255), t(2))
            cv2.putText(frame, label, (q[0] + t(16), q[1] - t(12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6 * ui, (255, 255, 255), t(2))
            cv2.drawMarker(frame, q, (0, 0, 255), cv2.MARKER_CROSS, t(
                18), t(2))   # where the robot's marker must be

    servo.overlay = overlay
    try:
        servo.prepare()
        rescan, note = True, ""
        while True:
            if rescan:                                             # ---- scan the floor markers
                previous = {r[0] for r in state["route"]}
                state["errs"] = {}
                try:
                    route = build_route(servo, a)
                    state["route"] = route
                    print(
                        "route:", " -> ".join(f"id {i} ({P[0]:.0f}, {P[1]:.0f}) mm" for i, P, _ in route))
                    missing = sorted(previous - {r[0] for r in route})
                    note = ""
                    if missing:
                        note = f"WARNING: id {missing} from the previous scan not found (covered by the robot?)"
                    if note:
                        print(note)
                except RuntimeError as e:
                    state["route"], note = [], str(e)
                    print("ERROR:", e)
                rescan = False

            info = ("G = go, " if state["route"]
                    else "") + "R = rescan, Q = quit"
            if note:
                info += "   |   " + note
            while True:                                            # ---- show the route, wait for a key
                servo.step(info)
                if servo.last_key == ord("r"):
                    rescan = True
                    break
                if servo.last_key == ord("g") and state["route"]:
                    break
            if rescan:
                continue

            # ---- drive the route
            state["errs"] = {}
            servo.busy = True
            try:
                while True:
                    for n, (i, P, _) in enumerate(state["route"], 1):
                        err = servo.goto(
                            P, None, f"id {i} ({n}/{len(state['route'])})")
                        state["errs"][i] = err
                        print(f"reached id {i}: position error {err:.1f} mm")
                    if not a.loop:
                        break
                    state["errs"] = {}
                note = f"route done, largest error {max(state['errs'].values()):.0f} mm"
            except Cancel:
                note = "run cancelled"
                print(note)
            except RuntimeError as e:
                servo.robot.stop()
                note = f"run failed: {e}"
                print("ERROR:", e)
            finally:
                servo.busy = False
                servo.waypoints = None
    except zs.Abort:
        print("quit")
    except RuntimeError as e:
        print("ERROR:", e)
    finally:
        zs.shutdown(servo)


if __name__ == "__main__":
    main()
