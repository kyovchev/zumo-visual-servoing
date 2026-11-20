#!/usr/bin/env python3
"""
Zumo robot: drive to the positions of ArUco markers lying on the floor, in the order of their IDs.

Needs zumo_square_servo.py in the same folder (camera, ArUco tracking, WebSocket, controllers).

  python zumo_marker_route.py --host 192.168.1.50 --cam 0
  python zumo_marker_route.py --ids 0,1,2,3 --target-mm 80 --loop

  - Floor markers use the same dictionary as the robot's marker (--dict). The robot's own ID (--id, 5)
    is never a target.
  - All markers are read once at the start, while the robot stands still, so it is fine if the robot
    later covers them. Do not move them during the run.
  - Floor markers are lower than the one on the robot, so they look smaller and their pixel positions
    are shifted by perspective. The script corrects this using each marker's apparent size
    (--target-mm must be the real outer side of the floor markers). If the floor markers are in the
    same plane as the robot's marker, or the camera is far away, --no-perspective gives the same result.

Keys: S = start (calibration), G = go after the route is shown, Q / ESC = quit (robot is stopped).
"""
import cv2
import numpy as np

import zumo_square_servo as zs


def main():
    ap = zs.common_parser("Drive the Zumo through floor markers in the order of their IDs")
    ap.add_argument("--target-mm", type=float, default=50.0, help="outer side of the floor markers, mm")
    ap.add_argument("--ids", default="", help="comma-separated IDs to visit (default: all found)")
    ap.add_argument("--loop", action="store_true", help="repeat the route until Q is pressed")
    ap.add_argument("--no-perspective", action="store_true", help="skip the marker-height perspective correction")
    a = ap.parse_args()
    servo = zs.build_servo(a)

    try:
        servo.prepare()
        found = servo.scan_markers()
        wanted = {int(x) for x in a.ids.split(",") if x.strip()} if a.ids else None
        ids = sorted(i for i in found if wanted is None or i in wanted)
        if wanted and wanted - set(ids):
            raise RuntimeError(f"markers not found in the image: {sorted(wanted - set(ids))}")
        if not ids:
            raise RuntimeError("no floor markers found (check --dict, lighting, marker sizes)")

        w, h = servo.frame_size
        c0, s_r = np.array([w / 2.0, h / 2.0]), servo.scale
        route = []
        for i in ids:
            p_px, side = found[i]
            s_f = a.target_mm / side                                  # mm per pixel at the floor marker
            P = p_px * s_r if a.no_perspective else c0 * s_r + (p_px - c0) * s_f
            route.append((i, P, p_px))
        print("route:", " -> ".join(f"id {i} ({P[0]:.0f}, {P[1]:.0f}) mm" for i, P, _ in route))

        def overlay(frame):
            for n, (i, P, p_px) in enumerate(route, 1):
                q = tuple(int(v) for v in p_px)
                cv2.circle(frame, q, 14, (255, 255, 255), 2)
                cv2.putText(frame, f"{n}: id {i}", (q[0] + 16, q[1] - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                t = (P / s_r).astype(int)                             # where the robot's marker must be
                cv2.drawMarker(frame, (int(t[0]), int(t[1])), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)

        servo.overlay = overlay
        while True:
            servo.step("route shown - G = go, Q = quit")
            if servo.last_key == ord("g"):
                break

        while True:
            for n, (i, P, _) in enumerate(route, 1):
                err = servo.goto(P, None, f"id {i} ({n}/{len(route)})")
                print(f"reached id {i}: position error {err:.1f} mm")
            if not a.loop:
                break
        servo.waypoints = None
        servo.wait(1.5, "DONE")
    except zs.Abort:
        print("aborted")
    except RuntimeError as e:
        print("ERROR:", e)
    finally:
        zs.shutdown(servo)


if __name__ == "__main__":
    main()
