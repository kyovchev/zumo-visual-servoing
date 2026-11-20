#!/usr/bin/env python3
"""
REST client for the Zumo robot (Wemos D1 mini firmware).

    pip install requests

    python zumo_rest.py status                  # print everything
    python zumo_rest.py demo                    # beep, blink LEDs, drive a little
    python zumo_rest.py --host 192.168.1.50 demo

The UNO stops the motors if no drive command arrives for 500 ms, so a
continuous movement must be re-sent about 10 times a second (see drive_for).
"""
import argparse
import json
import time

import requests


class Zumo:
    def __init__(self, host="zumo.local", timeout=2.0):
        self.base = f"http://{host}"
        self.timeout = timeout
        self.http = requests.Session()

    # ---- helpers ----
    def _post(self, path, **body):
        r = self.http.post(
            self.base + path, json=body or None, timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def _get(self, path):
        r = self.http.get(self.base + path, timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    # ---- commands ----
    def drive(self, left, right):
        """left/right: -400..400"""
        return self._post("/api/drive", left=int(left), right=int(right))

    def stop(self):
        return self._post("/api/stop")

    def led(self, led_id, on):
        """led_id 0 = pin 13, 1 = A4"""
        return self._post("/api/led", id=int(led_id), on=bool(on))

    def buzzer(self, freq, ms=200):
        """freq in Hz, 0 = silence"""
        return self._post("/api/buzzer", freq=int(freq), ms=int(ms))

    def calibrate(self):
        """Robot spins in place for ~2 s; put it over the line first."""
        return self._post("/api/calibrate")

    # ---- readings ----
    def status(self):
        return self._get("/api/status")

    def sensors(self):
        return self._get("/api/sensors")

    def battery(self):
        return self._get("/api/battery")

    def button(self):
        return self._get("/api/button")

    def leds(self):
        return self._get("/api/leds")

    def wifi(self):
        return self._get("/api/wifi")

    # ---- convenience ----
    def drive_for(self, left, right, seconds, hz=10):
        """Drive for a fixed time, re-sending the command to satisfy the 500 ms failsafe."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.drive(left, right)
            time.sleep(1.0 / hz)
        self.stop()


def demo(z):
    print("battery:", z.battery())
    time.sleep(3)
    print("button :", z.button())

    # LEDs and buzzer
    print("led")
    z.led(0, True)
    time.sleep(5)
    z.led(0, False)
    print("buzzer")
    z.buzzer(880, 1000)
    z.buzzer(1320, 2000)
    time.sleep(2)

    # Movement (keep the robot on the floor with free space around it)
    print("forward")
    z.drive_for(150, 150, 3.0)
    time.sleep(3)
    print("backward")
    z.drive_for(-150, -150, 3.0)
    time.sleep(3)
    print("spin")
    z.drive_for(150, -150, 3.0)
    time.sleep(3)
    print("sensors:", z.sensors())


def main():
    p = argparse.ArgumentParser(description="Zumo REST client")
    p.add_argument("--host", default="zumo.local",
                   help="hostname or IP (default zumo.local)")
    p.add_argument("command", choices=[
                   "status", "demo", "stop"], nargs="?", default="status")
    args = p.parse_args()

    z = Zumo(args.host)
    try:
        if args.command == "status":
            print(json.dumps(z.status(), indent=2))
        elif args.command == "stop":
            print(z.stop())
        else:
            try:
                demo(z)
            finally:
                z.stop()  # always leave the robot stopped
    except requests.RequestException as e:
        print(f"Cannot talk to the robot at {args.host}: {e}")


if __name__ == "__main__":
    main()
