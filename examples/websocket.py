#!/usr/bin/env python3
"""
WebSocket client for the Zumo robot (Wemos D1 mini firmware, port 81).

    pip install websockets

    python zumo_ws.py monitor               # live telemetry + button events
    python zumo_ws.py demo                  # beep, LEDs, drive a little
    python zumo_ws.py --host 192.168.1.50 demo

Notes
  * The server pushes telemetry about 10 times a second and button events immediately.
  * The UNO stops the motors if no drive command arrives for 500 ms, so movement
    commands are re-sent at 10 Hz (see drive_for).
  * The robot also stops whenever a WebSocket client disconnects.
"""
import argparse
import asyncio
import contextlib
import json
import time

import websockets


def fmt(t):
    r = t.get("reflectance", {})
    return (
        f"batt {t.get('battery_mv', 0) / 1000:.2f} V | "
        f"button {'DOWN' if t.get('button') else 'up'} | "
        f"leds {t.get('leds')} | "
        f"sensors {r.get('values')} | "
        f"rssi {t.get('rssi')} dBm"
    )


class Zumo:
    def __init__(self, ws):
        self.ws = ws
        self.telemetry = {}

    # ---- sending ----
    async def send(self, **msg):
        await self.ws.send(json.dumps(msg))

    async def drive(self, left, right):
        await self.send(cmd="drive", left=int(left), right=int(right))

    async def stop(self):
        await self.send(cmd="stop")

    async def led(self, led_id, on):
        await self.send(cmd="led", id=int(led_id), on=bool(on))

    async def buzzer(self, freq, ms=200):
        await self.send(cmd="buzzer", freq=int(freq), ms=int(ms))

    async def request_status(self):
        await self.send(cmd="status")

    async def drive_for(self, left, right, seconds, hz=10):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            await self.drive(left, right)
            await asyncio.sleep(1.0 / hz)
        await self.stop()

    # ---- receiving ----
    async def reader(self):
        """Runs in the background and keeps self.telemetry up to date."""
        async for raw in self.ws:
            msg = json.loads(raw)
            kind = msg.get("type")
            if kind == "telemetry":
                self.telemetry = msg
            elif kind == "event":
                state = "pressed" if msg.get("pressed") else "released"
                print(f"[event] {msg.get('name')} {state}")
            elif "error" in msg:
                print("[error]", msg["error"])
            # {"ok": true, "cmd": ...} acknowledgements are ignored

    async def wait_telemetry(self, timeout=3.0):
        end = time.monotonic() + timeout
        while not self.telemetry:
            if time.monotonic() > end:
                raise TimeoutError(
                    "no telemetry received (is the UNO connected?)")
            await asyncio.sleep(0.05)


# ---------------- modes ----------------
async def monitor(z):
    while True:
        print(fmt(z.telemetry))
        await asyncio.sleep(0.5)


async def demo(z):
    print(fmt(z.telemetry))

    await z.led(0, True)
    await asyncio.sleep(3)
    await z.led(0, False)
    await z.buzzer(880, 1000)
    await z.buzzer(1320, 2000)
    await asyncio.sleep(3)

    print("forward")
    await z.drive_for(150, 150, 3.0)
    await asyncio.sleep(3)
    print("backward")
    await z.drive_for(-150, -150, 3.0)
    await asyncio.sleep(3)
    print("spin")
    await z.drive_for(150, -150, 3.0)
    await asyncio.sleep(3)
    print(fmt(z.telemetry))


# ---------------- main ----------------
async def main(args):
    uri = f"ws://{args.host}:81"
    # ping_interval=None: the ESP does not need keep-alive pings
    async with websockets.connect(uri, ping_interval=None) as ws:
        z = Zumo(ws)
        reader = asyncio.create_task(z.reader())
        try:
            await z.wait_telemetry()
            if args.mode == "monitor":
                await monitor(z)
            elif args.mode == "demo":
                await demo(z)
        finally:
            with contextlib.suppress(Exception):
                await z.stop()  # always leave the robot stopped
            reader.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await reader


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Zumo WebSocket client")
    p.add_argument("--host", default="zumo.local",
                   help="hostname or IP (default zumo.local)")
    p.add_argument("mode", choices=[
                   "monitor", "demo", "follow"], nargs="?", default="monitor")
    p.add_argument("--seconds", type=float, default=15,
                   help="duration of follow mode")
    args = p.parse_args()
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        pass
    except (OSError, TimeoutError, websockets.exceptions.WebSocketException) as e:
        print(f"Cannot talk to the robot at {args.host}: {e}")
