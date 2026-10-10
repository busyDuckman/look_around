"""Testbed A (animation, open loop): lookaround drives a mannequin in the browser.

Runs the Python reference implementation at 60 Hz and streams each tick to the viewer page
over a WebSocket. The page sends back preset/path choices and the dragged ball position.

    python testbed/animation/server.py          # then open the URL it prints
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import functools
import json
import logging
import math
import random
import threading
import webbrowser
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from websockets.asyncio.server import ServerConnection, broadcast, serve

import lookaround as la

REPO = Path(__file__).resolve().parents[2]
WEB = REPO / "web"
PRESETS = ["human_standing", "human_seated", "human_swivel"]
PATHS = ["orbit", "lissajous", "bounce", "teleport", "manual", "idle"]
RATE = 60.0

Vec3 = tuple[float, float, float]


class Scene:
    """One shared simulation: a looker and a ball."""

    def __init__(self) -> None:
        self.path = "orbit"
        self.speed = 1.0
        self.paused = False
        self.path_t = 0.0
        self.manual: Vec3 = (1.5, 0.3, 1.3)
        self.rng = random.Random(1)
        self.teleport: Vec3 = (1.5, 0.0, 1.3)
        self.next_teleport = 0.0
        self.set_preset("human_standing")

    def set_preset(self, name: str) -> None:
        if name not in PRESETS:
            raise ValueError(name)
        self.preset = name
        path = la.preset_path(name)
        self.doc: dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))
        self.looker = la.Looker(la.Settings.load(self.doc))

    def config(self) -> str:
        return json.dumps({
            "type": "config", "preset": self.preset, "presets": PRESETS,
            "path": self.path, "paths": PATHS, "speed": self.speed, "paused": self.paused,
            "settings": self.doc,
        })

    def target(self, dt: float) -> Vec3 | None:
        t = self.path_t
        if self.path == "idle":
            return None
        if self.path == "manual":
            return self.manual
        if self.path == "orbit":  # full circle every 18 s, from knee height to overhead
            a = 0.35 * t
            return (1.8 * math.cos(a), 1.8 * math.sin(a), 1.1 + 0.8 * math.sin(0.5 * t))
        if self.path == "lissajous":  # in front of the face
            return (1.2, 0.9 * math.sin(0.9 * t), 1.3 + 0.5 * math.sin(1.3 * t))
        if self.path == "bounce":  # bouncing while circling at 2 m
            period, h_max = 1.2, 1.6
            phase = (t % period) / period
            a = 0.5 * t
            return (2.0 * math.cos(a), 2.0 * math.sin(a), 0.1 + 4 * h_max * phase * (1 - phase))
        if self.path == "teleport":  # random jumps every 1.5 s
            if t >= self.next_teleport:
                az = math.radians(self.rng.uniform(-150, 150))
                r = self.rng.uniform(1.5, 3.0)
                self.teleport = (r * math.cos(az), r * math.sin(az), self.rng.uniform(0.3, 2.2))
                self.next_teleport = t + 1.5
            return self.teleport
        raise ValueError(self.path)

    def tick(self, dt: float) -> str | None:
        if self.paused:
            return None
        self.path_t += dt * self.speed
        target = self.target(dt)
        out = self.looker.update(dt, target)
        msg = {"type": "frame", "t": out.t, "target": target, "pose": out.pose,
               **out.to_dict()}
        return json.dumps(msg)

    def handle(self, msg: dict[str, Any]) -> bool:
        """Apply a client message; True if every client needs the new config."""
        kind = msg.get("type")
        if kind == "preset":
            self.set_preset(str(msg["name"]))
            return True
        if kind == "path":
            if msg["name"] not in PATHS:
                raise ValueError(msg["name"])
            self.path = str(msg["name"])
            self.next_teleport = self.path_t
            return True
        if kind == "target":
            p = msg["pos"]
            self.manual = (float(p[0]), float(p[1]), max(0.05, float(p[2])))
            if self.path != "manual":
                self.path = "manual"
                return True
            return False
        if kind == "speed":
            self.speed = max(0.0, min(3.0, float(msg["value"])))
            return False
        if kind == "pause":
            self.paused = bool(msg["value"])
            return True
        raise ValueError(f"unknown message {kind!r}")


async def main(http_port: int, ws_port: int, open_browser: bool) -> None:
    scene = Scene()
    clients: set[ServerConnection] = set()

    async def handler(ws: ServerConnection) -> None:
        clients.add(ws)
        await ws.send(scene.config())
        try:
            async for raw in ws:
                try:
                    if scene.handle(json.loads(raw)):
                        broadcast(clients, scene.config())
                except (ValueError, KeyError, TypeError) as e:
                    await ws.send(json.dumps({"type": "error", "message": str(e)}))
        finally:
            clients.discard(ws)

    handler_cls = functools.partial(QuietHandler, directory=str(WEB))
    httpd, http_port = _bind(lambda p: ThreadingHTTPServer(("127.0.0.1", p), handler_cls),
                             http_port)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()

    ws_server = None
    for port in range(ws_port, ws_port + PORT_TRIES):
        try:
            ws_server = await serve(handler, "127.0.0.1", port)
            ws_port = port
            break
        except OSError:
            continue
    if ws_server is None:
        raise SystemExit(f"no free WebSocket port in {ws_port}..{ws_port + PORT_TRIES - 1}")

    url = f"http://localhost:{http_port}/testbed/?ws={ws_port}"
    print(f"lookaround testbed A: {url}  (Ctrl+C to stop)", flush=True)
    if open_browser:
        webbrowser.open(url)

    async with ws_server:
        loop = asyncio.get_running_loop()
        dt = 1.0 / RATE
        next_t = loop.time()
        while True:
            msg = scene.tick(dt)
            if msg is not None and clients:
                broadcast(clients, msg)
            next_t += dt
            delay = next_t - loop.time()
            if delay < -0.25:  # fell far behind (e.g. machine asleep): resynchronise
                next_t = loop.time()
            await asyncio.sleep(max(0.0, delay))


PORT_TRIES = 20


def _bind(make: Any, port: int) -> tuple[Any, int]:
    """Bind to the first free port from `port` upwards."""
    for p in range(port, port + PORT_TRIES):
        try:
            return make(p), p
        except OSError:
            continue
    raise SystemExit(f"no free HTTP port in {port}..{port + PORT_TRIES - 1}")


class QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, format: str, *args: Any) -> None:  # noqa: A002
        pass

    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--http-port", type=int, default=8610)
    ap.add_argument("--ws-port", type=int, default=8620)
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--debug", action="store_true", help="print lookaround's diagnostic log")
    args = ap.parse_args()
    logging.basicConfig(format="%(asctime)s %(name)s %(levelname)s: %(message)s",
                        level=logging.WARNING)
    if args.debug:
        logging.getLogger("lookaround").setLevel(logging.DEBUG)
    with contextlib.suppress(KeyboardInterrupt):
        asyncio.run(main(args.http_port, args.ws_port, not args.no_browser))
