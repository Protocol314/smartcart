"""ASCII radar client. Reads the same HTTP snapshot as the browser."""
from __future__ import annotations
import json
import math
import os
import shutil
import sys
import time
from urllib.request import urlopen
from urllib.error import URLError
from .core import project_scan


def render_ascii(envelope: dict, *, radius=5.0, yaw=0.0, x=0.0, y=0.0, columns=90, rows=22) -> str:
    state = envelope.get("state", "unknown").upper()
    scan = envelope.get("scan")
    width = max(25, min(columns - 2, 100))
    height = max(9, min(rows - 9, 31))
    if height % 2 == 0:
        height -= 1
    width = min(width, height * 2 + 1)
    grid = [[" " for _ in range(width)] for _ in range(height)]
    cx, cy = (width - 1) / 2, (height - 1) / 2
    scale_x, scale_y = cx / radius, cy / radius
    for degrees in range(360):
        a = math.radians(degrees)
        col, row = round(cx + math.cos(a) * cx), round(cy + math.sin(a) * cy)
        grid[row][col] = "."
    projection = None
    active = scan and state not in ("WAITING", "STALE", "ERROR", "DISCONNECTED")
    if active:
        projection = project_scan(scan, radius, yaw, x, y)
        for point in projection["points"]:
            col = round(cx - point["y"] * scale_x)
            row = round(cy - point["x"] * scale_y)
            if 0 <= row < height and 0 <= col < width:
                grid[row][col] = "*"
    grid[round(cy)][round(cx)] = "^"
    source = scan.get("source", {}).get("kind", "unknown") if scan else "none"
    age = envelope.get("age_ms")
    lines = [f"SMARTCART RADAR | {state} | SOURCE={source} | READ ONLY",
             f"Cart front=UP, y=LEFT | radius={radius:g}m | rx={envelope.get('receive_hz', 0):.1f}Hz | age={age}ms",
             " " * max(0, round(cx) - 3) + "FRONT"]
    lines += [" " + "".join(row) for row in grid]
    if projection:
        distances = []
        for name, value in projection["sectors"].items():
            minimum = value["min_m"]
            distances.append(f"{name}={minimum:.3f}m" if minimum is not None else f"{name}=UNKNOWN")
        lines.append(" | ".join(distances))
        lines.append(f"shown={len(projection['points'])}, rays={len(scan['ranges'])}/{scan['stats']['expected_rays']} | {' '.join(scan['warnings']) or 'No format warnings'}")
    else:
        lines.append("NO FRESH SCAN. A blank view is not proof of free space.")
    if source != "ros":
        lines.append("NOT LIVE HARDWARE DATA. Demo/replay is explicitly labeled.")
    if envelope.get("error"):
        lines.append(str(envelope["error"]))
    lines.append("Ctrl+C: quit viewer only; LiDAR driver keeps running.")
    return "\n".join(lines)


def run_cli(url: str, hz: float, *, radius=5.0, yaw=0.0, x=0.0, y=0.0, once=False):
    endpoint = url.rstrip("/") + "/api/frame"
    tty = sys.stdout.isatty()
    if os.name == "nt":
        os.system("")  # Enable ANSI processing in current Windows terminals.
    try:
        if tty:
            sys.stdout.write("\033[?25l")
        while True:
            begin = time.monotonic()
            try:
                with urlopen(endpoint, timeout=3) as response:
                    raw = response.read(8 * 1024 * 1024)
                envelope = json.loads(raw)
            except (URLError, TimeoutError, OSError, ValueError) as exc:
                envelope = {"state": "disconnected", "error": str(exc)}
            size = shutil.get_terminal_size((100, 36))
            frame = render_ascii(envelope, radius=radius, yaw=yaw, x=x, y=y, columns=size.columns, rows=size.lines)
            if tty:
                sys.stdout.write("\033[H\033[2J")
            print(frame, flush=True)
            if once:
                return
            time.sleep(max(0, 1 / hz - (time.monotonic() - begin)))
    except KeyboardInterrupt:
        pass
    finally:
        if tty:
            sys.stdout.write("\033[?25h\n")
