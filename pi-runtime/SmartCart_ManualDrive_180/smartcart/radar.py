#!/usr/bin/env python3
"""SmartCart Radar entry point. Python 3.10+; ROS packages only for ROS input."""
from __future__ import annotations
import argparse
import importlib.util
import json
import math
import queue
import sys
import threading
import time
from pathlib import Path

from smartcart_radar import __version__
from smartcart_radar.core import ScanStore
from smartcart_radar.scan_io import read_frames, write_scan
from smartcart_radar.server import RadarHTTPServer
from smartcart_radar.sources import source_loop, ros_loop
from smartcart_radar.terminal import run_cli


def finite_float(text):
    try:
        value = float(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Expected a number") from exc
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError("Expected a finite number")
    return value


def need_ros():
    if importlib.util.find_spec("rclpy") is None or importlib.util.find_spec("sensor_msgs") is None:
        raise RuntimeError("ROS 2 Python packages unavailable. On the Pi run:\n"
                           "  source /opt/ros/humble/setup.bash\n"
                           "  source ~/smartcart_ws/install/setup.bash\n"
                           "Use /usr/bin/python3 (not a virtualenv/Conda Python).\n"
                           "Offline only: python3 radar.py serve --mode demo")


def serve(args):
    if args.mode == "ros":
        need_ros()
    if not 1 <= args.port <= 65535:
        raise ValueError("port must be between 1 and 65535")
    if args.mode == "replay":
        if not args.file:
            raise ValueError("--mode replay requires --file")
        next(read_frames(args.file))  # Validate before opening a socket.
    if args.replay_hz is not None and not 0.2 <= args.replay_hz <= 100:
        raise ValueError("--replay-hz must be between 0.2 and 100")
    store = ScanStore(args.mode)
    server = RadarHTTPServer((args.host, args.port), store)
    source = threading.Thread(target=source_loop, kwargs={"store": store, "topic": args.topic,
                              "replay_file": args.file, "loop": args.loop, "replay_hz": args.replay_hz}, daemon=True)
    source.start()
    print(f"SmartCart Radar {__version__} | READ ONLY | source={args.mode}", flush=True)
    print(f"Listening: http://{args.host}:{args.port}", flush=True)
    if args.host == "0.0.0.0":
        print(f"Browser: http://<PI_IP>:{args.port} — trusted LAN only; no Internet port forwarding.", flush=True)
    if args.mode == "ros":
        print(f"Subscribing to {args.topic}; keep the existing sllidar_ros2 driver running.", flush=True)
    elif args.mode == "demo":
        print("SYNTHETIC DATA — NOT CONNECTED TO A LIDAR.", flush=True)
    print("Ctrl+C stops this viewer server, NOT the LiDAR motor or cart motors.", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        store.stop.set()
        server.server_close()
        source.join(timeout=3)
        print("Viewer server stopped.", flush=True)


def record(args):
    need_ros()
    if args.seconds <= 0:
        raise ValueError("--seconds must be positive")
    target = Path(args.output)
    target.parent.mkdir(parents=True, exist_ok=True)
    # Never overwrite an earlier experiment accidentally.
    handle = target.open("x", encoding="utf-8")
    store = ScanStore("ros")
    pending = queue.Queue(maxsize=128)
    dropped = 0
    def enqueue(scan):
        nonlocal dropped
        try:
            pending.put_nowait(scan)
        except queue.Full:
            dropped += 1
    worker = threading.Thread(target=ros_loop, args=(store, args.topic, enqueue), daemon=True)
    worker.start()
    deadline = time.monotonic() + args.seconds
    count = 0
    print(f"Recording FULL incoming scans, without the 5m display crop: {target}")
    interrupted = False
    try:
        with handle:
            while time.monotonic() < deadline and not interrupted:
                try:
                    scan = pending.get(timeout=0.2)
                    write_scan(handle, scan)
                    count += 1
                except queue.Empty:
                    if store.snapshot()["state"] == "error":
                        raise RuntimeError(store.snapshot()["error"])
                except KeyboardInterrupt:
                    interrupted = True
            store.stop.set()
            worker.join(timeout=3)
            while not pending.empty():
                write_scan(handle, pending.get_nowait())
                count += 1
    finally:
        store.stop.set()
        worker.join(timeout=3)
    print(f"Saved {count} complete messages; recorder queue drops={dropped}.")
    if not count:
        print("WARNING: No scans received. File is empty; check /scan and ROS_DOMAIN_ID.", file=sys.stderr)
        return 1
    return int(dropped > 0)


def main(argv=None):
    parser = argparse.ArgumentParser(description="SmartCart read-only LaserScan radar")
    parser.add_argument("--version", action="version", version=__version__)
    commands = parser.add_subparsers(dest="command", required=True)
    server = commands.add_parser("serve", help="Run ROS/demo/replay HTTP server")
    server.add_argument("--mode", choices=("ros", "demo", "replay"), default="ros")
    server.add_argument("--host", default="127.0.0.1", help="Use 0.0.0.0 only for a trusted LAN")
    server.add_argument("--port", type=int, default=8765)
    server.add_argument("--topic", default="/scan")
    server.add_argument("--file")
    server.add_argument("--loop", action="store_true")
    server.add_argument("--replay-hz", type=finite_float)
    cli = commands.add_parser("cli", help="ASCII radar from an already running server")
    cli.add_argument("--url", default="http://127.0.0.1:8765")
    cli.add_argument("--hz", type=int, choices=(1, 2), default=2)
    cli.add_argument("--radius", type=finite_float, default=5)
    cli.add_argument("--yaw", type=finite_float, default=0)
    cli.add_argument("--x", type=finite_float, default=0)
    cli.add_argument("--y", type=finite_float, default=0)
    cli.add_argument("--once", action="store_true")
    inspect = commands.add_parser("inspect", help="Validate and summarize the first scan")
    inspect.add_argument("file")
    convert = commands.add_parser("convert", help="Convert one ROS block-YAML scan or JSONL into JSON/JSONL")
    convert.add_argument("file")
    convert.add_argument("--output", required=True)
    rec = commands.add_parser("record", help="Record full ROS /scan messages to JSONL")
    rec.add_argument("--topic", default="/scan")
    rec.add_argument("--seconds", type=finite_float, default=30)
    rec.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "serve":
            serve(args)
        elif args.command == "cli":
            if not 0.1 <= args.radius <= 100:
                raise ValueError("radius must be between 0.1 and 100 m")
            run_cli(args.url, args.hz, radius=args.radius, yaw=args.yaw, x=args.x, y=args.y, once=args.once)
        elif args.command == "inspect":
            scan = next(read_frames(args.file))
            print(json.dumps({key: value for key, value in scan.items() if key not in ("ranges", "intensities")}, ensure_ascii=False, indent=2))
        elif args.command == "convert":
            frames = read_frames(args.file)
            target = Path(args.output)
            target.parent.mkdir(parents=True, exist_ok=True)
            count = 0
            with target.open("x", encoding="utf-8") as handle:
                for scan in frames:
                    write_scan(handle, scan)
                    count += 1
            print(f"Converted {count} scan(s): {target}; missing data was NOT filled.")
        elif args.command == "record":
            return record(args)
        return 0
    except (ValueError, RuntimeError, OSError, StopIteration) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
