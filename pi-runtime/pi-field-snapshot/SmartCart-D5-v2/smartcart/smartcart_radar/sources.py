"""ROS, explicit synthetic demo, and recorded-file producers."""
from __future__ import annotations
import math
import os
import time
from collections.abc import Callable
from .core import ScanStore, normalise_scan
from .scan_io import read_frames


def synthetic_scan(t: float) -> dict:
    """Raycast an invented room and obstacles. This is NEVER hardware data."""
    values = []
    n = 720
    circles = [(0.8, 0.45, 0.16), (1.5 + 0.5 * math.sin(t * 0.7), -1.1 + 0.3 * math.cos(t * 0.5), 0.24),
               (-1.9, 1.7, 0.32), (2.7, 2.4, 0.38)]
    # The room, an interior box and a short wall; dimensions are synthetic.
    segments = [((-3.5, -4.2), (4.3, -4.2)), ((4.3, -4.2), (4.3, 3.8)),
                ((4.3, 3.8), (-3.5, 3.8)), ((-3.5, 3.8), (-3.5, -4.2)),
                ((0.9, 2.0), (1.7, 2.0)), ((1.7, 2.0), (1.7, 2.8)),
                ((1.7, 2.8), (0.9, 2.8)), ((0.9, 2.8), (0.9, 2.0)),
                ((-1.8, -2.4), (-0.3, -2.4))]
    for i in range(n):
        angle = -math.pi + i * 2 * math.pi / (n - 1)
        dx, dy = math.cos(angle), math.sin(angle)
        hit = 16.0
        for (ax, ay), (bx, by) in segments:
            ex, ey = bx - ax, by - ay
            den = dx * ey - dy * ex
            if abs(den) < 1e-9:
                continue
            distance = (ax * ey - ay * ex) / den
            u = (ax * dy - ay * dx) / den
            if distance > 0 and 0 <= u <= 1:
                hit = min(hit, distance)
        for cx, cy, r in circles:
            b = dx * cx + dy * cy
            disc = b * b - (cx * cx + cy * cy - r * r)
            if disc >= 0 and b > 0:
                distance = b - math.sqrt(disc)
                if distance > 0:
                    hit = min(hit, distance)
        # Deterministic, visibly limited perturbation; no borrowed point cloud.
        values.append(None if i % 113 == 0 else hit + 0.006 * math.sin(i * 11.3 + t * 4))
    now = time.time()
    return normalise_scan({
        "header": {"frame_id": "synthetic_laser", "stamp": {"sec": int(now), "nanosec": int(now % 1 * 1e9)}},
        "angle_min": -math.pi, "angle_max": math.pi, "angle_increment": 2 * math.pi / (n - 1),
        "range_min": 0.05, "range_max": 16.0, "scan_time": 0.1,
        "time_increment": 0.1 / (n - 1), "ranges": values,
    }, kind="synthetic", label="SYNTHETIC DEMO — not sensor measurements")


def ros_loop(store: ScanStore, topic: str = "/scan", on_scan: Callable | None = None) -> None:
    """Use a private ROS context, without owning the LiDAR serial device."""
    context = node = executor = None
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import LaserScan

        context = Context()
        rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
        node = Node(f"smartcart_radar_reader_{os.getpid()}", context=context, enable_rosout=False, start_parameter_services=False)
        # A best-effort subscriber can consume best-effort or reliable sensor publishers.
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)

        def callback(message):
            try:
                data = {
                    "header": {"frame_id": message.header.frame_id,
                               "stamp": {"sec": message.header.stamp.sec, "nanosec": message.header.stamp.nanosec}},
                    "angle_min": message.angle_min, "angle_max": message.angle_max,
                    "angle_increment": message.angle_increment, "scan_time": message.scan_time,
                    "time_increment": message.time_increment, "range_min": message.range_min,
                    "range_max": message.range_max, "ranges": list(message.ranges),
                    "intensities": list(message.intensities),
                }
                scan = normalise_scan(data, kind="ros", label=topic)
                store.publish(scan)
                if on_scan:
                    on_scan(scan)
            except (ValueError, TypeError, OSError) as exc:
                store.set_error(f"Rejected scan: {exc}", rejected=True)

        subscription = node.create_subscription(LaserScan, topic, callback, qos)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        while not store.stop.is_set() and context.ok():
            executor.spin_once(timeout_sec=0.2)
        del subscription
    except Exception as exc:
        store.set_error(f"ROS source failed: {type(exc).__name__}: {exc}")
    finally:
        if executor:
            executor.shutdown(timeout_sec=1.0)
        if node:
            node.destroy_node()
        if context and context.ok():
            context.shutdown()


def source_loop(store: ScanStore, *, topic="/scan", replay_file=None, loop=False, replay_hz=None) -> None:
    try:
        if store.source_mode == "ros":
            ros_loop(store, topic)
            return
        if store.source_mode == "demo":
            start = time.monotonic()
            deadline = start
            while not store.stop.is_set():
                store.publish(synthetic_scan(time.monotonic() - start))
                deadline += 0.1
                store.stop.wait(max(0, deadline - time.monotonic()))
            return
        while not store.stop.is_set():
            count = 0
            deadline = time.monotonic()
            for scan in read_frames(replay_file):
                if store.stop.is_set():
                    return
                store.publish(scan)
                count += 1
                interval = 1 / replay_hz if replay_hz else (scan["scan_time"] or 0.1)
                deadline += max(0.01, min(5.0, interval))
                store.stop.wait(max(0, deadline - time.monotonic()))
            if not count:
                raise ValueError("Replay file contains no scans")
            if not loop:
                store.finish()
                return
    except Exception as exc:
        store.set_error(f"Source failed: {type(exc).__name__}: {exc}")
