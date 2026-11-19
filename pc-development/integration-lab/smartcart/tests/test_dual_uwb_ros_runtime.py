"""선택 ROS 런타임에서 합성 입력만 격리 DDS 도메인으로 전송한다."""
from __future__ import annotations

import json
import os
from pathlib import Path
import struct
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from smartcart_uwb.native import default_library_candidates

ROS_AVAILABLE = False
try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from smartcart_uwb_interfaces.msg import UwbObservation
    ROS_AVAILABLE = any(path.is_file() for path in default_library_candidates())
except ImportError:
    pass

ROOT = Path(__file__).resolve().parents[1]


def synthetic_frame(sequence=1):
    payload = struct.pack("<BHiIHii3h", sequence, 0x6E19, -21, 57, 0, -91, -83, 0, 0, 0)
    checksum = 0
    for byte in payload:
        checksum ^= byte
    return b"\x2a\x1b" + payload + bytes((checksum, 0x23))


@unittest.skipUnless(ROS_AVAILABLE, "실제 ROS 2·생성된 관측 메시지·네이티브 라이브러리가 필요합니다")
class DualUwbRosRuntimeTests(unittest.TestCase):
    def test_production_adapter_uses_own_context_and_real_dds_with_fake_usb(self):
        from smartcart_uwb import ros_adapter
        from smartcart_uwb.serial_reader import SerialReaders

        class FakePort:
            def __init__(self):
                self.sequence = 0
                self.closed = False
            def read(self, maximum):
                time.sleep(.03)
                self.sequence = (self.sequence + 1) & 255
                return synthetic_frame(self.sequence)
            def close(self):
                self.closed = True

        payload = json.loads((ROOT / "config/dual_uwb.d5.example.json").read_text(encoding="utf-8"))
        for index, sensor in enumerate(("uwb_left", "uwb_right"), 21):
            payload["sensors"][sensor].update(port=f"COM{index}", expected_device_identity=sensor,
                                              device_identity_verified=True)
        ports = {"COM21": FakePort(), "COM22": FakePort()}
        readers_created, outcomes, errors, messages = [], [], [], []

        def readers_factory(collector, *, observation_callback):
            reader = SerialReaders(collector, observation_callback=observation_callback,
                                   identity_reader=lambda port: "uwb_left" if port == "COM21" else "uwb_right",
                                   port_opener=ports.__getitem__)
            readers_created.append(reader)
            return reader

        def run_adapter(config):
            try:
                outcomes.append(ros_adapter.main(["--config", str(config)]))
            except Exception as error:
                errors.append(error)

        with tempfile.TemporaryDirectory(dir=ROOT / "tests") as directory, patch.dict(
                os.environ, {"ROS_DOMAIN_ID": "229", "ROS_LOCALHOST_ONLY": "1"}):
            config = Path(directory) / "devices.json"
            config.write_text(json.dumps(payload), encoding="utf-8")
            context = rclpy.context.Context()
            rclpy.init(args=[], context=context)
            node = rclpy.create_node("dual_uwb_test_receiver", context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
            subscriptions = [node.create_subscription(UwbObservation, topic, messages.append, qos)
                             for topic in ros_adapter.OBSERVATION_TOPICS.values()]
            worker = threading.Thread(target=run_adapter, args=(config,), daemon=True)
            try:
                with patch.object(ros_adapter, "SerialReaders", readers_factory):
                    worker.start()
                    deadline = time.monotonic() + 8
                    while time.monotonic() < deadline and not errors:
                        executor.spin_once(timeout_sec=.05)
                        if {message.sensor_id for message in messages} == {"uwb_left", "uwb_right"}:
                            break
                    self.assertEqual(errors, [])
                    self.assertEqual({message.sensor_id for message in messages}, {"uwb_left", "uwb_right"})
                    self.assertTrue(all(message.valid and not message.drive_authorized for message in messages))
                    self.assertTrue(all(message.timestamp_source == "HOST_RX" for message in messages))
                    readers_created[0].stop.set()
                    worker.join(timeout=3)
                    self.assertFalse(worker.is_alive())
                    self.assertEqual(outcomes, [0])
                    self.assertTrue(all(port.closed for port in ports.values()))
            finally:
                for reader in readers_created:
                    reader.stop.set()
                worker.join(timeout=3)
                executor.shutdown(timeout_sec=2)
                node.destroy_node()
                rclpy.shutdown(context=context)
                self.assertTrue(subscriptions)


if __name__ == "__main__":
    unittest.main()
