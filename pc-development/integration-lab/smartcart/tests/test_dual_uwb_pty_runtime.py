"""실제 Linux PTY 두 개와 pyserial·C 파서·ROS DDS를 연결하는 합성 통합 검사.

실물 USB 식별 대신 시험이 만든 PTY의 정확한 경로만 식별자로 대응시킨다.
프로덕션 포트 개방·읽기·파서·수집기·ROS 어댑터는 그대로 사용한다. 물리
장치·RF·제어선·Pi 성능을 검증하지 않으며 가상 장치로 송신하지 않는다.
"""
from collections import deque
import json
import math
import os
from pathlib import Path
import struct
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from uuid import uuid4

from smartcart_uwb.native import default_library_candidates

ROOT = Path(__file__).resolve().parents[1]
SENSORS = ("uwb_left", "uwb_right")
RUNTIME_AVAILABLE = False
RUNTIME_REASON = "Linux PTY가 필요합니다"
if sys.platform.startswith("linux"):
    try:
        import pty
        import select
        import termios
        import serial
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from rclpy.signals import SignalHandlerOptions
        from smartcart_uwb_interfaces.msg import UwbObservation
        from std_msgs.msg import String
        RUNTIME_REASON = "명시적으로 빌드한 Linux UWB 공유 라이브러리가 필요합니다"
        RUNTIME_AVAILABLE = any(path.is_file() for path in default_library_candidates())
    except ImportError:
        RUNTIME_REASON = "Linux pyserial·rclpy·생성된 UwbObservation 메시지가 필요합니다"


def binary_frame(sequence, *, tag_address=0x6E19, checksum_bad=False):
    """가상 센서가 송신하는 원본 31바이트 형태이며 실제 측정값이 아니다."""
    payload = struct.pack("<BHiIHii3h", sequence, tag_address, -21, 57, 0, -91, -83, 0, 0, 0)
    checksum = 0
    for value in payload:
        checksum ^= value
    if checksum_bad:
        checksum ^= 1
    return b"\x2a\x1b" + payload + bytes((checksum, 0x23))


@unittest.skipUnless(RUNTIME_AVAILABLE, RUNTIME_REASON)
class DualUwbPtyRuntimeTests(unittest.TestCase):
    def setUp(self):
        from smartcart_uwb import ros_adapter
        from smartcart_uwb.serial_reader import SerialReaders, canonical_port, open_readonly_port

        self.deadline = time.monotonic() + 25
        self.context = self.node = self.executor = self.worker = self.adapter_patch = None
        self.terminals = {}
        self.readers, self.outcomes, self.errors = [], [], []
        self.messages = {sensor_id: deque(maxlen=256) for sensor_id in SENSORS}
        self.statuses = deque(maxlen=256)
        self.sequences = {sensor_id: 0 for sensor_id in SENSORS}
        self.last_pump = 0.0
        self.temporary = tempfile.TemporaryDirectory(prefix="smartcart-dual-pty-")
        self.addCleanup(self.temporary.cleanup)
        self.environment = patch.dict(os.environ, {"ROS_DOMAIN_ID": "94", "ROS_LOCALHOST_ONLY": "1"})
        self.environment.start()
        self.addCleanup(self.environment.stop)
        self.addCleanup(self.cleanup_runtime)

        token = uuid4().hex[:12]
        payload = json.loads((ROOT / "config" / "dual_uwb.d5.example.json").read_text(encoding="utf-8"))
        identities = {}
        for sensor_id in SENSORS:
            master, slave = pty.openpty()
            os.set_blocking(master, False)
            port = os.ttyname(slave)
            self.terminals[sensor_id] = {"master": master, "slave": slave, "port": port}
            identity = "test-pty:" + sensor_id + ":" + token
            identities[canonical_port(port)] = identity
            payload["sensors"][sensor_id].update(port=port, expected_device_identity=identity,
                                                   device_identity_verified=True)
        self.config_path = Path(self.temporary.name) / "devices.json"
        self.config_path.write_text(json.dumps(payload), encoding="utf-8")

        def identify_test_pty(port):
            resolved = canonical_port(port)
            if resolved not in identities:
                raise ValueError("시험이 생성한 PTY 외의 장치는 허용하지 않습니다")
            return identities[resolved]

        def readers_factory(collector, *, observation_callback):
            reader = SerialReaders(collector, observation_callback=observation_callback,
                                   identity_reader=identify_test_pty, port_opener=open_readonly_port)
            self.readers.append(reader)
            return reader

        def run_adapter():
            try:
                self.outcomes.append(ros_adapter.main(["--config", str(self.config_path)]))
            except BaseException as error:
                # 위치·태그·원시 입력을 실패 메시지에 복사하지 않는다.
                self.errors.append(type(error).__name__)

        self.context = Context()
        rclpy.init(args=[], context=self.context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node("smartcart_dual_pty_observer_" + token, context=self.context,
                         enable_rosout=False, start_parameter_services=False)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.subscriptions = [self.node.create_subscription(
            UwbObservation, ros_adapter.OBSERVATION_TOPICS[sensor_id],
            lambda message, selected=sensor_id: self.messages[selected].append(message), qos)
            for sensor_id in SENSORS]
        self.subscriptions.append(self.node.create_subscription(
            String, ros_adapter.STATUS_TOPIC, lambda message: self.statuses.append(json.loads(message.data)), qos))
        self.adapter_patch = patch.object(ros_adapter, "SerialReaders", readers_factory)
        self.adapter_patch.start()
        self.worker = threading.Thread(target=run_adapter, name="dual-uwb-pty-adapter", daemon=True)
        self.worker.start()
        self.wait_until(lambda: bool(self.statuses) and all(
            self.statuses[-1]["sensors"][sensor_id]["connected"] for sensor_id in SENSORS),
            "좌우 PTY 개방과 실제 DDS 상태 발견")
        self.wait_until(lambda: all(not termios.tcgetattr(item["slave"])[3] &
                                   (termios.ECHO | termios.ICANON) for item in self.terminals.values()),
                        "실제 pyserial의 raw 모드 설정")
        self.assert_no_output()

    def cleanup_runtime(self):
        for reader in self.readers:
            reader.stop.set()
        if self.worker is not None and self.worker.ident is not None:
            self.worker.join(timeout=4)
        cleanup_errors = []
        for operation in (
            lambda: self.executor.shutdown(timeout_sec=2) if self.executor is not None else None,
            lambda: self.node.destroy_node() if self.node is not None else None,
            lambda: self.context.shutdown() if self.context is not None and self.context.ok() else None,
            lambda: self.adapter_patch.stop() if self.adapter_patch is not None else None,
        ):
            try:
                operation()
            except Exception:
                cleanup_errors.append("ros_cleanup_failed")
        for terminal in self.terminals.values():
            for key in ("master", "slave"):
                descriptor = terminal.get(key)
                if descriptor is not None:
                    try:
                        os.close(descriptor)
                    except OSError:
                        cleanup_errors.append("pty_close_failed")
                    terminal[key] = None
        self.assertFalse(self.worker is not None and self.worker.is_alive(), "ROS 어댑터가 종료되지 않았습니다")
        self.assertEqual(cleanup_errors, [])

    def wait_until(self, predicate, description, *, pump=None):
        while time.monotonic() < self.deadline:
            if pump is not None:
                pump()
            self.executor.spin_once(timeout_sec=.02)
            self.assertEqual(self.errors, [], "실제 ROS 어댑터에서 예외가 발생했습니다")
            if predicate():
                return
            if self.worker is not None and not self.worker.is_alive():
                self.fail("실제 ROS 어댑터가 관측 전에 종료되었습니다: " + description)
        self.fail("실제 PTY/DDS 대기 상한을 초과했습니다: " + description)

    def spin_for(self, seconds, *, pump=None):
        deadline = min(self.deadline, time.monotonic() + seconds)
        while time.monotonic() < deadline:
            if pump is not None:
                pump()
            self.executor.spin_once(timeout_sec=.02)
            self.assertEqual(self.errors, [])
        self.assertLess(time.monotonic(), self.deadline, "PTY/DDS 시험 전체 시간 상한을 초과했습니다")

    def send(self, sensor_id, data):
        master = self.terminals[sensor_id]["master"]
        offset = 0
        while offset < len(data):
            self.assertLess(time.monotonic(), self.deadline, "PTY 입력 전송 상한을 초과했습니다")
            if select.select([], [master], [], .02)[1]:
                try:
                    offset += os.write(master, data[offset:offset + 1024])
                except BlockingIOError:
                    pass

    def frame(self, sensor_id, **kwargs):
        self.sequences[sensor_id] = (self.sequences[sensor_id] + 1) & 255
        return binary_frame(self.sequences[sensor_id], **kwargs)

    def pump(self, *, left=True, checksum_bad=False, tag_address=0x6E19):
        now = time.monotonic()
        if now - self.last_pump < .03:
            return
        self.last_pump = now
        if left:
            self.send("uwb_left", self.frame("uwb_left", checksum_bad=checksum_bad, tag_address=tag_address))
        self.send("uwb_right", self.frame("uwb_right"))

    def sensor(self, sensor_id):
        return self.statuses[-1]["sensors"][sensor_id]

    def assert_no_output(self, *, sensors=SENSORS):
        for sensor_id in sensors:
            master = self.terminals[sensor_id]["master"]
            readable = select.select([master], [], [], .02)[0]
            output = os.read(master, 4096) if readable else b""
            self.assertEqual(output, b"", "읽기 전용 수신기가 가상 장치로 바이트를 보냈습니다")

    def wait_good_left(self):
        expected = self.sequences["uwb_left"]
        self.wait_until(lambda: bool(self.messages["uwb_left"]) and
                        self.messages["uwb_left"][-1].valid and
                        self.messages["uwb_left"][-1].device_sequence == expected and
                        self.sensor("uwb_left")["state"] == "live" and
                        self.sensor("uwb_left")["latest"]["device_sequence"] == expected and
                        self.sensor("uwb_right")["state"] == "live", "좌우 typed 관측·전체 상태 정상 수신",
                        pump=lambda: self.pump(left=False))

    def stop_and_check_resources(self, *, available=SENSORS):
        self.assertTrue(self.readers)
        for reader in self.readers:
            reader.stop.set()
        self.worker.join(timeout=4)
        self.assertFalse(self.worker.is_alive())
        self.assertEqual(self.errors, [])
        self.assertEqual(self.outcomes, [0])
        for reader in self.readers:
            self.assertTrue(all(not thread.is_alive() for thread in reader.threads))
            with self.assertRaises(RuntimeError):
                reader.collector.snapshot(time.monotonic_ns())
        # 실제 pyserial exclusive 잠금과 수신 파일 설명자의 반환을 확인한다.
        for sensor_id in available:
            with serial.Serial(self.terminals[sensor_id]["port"], baudrate=115200,
                               timeout=.05, exclusive=True) as probe:
                self.assertTrue(probe.is_open)

    def test_real_dual_pty_fragment_checksum_id_timeout_recovery_and_no_tx(self):
        split = self.frame("uwb_left")
        self.send("uwb_left", split[:12])
        self.spin_for(.10, pump=lambda: self.pump(left=False))
        self.assertFalse(self.messages["uwb_left"], "불완전한 31바이트를 관측으로 발행했습니다")
        self.send("uwb_left", split[12:])
        self.wait_good_left()
        typed = self.messages["uwb_left"][-1]
        self.assertEqual(typed.source_mode, "live")
        self.assertEqual(typed.timestamp_source, "HOST_RX")
        self.assertFalse(typed.device_timestamp_available)
        self.assertFalse(typed.drive_authorized)
        self.assertTrue(typed.frame_available)
        self.assertAlmostEqual(typed.range_m, .57, places=6)
        self.assertAlmostEqual(typed.bearing_rad, math.radians(21), places=6)
        self.assertEqual(self.statuses[-1]["schema"], "smartcart.dual_uwb_status.v1")
        self.assertFalse(self.statuses[-1]["drive_authorized"])
        self.assert_no_output()

        self.wait_until(lambda: self.sensor("uwb_left")["state"] == "error" and
                        self.sensor("uwb_left")["latest"]["invalid_reason"] == "bad_frame:checksum" and
                        self.messages["uwb_left"][-1].invalid_reason == "bad_frame:checksum" and
                        self.sensor("uwb_right")["state"] == "live", "불량 체크섬의 typed·상태 전달과 좌우 독립",
                        pump=lambda: self.pump(checksum_bad=True))
        self.assertFalse(self.messages["uwb_left"][-1].frame_available)
        self.assertFalse(self.messages["uwb_left"][-1].valid)

        self.send("uwb_left", self.frame("uwb_left"))
        self.wait_good_left()
        self.wait_until(lambda: self.sensor("uwb_left")["state"] == "error" and
                        self.sensor("uwb_left")["latest"]["invalid_reason"] == "id_mismatch" and
                        self.messages["uwb_left"][-1].invalid_reason == "id_mismatch" and
                        self.sensor("uwb_right")["state"] == "live", "태그 불일치의 typed·전체 상태 전달",
                        pump=lambda: self.pump(tag_address=0x1234))
        self.assertTrue(self.messages["uwb_left"][-1].frame_available)
        self.assertFalse(self.messages["uwb_left"][-1].id_valid)
        self.assertEqual(self.messages["uwb_left"][-1].logical_target_id, -1)

        self.send("uwb_left", self.frame("uwb_left"))
        self.wait_good_left()
        self.spin_for(.12, pump=lambda: self.pump(left=False))
        deadline = self.sensor("uwb_left")["latest"]["valid_until_monotonic_ns"]
        self.wait_until(lambda: self.sensor("uwb_left")["state"] == "stale" and
                        self.sensor("uwb_right")["state"] == "live", "한쪽 무입력의 원래 수명 만료",
                        pump=lambda: self.pump(left=False))
        self.assertFalse(self.sensor("uwb_left")["latest"]["valid"])
        self.assertEqual(self.sensor("uwb_left")["latest"]["valid_until_monotonic_ns"], deadline)
        self.send("uwb_left", self.frame("uwb_left"))
        self.wait_good_left()
        self.assert_no_output()
        self.stop_and_check_resources()

    def test_real_pty_disconnect_reaches_full_status_and_other_sensor_remains_live(self):
        self.send("uwb_left", self.frame("uwb_left"))
        self.wait_good_left()
        self.assert_no_output()
        os.close(self.terminals["uwb_left"]["master"])
        self.terminals["uwb_left"]["master"] = None
        self.wait_until(lambda: not self.sensor("uwb_left")["connected"] and
                        self.sensor("uwb_left")["state"] == "error" and
                        self.sensor("uwb_right")["state"] == "live", "실제 PTY 단절의 전체 상태와 좌우 독립",
                        pump=lambda: self.pump(left=False))
        self.assertFalse(self.sensor("uwb_left")["latest"]["valid"])
        self.assertFalse(self.sensor("uwb_left")["latest"]["drive_authorized"])
        self.assert_no_output(sensors=("uwb_right",))
        self.stop_and_check_resources(available=("uwb_right",))


if __name__ == "__main__":
    unittest.main()
