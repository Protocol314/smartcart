"""실제 pyserial과 Linux PTY를 사용하는 시리얼 통합 검증.

PTY가 만드는 가상 입력만 사용하며 ESP32·BU03·BU04에는 연결하지 않는다.
운영체제가 지원하는 PTY 데이터 경로를 검사하므로 실물 DTR/RTS·USB 리셋이나
전기 신호 검증을 대체하지 않는다. Windows와 pyserial 미설치 환경에서는 건너뛴다.
"""
from __future__ import annotations

import os
import sys
import threading
import time
import unittest

from smartcart_sensing.uwb import MAX_LINE_BYTES, UwbStore, serial_loop

SERIAL_RUNTIME_AVAILABLE = False
if sys.platform.startswith("linux"):
    try:
        import pty
        import select
        import serial
        import termios
        SERIAL_RUNTIME_AVAILABLE = hasattr(serial, "Serial")
    except ImportError:
        pass


def virtual_diagnostic_line(sequence: int, *, accepted_id: int = 0) -> bytes:
    """가상 직렬 장치에 주입할 진단 예제다. 하드웨어 측정값이 아니다."""
    return (f"C_KEY_DIAG_V3,{sequence * 20},{sequence % 256},28185,0,{accepted_id},"
            "1,1,1,143,8,-8123,-7544,1430.0,1412.0,7.50,0.184,1.400,1.112,"
            f"WELCOME,2,{sequence},0,1,9").encode("ascii")


@unittest.skipUnless(SERIAL_RUNTIME_AVAILABLE, "Linux PTY와 pyserial이 있어야 실제 시리얼 전송을 검사합니다")
class SerialRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.deadline = time.monotonic() + 7
        self.master = self.slave = self.worker = None
        self.store = UwbStore("serial")
        self.addCleanup(self.cleanup_serial)
        self.master, self.slave = pty.openpty()
        self.port = os.ttyname(self.slave)
        os.set_blocking(self.master, False)
        self.worker = threading.Thread(target=serial_loop, args=(self.store, self.port), daemon=True)
        self.worker.start()
        # pyserial이 ECHO/ICANON을 끈 뒤에만 입력한다. PTY 자체의 에코를 송신으로 오인하지 않는다.
        self.wait_until(lambda: not termios.tcgetattr(self.slave)[3] & (termios.ECHO | termios.ICANON),
                        "pyserial 포트 초기화")

    def cleanup_serial(self):
        self.store.stop.set()
        worker_alive = False
        if self.worker is not None:
            self.worker.join(timeout=1.5)
            worker_alive = self.worker.is_alive()
        for name in ("master", "slave"):
            descriptor = getattr(self, name)
            if descriptor is not None:
                os.close(descriptor)
                setattr(self, name, None)
        self.assertFalse(worker_alive, "시리얼 수신 worker가 종료되지 않았습니다")

    def wait_until(self, predicate, description):
        while time.monotonic() < self.deadline:
            if predicate():
                return
            if self.worker is not None and not self.worker.is_alive():
                self.fail(f"시리얼 worker가 종료되었습니다: {description}; {self.store.snapshot()['error']}")
            self.store.stop.wait(.01)
        self.fail(f"PTY 시리얼 검증 시간 초과: {description}; {self.store.snapshot()['error']}")

    def send(self, data: bytes):
        """부분 쓰기를 처리하며 가상 장치에서 수신 프로그램 쪽으로만 자료를 보낸다."""
        offset = 0
        while offset < len(data):
            self.assertLess(time.monotonic(), self.deadline, "PTY 입력 전송 시간 초과")
            if select.select([], [self.master], [], .05)[1]:
                try:
                    offset += os.write(self.master, data[offset:offset + 4096])
                except BlockingIOError:
                    pass

    def assert_no_output(self):
        """수신 프로그램→가상 장치 방향의 실제 바이트가 없는지 확인한다."""
        readable = select.select([self.master], [], [], .05)[0]
        output = os.read(self.master, 4096) if readable else b""
        self.assertEqual(output, b"", "읽기 전용 수신기가 가상 직렬 장치로 데이터를 보냈습니다")

    def test_fragmentation_id_failure_resynchronization_timeout_and_no_write(self):
        frame = virtual_diagnostic_line(1)
        midpoint = len(frame) // 2
        self.send(frame[:midpoint])
        self.store.stop.wait(.05)
        self.assertEqual(self.store.snapshot()["state"], "waiting")
        self.send(frame[midpoint:] + b"\r\n")
        self.wait_until(lambda: self.store.snapshot()["state"] == "live", "분할 진단 한 줄 수신")
        self.assertEqual(self.store.snapshot()["measurement"]["position"],
                         {"x": 1.4, "y": -.184, "frame_id": "base_link"})
        self.assert_no_output()

        self.send(virtual_diagnostic_line(2, accepted_id=1) + b"\n")
        self.wait_until(lambda: self.store.snapshot()["measurement"]["accepted_id"] == 1,
                        "ID 불일치 수신")
        self.assertFalse(self.store.snapshot()["measurement"]["valid"])
        self.assertIsNone(self.store.snapshot()["measurement"]["position"])

        self.send(b"C_KEY_DIAG_V3," + b"9" * (MAX_LINE_BYTES + 100) + b"\n")
        self.wait_until(lambda: self.store.snapshot()["rejected_frames"] >= 1,
                        "상한 초과 줄 거부")
        self.assertEqual(self.store.snapshot()["state"], "error")
        self.send(b"C_KEY_DIAG_V3,broken\n")
        self.wait_until(lambda: self.store.snapshot()["rejected_frames"] >= 2,
                        "길이 초과 뒤 다음 줄의 오류 검증")

        recovered = virtual_diagnostic_line(3)
        self.send(recovered[:7])
        self.send(recovered[7:] + b"\n")
        self.wait_until(lambda: self.store.snapshot()["state"] == "live"
                        and self.store.snapshot()["measurement"]["sequence"] == 3,
                        "다음 정상 진단으로 재동기화")
        self.assertTrue(self.store.snapshot()["measurement"]["valid"])
        self.assert_no_output()

        self.wait_until(lambda: self.store.snapshot()["state"] == "stale", "500 ms 입력 시간 초과")
        self.assertFalse(self.store.snapshot()["measurement"]["valid"])
        self.assertIsNone(self.store.snapshot()["measurement"]["position"])
        self.assert_no_output()
        self.store.stop.set()
        self.worker.join(timeout=1.5)
        self.assertFalse(self.worker.is_alive())
        # 수신기의 독점 잠금과 파일 설명자가 실제로 반환되었는지 재개방으로 확인한다.
        with serial.Serial(self.port, baudrate=115200, timeout=.05, exclusive=True) as probe:
            self.assertTrue(probe.is_open)

    def test_pty_disconnect_invalidates_position_and_worker_stops(self):
        self.send(virtual_diagnostic_line(1) + b"\n")
        self.wait_until(lambda: self.store.snapshot()["state"] == "live", "연결 중단 전 정상 진단 수신")
        self.assertTrue(self.store.snapshot()["measurement"]["valid"])
        os.close(self.master)
        self.master = None
        self.wait_until(lambda: self.store.snapshot()["state"] == "error", "PTY 연결 중단 감지")
        self.assertFalse(self.store.snapshot()["measurement"]["valid"])
        self.assertIsNone(self.store.snapshot()["measurement"]["position"])
        self.assertFalse(self.store.snapshot()["synthetic"])
        self.store.stop.set()
        self.worker.join(timeout=1.5)
        self.assertFalse(self.worker.is_alive())


if __name__ == "__main__":
    unittest.main()
