"""명시적으로 배정한 좌우 BU04 데이터 포트만 읽는다."""
from __future__ import annotations

import os
import posixpath
import re
import threading
import time
from typing import Callable

from .collector import DualCollector

SENSOR_IDS = ("uwb_left", "uwb_right")
MAX_READ_BYTES = 1024


class OutputError(Exception):
    """기록·발행 실패는 장치 재연결로 숨기지 않는다."""


def canonical_port(port: str) -> str:
    """별칭으로 같은 포트를 두 번 여는 경우를 검출한다."""
    if not isinstance(port, str) or not 1 <= len(port) <= 512:
        raise ValueError("포트 이름 길이가 올바르지 않습니다")
    if re.fullmatch(r"(?:\\\\\.\\)?COM[1-9][0-9]{0,3}", port, re.IGNORECASE):
        return port.removeprefix("\\\\.\\").upper()
    if not port.startswith("/dev/") or any(ord(c) < 32 for c in port):
        raise ValueError("로컬 COM 또는 /dev/ 직렬 포트만 허용합니다")
    if os.name == "nt":
        # PC 사전 점검은 Pi의 심볼릭 링크나 실제 장치 존재를 확인하지 않는다.
        normalized = posixpath.normpath(port)
        if not normalized.startswith("/dev/") or "\\" in port:
            raise ValueError("Pi 장치 경로가 /dev 범위를 벗어났습니다")
        return "posix:" + normalized
    return os.path.realpath(port)


def device_identity(info) -> str:
    """USB 고유번호가 없으면 물리적 연결 위치로 구분한다."""
    vid, pid = getattr(info, "vid", None), getattr(info, "pid", None)
    serial_number = getattr(info, "serial_number", None)
    location = getattr(info, "location", None)
    if vid is not None and pid is not None:
        prefix = f"vid={vid:04x};pid={pid:04x};"
        if serial_number:
            identity = prefix + "serial=" + str(serial_number)
            return identity + (";location=" + str(location) if location else "")
        if location:
            return prefix + "location=" + str(location)
    return "hwid=" + str(getattr(info, "hwid", ""))


def list_data_ports() -> list[dict]:
    """열거만 수행하며 포트를 열거나 AT 명령을 보내지 않는다."""
    from serial.tools import list_ports
    return [{"port": str(p.device), "identity": device_identity(p)}
            for p in list_ports.comports()]


def identify_port(port: str) -> str:
    resolved = canonical_port(port)
    matches = [p for p in list_data_ports() if canonical_port(p["port"]) == resolved]
    if len(matches) != 1 or matches[0]["identity"] in {"hwid=", "hwid=n/a"}:
        raise ValueError("배정한 USB 데이터 장치를 고유하게 확인할 수 없습니다")
    return matches[0]["identity"]


def open_readonly_port(port: str):
    """송신 API를 호출하지 않고 DTR/RTS 비활성을 요청한 후 연다."""
    import serial
    options = dict(port=None, baudrate=115200, timeout=.05, write_timeout=.05,
                   xonxoff=False, rtscts=False, dsrdtr=False)
    if os.name == "posix":
        options["exclusive"] = True
    device = serial.Serial(**options)
    try:
        device.dtr = False
        device.rts = False
        device.port = port
        device.open()
        # 포트를 다시 열기 전 운영체제에 쌓인 바이트를 새 측정으로 사용하지 않는다.
        device.reset_input_buffer()
        return device
    except BaseException:
        device.close()
        raise


class SerialReaders:
    """좌우 독립 수신과 직렬화된 이벤트 시각·콜백을 관리한다."""

    def __init__(self, collector: DualCollector, *,
                 port_opener: Callable = open_readonly_port,
                 identity_reader: Callable = identify_port,
                 capture_callback: Callable | None = None,
                 observation_callback: Callable | None = None,
                 sensor_ids: tuple[str, ...] = SENSOR_IDS,
                 retry_seconds: float = 1.0,
                 clock: Callable = time.monotonic_ns):
        if not .05 <= retry_seconds <= 10:
            raise ValueError("재연결 대기 시간이 범위를 벗어났습니다")
        if collector.source_mode != "live":
            raise ValueError("실물 수신기는 live 수집기에만 연결할 수 있습니다")
        if sensor_ids not in (SENSOR_IDS, ("uwb_left",)):
            raise ValueError("수신 구성은 좌우 또는 왼쪽 단독이어야 합니다")
        self.sensor_ids = sensor_ids
        self.collector = collector
        self.port_opener, self.identity_reader = port_opener, identity_reader
        self.capture_callback, self.observation_callback = capture_callback, observation_callback
        self.retry_seconds, self.clock = retry_seconds, clock
        self.stop = threading.Event()
        self.event_lock = threading.RLock()
        self.threads: list[threading.Thread] = []
        self.started = False
        self.origin_ns = self.clock()
        self.fatal_error: str | None = None
        sensors = collector.config.sensors
        ports, identities = [], []
        for sensor_id in self.sensor_ids:
            sensor = sensors[sensor_id]
            if not sensor.port or not sensor.expected_device_identity or not sensor.device_identity_verified:
                raise ValueError("실물 수집에는 좌우 데이터 포트와 확인된 장치 식별자가 필요합니다")
            ports.append(canonical_port(sensor.port))
            identities.append(sensor.expected_device_identity)
        if len(set(ports)) != len(self.sensor_ids) or len(set(identities)) != len(self.sensor_ids):
            raise ValueError("좌우에는 서로 다른 데이터 포트와 장치 식별자가 필요합니다")

    def _event(self, sensor_id: str | None, kind: str, data: bytes = b"") -> None:
        with self.event_lock:
            now = self.clock()
            observations = []
            if kind == "bytes":
                observations = self.collector.feed(sensor_id, data, now_ns=now)
            elif kind == "reconnect":
                self.collector.reconnect(sensor_id)
            elif kind == "disconnect":
                self.collector.disconnected(sensor_id)
            if self.capture_callback is not None:
                event = {"schema": "smartcart.uwb_capture.v1", "event": kind,
                         "at_ns": now - self.origin_ns, "source": "recorded"}
                if sensor_id is not None:
                    event["sensor_id"] = sensor_id
                if kind == "bytes":
                    event["data_hex"] = data.hex()
                try:
                    self.capture_callback(event)
                except Exception as exc:
                    raise OutputError("기록 콜백 실패") from exc
            if self.observation_callback is not None:
                for observation in observations:
                    try:
                        self.observation_callback(observation)
                    except Exception as exc:
                        raise OutputError("관측 발행 콜백 실패") from exc

    def tick(self) -> dict:
        with self.event_lock:
            self._event(None, "tick")
            return self.collector.snapshot(now_ns=self.clock())

    def _worker(self, sensor_id: str) -> None:
        sensor = self.collector.config.sensors[sensor_id]
        while not self.stop.is_set():
            device = None
            try:
                if self.identity_reader(sensor.port) != sensor.expected_device_identity:
                    raise ValueError("장치 식별자가 배정과 다릅니다")
                device = self.port_opener(sensor.port)
                # 열거와 open 사이에 장치가 교체된 경우도 읽기를 시작하지 않는다.
                if self.identity_reader(sensor.port) != sensor.expected_device_identity:
                    raise ValueError("포트 개방 중 장치 식별자가 변경되었습니다")
                self._event(sensor_id, "reconnect")
                while not self.stop.is_set():
                    data = device.read(MAX_READ_BYTES)
                    if not isinstance(data, bytes) or len(data) > MAX_READ_BYTES:
                        raise ValueError("직렬 수신 블록이 올바르지 않습니다")
                    if data:
                        self._event(sensor_id, "bytes", data)
            except (OSError, ValueError, RuntimeError) as exc:
                print(f"BU04_CONNECT_ERROR [{sensor_id}]: {type(exc).__name__}: {exc}", flush=True)
                self.collector.disconnected(sensor_id, reason="serial_or_identity_error")
            except Exception:
                # 기록 실패 등 예상 밖 오류를 정상 수신으로 숨기지 않는다.
                self.fatal_error = "수집 콜백 또는 직렬 수신 처리에 실패했습니다"
                self.stop.set()
            finally:
                if device is not None:
                    try:
                        device.close()
                    except Exception:
                        self.fatal_error = "직렬 포트 종료에 실패했습니다"
                        self.stop.set()
                try:
                    self._event(sensor_id, "disconnect")
                except Exception:
                    self.fatal_error = "수집 종료 이벤트 처리에 실패했습니다"
                    self.stop.set()
            self.stop.wait(self.retry_seconds)

    def start(self) -> None:
        if self.started:
            raise RuntimeError("수신기를 두 번 시작할 수 없습니다")
        self.started = True
        try:
            for sensor_id in self.sensor_ids:
                worker = threading.Thread(target=self._worker, args=(sensor_id,),
                                          name=f"bu04-{sensor_id}", daemon=True)
                worker.start()
                self.threads.append(worker)
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        self.stop.set()
        for worker in self.threads:
            worker.join(timeout=2)
        if any(worker.is_alive() for worker in self.threads):
            raise RuntimeError("직렬 수신기가 제한 시간 안에 종료되지 않았습니다")
        if self.fatal_error is not None:
            raise RuntimeError(self.fatal_error)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.close()
