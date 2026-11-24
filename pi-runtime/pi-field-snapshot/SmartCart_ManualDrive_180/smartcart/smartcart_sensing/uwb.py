"""ESP32 진단 수신·검증·좌표 변환. 장치에는 어떠한 명령도 보내지 않는다."""
from __future__ import annotations

import copy
import math
import re
import threading
import time
from collections import deque
from typing import Any

SCHEMA = "smartcart.uwb.v1"
MAX_LINE_BYTES = 2048
MAX_ROS_JSON_BYTES = 16384
KNOWN_TAG_ADDRESS = 0x6E19
FIRMWARE_STATES = frozenset({
    "NO_KEY", "INVALID_ID", "OUT_OF_ANGLE", "SENSING", "WELCOME", "UNLOCKED", "FAULT",
})
V2_FIELDS = (
    "timestamp_ms", "sequence", "tag_address", "tag_id", "accepted_id", "link_ok",
    "measurement_ready", "pose_valid", "raw_distance_cm", "raw_angle_deg",
    "corrected_distance_mm", "filtered_distance_mm", "filtered_angle_deg",
    "x_m", "y_m", "boundary_m", "state", "events", "accepted_frames", "rejected_frames",
)
V3_FIELDS = V2_FIELDS[:10] + ("first_path_power", "rx_level") + V2_FIELDS[10:] + (
    "angle_sample_rejected", "angle_rejected_samples",
)
BOOL_FIELDS = frozenset({"link_ok", "measurement_ready", "pose_valid", "angle_sample_rejected"})
FLOAT_RANGES = {
    "raw_angle_deg": (-180, 180), "corrected_distance_mm": (0, 30000),
    "filtered_distance_mm": (0, 30000), "filtered_angle_deg": (-180, 180),
    "x_m": (-30, 30), "y_m": (-30, 30), "boundary_m": (0, 30),
}
INT_RANGES = {
    "timestamp_ms": (0, 0xFFFFFFFF), "sequence": (0, 255), "tag_address": (0, 65535),
    "tag_id": (0, 255), "accepted_id": (0, 15), "raw_distance_cm": (0, 0xFFFFFFFF),
    "first_path_power": (-0x80000000, 0x7FFFFFFF), "rx_level": (-0x80000000, 0x7FFFFFFF),
    "events": (0, 0xFFFFFFFF), "accepted_frames": (0, 0xFFFFFFFF),
    "rejected_frames": (0, 0xFFFFFFFF), "angle_rejected_samples": (0, 0xFFFFFFFF),
}


def _number(value: Any, name: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name}: 숫자가 필요합니다")
    number = float(value)
    if not math.isfinite(number) or not minimum <= number <= maximum:
        raise ValueError(f"{name}: 허용 범위를 벗어났습니다")
    return number


def validate_transform(yaw_deg: float, offset_x: float, offset_y: float) -> dict:
    """ROS 카트 좌표 기준 장착 보정값의 범위를 확인한다."""
    return {
        "frame_id": "base_link",
        "yaw_deg": _number(yaw_deg, "yaw_deg", -360, 360),
        "offset_x": _number(offset_x, "offset_x", -10, 10),
        "offset_y": _number(offset_y, "offset_y", -10, 10),
    }


def transform_position(x_m: float, y_m: float, *, yaw_deg: float = 0,
                       offset_x: float = 0, offset_y: float = 0) -> dict:
    """UWB +X 오른쪽/+Y 전방을 카트 +X 전방/+Y 왼쪽으로 바꾼다."""
    transform = validate_transform(yaw_deg, offset_x, offset_y)
    raw_x = _number(x_m, "uwb.x_m", -30, 30)
    raw_y = _number(y_m, "uwb.y_m", -30, 30)
    yaw = math.radians(transform["yaw_deg"])
    forward, left = raw_y, -raw_x
    return {"x": transform["offset_x"] + forward * math.cos(yaw) - left * math.sin(yaw),
            "y": transform["offset_y"] + forward * math.sin(yaw) + left * math.cos(yaw),
            "frame_id": "base_link"}


def parse_diagnostic_line(line: str) -> dict | None:
    """기존 C_KEY_DIAG_V3/V2 열 정의를 그대로 읽되 타입·상한을 엄격하게 검사한다.

    ESP-IDF 로그 접두사는 허용한다. 일반 부팅 로그는 무시하며 진단 프레임의
    오류는 예외로 알린다. 보존된 C_KEY_DIAG_V2/V3 회귀 형식의 열 순서를 유지한다.
    """
    if not isinstance(line, str) or len(line) > MAX_LINE_BYTES:
        raise ValueError("UWB 진단 줄이 허용 길이를 벗어났습니다")
    match = re.search(r"C_KEY_DIAG_V[23],", line)
    if match is None:
        if "C_KEY_DIAG_" in line:
            raise ValueError("지원하지 않는 UWB 진단 형식입니다")
        return None
    payload = line[match.start():].strip()
    if any(ord(char) < 32 or ord(char) > 126 for char in payload):
        raise ValueError("UWB 진단은 ASCII 한 줄이어야 합니다")
    row = payload.split(",")
    fields = V3_FIELDS if row[0] == "C_KEY_DIAG_V3" else V2_FIELDS
    if len(row) != len(fields) + 1:
        raise ValueError("UWB 진단 열 수가 일치하지 않습니다")
    frame: dict = {"version": row[0], "raw_diagnostic_line": payload}
    for name, value in zip(fields, row[1:]):
        if name == "state":
            if value not in FIRMWARE_STATES:
                raise ValueError("알 수 없는 펌웨어 상태입니다")
            frame[name] = value
        elif name in BOOL_FIELDS:
            if value not in ("0", "1"):
                raise ValueError(f"{name}: 0 또는 1이어야 합니다")
            frame[name] = value == "1"
        elif name in INT_RANGES:
            if not re.fullmatch(r"-?[0-9]{1,11}", value):
                raise ValueError(f"{name}: 정수 형식이 아닙니다")
            number = int(value)
            lo, hi = INT_RANGES[name]
            if not lo <= number <= hi:
                raise ValueError(f"{name}: 허용 범위를 벗어났습니다")
            frame[name] = number
        else:
            if not re.fullmatch(r"-?(?:[0-9]{1,8})(?:\.[0-9]{1,8})?", value):
                raise ValueError(f"{name}: 유한 실수 형식이 아닙니다")
            frame[name] = _number(float(value), name, *FLOAT_RANGES[name])
    frame.setdefault("first_path_power", 0)
    frame.setdefault("rx_level", 0)
    frame.setdefault("angle_sample_rejected", False)
    frame.setdefault("angle_rejected_samples", 0)
    return frame


def _measurement(frame: dict, transform: dict) -> dict:
    authenticated = (frame["tag_address"] == KNOWN_TAG_ADDRESS
                     and frame["tag_id"] == 0 and frame["accepted_id"] == 0)
    valid = (authenticated and frame["link_ok"] and frame["measurement_ready"]
             and frame["pose_valid"] and frame["state"] not in {"FAULT", "NO_KEY", "INVALID_ID"}
             and 5 <= frame["raw_distance_cm"] <= 2000
             and 50 <= frame["filtered_distance_mm"] <= 20000)
    position = transform_position(frame["x_m"], frame["y_m"],
                                  yaw_deg=transform["yaw_deg"],
                                  offset_x=transform["offset_x"], offset_y=transform["offset_y"])
    return {
        "tag_address": f"0x{frame['tag_address']:04X}", "tag_id": frame["tag_id"],
        "accepted_id": frame["accepted_id"], "authenticated": authenticated, "valid": bool(valid),
        "raw_distance_m": frame["raw_distance_cm"] / 100.0,
        "filtered_distance_m": frame["filtered_distance_mm"] / 1000.0,
        "angle_deg": frame["filtered_angle_deg"], "raw_angle_deg": frame["raw_angle_deg"],
        "position": position if valid else None,
        "uwb_position": {"x": frame["x_m"], "y": frame["y_m"], "frame_id": "uwb"} if valid else None,
        "firmware_state": frame["state"], "firmware_timestamp_ms": frame["timestamp_ms"],
        "sequence": frame["sequence"], "link_ok": frame["link_ok"],
        "measurement_ready": frame["measurement_ready"], "pose_valid": frame["pose_valid"],
        "boundary_distance_m": frame["boundary_m"],
        "corrected_distance_m": frame["corrected_distance_mm"] / 1000.0,
        "first_path_power": frame["first_path_power"], "rx_level": frame["rx_level"],
        "angle_sample_rejected": frame["angle_sample_rejected"],
        "angle_rejected_samples": frame["angle_rejected_samples"], "events": frame["events"],
        "accepted_frames": frame["accepted_frames"], "rejected_frames": frame["rejected_frames"],
        "protocol": frame["version"], "raw_diagnostic_line": frame["raw_diagnostic_line"],
    }


class UwbStore:
    """최신 UWB 한 프레임만 메모리에 보유하고 오래된 위치는 표시하지 않는다."""
    def __init__(self, source_mode: str = "serial", stale_seconds: float = .5,
                 stop: threading.Event | None = None, yaw_deg: float = 0,
                 offset_x: float = 0, offset_y: float = 0):
        if source_mode not in {"serial", "demo", "ros", "off"}:
            raise ValueError("지원하지 않는 UWB 입력 모드입니다")
        self.source_mode = source_mode
        self.stale_seconds = _number(stale_seconds, "stale_seconds", .05, 10)
        self.transform = validate_transform(yaw_deg, offset_x, offset_y)
        self.stop = stop if stop is not None else threading.Event()
        self.lock = threading.Lock()
        self.latest: dict | None = None
        self.last_received: float | None = None
        self.arrivals: deque = deque(maxlen=300)
        self.error: str | None = None
        self.rejected = 0
        self.synthetic = source_mode == "demo"
        self.upstream_state: str | None = None
        self.last_ros_line: str | None = None

    def _publish_frame(self, frame: dict, *, now: float, transform: dict,
                       synthetic: bool = False) -> None:
        measurement = _measurement(frame, transform)
        with self.lock:
            self.latest = measurement
            self.transform = transform
            self.last_received = now
            self.arrivals.append(now)
            self.synthetic = synthetic
            self.upstream_state = None
            self.error = None

    def publish_line(self, line: str, now: float | None = None) -> bool:
        try:
            frame = parse_diagnostic_line(line)
            if frame is None:
                return False
            with self.lock:
                if (self.latest is not None
                        and self.latest["raw_diagnostic_line"] == frame["raw_diagnostic_line"]):
                    # 좌표가 같아도 정상 ESP 진단은 시간·순번이 변한다. 재전송은 새 측정이 아니다.
                    return False
            self._publish_frame(frame, now=time.monotonic() if now is None else now,
                                transform=self.transform, synthetic=self.source_mode == "demo")
            return True
        except (ValueError, TypeError, OverflowError):
            self.set_error("잘못된 UWB 진단 프레임을 거부했습니다", rejected=True)
            return False

    def publish_payload(self, payload: dict, now: float | None = None) -> bool:
        """ROS 스냅샷의 원시 진단과 장착값을 재검증한다. 중복 재발행은 생존 시간을 늘리지 않는다."""
        try:
            if not isinstance(payload, dict) or payload.get("schema") != SCHEMA:
                raise ValueError("UWB ROS 스키마가 일치하지 않습니다")
            state = payload.get("state")
            if state not in {"waiting", "live", "synthetic", "stale", "error", "off"}:
                raise ValueError("UWB ROS 상태가 잘못되었습니다")
            if state in {"stale", "error", "waiting", "off"}:
                with self.lock:
                    self.upstream_state = state
                    self.error = "UWB ROS 송신 소스 오류" if state == "error" else None
                return False
            measurement = payload.get("measurement")
            transform = payload.get("transform")
            if not isinstance(measurement, dict) or not isinstance(transform, dict):
                raise ValueError("UWB ROS 측정값 또는 변환이 없습니다")
            if transform.get("frame_id") != "base_link":
                raise ValueError("지원하지 않는 UWB 출력 좌표계입니다")
            transform = validate_transform(transform.get("yaw_deg"), transform.get("offset_x"),
                                           transform.get("offset_y"))
            raw_line = measurement.get("raw_diagnostic_line")
            frame = parse_diagnostic_line(raw_line)
            if frame is None:
                raise ValueError("UWB 원시 진단이 없습니다")
            age = _number(payload.get("age_ms"), "age_ms", 0, 86400000) / 1000
            sent = _number(payload.get("server_unix"), "server_unix", 0, 100000000000)
            transit = time.time() - sent
            if transit < -5:
                raise ValueError("ROS 송신 시각이 미래입니다. 시계를 확인하십시오")
            # 동일 Pi의 ROS 분리에 사용한다. 오래 대기한 메시지도 현재 위치로 되살리지 않는다.
            age += max(0, transit)
            synthetic = payload.get("synthetic")
            if not isinstance(synthetic, bool) or (state == "synthetic") != synthetic:
                raise ValueError("UWB 합성 출처 표시가 일치하지 않습니다")
            if raw_line == self.last_ros_line:
                return False
            self.last_ros_line = raw_line
            self._publish_frame(frame, now=(time.monotonic() if now is None else now) - age,
                                transform=transform, synthetic=synthetic)
            return True
        except (ValueError, TypeError, OverflowError):
            self.set_error("잘못된 UWB ROS 메시지를 거부했습니다", rejected=True)
            return False

    def set_error(self, message: str, *, rejected: bool = False) -> None:
        with self.lock:
            self.error = str(message)[:300]
            self.rejected += int(rejected)

    def snapshot(self, now: float | None = None) -> dict:
        mono = time.monotonic() if now is None else now
        with self.lock:
            age = None if self.last_received is None else max(0, mono - self.last_received)
            samples = [v for v in self.arrivals if 0 <= mono - v <= 3]
            hz = ((len(samples) - 1) / (samples[-1] - samples[0])
                  if len(samples) >= 2 and samples[-1] > samples[0] else 0)
            if self.source_mode == "off":
                state = "off"
            elif self.error:
                state = "error"
            elif self.upstream_state is not None:
                state = self.upstream_state
            elif age is None:
                state = "waiting"
            elif age > self.stale_seconds:
                state = "stale"
            else:
                state = "synthetic" if self.synthetic else "live"
            measurement = copy.deepcopy(self.latest)
            if measurement is not None and state not in {"live", "synthetic"}:
                measurement.update(valid=False, position=None, uwb_position=None)
            return {"schema": SCHEMA, "state": state, "source_mode": self.source_mode,
                    "synthetic": self.synthetic, "server_unix": time.time(),
                    "age_ms": None if age is None else round(age * 1000, 2),
                    "stale_after_ms": self.stale_seconds * 1000,
                    "receive_hz": round(hz, 2), "error": self.error,
                    "rejected_frames": self.rejected, "transform": dict(self.transform),
                    "measurement": measurement}


def validate_serial_port(port: str) -> str:
    """로컬 시리얼 장치만 허용하며 pyserial 네트워크 URL은 허용하지 않는다."""
    if not isinstance(port, str) or not 1 <= len(port) <= 256:
        raise ValueError("ESP32 로컬 시리얼 장치 경로가 필요합니다")
    if re.fullmatch(r"COM[1-9][0-9]{0,3}", port, re.IGNORECASE):
        return port
    if (port.startswith("/dev/") and not any(ord(char) < 32 for char in port)
            and "\\" not in port and ":" not in port
            and all(part not in {"", ".", ".."} for part in port.split("/")[2:])):
        return port
    raise ValueError("/dev/ 아래 로컬 시리얼 장치 또는 COM 번호를 지정하십시오")


def serial_loop(store: UwbStore, port: str, baud: int = 115200) -> None:
    """시리얼 단일 소유자. 길이 제한·재연결·종료를 지원하며 송신은 수행하지 않는다."""
    try:
        validate_serial_port(port)
        if isinstance(baud, bool) or baud not in {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600}:
            raise ValueError("지원하지 않는 시리얼 속도입니다")
        import serial
    except ImportError:
        store.set_error("pyserial이 없습니다. 라즈베리파이에 python3-serial을 설치하십시오")
        return
    except ValueError as exc:
        store.set_error(str(exc))
        return
    while not store.stop.is_set():
        connection = None
        try:
            # 포트를 열기 전에 DTR/RTS를 낮춰 일반적인 ESP32 자동 리셋을 줄인다.
            connection = serial.Serial(port=None, baudrate=baud, timeout=.2, write_timeout=.2)
            connection.dtr = False
            connection.rts = False
            connection.port = port
            connection.exclusive = True
            connection.open()
            buffer = bytearray()
            discarding = False
            while not store.stop.is_set():
                data = connection.read(min(max(connection.in_waiting, 1), 4096))
                for value in data:
                    if value == 10:
                        if not discarding:
                            try:
                                store.publish_line(buffer.decode("utf-8", errors="strict"))
                            except UnicodeDecodeError:
                                store.set_error("문자 인코딩이 잘못된 UWB 줄을 거부했습니다", rejected=True)
                        buffer.clear()
                        discarding = False
                    elif not discarding:
                        if len(buffer) >= MAX_LINE_BYTES:
                            store.set_error("과도하게 긴 UWB 진단 줄을 거부했습니다", rejected=True)
                            buffer.clear()
                            discarding = True
                        else:
                            buffer.append(value)
        except (OSError, serial.SerialException):
            store.set_error("ESP32 시리얼 연결이 끊어졌습니다. 재연결을 기다립니다")
        finally:
            if connection is not None:
                try:
                    connection.close()
                except (OSError, serial.SerialException):
                    pass
        store.stop.wait(1.0)


def synthetic_line(t: float, index: int = 0) -> str:
    """하드웨어가 아닌 합성 UWB 예제다. demo 모드에서만 사용한다."""
    distance = 1.8 + .4 * math.sin(t * .4)
    angle = 30 * math.sin(t * .3)
    x_m = distance * math.sin(math.radians(angle))
    y_m = distance * math.cos(math.radians(angle))
    return (f"C_KEY_DIAG_V3,{int(t * 1000) % 4294967296},{index % 256},28185,0,0,1,1,1,"
            f"{int(distance * 100)},{angle:.2f},-8100,-7500,{distance * 1000:.1f},"
            f"{distance * 1000:.1f},{angle:.2f},{x_m:.3f},{y_m:.3f},"
            f"{max(0, distance - .3):.3f},WELCOME,0,{index % 4294967296},0,0,0")


def demo_loop(store: UwbStore) -> None:
    if store.source_mode != "demo":
        store.set_error("합성 UWB는 명시적인 demo 모드에서만 실행할 수 있습니다")
        return
    start = time.monotonic()
    index = 0
    while not store.stop.is_set():
        store.publish_line(synthetic_line(time.monotonic() - start, index))
        index += 1
        store.stop.wait(.1)
