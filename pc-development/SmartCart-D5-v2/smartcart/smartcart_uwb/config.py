"""명시적으로 지정한 센서 설정만 허용하며 장치나 라이브러리를 자동 탐색하지 않는다."""

from __future__ import annotations

import json
import math
import os
import posixpath
from dataclasses import dataclass
from pathlib import Path
import re
import sys
import sys
from types import MappingProxyType
from typing import Mapping

CONFIG_SCHEMA = "smartcart.dual_uwb_config.v1"
SENSOR_IDS = ("uwb_left", "uwb_right")
MAX_CONFIG_BYTES = 65536
MAX_PORT_LENGTH = 512
_TOP_KEYS = {"schema", "allowed_tag_address", "logical_target_id", "timeout_ms", "sensors", "lidar", "metadata"}
_MOUNT_KEYS = {"frame_id", "translation_m", "yaw_deg", "mounting_verified", "calibration_profile"}
_SENSOR_KEYS = _MOUNT_KEYS | {"port", "expected_device_identity", "device_identity_verified"}
_METADATA_KEYS = {"coordinate_basis", "translation_reference", "measurement_reference", "notes"}


def example_config_path() -> Path:
    """소스·일반 Python 설치·ament 설치의 명시적인 패키지 자료만 조회한다."""
    package_root = Path(__file__).resolve().parent.parent
    candidates = [package_root / "config" / "dual_uwb.d5.example.json",
                  package_root / "share" / "smartcart_sensing" / "config" / "dual_uwb.d5.example.json",
                  Path(sys.prefix) / "share" / "smartcart_sensing" / "config" / "dual_uwb.d5.example.json"]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("smartcart_sensing")) / "config" / "dual_uwb.d5.example.json"
    except (ImportError, LookupError):
        return candidates[0]


def example_config_path() -> Path:
    """소스·일반 Python 설치·ament 설치의 명시적인 패키지 자료만 조회한다."""
    package_root = Path(__file__).resolve().parent.parent
    candidates = [package_root / "config" / "dual_uwb.d5.example.json",
                  package_root / "share" / "smartcart_sensing" / "config" / "dual_uwb.d5.example.json",
                  Path(sys.prefix) / "share" / "smartcart_sensing" / "config" / "dual_uwb.d5.example.json"]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("smartcart_sensing")) / "config" / "dual_uwb.d5.example.json"
    except (ImportError, LookupError):
        return candidates[0]


@dataclass(frozen=True)
class MountingConfig:
    frame_id: str
    translation_m: tuple[float, float, float]
    yaw_deg: float
    mounting_verified: bool
    calibration_profile: str = "raw_unverified"

    def __post_init__(self):
        values = _mount({key: getattr(self, key) for key in _MOUNT_KEYS}, "mounting")
        for key, value in values.items():
            object.__setattr__(self, key, value)


@dataclass(frozen=True)
class SensorConfig:
    sensor_id: str
    port: str | None
    expected_device_identity: str
    device_identity_verified: bool
    frame_id: str
    translation_m: tuple[float, float, float]
    yaw_deg: float
    mounting_verified: bool
    calibration_profile: str = "raw_unverified"

    def __post_init__(self):
        if self.sensor_id not in SENSOR_IDS:
            raise ValueError("알 수 없는 센서 식별자입니다")
        if self.port is not None:
            canonical_port(self.port)
        _text(self.expected_device_identity, "expected_device_identity", 512, empty=True)
        _boolean(self.device_identity_verified, "device_identity_verified")
        if self.device_identity_verified and not self.expected_device_identity:
            raise ValueError("장치 식별 확인에는 비어 있지 않은 식별값이 필요합니다")
        values = _mount({key: getattr(self, key) for key in _MOUNT_KEYS}, self.sensor_id)
        for key, value in values.items():
            object.__setattr__(self, key, value)


@dataclass(frozen=True)
class CollectorConfig:
    sensors: Mapping[str, SensorConfig]
    lidar: MountingConfig
    allowed_tag_address: int = 0x6E19
    logical_target_id: int = 0
    timeout_ms: int = 500
    metadata: Mapping[str, str] | None = None
    schema: str = CONFIG_SCHEMA

    def __post_init__(self):
        if self.schema != CONFIG_SCHEMA or not isinstance(self.sensors, Mapping):
            raise ValueError("지원하지 않는 수집기 설정입니다")
        if set(self.sensors) != set(SENSOR_IDS):
            raise ValueError("좌우 센서 설정이 필요합니다")
        sensors = dict(self.sensors)
        for key, sensor in sensors.items():
            if not isinstance(sensor, SensorConfig) or sensor.sensor_id != key:
                raise ValueError("센서 설정과 식별자가 일치하지 않습니다")
        if not isinstance(self.lidar, MountingConfig):
            raise ValueError("LiDAR 장착 설정이 필요합니다")
        if len({self.lidar.frame_id, *(sensor.frame_id for sensor in sensors.values())}) != 3:
            raise ValueError("센서 frame_id가 중복되었습니다")
        _integer(self.allowed_tag_address, "allowed_tag_address", 0, 65535)
        _integer(self.logical_target_id, "logical_target_id", 0, 15)
        _integer(self.timeout_ms, "timeout_ms", 1, 5000)
        metadata = {} if self.metadata is None else dict(self.metadata)
        _object(metadata, _METADATA_KEYS, "metadata", required=False)
        metadata = {key: _text(value, "metadata." + key, 2048, empty=True) for key, value in metadata.items()}
        object.__setattr__(self, "sensors", MappingProxyType(sensors))
        object.__setattr__(self, "metadata", MappingProxyType(metadata))
        validate_distinct_ports(self)

    @property
    def drive_authorized(self) -> bool:
        """측정 설정의 검수 여부와 관계없이 이 어댑터는 구동을 승인하지 않는다."""
        return False


def _object(value: object, keys: set[str], where: str, *, required: bool = True) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f"{where}: 객체가 필요합니다")
    if set(value) - keys or (required and set(value) != keys):
        raise ValueError(f"{where}: 허용되지 않거나 누락된 설정 키입니다")
    return value


def _text(value: object, where: str, maximum: int, *, empty: bool = False) -> str:
    if not isinstance(value, str) or len(value) > maximum or (not empty and not value):
        raise ValueError(f"{where}: 문자열 길이가 허용 범위를 벗어났습니다")
    if value != value.strip() or any(ord(c) < 32 or ord(c) == 127 or 0xD800 <= ord(c) <= 0xDFFF for c in value):
        raise ValueError(f"{where}: 제어 문자나 앞뒤 공백은 허용하지 않습니다")
    return value


def _integer(value: object, where: str, minimum: int, maximum: int) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{where}: 정수 범위가 잘못되었습니다")
    return value


def _number(value: object, where: str, minimum: float, maximum: float) -> float:
    if type(value) not in (float, int) or not minimum <= value <= maximum or not math.isfinite(value):
        raise ValueError(f"{where}: 유한한 수와 허용 범위가 필요합니다")
    return float(value)


def _boolean(value: object, where: str) -> bool:
    if type(value) is not bool:
        raise ValueError(f"{where}: 참 또는 거짓이 필요합니다")
    return value


def canonical_port(port: str) -> str:
    """COM 별칭과 심볼릭 링크를 읽기 전용으로 정규화한다."""
    port = _text(port, "port", MAX_PORT_LENGTH)
    match = re.fullmatch(r"(?:\\\\\.\\)?COM([1-9][0-9]{0,5})", port, re.IGNORECASE)
    if match:
        return "com:" + str(int(match.group(1)))
    if os.name == "nt" and port.startswith("/dev/"):
        # 아직 연결할 수 없는 Pi의 경로를 PC 드라이브 경로로 해석하지 않는다.
        normalized = posixpath.normpath(port)
        if not normalized.startswith("/dev/") or "\\" in port:
            raise ValueError("port: Pi 장치 경로가 /dev 범위를 벗어났습니다")
        return "posix:" + normalized
    if not os.path.isabs(port):
        raise ValueError("port: 절대 장치 경로나 COM 번호를 명시해야 합니다")
    return os.path.normcase(os.path.realpath(port))


def validate_distinct_ports(config: CollectorConfig) -> None:
    assigned: set[str] = set()
    for sensor in config.sensors.values():
        if sensor.port is None:
            continue
        canonical = canonical_port(sensor.port)
        if canonical in assigned:
            raise ValueError("좌우 센서에 같은 장치 또는 포트 별칭을 지정할 수 없습니다")
        assigned.add(canonical)


def _mount(payload: dict, where: str) -> dict:
    frame_id = _text(payload["frame_id"], where + ".frame_id", 128)
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_/.-]{0,127}", frame_id) or ".." in frame_id:
        raise ValueError(f"{where}.frame_id: 허용되지 않는 프레임 이름입니다")
    translation = payload["translation_m"]
    if not isinstance(translation, (list, tuple)) or len(translation) != 3:
        raise ValueError(f"{where}.translation_m: 좌표 세 개가 필요합니다")
    profile = _text(payload["calibration_profile"], where + ".calibration_profile", 64)
    if profile != "raw_unverified":
        raise ValueError("이 수집기는 raw_unverified 외의 보정을 적용하지 않습니다")
    return {
        "frame_id": frame_id,
        "translation_m": tuple(_number(x, where + ".translation_m", -10, 10) for x in translation),
        "yaw_deg": _number(payload["yaw_deg"], where + ".yaw_deg", -180, 180),
        "mounting_verified": _boolean(payload["mounting_verified"], where + ".mounting_verified"),
        "calibration_profile": profile,
    }


def config_from_dict(payload: object) -> CollectorConfig:
    data = _object(payload, _TOP_KEYS, "config")
    if data["schema"] != CONFIG_SCHEMA:
        raise ValueError("지원하지 않는 설정 스키마입니다")
    sensor_payload = _object(data["sensors"], set(SENSOR_IDS), "sensors")
    sensors = {}
    for sensor_id in SENSOR_IDS:
        item = _object(sensor_payload[sensor_id], _SENSOR_KEYS, sensor_id)
        port = item["port"]
        if port is not None:
            port = _text(port, sensor_id + ".port", MAX_PORT_LENGTH)
            canonical_port(port)
        identity = _text(item["expected_device_identity"], sensor_id + ".expected_device_identity", 512, empty=True)
        verified = _boolean(item["device_identity_verified"], sensor_id + ".device_identity_verified")
        if verified and not identity:
            raise ValueError("장치 식별 확인에는 비어 있지 않은 식별값이 필요합니다")
        sensors[sensor_id] = SensorConfig(sensor_id, port, identity, verified, **_mount(item, sensor_id))
    if sensors["uwb_left"].frame_id == sensors["uwb_right"].frame_id:
        raise ValueError("좌우 센서는 서로 다른 frame_id를 사용해야 합니다")
    lidar = MountingConfig(**_mount(_object(data["lidar"], _MOUNT_KEYS, "lidar"), "lidar"))
    if lidar.frame_id in {s.frame_id for s in sensors.values()}:
        raise ValueError("LiDAR와 UWB의 frame_id가 중복되었습니다")
    metadata = _object(data["metadata"], _METADATA_KEYS, "metadata", required=False)
    metadata = {key: _text(value, "metadata." + key, 2048, empty=True) for key, value in metadata.items()}
    config = CollectorConfig(
        sensors=MappingProxyType(sensors), lidar=lidar,
        allowed_tag_address=_integer(data["allowed_tag_address"], "allowed_tag_address", 0, 65535),
        logical_target_id=_integer(data["logical_target_id"], "logical_target_id", 0, 15),
        timeout_ms=_integer(data["timeout_ms"], "timeout_ms", 1, 5000),
        metadata=MappingProxyType(metadata),
    )
    validate_distinct_ports(config)
    return config


def _unique_object(pairs: list[tuple[str, object]]) -> dict:
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("중복된 JSON 설정 키입니다")
        result[key] = value
    return result


def _reject_constant(value: str) -> None:
    raise ValueError("JSON에는 유한한 수만 허용합니다")


def load_config(path: str | os.PathLike[str]) -> CollectorConfig:
    file = Path(path)
    if not file.is_file() or file.stat().st_size > MAX_CONFIG_BYTES:
        raise ValueError("설정은 크기 상한 이내의 일반 파일이어야 합니다")
    with file.open("rb") as source:
        raw = source.read(MAX_CONFIG_BYTES + 1)
    if len(raw) > MAX_CONFIG_BYTES:
        raise ValueError("설정 파일 크기 상한을 넘었습니다")
    try:
        payload = json.loads(raw.decode("utf-8"), object_pairs_hook=_unique_object, parse_constant=_reject_constant)
    except (UnicodeError, json.JSONDecodeError, RecursionError) as error:
        raise ValueError("올바른 UTF-8 JSON 설정이 필요합니다") from error
    return config_from_dict(payload)
