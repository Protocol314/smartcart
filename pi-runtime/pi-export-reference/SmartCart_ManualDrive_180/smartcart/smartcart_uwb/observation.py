"""공간 거리와 원시 각도를 관측으로 표현하며 위치나 구동 승인을 만들지 않는다."""

from __future__ import annotations

import math
from typing import Mapping

from .config import SensorConfig

OBSERVATION_SCHEMA = "smartcart.uwb_observation.v1"
MAX_MONOTONIC_NS = (1 << 63) - 1
_FRAME_BOUNDS = {
    "sequence": (0, 255), "tag_address": (0, 65535),
    "angle_deg": (-(1 << 31), (1 << 31) - 1),
    "distance_cm": (0, (1 << 32) - 1),
    "user_command": (0, 65535),
    "first_path_power": (-(1 << 31), (1 << 31) - 1),
    "rx_level": (-(1 << 31), (1 << 31) - 1),
    "acceleration_x": (-32768, 32767), "acceleration_y": (-32768, 32767),
    "acceleration_z": (-32768, 32767),
}


def timestamp_ns(value: object, name: str = "now_ns") -> int:
    if type(value) is not int or not 0 <= value <= MAX_MONOTONIC_NS:
        raise ValueError(f"{name}: 음수가 아닌 63비트 정수가 필요합니다")
    return value


def validate_frame(frame: object) -> dict:
    if not isinstance(frame, Mapping) or set(frame) != set(_FRAME_BOUNDS):
        raise ValueError("파서 프레임 필드가 잘못되었습니다")
    result = {}
    for key, (minimum, maximum) in _FRAME_BOUNDS.items():
        value = frame[key]
        if type(value) is not int or not minimum <= value <= maximum:
            raise ValueError("파서 프레임 정수 범위가 잘못되었습니다")
        result[key] = value
    return result


def make_observation(*, sensor: SensorConfig, frame: dict | None, allowed_tag_address: int,
                     logical_target_id: int, sample_sequence: int, stream_epoch: int,
                     adapter_session: str, source_mode: str, rx_ns: int,
                     source_rx_ns: int | None, valid_until_ns: int,
                     invalid_reason: str | None = None) -> dict:
    """무효 프레임도 원인이 보이는 동일 스키마로 반환한다."""
    measurement_valid = bool(frame is not None and 5 <= frame["distance_cm"] <= 2000
                             and -180 <= frame["angle_deg"] <= 180)
    id_valid = bool(frame is not None and frame["tag_address"] == allowed_tag_address)
    reason = invalid_reason
    if reason is None and not measurement_valid:
        reason = "invalid_measurement"
    if reason is None and not id_valid:
        reason = "id_mismatch"
    valid = reason is None and measurement_valid and id_valid
    return {
        "schema": OBSERVATION_SCHEMA,
        "sensor_id": sensor.sensor_id, "frame_id": sensor.frame_id,
        "tag_address": frame["tag_address"] if frame is not None else None,
        "logical_target_id": logical_target_id if id_valid else None,
        "device_sequence": frame["sequence"] if frame is not None else None,
        "sample_sequence": sample_sequence, "stream_epoch": stream_epoch,
        "adapter_session": adapter_session, "source_mode": source_mode,
        "timestamp_source": "HOST_RX", "rx_monotonic_ns": rx_ns,
        "source_rx_monotonic_ns": source_rx_ns,
        "valid_until_monotonic_ns": valid_until_ns,
        "device_timestamp_ns": None,
        "range_m": frame["distance_cm"] / 100.0 if frame is not None else None,
        "bearing_rad": -math.radians(frame["angle_deg"]) if frame is not None else None,
        "raw_angle_deg": frame["angle_deg"] if frame is not None else None,
        "first_path_power": frame["first_path_power"] if frame is not None else None,
        "rx_level": frame["rx_level"] if frame is not None else None,
        "measurement_valid": measurement_valid, "id_valid": id_valid,
        "valid": bool(valid), "invalid_reason": reason,
        "mounting_verified": sensor.mounting_verified,
        "calibration_profile": sensor.calibration_profile,
        "drive_authorized": False,
    }
