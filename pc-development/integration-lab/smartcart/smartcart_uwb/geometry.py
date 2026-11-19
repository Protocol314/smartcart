"""명시된 거리·높이 가정에 따른 평면 투영과 보수적인 좌우 위치 경계.

오차 반경은 설정된 상한이 성립할 때의 기하 경계다. 현장 오차를 보증하거나
통계적 독립성, 공통 전파 편향 제거, 장치 측정 시각 동기화를 가정하지 않는다.
"""
from __future__ import annotations

import math

from .config import CollectorConfig, SensorConfig
from .estimation_config import EstimationConfig
from .observation import MAX_MONOTONIC_NS


ALIGNMENT_METHOD = "bounded_host_rx_pair_no_motion_compensation"


class GeometryError(ValueError):
    """원시 관측값을 오류 문자열에 노출하지 않는 기하 계산 오류."""

    def __init__(self, reason: str):
        self.reason = reason
        super().__init__("UWB 기하 계산을 거부했습니다: " + reason)


def _finite(value: object, reason: str) -> float:
    if type(value) not in (int, float):
        raise GeometryError(reason)
    try:
        number = float(value)
    except (OverflowError, ValueError):
        raise GeometryError(reason) from None
    if not math.isfinite(number):
        raise GeometryError(reason)
    return number


def _settings(settings: EstimationConfig) -> None:
    if not isinstance(settings, EstimationConfig):
        raise GeometryError("invalid_settings")
    if settings.range_model == "unknown":
        raise GeometryError("unknown_range_model")
    if settings.range_model not in ("horizontal", "slant_known_height"):
        raise GeometryError("invalid_settings")


def _horizontal_interval(range_m: float, sensor: SensorConfig,
                         settings: EstimationConfig) -> tuple[float, float, float]:
    lower_range = range_m - settings.range_error_m
    upper_range = range_m + settings.range_error_m
    minimum = settings.min_horizontal_range_m
    if settings.range_model == "horizontal":
        if lower_range <= minimum:
            raise GeometryError("singular_projection")
        return range_m, lower_range, upper_range

    if settings.tag_height_m is None:
        raise GeometryError("invalid_settings")
    dz = settings.tag_height_m - sensor.translation_m[2]
    # 센서 설치 Z 오차도 높이 차의 불확실성에 더한다. XY 설치 오차는 별도로 상계한다.
    height_error = settings.tag_height_tolerance_m + settings.mount_translation_error_m
    dz_lower, dz_upper = dz - height_error, dz + height_error
    abs_dz_upper = max(abs(dz_lower), abs(dz_upper))
    abs_dz_lower = 0.0 if dz_lower <= 0 <= dz_upper else min(abs(dz_lower), abs(dz_upper))

    # 차의 곱 형태를 사용해 수직에 가까운 제곱값끼리의 소거를 줄인다.
    nominal_squared = (range_m - abs(dz)) * (range_m + abs(dz))
    lower_squared = (lower_range - abs_dz_upper) * (lower_range + abs_dz_upper)
    upper_squared = (upper_range - abs_dz_lower) * (upper_range + abs_dz_lower)
    # 구간 일부만 성립해도 위치를 만들지 않는다. 최소 수평거리 경계 접촉도 거부한다.
    if (lower_range <= 0 or nominal_squared <= minimum * minimum
            or lower_squared <= minimum * minimum or upper_squared <= 0):
        raise GeometryError("singular_projection")
    return math.sqrt(nominal_squared), math.sqrt(lower_squared), math.sqrt(upper_squared)


def project_observation(observation: dict, sensor: SensorConfig,
                        settings: EstimationConfig) -> dict:
    """센서 로컬의 왼쪽 양수 방위각을 장착 yaw·XY 이동으로 base_link에 투영한다.

    수신 세션·순번·현재 시각에 대한 만료 검증은 상위 추정기가 담당한다.
    이 함수도 무효 플래그, 잘못된 입력 출처와 비유한 측정은 직접 거부한다.
    """
    _settings(settings)
    if not isinstance(sensor, SensorConfig):
        raise GeometryError("invalid_settings")
    if not isinstance(observation, dict) or any(
            observation.get(key) is not True for key in ("valid", "measurement_valid", "id_valid")):
        raise GeometryError("invalid_observation")
    if observation.get("sensor_id") != sensor.sensor_id or observation.get("frame_id") != sensor.frame_id:
        raise GeometryError("sensor_mismatch")
    range_m = _finite(observation.get("range_m"), "invalid_observation")
    bearing = _finite(observation.get("bearing_rad"), "invalid_observation")
    if not .05 <= range_m <= 20 or not -math.pi <= bearing <= math.pi:
        raise GeometryError("invalid_observation")
    horizontal, lower, upper = _horizontal_interval(range_m, sensor, settings)
    radius_error = max(horizontal - lower, upper - horizontal)
    angular_error = math.radians(settings.bearing_error_deg + settings.mount_yaw_error_deg)
    # 현 설정 상한의 합은 90도 이하이며 현 길이는 그 각도 구간 전체를 포함한다.
    error = radius_error + 2 * upper * math.sin(angular_error / 2) + settings.mount_translation_error_m
    angle = bearing + math.radians(sensor.yaw_deg)
    x = sensor.translation_m[0] + horizontal * math.cos(angle)
    y = sensor.translation_m[1] + horizontal * math.sin(angle)
    if not all(math.isfinite(value) for value in (x, y, error)) or error < 0:
        raise GeometryError("invalid_geometry")
    return {"position_xy_m": [x, y], "error_radius_m": error}


def _rx_time(observation: dict) -> int:
    value = observation.get("rx_monotonic_ns")
    if type(value) is not int or not 0 <= value <= MAX_MONOTONIC_NS:
        raise GeometryError("invalid_timestamp")
    return value


def fuse_pair(left: dict, right: dict, collector_config: CollectorConfig,
              settings: EstimationConfig) -> dict:
    """두 위치의 중점과 잔차를 계산하고 관측 시각 차에 따른 이동 여유를 더한다.

    최신 호스트 수신 시각을 기준으로 삼되 관측값을 이동 보상하지 않는다.
    입력의 세션·식별·만료 일관성은 호출하는 추정기가 먼저 검증해야 한다.
    """
    _settings(settings)
    if not isinstance(collector_config, CollectorConfig):
        raise GeometryError("invalid_settings")
    if not isinstance(left, dict) or not isinstance(right, dict):
        raise GeometryError("invalid_observation")
    left_projected = project_observation(left, collector_config.sensors["uwb_left"], settings)
    right_projected = project_observation(right, collector_config.sensors["uwb_right"], settings)
    left_time, right_time = _rx_time(left), _rx_time(right)
    skew_ns = abs(left_time - right_time)
    if skew_ns > settings.max_pair_skew_ms * 1_000_000:
        raise GeometryError("pair_skew")
    lx, ly = left_projected["position_xy_m"]
    rx, ry = right_projected["position_xy_m"]
    residual = math.hypot(lx - rx, ly - ry)
    skew_seconds = skew_ns / 1_000_000_000
    if residual > settings.max_position_disagreement_m + settings.max_relative_speed_mps * skew_seconds:
        raise GeometryError("inconsistent_positions")
    # 공통 지연에 대한 가정도 별도 여유에 넣으며 독립 가우시안 평균으로 오차를 줄이지 않는다.
    motion_pad = settings.max_relative_speed_mps * (
        skew_seconds + settings.max_host_rx_delay_ms / 1000)
    error = max(settings.position_error_floor_m,
                max(left_projected["error_radius_m"], right_projected["error_radius_m"])
                + residual / 2 + motion_pad)
    return {
        "position_xy_m": [(lx + rx) / 2, (ly + ry) / 2],
        "tag_height_m": settings.tag_height_m if settings.range_model == "slant_known_height" else None,
        "error_radius_m": error,
        "pair_residual_m": residual,
        "rx_skew_ns": skew_ns,
        "reference_monotonic_ns": max(left_time, right_time),
        "alignment_method": ALIGNMENT_METHOD,
    }
