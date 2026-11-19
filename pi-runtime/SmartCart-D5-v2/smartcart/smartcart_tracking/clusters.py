"""연속 LiDAR 반사점을 제한된 평면 묶음으로 표현한다. 사람 분류나 보간은 하지 않는다."""
from __future__ import annotations

import math
import re

from smartcart_radar.core import normalise_scan, project_scan
from smartcart_uwb.config import CollectorConfig
from smartcart_uwb.observation import MAX_MONOTONIC_NS

from .config import TrackingConfig


OBSERVATION_SCHEMA = "smartcart.lidar_observation.v1"
CLUSTERS_SCHEMA = "smartcart.lidar_clusters.v1"
_OBSERVATION_KEYS = {"schema", "sensor_id", "frame_id", "adapter_session", "stream_epoch",
                     "sample_sequence", "source_mode", "timestamp_source", "rx_monotonic_ns",
                     "valid_until_monotonic_ns", "valid", "invalid_reason", "scan"}
_SCAN_KEYS = {"angle_min", "angle_max", "angle_increment", "range_min", "range_max",
              "ranges", "scan_time", "time_increment"}
_COMMON_KEYS = _OBSERVATION_KEYS - {"schema", "frame_id", "valid", "invalid_reason", "scan"}
_TAU = 2 * math.pi


class _Rejected(ValueError):
    def __init__(self, reason):
        self.reason = reason


def _number(value):
    if type(value) not in (int, float):
        raise _Rejected("invalid_scan")
    try:
        result = float(value)
    except (OverflowError, ValueError):
        raise _Rejected("invalid_scan") from None
    if not math.isfinite(result):
        raise _Rejected("invalid_scan")
    return result


def _integer(value, *, minimum=0):
    if type(value) is not int or not minimum <= value <= MAX_MONOTONIC_NS:
        raise _Rejected("invalid_metadata")
    return value


def _code(value):
    return type(value) is str and re.fullmatch(r"[A-Za-z0-9_.:-]{1,128}", value) is not None


def _blank(mounting):
    return {"schema": CLUSTERS_SCHEMA, "sensor_id": "lidar_center", "frame_id": "base_link",
            "input_frame_id": mounting.lidar.frame_id if isinstance(mounting, CollectorConfig) else None,
            "adapter_session": None, "stream_epoch": 0, "sample_sequence": 0,
            "source_mode": None, "timestamp_source": "HOST_RX", "rx_monotonic_ns": 0,
            "valid_until_monotonic_ns": 0, "valid": False, "invalid_reason": "invalid_observation",
            "clusters": [], "drive_authorized": False, "provisional": True}


def _metadata(observation, mounting, settings):
    if type(observation) is not dict or set(observation) != _OBSERVATION_KEYS:
        raise _Rejected("invalid_observation")
    if (observation["schema"] != OBSERVATION_SCHEMA or observation["sensor_id"] != "lidar_center"
            or observation["frame_id"] != mounting.lidar.frame_id
            or observation["source_mode"] not in ("live", "replay")
            or observation["timestamp_source"] != "HOST_RX"
            or not _code(observation["adapter_session"]) or type(observation["valid"]) is not bool):
        raise _Rejected("invalid_metadata")
    _integer(observation["stream_epoch"])
    sample = _integer(observation["sample_sequence"])
    rx = _integer(observation["rx_monotonic_ns"])
    deadline = _integer(observation["valid_until_monotonic_ns"])
    if observation["valid"]:
        if (sample < 1 or observation["invalid_reason"] is not None
                or deadline != rx + settings.lidar_timeout_ms * 1_000_000):
            raise _Rejected("invalid_metadata")
    elif deadline != 0 or not _code(observation["invalid_reason"]) or observation["scan"] is not None:
        raise _Rejected("invalid_metadata")
    return {key: observation[key] for key in _COMMON_KEYS}


def _scan(raw, settings):
    if type(raw) is not dict or set(raw) != _SCAN_KEYS:
        raise _Rejected("invalid_scan")
    ranges = raw["ranges"]
    if type(ranges) not in (list, tuple) or not 1 <= len(ranges) <= settings.max_rays:
        raise _Rejected("invalid_scan")
    for value in ranges:
        if value is not None and type(value) not in (int, float):
            raise _Rejected("invalid_scan")
        if type(value) is int:
            # 무한대 부동소수점은 무반사로 허용하지만 변환 불가능한 임의 큰 정수는 거부한다.
            _number(value)
    amin, amax, step = (_number(raw[key]) for key in ("angle_min", "angle_max", "angle_increment"))
    if (step == 0 or (amax - amin) / step < 0 or abs(amax - amin) > _TAU + 1e-6
            or not math.isclose(amax - amin, step * (len(ranges) - 1), rel_tol=1e-6, abs_tol=1e-9)):
        raise _Rejected("invalid_scan")
    expected = round((amax - amin) / step) + 1
    if expected != len(ranges):
        raise _Rejected("invalid_scan")
    scan_time, time_increment = (_number(raw[key]) for key in ("scan_time", "time_increment"))
    if scan_time < 0 or time_increment < 0:
        raise _Rejected("invalid_scan")
    duration = max(scan_time, time_increment * (len(ranges) - 1))
    maximum_duration = settings.max_scan_duration_ms / 1000
    if duration > maximum_duration:
        raise _Rejected("scan_duration")
    if duration == 0:
        duration = maximum_duration
    for key in ("range_min", "range_max"):
        _number(raw[key])
    # 기존 정규화가 NaN/Inf를 null로 바꾸되 원래 ray 인덱스를 보존한다.
    scan = normalise_scan(raw, kind="replay", label="추적 기하 입력")
    if scan["truncated"] or scan["stats"]["expected_rays"] != len(scan["ranges"]):
        raise _Rejected("invalid_scan")
    # 최대 거리 밖의 점도 인덱스를 삭제하지 않아 연결 사슬이 그 자리에서 끊긴다.
    scan["range_max"] = min(scan["range_max"], settings.max_range_m)
    return scan, duration


def _near(first, second, settings, step):
    threshold = settings.cluster_gap_base_m + settings.cluster_gap_scale * min(first["range"], second["range"]) * abs(step)
    return math.hypot(first["x"] - second["x"], first["y"] - second["y"]) <= threshold


def _groups(points, scan, settings):
    runs = []
    previous = None
    for point in points:
        if (previous is None or point["index"] != previous["index"] + 1
                or not _near(previous, point, settings, scan["angle_increment"])):
            runs.append([])
        runs[-1].append(point)
        previous = point
    count = len(scan["ranges"])
    step = abs(scan["angle_increment"])
    span = abs(scan["angle_max"] - scan["angle_min"])
    # 끝점 포함형과 끝점 미포함형의 실제 한 바퀴 스캔만 경계 연결을 허용한다.
    tolerance = max(1e-8, step * .001)
    full_circle = (count >= 3 and (abs(span - _TAU) <= tolerance
                                  or abs(count * step - _TAU) <= tolerance))
    if (full_circle and len(runs) > 1 and runs[0][0]["index"] == 0
            and runs[-1][-1]["index"] == count - 1
            and _near(runs[-1][-1], runs[0][0], settings, scan["angle_increment"])):
        runs = [runs[-1] + runs[0], *runs[1:-1]]
    return runs


def _feature(points, cluster_id, duration, settings):
    count = len(points)
    if not settings.min_cluster_points <= count <= settings.max_cluster_points:
        return None
    xs, ys = [point["x"] for point in points], [point["y"] for point in points]
    width = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
    if not settings.min_cluster_width_m <= width <= settings.max_cluster_width_m:
        return None
    cx, cy = math.fsum(xs) / count, math.fsum(ys) / count
    radius = max(math.hypot(x - cx, y - cy) for x, y in zip(xs, ys))
    maximum_range = max(point["range"] for point in points)
    error = (settings.lidar_range_error_m + settings.mount_translation_error_m
             + 2 * maximum_range * math.sin(math.radians(settings.mount_yaw_error_deg) / 2)
             + settings.max_relative_speed_mps * (duration + settings.max_host_rx_delay_ms / 1000))
    return {"cluster_id": cluster_id, "centroid_xy_m": [cx, cy], "radius_m": radius,
            "width_m": width, "point_count": count,
            "first_ray": points[0]["index"], "last_ray": points[-1]["index"], "error_radius_m": error}


def cluster_scan(observation: dict, mounting: CollectorConfig, settings: TrackingConfig) -> dict:
    """오류를 빈 무효 결과로 바꾸고 상한 내 군집 특징만 반환한다.

    반경은 반사점과 설정 가정의 경계이며 사람의 몸체나 안전한 자유 공간을 뜻하지 않는다.
    현재 시각에 대한 만료 검사와 이전 결과 폐기는 상위 추적기가 담당한다.
    """
    output = _blank(mounting)
    try:
        if not isinstance(mounting, CollectorConfig) or not isinstance(settings, TrackingConfig):
            raise _Rejected("invalid_settings")
        output.update(_metadata(observation, mounting, settings))
        if not observation["valid"]:
            raise _Rejected(observation["invalid_reason"])
        scan, duration = _scan(observation["scan"], settings)
        offset = mounting.lidar.translation_m
        # 기존 투영의 카트 원점 반경 자르기가 센서 기준 최대거리의 점을 누락하지 않게 한다.
        projection_radius = settings.max_range_m + math.hypot(offset[0], offset[1]) + 1e-9
        points = project_scan(scan, projection_radius, mounting.lidar.yaw_deg, offset[0], offset[1])["points"]
        clusters = []
        for group in _groups(points, scan, settings):
            feature = _feature(group, len(clusters), duration, settings)
            if feature is not None:
                clusters.append(feature)
                if len(clusters) > settings.max_clusters:
                    raise _Rejected("cluster_overflow")
        output.update(valid=True, invalid_reason=None, clusters=clusters)
    except _Rejected as error:
        output.update(valid=False, invalid_reason=error.reason, clusters=[])
    except Exception:
        # 이전 점이나 사용자 입력·예외 내용을 유효 결과 또는 로그로 노출하지 않는다.
        output.update(valid=False, invalid_reason="invalid_scan", clusters=[])
    return output
