"""UWB 게이트 안의 유일한 LiDAR 군집만 연관하며 사람 분류나 모터 허가를 만들지 않는다."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
import re
from threading import RLock

from smartcart_uwb.config import CollectorConfig
from smartcart_uwb.estimator import ESTIMATE_SCHEMA
from smartcart_uwb.geometry import ALIGNMENT_METHOD
from smartcart_uwb.observation import MAX_MONOTONIC_NS
from .config import TrackingConfig

TRACK_SCHEMA = "smartcart.target_track.v1"
CLUSTERS_SCHEMA = "smartcart.lidar_clusters.v1"
_UINT64 = (1 << 64) - 1
_CLUSTER_FRAME_FIELDS = {
    "schema", "sensor_id", "frame_id", "input_frame_id", "adapter_session", "stream_epoch",
    "sample_sequence", "source_mode", "timestamp_source", "rx_monotonic_ns",
    "valid_until_monotonic_ns", "valid", "invalid_reason", "clusters", "drive_authorized", "provisional",
}
_CLUSTER_FIELDS = {"cluster_id", "centroid_xy_m", "radius_m", "width_m", "point_count",
                   "first_ray", "last_ray", "error_radius_m"}
_UWB_FIELDS = {
    "schema", "valid", "invalid_reason", "frame_id", "position_xy_m", "tag_height_m",
    "error_radius_m", "error_at_reference_m", "estimate_age_ms", "pair_residual_m", "rx_skew_ns",
    "reference_monotonic_ns", "valid_until_monotonic_ns", "estimate_sequence", "adapter_session",
    "source_mode", "provisional", "range_model", "alignment_method", "drive_authorized", "samples",
}
_SAMPLE_FIELDS = {"stream_epoch", "sample_sequence", "rx_monotonic_ns", "device_sequence"}


class _Fault(ValueError):
    def __init__(self, reason="invalid_input"):
        self.reason = reason
        super().__init__(reason)


def _object(value, fields):
    if type(value) is not dict or len(value) != len(fields) or set(value) != fields:
        raise _Fault()
    return value


def _int(value, low=0, high=_UINT64):
    if type(value) is not int or not low <= value <= high:
        raise _Fault()
    return value


def _num(value, low=0, high=2000):
    if type(value) not in (int, float) or not low <= value <= high or not math.isfinite(value):
        raise _Fault()
    return float(value)


def _code(value, optional=False):
    if optional and value is None:
        return None
    if type(value) is not str or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,128}", value):
        raise _Fault()
    return value


def _xy(value):
    if type(value) not in (tuple, list) or len(value) != 2:
        raise _Fault()
    return [_num(item, -100, 100) for item in value]


def _distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


@dataclass
class _Seen:
    context: tuple
    sequence: int
    rx_ns: int | None
    value: dict

    @property
    def key(self):
        return self.context + (self.sequence,)


class TargetTracker:
    """최신 입력과 단일 상대 궤적만 보관한다. 예측은 유효한 측정 위치와 분리한다."""

    def __init__(self, config: CollectorConfig, settings: TrackingConfig, source_mode="replay"):
        if not isinstance(config, CollectorConfig) or not isinstance(settings, TrackingConfig):
            raise ValueError("검증된 장착·추적 설정이 필요합니다")
        if source_mode not in ("replay", "live"):
            raise ValueError("추적 출처는 replay 또는 live여야 합니다")
        self.config, self.settings, self.source_mode = config, settings, source_mode
        self._lock = RLock()
        self._now = None
        self._seen = {}
        self._retired = {"lidar": set(), "uwb": set()}
        self._barriers = {}
        self._fault_token = None
        self._had_pair = False
        self._ever_track = False
        self._uncertainty = None
        self._track_id = None
        self._next_id = 1
        self._track_sequence = 0
        self._anchor = None
        self._velocity = [0.0, 0.0]
        self._confirmed = False
        self._hits = 0
        self._confirmation_keys = None
        self._uncertainty = None
        self._current = None

    def _metadata(self):
        result = {}
        for name, seen in self._seen.items():
            result[name] = {"adapter_session": seen.context[0], "sequence": seen.sequence,
                            "reference_monotonic_ns": seen.rx_ns}
        return result

    def _prediction(self, now):
        if self._anchor is None or now is None:
            return None
        elapsed = now - self._anchor["reference"]
        if elapsed < 0 or elapsed >= self.settings.occlusion_timeout_ms * 1_000_000:
            return None
        seconds = elapsed / 1e9
        return [self._anchor["xy"][axis] + self._velocity[axis] * seconds for axis in (0, 1)]

    def _result(self, state, reason, *, candidates=0, valid=False, now=None):
        current = self._current if valid else None
        elapsed = (now - current["reference"]) / 1e9 if current is not None else 0
        return {
            "schema": TRACK_SCHEMA, "frame_id": "base_link", "state": state, "valid": valid,
            "invalid_reason": reason, "track_id": self._track_id,
            "position_xy_m": list(current["xy"]) if current is not None else None,
            "velocity_xy_mps": list(self._velocity) if current is not None else None,
            "predicted_position_xy_m": None if valid else self._prediction(now),
            "error_radius_m": (current["error"] + self.settings.max_relative_speed_mps * elapsed
                               if current is not None else None),
            "reference_monotonic_ns": (current["reference"] if current is not None else
                                        self._anchor["reference"] if self._anchor is not None else None),
            "valid_until_monotonic_ns": current["deadline"] if current is not None else 0,
            "confirmation_count": self._hits, "candidate_count": candidates,
            "track_sequence": self._track_sequence, "source_mode": self.source_mode,
            "provisional": True, "drive_authorized": False, "inputs": self._metadata(),
            "associated_cluster_id": current["cluster_id"] if current is not None else None,
            "association_distance_m": current["association_distance"] if current is not None else None,
            "association_gate_m": current["association_gate"] if current is not None else None,
        }

    def _barrier(self):
        self._barriers.update({name: seen.key for name, seen in self._seen.items()})

    def _clear_track(self):
        self._track_id = None
        self._anchor = self._current = None
        self._velocity = [0.0, 0.0]
        self._hits = 0
        self._confirmed = False
        self._confirmation_keys = None
        self._uncertainty = None

    def _hard_fault(self, reason, source=None, state="error"):
        seen = self._seen.get(source)
        boundary = (seen.key if seen is not None else
                    tuple((name, item.key) for name, item in sorted(self._seen.items())))
        token = (reason, source, boundary)
        if token != self._fault_token:
            self._barrier()
            self._fault_token = token
        self._clear_track()
        return self._result(state, reason)

    def reset(self, reason="reset"):
        with self._lock:
            try:
                _code(reason)
            except _Fault:
                reason = "invalid_reset_reason"
            self._barrier()
            self._clear_track()
            self._fault_token = None
            return self._result("waiting", reason)

    def _common(self, value):
        if value["source_mode"] != self.source_mode:
            raise _Fault("source_mode_mismatch")
        if value["drive_authorized"] is not False or type(value["valid"]) is not bool:
            raise _Fault()
        if type(value["provisional"]) is not bool:
            raise _Fault()
        _code(value["invalid_reason"], optional=True)
        if value["valid"] != (value["invalid_reason"] is None):
            raise _Fault()

    def _lidar(self, value, now):
        frame = _object(value, _CLUSTER_FRAME_FIELDS)
        self._common(frame)
        if (frame["schema"] != CLUSTERS_SCHEMA or frame["sensor_id"] != "lidar_center"
                or frame["frame_id"] != "base_link" or frame["input_frame_id"] != self.config.lidar.frame_id
                or frame["timestamp_source"] != "HOST_RX" or frame["provisional"] is not True):
            raise _Fault()
        session = _code(frame["adapter_session"])
        epoch = _int(frame["stream_epoch"])
        sequence = _int(frame["sample_sequence"], 1 if frame["valid"] else 0)
        rx = _int(frame["rx_monotonic_ns"], high=MAX_MONOTONIC_NS)
        deadline = _int(frame["valid_until_monotonic_ns"], high=MAX_MONOTONIC_NS)
        if rx > now or deadline > rx + self.settings.lidar_timeout_ms * 1_000_000:
            raise _Fault("invalid_timestamp")
        if frame["valid"] and deadline <= rx:
            raise _Fault()
        clusters = frame["clusters"]
        if type(clusters) is not list or len(clusters) > self.settings.max_clusters:
            raise _Fault()
        if not frame["valid"] and clusters:
            raise _Fault()
        identifiers = set()
        for cluster in clusters:
            _object(cluster, _CLUSTER_FIELDS)
            identifier = _int(cluster["cluster_id"], high=self.settings.max_clusters - 1)
            if identifier in identifiers:
                raise _Fault()
            identifiers.add(identifier)
            _xy(cluster["centroid_xy_m"])
            radius = _num(cluster["radius_m"], 0, 100)
            width = _num(cluster["width_m"], self.settings.min_cluster_width_m, self.settings.max_cluster_width_m)
            if radius > width + 1e-8 or width > 2 * math.sqrt(2) * radius + 1e-8:
                raise _Fault()
            _num(cluster["error_radius_m"], 0, 100)
            _int(cluster["point_count"], self.settings.min_cluster_points, self.settings.max_cluster_points)
            _int(cluster["first_ray"], high=self.settings.max_rays - 1)
            _int(cluster["last_ray"], high=self.settings.max_rays - 1)
        return _Seen((session, epoch), sequence, rx, deepcopy(frame))

    def _uwb(self, value, now):
        if type(value) is dict and value.get("schema") == "smartcart.single_uwb_estimate.v1":
            return self._single_uwb(value, now)
        estimate = _object(value, _UWB_FIELDS)
        self._common(estimate)
        if estimate["schema"] != ESTIMATE_SCHEMA or estimate["frame_id"] != "base_link":
            raise _Fault()
        sequence = _int(estimate["estimate_sequence"])
        session = _code(estimate["adapter_session"], optional=not estimate["valid"])
        if estimate["range_model"] not in ("unknown", "horizontal", "slant_known_height"):
            raise _Fault()
        samples = estimate["samples"]
        if type(samples) is not dict or len(samples) > 2 or set(samples) - {"uwb_left", "uwb_right"}:
            raise _Fault()
        epochs = []
        for name in ("uwb_left", "uwb_right"):
            if name not in samples:
                epochs.append(None)
                continue
            sample = _object(samples[name], _SAMPLE_FIELDS)
            epochs.append(_int(sample["stream_epoch"]))
            _int(sample["sample_sequence"], 1)
            if _int(sample["rx_monotonic_ns"], high=MAX_MONOTONIC_NS) > now:
                raise _Fault("invalid_timestamp")
            if sample["device_sequence"] is not None:
                _int(sample["device_sequence"], high=255)
            elif estimate["valid"]:
                raise _Fault()
        deadline = _int(estimate["valid_until_monotonic_ns"], high=MAX_MONOTONIC_NS)
        if not estimate["valid"]:
            fields = ("position_xy_m", "tag_height_m", "error_radius_m", "error_at_reference_m", "estimate_age_ms",
                      "pair_residual_m", "rx_skew_ns", "reference_monotonic_ns", "alignment_method")
            if deadline != 0 or any(estimate[field] is not None for field in fields):
                raise _Fault()
            return _Seen((session, *epochs), sequence, None, deepcopy(estimate))
        _int(sequence, 1)
        if len(samples) != 2 or estimate["range_model"] == "unknown" or estimate["alignment_method"] != ALIGNMENT_METHOD:
            raise _Fault()
        _xy(estimate["position_xy_m"])
        if estimate["range_model"] == "slant_known_height":
            _num(estimate["tag_height_m"], -5, 5)
        elif estimate["tag_height_m"] is not None:
            raise _Fault()
        reference = _int(estimate["reference_monotonic_ns"], high=MAX_MONOTONIC_NS)
        times = [samples[name]["rx_monotonic_ns"] for name in ("uwb_left", "uwb_right")]
        if reference != max(times) or reference > now or not reference < deadline <= min(times) + self.config.timeout_ms * 1_000_000:
            raise _Fault("invalid_timestamp")
        if _int(estimate["rx_skew_ns"], high=MAX_MONOTONIC_NS) != abs(times[0] - times[1]):
            raise _Fault()
        error = _num(estimate["error_radius_m"])
        base_error = _num(estimate["error_at_reference_m"])
        age_ms = _num(estimate["estimate_age_ms"], 0, 5000)
        if base_error > error or age_ms > (now - reference) / 1e6 + 1e-6:
            raise _Fault()
        _num(estimate["pair_residual_m"], 0, 100)
        return _Seen((session, *epochs), sequence, reference, deepcopy(estimate))

    def _single_uwb(self, value, now):
        fields = {"schema", "valid", "invalid_reason", "position_xy_m", "error_radius_m",
                  "valid_until_monotonic_ns", "sensor_id", "source_mode", "provisional", "drive_authorized",
                  "frame_id", "reference_monotonic_ns", "sample_sequence", "stream_epoch", "adapter_session", "estimate_age_ms"}
        value = _object(value, fields)
        self._common(value)
        if value["sensor_id"] != "uwb_left" or value["frame_id"] != "base_link" or value["provisional"] is not True:
            raise _Fault()
        session = _code(value["adapter_session"], optional=not value["valid"])
        epoch = value["stream_epoch"]
        if epoch is not None:
            _int(epoch)
        seq = _int(value["sample_sequence"], 1 if value["valid"] else 0)
        deadline = _int(value["valid_until_monotonic_ns"], high=MAX_MONOTONIC_NS)
        if not value["valid"]:
            if deadline or any(value[k] is not None for k in ("position_xy_m", "error_radius_m", "reference_monotonic_ns", "estimate_age_ms")):
                raise _Fault()
            return _Seen((session, epoch), seq, None, deepcopy(value))
        if epoch is None:
            raise _Fault()
        _xy(value["position_xy_m"])
        _num(value["error_radius_m"])
        rx = _int(value["reference_monotonic_ns"], high=MAX_MONOTONIC_NS)
        age = _num(value["estimate_age_ms"], 0, 5000)
        if rx > now or not rx < deadline <= rx + self.config.timeout_ms * 1_000_000 or age > (now - rx) / 1e6 + 1e-6:
            raise _Fault("invalid_timestamp")
        return _Seen((session, epoch), seq, rx, deepcopy(value))

    def _accept_inputs(self, prepared):
        changed = False
        for name, seen in prepared.items():
            old = self._seen.get(name)
            if old is None:
                continue
            if seen.context != old.context:
                if (not self._ever_track and not old.value["valid"] and
                        (old.context[0] is None or (seen.context[0] == old.context[0] and all(
                            before is None or after == before
                            for after, before in zip(seen.context[1:], old.context[1:]))))):
                    continue
                if seen.context[0] == old.context[0]:
                    for new_epoch, old_epoch in zip(seen.context[1:], old.context[1:]):
                        if new_epoch is not None and old_epoch is not None and new_epoch < old_epoch:
                            raise _Fault("epoch_regression")
                    # 무효 UWB 결과의 누락 epoch는 전환으로 해석하지 않는다.
                    if not seen.value["valid"]:
                        continue
                else:
                    if seen.context[0] in self._retired[name]:
                        raise _Fault("retired_session")
                    if len(self._retired[name]) >= 32:
                        raise _Fault("session_limit")
                changed = True
                continue
            if seen.sequence < old.sequence:
                raise _Fault("sequence_regression")
            if seen.rx_ns is not None and old.rx_ns is not None and seen.rx_ns < old.rx_ns:
                raise _Fault("invalid_timestamp")
            if seen.sequence == old.sequence:
                if seen.value["valid"] and not old.value["valid"]:
                    raise _Fault("sample_reuse")
                if seen.value["valid"] and old.value["valid"]:
                    a, b = deepcopy(seen.value), deepcopy(old.value)
                    if name == "uwb":
                        for field in ("error_radius_m", "estimate_age_ms"):
                            if a[field] < b[field]:
                                raise _Fault("sample_reuse")
                            a.pop(field)
                            b.pop(field)
                    if a != b:
                        raise _Fault("sample_reuse")
            elif name == "uwb" and seen.value["valid"] and old.value["valid"]:
                for sensor in (tuple(seen.value.get("samples", {})) if "samples" in seen.value else ()):
                    if seen.value["samples"][sensor]["sample_sequence"] <= old.value["samples"][sensor]["sample_sequence"]:
                        raise _Fault("sample_reuse")
        for name, seen in prepared.items():
            old = self._seen.get(name)
            if old is not None and seen.context[0] != old.context[0] and old.context[0] is not None:
                self._retired[name].add(old.context[0])
        self._seen.update(prepared)
        return changed

    def _fresh(self, key, previous):
        return previous is None or (key[:-1] == previous[:-1] and key[-1] > previous[-1])

    def _continuity(self, point, reference):
        if self._anchor is None:
            return True
        elapsed = reference - self._anchor["reference"]
        if elapsed < 0:
            return False
        seconds = elapsed / 1e9
        predicted = [self._anchor["xy"][axis] + self._velocity[axis] * seconds for axis in (0, 1)]
        limit = self.settings.max_relative_speed_mps * seconds + self.settings.continuity_slack_m
        return _distance(point, self._anchor["xy"]) <= limit and _distance(point, predicted) <= limit

    def _update_anchor(self, point, reference, lidar_key):
        if self._anchor is not None and lidar_key == self._anchor["lidar_key"]:
            return
        if self._anchor is not None and reference > self._anchor["reference"]:
            seconds = (reference - self._anchor["reference"]) / 1e9
            measured = [(point[axis] - self._anchor["xy"][axis]) / seconds for axis in (0, 1)]
            speed = math.hypot(*measured)
            if speed > self.settings.max_relative_speed_mps:
                measured = [value * self.settings.max_relative_speed_mps / speed for value in measured]
            alpha = self.settings.velocity_alpha
            self._velocity = [alpha * measured[axis] + (1 - alpha) * self._velocity[axis] for axis in (0, 1)]
        self._anchor = {"xy": list(point), "reference": reference, "lidar_key": lidar_key}
        self._track_sequence += 1

    def update(self, cluster_frame: dict | None, uwb_estimate: dict | None, now_ns: int) -> dict:
        with self._lock:
            try:
                _int(now_ns, high=MAX_MONOTONIC_NS)
                if self._now is not None and now_ns < self._now:
                    return self._hard_fault("non_monotonic_time")
                self._now = now_ns
                prepared = {}
                if cluster_frame is not None:
                    prepared["lidar"] = self._lidar(cluster_frame, now_ns)
                if uwb_estimate is not None:
                    prepared["uwb"] = self._uwb(uwb_estimate, now_ns)
                changed = self._accept_inputs(prepared)
                if len(prepared) != 2:
                    if self._had_pair:
                        return self._hard_fault("missing_input")
                    return self._result("waiting", "missing_input")
                if not self._ever_track and any(
                        not item.value["valid"] and item.value["invalid_reason"] in (
                            "waiting_for_scan", "waiting_for_pair", "awaiting_fresh_pair")
                        for item in prepared.values()):
                    return self._result("waiting", "waiting_for_inputs")
                self._had_pair = True
                if changed:
                    return self._hard_fault("source_reset")
                lidar, uwb = prepared["lidar"], prepared["uwb"]
                if not lidar.value["valid"]:
                    return self._hard_fault("lidar_invalid", "lidar")
                if not uwb.value["valid"]:
                    return self._hard_fault("uwb_invalid", "uwb")
                if now_ns >= lidar.value["valid_until_monotonic_ns"]:
                    return self._hard_fault("lidar_stale", "lidar")
                if now_ns >= uwb.value["valid_until_monotonic_ns"]:
                    return self._hard_fault("uwb_stale", "uwb")
                skew = abs(lidar.rx_ns - uwb.rx_ns)
                if skew > self.settings.max_sensor_skew_ms * 1_000_000:
                    return self._hard_fault("sensor_skew")
                additional_age = max(0, (now_ns - uwb.rx_ns) / 1e9 - uwb.value["estimate_age_ms"] / 1000)
                uwb_error = uwb.value["error_radius_m"] + self.settings.max_relative_speed_mps * additional_age
                if uwb_error > self.settings.max_uwb_error_m:
                    return self._hard_fault("uwb_uncertain", "uwb")
                keys = {name: seen.key for name, seen in prepared.items()}
                if any(not self._fresh(keys[name], barrier) for name, barrier in self._barriers.items()):
                    return self._result("waiting", "awaiting_fresh_inputs")
                self._fault_token = None
                candidates = []
                for cluster in lidar.value["clusters"]:
                    gate = min(self.settings.max_association_radius_m,
                               uwb_error + self.settings.tag_cluster_offset_m + cluster["error_radius_m"])
                    if _distance(cluster["centroid_xy_m"], uwb.value["position_xy_m"]) <= gate:
                        candidates.append(cluster)
                count = len(candidates)
                if count != 1:
                    self._current = None
                    if self._anchor is not None and self._prediction(now_ns) is None:
                        self._clear_track()
                        return self._result("lost", "occlusion_timeout", candidates=count)
                    if count > 1:
                        self._hits = 0
                        self._confirmed = False
                        if self._uncertainty != "ambiguous":
                            self._confirmation_keys = keys
                        self._uncertainty = "ambiguous"
                        return self._result("ambiguous", "ambiguous_candidates", candidates=count, now=now_ns)
                    self._hits = 0
                    self._confirmed = False
                    if self._uncertainty != "coasting":
                        self._confirmation_keys = keys
                    self._uncertainty = "coasting"
                    return self._result("coasting" if self._anchor is not None else "waiting",
                                        "no_candidate", now=now_ns)
                point = candidates[0]["centroid_xy_m"]
                self._uncertainty = None
                if (self._anchor is not None and lidar.key == self._anchor["lidar_key"]
                        and list(point) != self._anchor["xy"]):
                    return self._hard_fault("association_changed", state="lost")
                if not self._continuity(point, lidar.rx_ns):
                    return self._hard_fault("continuity_jump", state="lost")
                if self._anchor is not None and self._prediction(now_ns) is None:
                    self._clear_track()
                if self._track_id is None:
                    self._track_id = self._next_id
                    self._next_id += 1
                    self._ever_track = True
                self._update_anchor(point, lidar.rx_ns, lidar.key)
                fresh_pair = self._confirmation_keys is None or all(
                    self._fresh(keys[name], self._confirmation_keys[name]) for name in ("lidar", "uwb"))
                if fresh_pair:
                    self._hits = min(self.settings.confirmation_hits, self._hits + 1)
                    self._confirmation_keys = keys
                self._confirmed = self._confirmed or self._hits >= self.settings.confirmation_hits
                deadline = min(lidar.value["valid_until_monotonic_ns"], uwb.value["valid_until_monotonic_ns"])
                if not self._confirmed:
                    self._current = None
                    return self._result("acquiring", "confirming", candidates=1, now=now_ns)
                if self._current is not None and self._current["lidar_key"] == lidar.key:
                    self._current["deadline"] = min(self._current["deadline"], deadline)
                    self._current["association_distance"] = _distance(point, uwb.value["position_xy_m"])
                    self._current["association_gate"] = min(self.settings.max_association_radius_m,
                        uwb_error + self.settings.tag_cluster_offset_m + candidates[0]["error_radius_m"])
                else:
                    self._current = {"xy": list(point), "reference": lidar.rx_ns, "deadline": deadline,
                                     "lidar_key": lidar.key,
                                     "cluster_id": candidates[0]["cluster_id"],
                                     "association_distance": _distance(point, uwb.value["position_xy_m"]),
                                     "association_gate": min(self.settings.max_association_radius_m,
                                         uwb_error + self.settings.tag_cluster_offset_m + candidates[0]["error_radius_m"]),
                                     "error": max(uwb_error, candidates[0]["error_radius_m"])
                                              + self.settings.tag_cluster_offset_m
                                              + self.settings.max_relative_speed_mps * skew / 1e9}
                if now_ns >= self._current["deadline"]:
                    return self._hard_fault("track_stale")
                self._barriers.clear()
                return self._result("tracking", None, candidates=1, valid=True, now=now_ns)
            except _Fault as error:
                return self._hard_fault(error.reason)
            except Exception:
                return self._hard_fault("tracking_error")
