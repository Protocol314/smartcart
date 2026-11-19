"""최신 좌우 관측만으로 조건부 위치를 추정하며 모든 실패에서 위치를 즉시 지운다."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
import re
from threading import RLock

from .collector import STATUS_SCHEMA
from .config import CollectorConfig, SENSOR_IDS
from .estimation_config import EstimationConfig
from .geometry import ALIGNMENT_METHOD, GeometryError, fuse_pair
from .observation import MAX_MONOTONIC_NS, OBSERVATION_SCHEMA

ESTIMATE_SCHEMA = "smartcart.uwb_tag_estimate.v1"
_UINT64_MAX = (1 << 64) - 1
_SNAPSHOT_FIELDS = {"schema", "source_mode", "adapter_session", "drive_authorized", "sensors"}
_STATE_FIELDS = {"state", "connected", "stream_epoch", "latest", "last_error"}
_OBSERVATION_FIELDS = {
    "schema", "sensor_id", "frame_id", "tag_address", "logical_target_id", "device_sequence",
    "sample_sequence", "stream_epoch", "adapter_session", "source_mode", "timestamp_source",
    "rx_monotonic_ns", "source_rx_monotonic_ns", "valid_until_monotonic_ns", "device_timestamp_ns",
    "range_m", "bearing_rad", "raw_angle_deg", "first_path_power", "rx_level", "measurement_valid",
    "id_valid", "valid", "invalid_reason", "mounting_verified", "calibration_profile", "drive_authorized",
}
_GEOMETRY_FIELDS = {"position_xy_m", "tag_height_m", "error_radius_m", "pair_residual_m",
                    "rx_skew_ns", "reference_monotonic_ns", "alignment_method"}
_UNSTABLE_FIELDS = {"valid", "invalid_reason"}


class _SnapshotError(ValueError):
    def __init__(self, reason="invalid_snapshot"):
        self.reason = reason
        super().__init__(reason)


def _object(value, fields):
    if type(value) is not dict or len(value) != len(fields) or set(value) != fields:
        raise _SnapshotError()
    return value


def _integer(value, minimum=0, maximum=_UINT64_MAX):
    if type(value) is not int or not minimum <= value <= maximum:
        raise _SnapshotError()
    return value


def _number(value, minimum, maximum):
    if type(value) not in (int, float) or not minimum <= value <= maximum or not math.isfinite(value):
        raise _SnapshotError()
    return float(value)


def _code(value, *, optional=False):
    if optional and value is None:
        return None
    if type(value) is not str or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,128}", value):
        raise _SnapshotError()
    return value


@dataclass
class _Seen:
    epoch: int
    sequence: int
    rx_ns: int
    fingerprint: tuple
    valid: bool
    metadata: dict


class TagEstimator:
    """오류 뒤 양쪽 새 표본을 요구하고 같은 쌍이나 tick으로 수명을 연장하지 않는다."""

    def __init__(self, config: CollectorConfig, settings: EstimationConfig, source_mode="replay"):
        if not isinstance(config, CollectorConfig) or not isinstance(settings, EstimationConfig):
            raise ValueError("검증된 수집·추정 설정이 필요합니다")
        if source_mode not in ("live", "replay"):
            raise ValueError("추정 출처는 live 또는 replay여야 합니다")
        self.config = config
        self.settings = settings
        self.source_mode = source_mode
        self._lock = RLock()
        self._now_ns = None
        self._session = None
        self._retired_sessions = set()
        self._epochs = {}
        self._seen: dict[str, _Seen] = {}
        self._device_sequences = {}
        self._barriers = {}
        self._fault_tokens = {}
        self._success_keys = None
        self._last_attempt = None
        self._geometry_key = None
        self._geometry_value = None
        self._current = None
        self._estimate_sequence = 0

    def _provisional(self):
        return (not self.settings.assumptions_verified
                or any(not sensor.mounting_verified or sensor.calibration_profile == "raw_unverified"
                       for sensor in self.config.sensors.values()))

    def _samples(self):
        return {sensor_id: deepcopy(seen.metadata) for sensor_id, seen in self._seen.items()}

    def _invalid(self, reason):
        self._current = None
        return {
            "schema": ESTIMATE_SCHEMA, "valid": False, "invalid_reason": reason,
            "frame_id": "base_link", "position_xy_m": None, "tag_height_m": None,
            "error_radius_m": None, "error_at_reference_m": None, "estimate_age_ms": None,
            "pair_residual_m": None, "rx_skew_ns": None, "reference_monotonic_ns": None,
            "valid_until_monotonic_ns": 0, "estimate_sequence": self._estimate_sequence,
            "adapter_session": self._session, "source_mode": self.source_mode,
            "provisional": self._provisional(), "range_model": self.settings.range_model,
            "alignment_method": None, "drive_authorized": False, "samples": self._samples(),
        }

    def _barrier(self, states=None):
        for sensor_id in SENSOR_IDS:
            epoch = self._epochs.get(sensor_id, 0)
            sample = self._seen[sensor_id].sequence if sensor_id in self._seen else 0
            if states is not None:
                latest = states[sensor_id]["latest"]
                epoch = states[sensor_id]["stream_epoch"]
                sample = latest["sample_sequence"] if latest is not None else sample
            previous = self._barriers.get(sensor_id)
            if previous is None or previous[0] != epoch or sample > previous[1]:
                self._barriers[sensor_id] = (epoch, sample)

    def reset(self, reason="reset"):
        """외부 입력 경로의 실패도 현재 쌍을 소비하고 양쪽 새 표본을 요구한다."""
        with self._lock:
            try:
                _code(reason)
            except _SnapshotError:
                reason = "invalid_reset_reason"
            self._barrier()
            self._last_attempt = None
            self._geometry_key = self._geometry_value = None
            return self._invalid(reason)

    def _validate_observation(self, value, sensor_id, session, epoch, now_ns):
        observation = _object(value, _OBSERVATION_FIELDS)
        sensor = self.config.sensors[sensor_id]
        if (observation["schema"] != OBSERVATION_SCHEMA or observation["sensor_id"] != sensor_id
                or observation["frame_id"] != sensor.frame_id
                or observation["adapter_session"] != session
                or observation["source_mode"] != self.source_mode
                or observation["timestamp_source"] != "HOST_RX"
                or observation["device_timestamp_ns"] is not None
                or observation["calibration_profile"] != sensor.calibration_profile):
            raise _SnapshotError()
        for key in ("valid", "measurement_valid", "id_valid", "mounting_verified", "drive_authorized"):
            if type(observation[key]) is not bool:
                raise _SnapshotError()
        if observation["drive_authorized"] or observation["mounting_verified"] != sensor.mounting_verified:
            raise _SnapshotError()
        if _integer(observation["stream_epoch"]) != epoch:
            raise _SnapshotError()
        _integer(observation["sample_sequence"], 1)
        rx_ns = _integer(observation["rx_monotonic_ns"], maximum=MAX_MONOTONIC_NS)
        deadline = _integer(observation["valid_until_monotonic_ns"], maximum=MAX_MONOTONIC_NS)
        if rx_ns > now_ns or deadline > rx_ns + self.config.timeout_ms * 1_000_000:
            raise _SnapshotError("invalid_timestamp")
        original = observation["source_rx_monotonic_ns"]
        if original is not None:
            _integer(original, maximum=MAX_MONOTONIC_NS)
        _code(observation["invalid_reason"], optional=True)
        raw_fields = ("tag_address", "device_sequence", "range_m", "bearing_rad", "raw_angle_deg",
                      "first_path_power", "rx_level")
        available = observation["tag_address"] is not None
        if not available:
            if any(observation[key] is not None for key in raw_fields):
                raise _SnapshotError()
            measured = identified = False
        else:
            _integer(observation["tag_address"], maximum=65535)
            _integer(observation["device_sequence"], maximum=255)
            raw_angle = _integer(observation["raw_angle_deg"], -(1 << 31), (1 << 31) - 1)
            _integer(observation["first_path_power"], -(1 << 31), (1 << 31) - 1)
            _integer(observation["rx_level"], -(1 << 31), (1 << 31) - 1)
            distance = _number(observation["range_m"], 0, ((1 << 32) - 1) / 100)
            bearing = _number(observation["bearing_rad"], -40_000_000, 40_000_000)
            if not math.isclose(bearing, -math.radians(raw_angle), rel_tol=1e-12, abs_tol=1e-12):
                raise _SnapshotError()
            measured = .05 <= distance <= 20 and -180 <= raw_angle <= 180
            identified = observation["tag_address"] == self.config.allowed_tag_address
        if observation["measurement_valid"] != measured or observation["id_valid"] != identified:
            raise _SnapshotError()
        target = observation["logical_target_id"]
        if identified:
            if type(target) is not int or target != self.config.logical_target_id:
                raise _SnapshotError()
        elif target is not None:
            raise _SnapshotError()
        if observation["valid"]:
            if not measured or not identified or observation["invalid_reason"] is not None:
                raise _SnapshotError()
            if deadline != rx_ns + self.config.timeout_ms * 1_000_000:
                raise _SnapshotError()
        elif observation["invalid_reason"] is None:
            raise _SnapshotError()
        return observation

    def _validate_snapshot(self, snapshot, now_ns):
        value = _object(snapshot, _SNAPSHOT_FIELDS)
        if value["schema"] != STATUS_SCHEMA or value["drive_authorized"] is not False:
            raise _SnapshotError()
        if value["source_mode"] != self.source_mode:
            raise _SnapshotError("source_mode_mismatch")
        session = _code(value["adapter_session"])
        states = _object(value["sensors"], set(SENSOR_IDS))
        for sensor_id in SENSOR_IDS:
            state = _object(states[sensor_id], _STATE_FIELDS)
            if type(state["connected"]) is not bool or state["state"] not in ("waiting", "live", "stale", "error"):
                raise _SnapshotError()
            epoch = _integer(state["stream_epoch"])
            _code(state["last_error"], optional=True)
            latest = state["latest"]
            if latest is None:
                if state["state"] == "waiting":
                    if not state["connected"] or state["last_error"] is not None:
                        raise _SnapshotError()
                elif state["state"] != "error" or state["last_error"] is None:
                    raise _SnapshotError()
                continue
            observation = self._validate_observation(latest, sensor_id, session, epoch, now_ns)
            if state["state"] == "live":
                if not state["connected"] or not observation["valid"] or state["last_error"] is not None:
                    raise _SnapshotError()
            elif state["state"] == "waiting" or observation["valid"]:
                raise _SnapshotError()
            elif state["last_error"] != observation["invalid_reason"]:
                raise _SnapshotError()
        return session, states

    def _context(self, session, states):
        epochs = {sensor_id: states[sensor_id]["stream_epoch"] for sensor_id in SENSOR_IDS}
        if self._session is None:
            self._session, self._epochs = session, epochs
            return None
        if session != self._session:
            if session in self._retired_sessions:
                raise _SnapshotError("retired_session")
            # 세션 메타데이터도 유한하게 제한한다. 초과 시 새 추정기를 명시적으로 만들어야 한다.
            if len(self._retired_sessions) >= 32:
                raise _SnapshotError("session_limit")
            self._retired_sessions.add(self._session)
            self._session, self._epochs = session, epochs
            self._seen.clear()
            self._device_sequences.clear()
            self._barriers.clear()
            self._fault_tokens.clear()
            self._success_keys = self._last_attempt = None
            self._geometry_key = self._geometry_value = None
            self._barrier(states)
            return "session_changed"
        changed = False
        for sensor_id in SENSOR_IDS:
            if epochs[sensor_id] < self._epochs[sensor_id]:
                raise _SnapshotError("epoch_regression")
            if epochs[sensor_id] != self._epochs[sensor_id]:
                self._seen.pop(sensor_id, None)
                self._device_sequences.pop(sensor_id, None)
                changed = True
        self._epochs = epochs
        if changed:
            self._last_attempt = None
            self._geometry_key = self._geometry_value = None
            self._barrier(states)
            return "epoch_changed"
        return None

    def _observe(self, states):
        pending = {}
        device_sequences = dict(self._device_sequences)
        for sensor_id in SENSOR_IDS:
            observation = states[sensor_id]["latest"]
            if observation is None:
                continue
            epoch = observation["stream_epoch"]
            sequence = observation["sample_sequence"]
            fingerprint = tuple(observation[key] for key in sorted(_OBSERVATION_FIELDS - _UNSTABLE_FIELDS))
            previous = self._seen.get(sensor_id)
            if previous is not None and previous.epoch == epoch:
                if sequence < previous.sequence or observation["rx_monotonic_ns"] < previous.rx_ns:
                    raise _SnapshotError("sample_regression")
                if sequence == previous.sequence:
                    if fingerprint != previous.fingerprint or (not previous.valid and observation["valid"]):
                        raise _SnapshotError("sample_reuse")
                elif observation["valid"] and sensor_id in device_sequences:
                    delta = (observation["device_sequence"] - device_sequences[sensor_id]) & 255
                    if not 1 <= delta <= 127:
                        raise _SnapshotError("device_sequence_regression")
            if observation["valid"]:
                device_sequences[sensor_id] = observation["device_sequence"]
            metadata = {key: observation[key] for key in (
                "stream_epoch", "sample_sequence", "rx_monotonic_ns", "device_sequence")}
            pending[sensor_id] = _Seen(epoch, sequence, observation["rx_monotonic_ns"],
                                      fingerprint, observation["valid"], metadata)
        self._seen.update(pending)
        self._device_sequences = device_sequences

    def _health(self, states, now_ns):
        failure = None
        for sensor_id in SENSOR_IDS:
            state = states[sensor_id]
            observation = state["latest"]
            reason = None
            if not state["connected"]:
                reason = state["last_error"] or "disconnected"
            elif observation is None:
                reason = state["last_error"] or "waiting_for_pair"
            elif not observation["valid"]:
                reason = observation["invalid_reason"]
            elif now_ns >= observation["valid_until_monotonic_ns"]:
                reason = "stale"
            elif now_ns >= observation["rx_monotonic_ns"] + self.settings.max_observation_age_ms * 1_000_000:
                reason = "observation_stale"
            if reason is None:
                self._fault_tokens.pop(sensor_id, None)
                continue
            token = (state["stream_epoch"], observation["sample_sequence"] if observation else 0, reason)
            if self._fault_tokens.get(sensor_id) != token:
                if reason != "waiting_for_pair":
                    self._barrier(states)
                self._fault_tokens[sensor_id] = token
            failure = failure or reason
        return failure

    def _validate_geometry(self, geometry, left, right):
        result = _object(geometry, _GEOMETRY_FIELDS)
        position = result["position_xy_m"]
        if type(position) not in (list, tuple) or len(position) != 2:
            raise _SnapshotError("invalid_geometry")
        xy = [_number(value, -100, 100) for value in position]
        error = _number(result["error_radius_m"], self.settings.position_error_floor_m, 1000)
        residual = _number(result["pair_residual_m"], 0, 100)
        skew = abs(left["rx_monotonic_ns"] - right["rx_monotonic_ns"])
        reference = max(left["rx_monotonic_ns"], right["rx_monotonic_ns"])
        if (_integer(result["rx_skew_ns"], maximum=MAX_MONOTONIC_NS) != skew
                or _integer(result["reference_monotonic_ns"], maximum=MAX_MONOTONIC_NS) != reference
                or result["alignment_method"] != ALIGNMENT_METHOD):
            raise _SnapshotError("invalid_geometry")
        if self.settings.range_model == "slant_known_height":
            if _number(result["tag_height_m"], -5, 5) != self.settings.tag_height_m:
                raise _SnapshotError("invalid_geometry")
        elif result["tag_height_m"] is not None:
            raise _SnapshotError("invalid_geometry")
        if residual > self.settings.max_position_disagreement_m + self.settings.max_relative_speed_mps * skew / 1e9:
            raise _SnapshotError("invalid_geometry")
        return {**result, "position_xy_m": xy, "error_radius_m": error,
                "pair_residual_m": residual}

    def _held(self, now_ns):
        if self._current is None:
            return self._invalid("awaiting_fresh_pair")
        if now_ns >= self._current["valid_until_monotonic_ns"]:
            self._barrier()
            return self._invalid("estimate_stale")
        result = deepcopy(self._current)
        age_seconds = (now_ns - result["reference_monotonic_ns"]) / 1e9
        result["estimate_age_ms"] = age_seconds * 1000
        result["error_radius_m"] = result["error_at_reference_m"] + self.settings.max_relative_speed_mps * age_seconds
        return result

    def update(self, snapshot: dict, now_ns: int) -> dict:
        with self._lock:
            try:
                _integer(now_ns, maximum=MAX_MONOTONIC_NS)
                if self._now_ns is not None and now_ns < self._now_ns:
                    self._barrier()
                    return self._invalid("non_monotonic_time")
                self._now_ns = now_ns
                session, states = self._validate_snapshot(snapshot, now_ns)
                transition = self._context(session, states)
                self._observe(states)
                if transition is not None:
                    return self._invalid(transition)
                failure = self._health(states, now_ns)
                if failure is not None:
                    return self._invalid(failure)
                left, right = (states[sensor_id]["latest"] for sensor_id in SENSOR_IDS)
                if abs(left["rx_monotonic_ns"] - right["rx_monotonic_ns"]) > self.settings.max_pair_skew_ms * 1_000_000:
                    return self._invalid("pair_skew")
                if self.settings.range_model == "unknown":
                    return self._invalid("range_model_unknown")
                keys = tuple((self._seen[sensor_id].epoch, self._seen[sensor_id].sequence) for sensor_id in SENSOR_IDS)
                if any(sensor_id in self._barriers and (key[0] != self._barriers[sensor_id][0]
                       or key[1] <= self._barriers[sensor_id][1]) for sensor_id, key in zip(SENSOR_IDS, keys)):
                    return self._invalid("awaiting_fresh_pair")
                # 한쪽만 새 관측이어도 현재 쌍의 기하 모순은 이전 위치를 즉시 제거해야 한다.
                # 위치·수명 갱신은 아래의 양쪽 새 표본 조건을 별도로 통과해야 한다.
                if keys != self._geometry_key:
                    self._geometry_value = self._validate_geometry(
                        fuse_pair(left, right, self.config, self.settings), left, right)
                    self._geometry_key = keys
                geometry = self._geometry_value
                if self._success_keys is not None and not all(
                        key[0] != previous[0] or key[1] > previous[1]
                        for key, previous in zip(keys, self._success_keys)):
                    return self._held(now_ns)
                if keys == self._last_attempt:
                    return self._invalid("awaiting_fresh_pair")
                self._last_attempt = keys
                deadline = min(observation["valid_until_monotonic_ns"] for observation in (left, right))
                deadline = min(deadline, *(observation["rx_monotonic_ns"]
                               + int(self.settings.max_observation_age_ms * 1_000_000)
                               for observation in (left, right)))
                self._estimate_sequence += 1
                self._success_keys = keys
                self._barriers.clear()
                self._current = {
                    "schema": ESTIMATE_SCHEMA, "valid": True, "invalid_reason": None,
                    "frame_id": "base_link", **geometry,
                    "error_at_reference_m": geometry["error_radius_m"], "estimate_age_ms": 0.0,
                    "valid_until_monotonic_ns": deadline, "estimate_sequence": self._estimate_sequence,
                    "adapter_session": self._session, "source_mode": self.source_mode,
                    "provisional": self._provisional(), "range_model": self.settings.range_model,
                    "drive_authorized": False, "samples": self._samples(),
                }
                return self._held(now_ns)
            except GeometryError as error:
                self._barrier(states)
                self._geometry_key = self._geometry_value = None
                reason = "range_model_unknown" if error.reason == "unknown_range_model" else error.reason
                return self._invalid(reason)
            except (_SnapshotError, TypeError, ValueError, OverflowError) as error:
                self._barrier()
                reason = error.reason if isinstance(error, _SnapshotError) else "invalid_snapshot"
                return self._invalid(reason)
            except Exception:
                self._barrier()
                return self._invalid("estimation_error")
