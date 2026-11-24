"""기존 장착 설정을 바꾸지 않는 현장 식별·실측 증빙 요구사항."""
from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
from types import MappingProxyType

from smartcart_uwb.config import MAX_CONFIG_BYTES, _boolean, _number, _object, _text as _core_text
from smartcart_uwb.config import _reject_constant, _unique_object
from smartcart_uwb.serial_reader import canonical_port

FIELD_CONFIG_SCHEMA = "smartcart.field_acceptance_config.v1"
ROLES = ("uwb_left", "uwb_right", "lidar_center")


def _text(value, where, maximum, *, empty=False):
    result = _core_text(value, where, maximum, empty=empty)
    try:
        value.encode("utf-8")
    except UnicodeError as error:
        raise ValueError("현장 설정 문자열은 올바른 UTF-8이어야 합니다") from error
    return result


@dataclass(frozen=True)
class DeviceBinding:
    port: str | None = None
    expected_device_identity: str = ""
    device_identity_verified: bool = False

    def __post_init__(self):
        if self.port is not None:
            _text(self.port, "port", 512)
            canonical_port(self.port)
        _text(self.expected_device_identity, "expected_device_identity", 512, empty=True)
        _boolean(self.device_identity_verified, "device_identity_verified")
        if self.device_identity_verified and (not self.port or not self.expected_device_identity):
            raise ValueError("식별 확인에는 포트와 정확한 장치 식별값이 필요합니다")


@dataclass(frozen=True)
class CoordinateContract:
    frame_id: str = "base_link"
    origin: str = "rear_axle_center"
    x_axis: str = "forward"
    y_axis: str = "left"
    z_axis: str = "up"
    translation_unit: str = "m"
    yaw_unit: str = "deg"

    def __post_init__(self):
        expected = {"frame_id": "base_link", "origin": "rear_axle_center", "x_axis": "forward",
                    "y_axis": "left", "z_axis": "up", "translation_unit": "m", "yaw_unit": "deg"}
        if asdict(self) != expected:
            raise ValueError("좌표는 후륜축 실제 중앙의 base_link, 전방 X·좌측 Y·상방 Z, m·deg여야 합니다")


@dataclass(frozen=True)
class MountingEvidence:
    translation_verified: bool = False
    yaw_verified: bool = False
    height_verified: bool = False
    level_verified: bool = False
    reference_point: str = "unknown"
    evidence_note: str = ""

    def __post_init__(self):
        for name in ("translation_verified", "yaw_verified", "height_verified", "level_verified"):
            _boolean(getattr(self, name), name)
        if self.reference_point not in ("unknown", "rf_phase_center", "scan_origin"):
            raise ValueError("센서 기준점은 unknown·rf_phase_center·scan_origin 중 하나여야 합니다")
        _text(self.evidence_note, "evidence_note", 2048, empty=True)


def _default_evidence():
    return MappingProxyType({role: MountingEvidence() for role in ROLES})


@dataclass(frozen=True)
class FieldConfig:
    schema: str = FIELD_CONFIG_SCHEMA
    coordinate_contract: CoordinateContract = field(default_factory=CoordinateContract)
    origin_verified: bool = False
    axes_verified: bool = False
    lidar_device: DeviceBinding = field(default_factory=DeviceBinding)
    expected_lidar_frame_id: str = "lidar_scan"
    mounting_evidence: object = field(default_factory=_default_evidence)
    range_model_verified: bool = False
    tag_height_verified: bool = False
    min_uwb_baseline_m: float = .05
    max_lidar_lateral_offset_m: float = .05

    def __post_init__(self):
        if self.schema != FIELD_CONFIG_SCHEMA:
            raise ValueError("지원하지 않는 현장 검수 설정입니다")
        if not isinstance(self.coordinate_contract, CoordinateContract) or not isinstance(self.lidar_device, DeviceBinding):
            raise ValueError("검증된 좌표계와 LiDAR 장치 배정이 필요합니다")
        for name in ("origin_verified", "axes_verified", "range_model_verified", "tag_height_verified"):
            _boolean(getattr(self, name), name)
        _text(self.expected_lidar_frame_id, "expected_lidar_frame_id", 128)
        if (not hasattr(self.mounting_evidence, "keys") or set(self.mounting_evidence) != set(ROLES)
                or not all(isinstance(value, MountingEvidence) for value in self.mounting_evidence.values())):
            raise ValueError("좌우 UWB와 중앙 LiDAR 세 역할의 실측 증빙이 필요합니다")
        object.__setattr__(self, "mounting_evidence", MappingProxyType(dict(self.mounting_evidence)))
        for name, minimum, maximum in (("min_uwb_baseline_m", .01, 2),
                                       ("max_lidar_lateral_offset_m", 0, 1)):
            object.__setattr__(self, name, _number(getattr(self, name), name, minimum, maximum))


def field_config_dict(settings):
    """설정 원문을 명시적으로 요청한 호출자에게만 반환한다. 식별값은 요약 보고서와 다르다."""
    if not isinstance(settings, FieldConfig):
        raise ValueError("검증된 현장 검수 설정이 필요합니다")
    return {"schema": settings.schema, "coordinate_contract": asdict(settings.coordinate_contract),
            "origin_verified": settings.origin_verified, "axes_verified": settings.axes_verified,
            "lidar_device": asdict(settings.lidar_device), "expected_lidar_frame_id": settings.expected_lidar_frame_id,
            "mounting_evidence": {role: asdict(settings.mounting_evidence[role]) for role in ROLES},
            "range_model_verified": settings.range_model_verified, "tag_height_verified": settings.tag_height_verified,
            "min_uwb_baseline_m": settings.min_uwb_baseline_m,
            "max_lidar_lateral_offset_m": settings.max_lidar_lateral_offset_m}


def field_config_from_dict(payload):
    data = _object(payload, set(FieldConfig.__dataclass_fields__), "field_config")
    values = dict(data)
    values["coordinate_contract"] = CoordinateContract(**_object(
        data["coordinate_contract"], set(CoordinateContract.__dataclass_fields__), "coordinate_contract"))
    values["lidar_device"] = DeviceBinding(**_object(
        data["lidar_device"], set(DeviceBinding.__dataclass_fields__), "lidar_device"))
    evidence = _object(data["mounting_evidence"], set(ROLES), "mounting_evidence")
    values["mounting_evidence"] = {role: MountingEvidence(**_object(
        evidence[role], set(MountingEvidence.__dataclass_fields__), role)) for role in ROLES}
    return FieldConfig(**values)


def load_field_config(path):
    path = Path(path)
    if not path.is_file() or path.stat().st_size > MAX_CONFIG_BYTES:
        raise ValueError("현장 검수 설정은 64 KiB 이내의 일반 JSON 파일이어야 합니다")
    with path.open("rb") as handle:
        raw = handle.read(MAX_CONFIG_BYTES + 1)
    if len(raw) > MAX_CONFIG_BYTES:
        raise ValueError("현장 검수 설정 크기 상한을 넘었습니다")
    try:
        payload = json.loads(raw.decode("utf-8"), object_pairs_hook=_unique_object, parse_constant=_reject_constant)
    except (UnicodeError, json.JSONDecodeError, RecursionError) as error:
        raise ValueError("올바른 UTF-8 현장 검수 JSON이 필요합니다") from error
    return field_config_from_dict(payload)
