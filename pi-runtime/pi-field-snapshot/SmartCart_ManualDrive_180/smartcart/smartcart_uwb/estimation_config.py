"""수집 계약을 바꾸지 않는 별도의 위치 추정 가정과 입력 상한."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path

from .config import MAX_CONFIG_BYTES, _boolean, _number, _object, _reject_constant, _unique_object

ESTIMATION_CONFIG_SCHEMA = "smartcart.uwb_estimation_config.v1"


@dataclass(frozen=True)
class EstimationConfig:
    """오차 항목은 표준편차가 아닌 사용자가 정하는 가정상의 상한이다.

    기본 거리 의미는 미확인이므로 위치를 만들지 않는다. 이 설정만으로
    현장 오차 상한이나 물리 동기화가 검증되지는 않는다.
    """
    schema: str = ESTIMATION_CONFIG_SCHEMA
    range_model: str = "unknown"
    tag_height_m: float | None = None
    tag_height_tolerance_m: float = .05
    max_pair_skew_ms: float = 40.0
    max_observation_age_ms: float = 200.0
    range_error_m: float = .10
    bearing_error_deg: float = 8.0
    mount_translation_error_m: float = .03
    mount_yaw_error_deg: float = 3.0
    min_horizontal_range_m: float = .10
    max_position_disagreement_m: float = .60
    position_error_floor_m: float = .20
    max_relative_speed_mps: float = 3.0
    max_host_rx_delay_ms: float = 50.0
    assumptions_verified: bool = False

    def __post_init__(self):
        if self.schema != ESTIMATION_CONFIG_SCHEMA:
            raise ValueError("지원하지 않는 위치 추정 설정입니다")
        if not isinstance(self.range_model, str) or self.range_model not in (
                "unknown", "horizontal", "slant_known_height"):
            raise ValueError("명시적인 거리 모델이 필요합니다")
        if self.range_model == "slant_known_height":
            object.__setattr__(self, "tag_height_m", _number(self.tag_height_m, "tag_height_m", -5, 5))
        elif self.tag_height_m is not None:
            raise ValueError("태그 높이는 slant_known_height 모델에서만 지정합니다")
        limits = {
            "tag_height_tolerance_m": (0, 1), "max_pair_skew_ms": (1, 200),
            "max_observation_age_ms": (1, 5000), "range_error_m": (.01, 2),
            "bearing_error_deg": (1, 45), "mount_translation_error_m": (.001, 1),
            "mount_yaw_error_deg": (0, 45), "min_horizontal_range_m": (.05, 1),
            "max_position_disagreement_m": (.01, 5), "position_error_floor_m": (.01, 2),
            "max_relative_speed_mps": (.1, 10), "max_host_rx_delay_ms": (0, 1000),
        }
        for name, (minimum, maximum) in limits.items():
            object.__setattr__(self, name, _number(getattr(self, name), name, minimum, maximum))
        _boolean(self.assumptions_verified, "assumptions_verified")
        if self.max_pair_skew_ms > self.max_observation_age_ms:
            raise ValueError("좌우 시각차 한도는 관측 수명 이하여야 합니다")


def config_from_dict(payload: object) -> EstimationConfig:
    """누락·미지 필드와 암묵적 형변환을 허용하지 않는다."""
    values = _object(payload, set(EstimationConfig.__dataclass_fields__), "estimation_config")
    return EstimationConfig(**values)


def load_estimation_config(path: str | Path) -> EstimationConfig:
    path = Path(path)
    if not path.is_file() or path.stat().st_size > MAX_CONFIG_BYTES:
        raise ValueError("추정 설정은 64 KiB 이내의 일반 파일이어야 합니다")
    with path.open("rb") as handle:
        raw = handle.read(MAX_CONFIG_BYTES + 1)
    if len(raw) > MAX_CONFIG_BYTES:
        raise ValueError("추정 설정 크기 상한을 넘었습니다")
    try:
        payload = json.loads(raw.decode("utf-8"), object_pairs_hook=_unique_object,
                             parse_constant=_reject_constant)
    except (UnicodeError, json.JSONDecodeError, RecursionError) as exc:
        raise ValueError("올바른 UTF-8 추정 설정이 필요합니다") from exc
    return config_from_dict(payload)


def config_dict(settings: EstimationConfig) -> dict:
    """검증된 설정을 JSON에 쓸 때 사용하는 작은 값 객체다."""
    if not isinstance(settings, EstimationConfig):
        raise ValueError("검증된 추정 설정이 필요합니다")
    return asdict(settings)
