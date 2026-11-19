"""실측 입력과 현장 검수 기록에서 SafetyFrame과 우선순위 정지 사유를 만든다.

검수되지 않은 항목이나 존재하지 않는 비상정지 피드백을 참으로 만들지 않는다.
입력은 같은 호스트의 단조 시각(rx/valid_until)을 그대로 유지한 원래 관측이어야 한다.
"""
from dataclasses import dataclass
import re

from .control import NS, SafetyFrame, _number, _time
from .lidar_clearance import ClearanceConfig, evaluate_clearance

_RECORD = re.compile(r'[A-Za-z0-9_.:-]{1,64}')
_COMMON = ('forward_polarity', 'independent_cutoff', 'estop_feedback', 'lidar_geometry')
_FOLLOW = ('uwb_model', 'follow_trial')
_TRACK_STATES = {'waiting', 'acquiring', 'coasting', 'ambiguous', 'lost', 'error'}


@dataclass(frozen=True)
class DriveAcceptance:
    """현장 검수 결과. 기본은 모두 미검수이며 참 값에는 검수 기록 ID가 필요하다.

    가상 시험 통과나 이전 Arduino 실험은 이 기록의 근거가 아니다.
    """
    record_id: str | None = None
    forward_polarity: bool = False    # 차체 기준 좌우 전진 채널 실물 확인
    independent_cutoff: bool = False  # Pi와 무관한 모터 전원 차단·재투입 방지
    estop_feedback: bool = False      # 비상정지 상태를 Pi가 읽는 회로 검수
    lidar_geometry: bool = False      # 장착·차체·정지 영역과 단일 스캔면 제약
    uwb_model: bool = False           # 거리·각도 모델과 사람 위치 정확도 (추종 전용)
    follow_trial: bool = False        # 저속 추종·정지 실물 시험 (추종 전용)

    def __post_init__(self):
        names = _COMMON + _FOLLOW
        for name in names:
            if type(getattr(self, name)) is not bool:
                raise ValueError(name + ' 검수 값은 참 또는 거짓이어야 합니다')
        if self.record_id is not None and (type(self.record_id) is not str
                                           or not _RECORD.fullmatch(self.record_id)):
            raise ValueError('검수 기록 ID 형식이 잘못되었습니다')
        if self.record_id is None and any(getattr(self, name) for name in names):
            raise ValueError('검수 항목을 참으로 두려면 검수 기록 ID가 필요합니다')

    def missing(self, mode):
        if mode not in ('manual', 'follow'):
            raise ValueError('검수 대상 모드는 manual 또는 follow여야 합니다')
        names = _COMMON + (_FOLLOW if mode == 'follow' else ())
        return tuple(name for name in names if not getattr(self, name))


@dataclass(frozen=True)
class EstopReading:
    """비상정지 피드백 회로의 최신 판독. 회로가 없으면 ESTOP_UNAVAILABLE을 쓴다."""
    valid: bool
    released: bool
    until_ns: int

    def __post_init__(self):
        if type(self.valid) is not bool or type(self.released) is not bool:
            raise ValueError('비상정지 판독 값은 참 또는 거짓이어야 합니다')
        _time(self.until_ns)


# 사용자 보고상 현재 비상정지는 모터 전원만 끊으며 Pi가 읽을 수 있는 피드백이 없다.
ESTOP_UNAVAILABLE = EstopReading(False, False, 0)


def _window(value, now, prefix):
    try:
        rx = _time(value.get('rx_monotonic_ns'))
        until = _time(value.get('valid_until_monotonic_ns'))
    except ValueError:
        return False, 0, prefix + '_invalid'
    if rx > now or now >= until:
        return False, 0, prefix + '_stale'
    return True, until, None


def _lidar(observation, now):
    if not isinstance(observation, dict) or observation.get('schema') != 'smartcart.lidar_observation.v1':
        return False, 0, 'lidar_missing'
    if observation.get('source_mode') != 'live':
        return False, 0, 'lidar_not_live'
    if observation.get('valid') is not True or observation.get('sensor_id') != 'lidar_center':
        return False, 0, 'lidar_invalid'
    return _window(observation, now, 'lidar')


def _uwb(status, sensor_id, now):
    """단일 태그 식별(id_valid)과 측정 유효를 모두 요구한다. 거리 정확도 검수는 아니다."""
    if (not isinstance(status, dict) or status.get('schema') != 'smartcart.dual_uwb_status.v1'
            or status.get('source_mode') != 'live' or not isinstance(status.get('sensors'), dict)):
        return False, 0, 'uwb_status_missing'
    sensor = status['sensors'].get(sensor_id)
    if not isinstance(sensor, dict) or sensor.get('connected') is not True:
        return False, 0, sensor_id + '_disconnected'
    latest = sensor.get('latest')
    if sensor.get('state') != 'live' or not isinstance(latest, dict):
        return False, 0, sensor_id + '_invalid'
    if (latest.get('schema') != 'smartcart.uwb_observation.v1' or latest.get('sensor_id') != sensor_id
            or latest.get('source_mode') != 'live'):
        return False, 0, sensor_id + '_invalid'
    if latest.get('id_valid') is not True or type(latest.get('logical_target_id')) is not int:
        return False, 0, sensor_id + '_id_mismatch'
    if latest.get('valid') is not True or latest.get('measurement_valid') is not True:
        return False, 0, sensor_id + '_invalid'
    return _window(latest, now, sensor_id)


def _track(track, now):
    """반환: (유효, 만료, 임시값 여부, 사유)."""
    if (not isinstance(track, dict) or track.get('schema') != 'smartcart.target_track.v1'
            or track.get('source_mode') != 'live' or track.get('frame_id') != 'base_link'):
        return False, 0, True, 'tracking_missing'
    provisional = track.get('provisional') is not False
    state = track.get('state')
    if state != 'tracking' or track.get('valid') is not True:
        return False, 0, provisional, 'tracking_' + (state if state in _TRACK_STATES else 'invalid')
    if track.get('candidate_count') != 1:
        return False, 0, provisional, 'tracking_ambiguous'
    if track.get('drive_authorized') is not True:
        return False, 0, provisional, 'tracking_not_authorized'
    try:
        reference = _time(track.get('reference_monotonic_ns'))
        until = _time(track.get('valid_until_monotonic_ns'))
    except ValueError:
        return False, 0, provisional, 'tracking_invalid'
    if reference > now or now >= until:
        return False, 0, provisional, 'tracking_stale'
    if provisional:
        return True, until, True, 'tracking_provisional'
    return True, until, False, None


def build_safety_frame(mode, *, lidar_observation, uwb_status, track, estop, acceptance,
                       clearance, now_ns, uwb_profile='single_left', speed_limit=1.0,
                       safety_timeout_ms=300):
    """반환: (SafetyFrame, 정지 사유 튜플). 사유가 비어 있을 때만 구동 조건을 만족한다.

    사유 순서가 정지 우선순위다: 비상정지 → 현장 미검수 → LiDAR → UWB → 장애물 → 추적.
    """
    if mode not in ('manual', 'follow'):
        raise ValueError('안전 상태 모드는 manual 또는 follow여야 합니다')
    if uwb_profile not in ('single_left', 'dual'):
        raise ValueError('UWB 안전 구성은 single_left 또는 dual이어야 합니다')
    if (not isinstance(acceptance, DriveAcceptance) or not isinstance(clearance, ClearanceConfig)
            or not isinstance(estop, EstopReading)):
        raise ValueError('검수 기록·장애물 설정·비상정지 판독이 필요합니다')
    if clearance.geometry_verified and not acceptance.lidar_geometry:
        raise ValueError('장착 검수 기록 없이 정지 영역을 검수됨으로 둘 수 없습니다')
    now = _time(now_ns)
    speed_limit = _number(speed_limit, '안전 속도 한도', .01, 1)
    reasons = []

    estop_valid = estop.valid and now < estop.until_ns <= now + 2 * NS
    if not estop_valid:
        reasons.append('estop_feedback_unavailable')
    elif not estop.released:
        reasons.append('estop_engaged')

    missing = acceptance.missing(mode)
    reasons.extend('unverified_' + name for name in missing)

    lidar_ok, lidar_until, why = _lidar(lidar_observation, now)
    if why:
        reasons.append(why)
    left_ok, left_until, why = _uwb(uwb_status, 'uwb_left', now)
    if why:
        reasons.append(why)
    right_ok, right_until = False, 0
    if uwb_profile == 'dual':
        right_ok, right_until, why = _uwb(uwb_status, 'uwb_right', now)
        if why:
            reasons.append(why)

    clear = {'valid': False, 'clear': False, 'reason': 'lidar_unavailable', 'valid_until_ns': 0}
    if lidar_ok:
        clear = evaluate_clearance(lidar_observation, now, clearance)
        if not clear['clear']:
            reasons.append('obstacle' if clear['reason'] == 'obstacle' else 'clearance_' + clear['reason'])

    tracking_ok, tracking_until, track_provisional = False, 0, False
    if mode == 'follow':
        tracking_ok, tracking_until, track_provisional, why = _track(track, now)
        if why:
            reasons.append(why)

    frame = SafetyFrame(
        source_mode='live', drive_authorized=not missing,
        provisional=bool(missing) or track_provisional,
        estop_feedback_valid=estop_valid, estop_released=estop_valid and estop.released,
        uwb_left_valid=left_ok, uwb_right_valid=right_ok, lidar_valid=lidar_ok,
        tracking_valid=tracking_ok, clearance_valid=clear['valid'], clearance_ok=clear['clear'],
        speed_limit=speed_limit, observed_ns=now, uwb_left_until_ns=left_until,
        uwb_right_until_ns=right_until, lidar_until_ns=lidar_until,
        tracking_until_ns=tracking_until, clearance_until_ns=clear['valid_until_ns'],
        uwb_profile=uwb_profile, control_mode=mode)
    if not reasons and not frame.permits(now, safety_timeout_ms):
        # 예: 입력 수명이 2초를 넘는 등 프레임 규칙 위반. 참으로 보정하지 않는다.
        reasons.append('safety_frame_rejected')
    return frame, tuple(reasons)
