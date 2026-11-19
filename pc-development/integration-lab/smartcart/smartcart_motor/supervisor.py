"""수동 조종·사용자 추종의 모드 전환, 정지 우선순위, 고장 래치와 명시적 재무장.

GPIO·네트워크·ROS를 직접 열지 않는다. 호출자는 같은 호스트의 단조 시각을 유지한
실측 입력과 조종 명령을 넘기고, 별도 스레드에서 cycle(또는 run)을 주기 실행한다.
모든 정지는 PWM 0 요청이며 Pi·OS·프로세스 고장 때의 독립 전원 차단을 대신하지 않는다.
"""
from threading import RLock
import time

from .control import MotorController, _number, _time, mix
from .gpio_backend import GpioBackend
from .lidar_clearance import ClearanceConfig
from .live_follow import LiveFollow, LiveFollowConfig
from .live_manual import LiveManual
from .safety_provider import DriveAcceptance, EstopReading, build_safety_frame
from .service import MotorService

SCHEMA = 'smartcart.drive_supervisor.v1'
MODES = ('locked', 'manual', 'follow')
STOP_REASONS = ('operator_stop', 'operator_disconnected', 'shutdown')


class DriveSupervisor:
    """하나의 잠금으로 입력·명령·제어 주기를 직렬화한다.

    정지 사유는 첫 원인을 래치하며, 센서가 곧바로 회복돼도 자동 재개하지 않는다.
    재무장은 현재 입력이 모두 유효하고(수동은 방향 입력이 중립일 때) 명시적으로 요청할 때만 된다.
    """

    def __init__(self, controller, *, acceptance, clearance, follow, uwb_profile='single_left',
                 speed_limit=1.0, clock=time.monotonic_ns):
        if (not isinstance(controller, MotorController) or not isinstance(acceptance, DriveAcceptance)
                or not isinstance(clearance, ClearanceConfig) or not isinstance(follow, LiveFollowConfig)
                or not callable(clock)):
            raise ValueError('제어기·검수 기록·장애물/추종 설정·단조 시계가 필요합니다')
        if uwb_profile not in ('single_left', 'dual'):
            raise ValueError('UWB 안전 구성은 single_left 또는 dual이어야 합니다')
        # 설정·출력부·검수 기록이 서로 다른 말을 하지 않게 한다. 기록 없는 참을 거부한다.
        if controller.config.polarity_verified and not acceptance.forward_polarity:
            raise ValueError('전진 극성 검수 기록 없이 모터 설정을 검수됨으로 둘 수 없습니다')
        if getattr(controller.backend, 'cutoff_verified', False) is not False and not acceptance.independent_cutoff:
            raise ValueError('독립 차단 검수 기록 없이 출력부를 검수됨으로 둘 수 없습니다')
        if clearance.geometry_verified and not acceptance.lidar_geometry:
            raise ValueError('장착 검수 기록 없이 정지 영역을 검수됨으로 둘 수 없습니다')
        self.controller, self.acceptance = controller, acceptance
        self.clearance, self.follow_config = clearance, follow
        self.uwb_profile = uwb_profile
        self.speed_limit = _number(speed_limit, '안전 속도 한도', .01, 1)
        self.clock = clock
        self._lock = RLock()
        self.mode = 'locked'
        self.latch = 'startup'
        self.last_stop = 'startup'
        self._arming = False
        self._output = (0.0, 0.0)
        self._new_session()

    # 내부 상태 ---------------------------------------------------------------
    def _now(self, now_ns):
        return _time(self.clock() if now_ns is None else now_ns)

    def _new_session(self):
        """모드 선택마다 명령 순번 공간과 서비스 상태를 새로 시작한다."""
        self.service = MotorService(self.controller, clock=self.clock)
        self._manual = LiveManual(self.service, self.clearance)
        self._follow = LiveFollow(self.service, self.follow_config, self.clearance)
        self._frame, self._reasons = None, ('awaiting_inputs',)
        self._lidar = self._track = None
        self._neutral = True

    def _halt(self, reason, now):
        """대기 명령·무장 요청을 지우고 즉시 PWM 0을 요청한다. 첫 원인을 래치한다."""
        self.service.invalidate()
        self._arming = False
        if self.latch is None:
            self.latch = reason
        self.last_stop = reason
        self.controller.disarm(reason, now_ns=now)
        self._output = (0.0, 0.0)

    @staticmethod
    def _result(accepted, reason=None):
        return {'accepted': accepted, 'reason': reason}

    # 운영자 요청 -------------------------------------------------------------
    def set_mode(self, mode, now_ns=None):
        """모드 변경(같은 모드 재선택 포함)은 즉시 정지하고 새 입력과 재무장을 요구한다."""
        with self._lock:
            now = self._now(now_ns)
            if mode not in MODES:
                raise ValueError('지원하지 않는 구동 모드입니다')
            self._halt('mode_change', now)
            self.mode = mode
            self._new_session()
            if mode == 'locked':
                self._reasons = ('mode_locked',)
            return self.status(now)

    def stop(self, reason='operator_stop', now_ns=None):
        """가장 높은 우선순위. 어떤 모드·상태에서도 즉시 정지하고 래치한다."""
        with self._lock:
            now = self._now(now_ns)
            if reason not in STOP_REASONS:
                reason = 'operator_stop'
            self._halt(reason, now)
            return self.status(now)

    def request_arm(self, now_ns=None):
        with self._lock:
            now = self._now(now_ns)
            if self.mode == 'locked':
                return self._result(False, 'mode_locked')
            if self.latch is None:
                return self._result(False, 'already_active')
            if self._frame is None or self._reasons:
                return self._result(False, self._reasons[0] if self._reasons else 'awaiting_inputs')
            if not self._frame.permits(now, self.controller.config.safety_timeout_ms):
                return self._result(False, 'inputs_expired')
            if self.mode == 'manual' and not self._neutral:
                return self._result(False, 'controls_not_neutral')
            if self.controller.armed:
                return self._result(False, 'controller_active')
            self.latch = None
            self._arming = True
            self.service.update_safety(self._frame)
            self.service.request_arm()
            return self._result(True)

    def manual_command(self, forward, turn, sequence, now_ns=None):
        """조종자 방향 입력. 래치 중에도 중립 여부는 기록해 재무장 조건에 쓴다."""
        with self._lock:
            now = self._now(now_ns)
            try:
                mix(forward, turn)
                if type(sequence) is not int or not 0 <= sequence < (1 << 53):
                    raise ValueError('명령 순번이 잘못되었습니다')
            except (TypeError, ValueError):
                self._neutral = False
                self._halt('invalid_command', now)
                return self._result(False, 'invalid_command')
            self._neutral = forward == 0 and turn == 0
            if self.mode != 'manual':
                # 추종 중 비중립 수동 입력은 의도 충돌이므로 정지한다. 중립 유지 신호는 무시한다.
                if not self._neutral:
                    self._halt('mode_mismatch', now)
                return self._result(False, 'mode_mismatch')
            if self.latch is not None:
                return self._result(False, 'latched')
            if self._frame is None or self._reasons:
                reason = self._reasons[0] if self._reasons else 'awaiting_inputs'
                self._halt(reason, now)
                return self._result(False, reason)
            result = self._manual.update(forward, turn, sequence, self._lidar, self._frame, now)
            if not result['accepted']:
                self._halt('manual_' + result['reason'], now)
            return result

    # 센서 입력 ---------------------------------------------------------------
    def update_inputs(self, *, lidar_observation, uwb_status, estop, track=None, now_ns=None):
        """최신 실측 입력. 원래 관측 시각과 만료를 보존한 dict를 그대로 넘겨야 한다."""
        with self._lock:
            now = self._now(now_ns)
            if self.mode == 'locked':
                self._frame, self._reasons = None, ('mode_locked',)
                return self.status(now)
            try:
                frame, reasons = build_safety_frame(
                    self.mode, lidar_observation=lidar_observation, uwb_status=uwb_status,
                    track=track, estop=estop, acceptance=self.acceptance, clearance=self.clearance,
                    now_ns=now, uwb_profile=self.uwb_profile, speed_limit=self.speed_limit,
                    safety_timeout_ms=self.controller.config.safety_timeout_ms)
            except (TypeError, ValueError, OverflowError):
                frame, reasons = None, ('invalid_inputs',)
            self._frame, self._reasons = frame, reasons
            self._lidar, self._track = lidar_observation, track
            if reasons:
                if self.latch is None:
                    self._halt(reasons[0], now)
                return self.status(now)
            if self.latch is None:
                # 유효한 최신 상태만 갱신한다. 이동은 여전히 명령 수명이 결정하므로
                # 조종 입력이 끊기면 command_timeout으로 정지한다.
                self.service.update_safety(frame)
            if self.mode == 'follow' and self.latch is None:
                result = self._follow.update(track, lidar_observation, frame, now)
                if not result['accepted'] and result['reason'] != 'no_new_track':
                    self._halt('follow_' + result['reason'], now)
            return self.status(now)

    # 제어 주기 ---------------------------------------------------------------
    def cycle(self, now_ns=None):
        """GPIO를 쓰는 유일한 경로. 입력 공급이 끊기면 마지막 상태가 만료돼 정지한다."""
        with self._lock:
            now = self._now(now_ns)
            if self.latch is None and (self._frame is None or not self._frame.permits(
                    now, self.controller.config.safety_timeout_ms)):
                self._halt('inputs_expired', now)
            arming = self._arming
            try:
                output = self.service.cycle(now)
            except Exception:
                self._halt('control_error', now)
                raise
            self._arming = False
            if self.latch is None and not self.controller.armed:
                self._halt('arm_refused' if arming else 'controller_' + self.controller.reason, now)
                output = (0.0, 0.0)
            self._output = tuple(output)
            return self._output

    def run(self, stop_event, period_ms=20):
        """현재 스레드에서 주기 실행한다. 종료·예외 때 정지 후 출력부를 닫는다."""
        if type(period_ms) is not int or not 10 <= period_ms <= 100:
            raise ValueError('제어 주기는 10~100ms 정수여야 합니다')
        try:
            while not stop_event.is_set():
                started = self.clock()
                self.cycle(started)
                elapsed = max(0, self.clock() - started) / 1_000_000_000
                stop_event.wait(max(0, period_ms / 1000 - elapsed))
        finally:
            with self._lock:
                try:
                    self._halt('shutdown', self._now(None))
                finally:
                    self.controller.close()

    def status(self, now_ns=None):
        """조종 화면용 요약. 위치·태그 식별자 같은 원시 측정값은 넣지 않는다."""
        with self._lock:
            self._now(now_ns)
            mode = self.mode if self.mode != 'locked' else 'follow'
            return {
                'schema': SCHEMA, 'mode': self.mode,
                'active': self.latch is None and self.controller.armed,
                'arming': self._arming, 'latched_reason': self.latch,
                'last_stop_reason': self.last_stop, 'blocking_reasons': list(self._reasons),
                'acceptance_missing': list(self.acceptance.missing(mode)),
                'output_duty': list(self._output),
                'hardware_output': isinstance(self.controller.backend, GpioBackend),
                'uwb_profile': self.uwb_profile, 'measured_rpm': None,
                'stop_is_pwm_zero_only': True,
            }
