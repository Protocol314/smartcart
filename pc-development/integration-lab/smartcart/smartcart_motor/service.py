"""단일 소유 작업자의 주기 평가와 크기 제한 최신 명령 보관함."""
from threading import Event, Lock
import time

from .control import MotorController, SafetyFrame, _time, mix


class MotorService:
    """신뢰된 로컬 통합 계층이 상태와 명령을 제공하며 네트워크 포트를 열지 않는다.

    이 루프가 실행 중일 때만 소프트웨어 시간 초과를 평가한다. 프로세스·커널·전원
    정지는 소프트웨어만으로 차단할 수 없으므로 독립 하드웨어 차단이 필요하다.
    """
    def __init__(self, controller, *, clock=time.monotonic_ns, period_ms=20):
        if not isinstance(controller, MotorController) or not callable(clock):
            raise ValueError('검증된 제어기와 단조 시계가 필요합니다')
        if type(period_ms) is not int or not 10 <= period_ms <= 100:
            raise ValueError('제어 주기는 10~100ms 정수여야 합니다')
        self.controller, self.clock, self.period_ms = controller, clock, period_ms
        self.stop = Event()
        self._lock = Lock()
        self._safety = self._command = self._last_sequence = None
        self._arm_requested = self._fault = self._running = False

    def update_safety(self, frame):
        """상태는 최신 하나만 보관한다. 잘못된 입력은 다음 평가에서 잠금을 건다."""
        with self._lock:
            self._safety = frame if isinstance(frame, SafetyFrame) else None

    def invalidate(self):
        """한 주기 안에 복구되더라도 오류·정지 요청을 잃지 않고 잠근다."""
        with self._lock:
            self._safety = None
            self._command = None
            self._arm_requested = False
            self._fault = True

    def request_arm(self):
        """재무장은 명시적 로컬 요청이며 다음 주기에서 안전 상태를 다시 검사한다."""
        with self._lock:
            self._arm_requested = True

    def submit(self, forward, turn, sequence):
        """순번·입력을 제한하고 수신 시각을 보존한다. 동일 순번은 수명을 늘리지 않는다."""
        with self._lock:
            try:
                mix(forward, turn)
                if type(sequence) is not int or not 0 <= sequence < (1 << 53):
                    raise ValueError('명령 순번이 잘못되었습니다')
                if self._last_sequence is not None and sequence <= self._last_sequence:
                    raise ValueError('반복되거나 역행한 명령을 거부합니다')
                received = _time(self.clock())
                self._last_sequence = sequence
                self._command = (forward, turn, sequence, received)
            except Exception:
                self._command = None
                self._fault = True
                raise

    def cycle(self, now_ns):
        """GPIO 소유 단일 스레드에서만 부른다. 안전 상태가 없으면 항상 잠긴다."""
        try:
            return self._cycle(now_ns)
        except Exception:
            self.controller.disarm('service_error')
            raise

    def _cycle(self, now_ns):
        now_ns = _time(now_ns)
        with self._lock:
            safety, command = self._safety, self._command
            arm_requested, fault = self._arm_requested, self._fault
            self._command, self._arm_requested, self._fault = None, False, False
        if fault:
            self.controller.disarm('invalid_command', now_ns=now_ns)
            return (0.0, 0.0)
        if arm_requested and not self.controller.arm(safety, now_ns):
            return (0.0, 0.0)
        if command is not None:
            self.controller.submit(*command[:3], safety, now_ns, issued_ns=command[3])
        return self.controller.step(safety, now_ns)

    def run(self):
        """현재 스레드에서 주기적으로 평가하고 중단·예외 때 출력을 종료한다."""
        with self._lock:
            if self._running:
                raise RuntimeError('모터 작업자는 한 번만 실행할 수 있습니다')
            self._running = True
        try:
            while not self.stop.is_set():
                started = self.clock()
                self.cycle(started)
                elapsed = max(0, self.clock() - started) / 1_000_000_000
                self.stop.wait(max(0, self.period_ms / 1000 - elapsed))
        finally:
            try:
                self.controller.close()
            finally:
                with self._lock:
                    self._running = False
