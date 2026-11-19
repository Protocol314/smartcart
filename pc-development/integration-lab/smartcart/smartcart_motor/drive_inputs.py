"""같은 Pi의 ROS 토픽에서 원래 관측을 받아 구동 감독기에 공급한다.

장치·USB를 열지 않고 기존 `/scan`과 좌우 UWB 상태 토픽만 구독한다. 추적 노드와 같은
추적 코어를 별도로 실행해 원래 수신 시각과 만료를 그대로 보존한다. 비상정지 피드백
회로가 없으므로 기본 판독은 항상 ESTOP_UNAVAILABLE이다.
"""
from copy import deepcopy
from threading import RLock
import time

from smartcart_tracking.runtime import LiveTrackingRuntime
from .control import _time
from .safety_provider import ESTOP_UNAVAILABLE, EstopReading
from .supervisor import DriveSupervisor

SCAN_KEYS = ('angle_min', 'angle_max', 'angle_increment', 'range_min', 'range_max',
             'scan_time', 'time_increment')


def _unavailable(_now_ns):
    return ESTOP_UNAVAILABLE


class DriveInputs:
    """ROS 콜백·타이머에서 부른다. 오류 입력은 해당 센서를 무효로 만들어 정지시킨다."""

    def __init__(self, supervisor, runtime, *, estop=_unavailable):
        if (not isinstance(supervisor, DriveSupervisor) or not isinstance(runtime, LiveTrackingRuntime)
                or not callable(estop)):
            raise ValueError('감독기·실측 추적 코어·비상정지 판독기가 필요합니다')
        self.supervisor, self.runtime, self.estop = supervisor, runtime, estop
        self._lock = RLock()
        self._status = None

    def on_status(self, text, now_ns):
        from smartcart_tracking.ros_runtime import decode_status
        with self._lock:
            now = _time(now_ns)
            try:
                status = decode_status(text)
                self.runtime.consume_status(status, now)
                self._status = status
            except Exception:
                self._status = None
                self.runtime.invalidate_status('invalid_status_json')
            return self._feed(now)

    def on_scan(self, scan, frame_id, now_ns):
        with self._lock:
            now = _time(now_ns)
            try:
                settings = self.runtime.settings
                if (frame_id != self.runtime.config.lidar.frame_id or type(scan) is not dict
                        or set(scan) != set(SCAN_KEYS) | {'ranges'}
                        or not 1 <= len(scan['ranges']) <= settings.max_rays):
                    raise ValueError('LiDAR 프레임 또는 배열 크기가 잘못되었습니다')
                self.runtime.consume_scan(scan, now)
            except Exception:
                self.runtime.invalidate_scan('invalid_ros_scan')
            return self._feed(now)

    def tick(self, now_ns):
        with self._lock:
            return self._feed(_time(now_ns))

    def _feed(self, now):
        try:
            track = self.runtime.snapshot(now)['track']
            lidar = self.runtime.lidar.snapshot(now)
            estop = self.estop(now)
            if not isinstance(estop, EstopReading):
                estop = ESTOP_UNAVAILABLE
        except Exception:
            track = lidar = None
            estop = ESTOP_UNAVAILABLE
        return self.supervisor.update_inputs(
            lidar_observation=lidar, uwb_status=deepcopy(self._status), track=track,
            estop=estop, now_ns=now)


def drive_ros_loop(inputs, stop, *, scan_topic='/scan', status_topic='/smartcart/uwb/dual/status',
                   ros_args=None, on_error=None):
    """구독만 하며 발행·장치 개방·주행 토픽 생성은 없다. 오류 시 stop을 설정한다."""
    from smartcart_sensing.ros import validate_topic
    from smartcart_tracking.ros_runtime import _cleanup, _environment, _ros_types
    context = node = executor = None
    try:
        validate_topic(scan_topic)
        validate_topic(status_topic)
        if scan_topic == status_topic:
            raise ValueError('입력 토픽은 서로 달라야 합니다')
        _environment()
        (rclpy, Context, Executor, Node, QoS, Reliability, History, Durability, Signals, String) = _ros_types()
        from sensor_msgs.msg import LaserScan
        context = Context()
        rclpy.init(args=[] if ros_args is None else ros_args, context=context,
                   signal_handler_options=Signals.NO)
        node = Node('smartcart_drive_inputs', context=context, use_global_arguments=False,
                    enable_rosout=False, start_parameter_services=False)
        qos = QoS(history=History.KEEP_LAST, depth=5, reliability=Reliability.BEST_EFFORT,
                  durability=Durability.VOLATILE)

        def scan_callback(message):
            scan = {key: getattr(message, key) for key in SCAN_KEYS}
            scan['ranges'] = list(message.ranges)
            inputs.on_scan(scan, message.header.frame_id, time.monotonic_ns())

        node.create_subscription(LaserScan, scan_topic, scan_callback, qos)
        node.create_subscription(String, status_topic,
                                 lambda message: inputs.on_status(message.data, time.monotonic_ns()), qos)
        node.create_timer(.05, lambda: inputs.tick(time.monotonic_ns()))
        executor = Executor(context=context)
        executor.add_node(node)
        while not stop.is_set() and context.ok():
            executor.spin_once(timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if on_error is not None:
            on_error(error)
    finally:
        stop.set()
        _cleanup(executor, node, context)
