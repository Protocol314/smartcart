"""구동 감독기 실행기: ROS 입력 구독 + 제어 주기 + 조종 화면 서버.

출력은 가상 전용이며 GPIO를 열지 않는다. 비상정지 피드백 회로와 현장 검수 기록이 없으므로
감독기는 항상 잠긴다. 화면은 실제 센서 상태에 따른 잠금 사유를 보여 준다.
센싱(start-single-left-trial.sh 등)이 먼저 실행 중이어야 한다.
"""
import argparse
import getpass
import ipaddress
from pathlib import Path
import ssl
import sys
import threading

from .control import MotorConfig, MotorController
from .drive_inputs import DriveInputs, drive_ros_loop
from .drive_server import DriveAccess, DriveServer
from .lidar_clearance import ClearanceConfig
from .live_follow import LiveFollowConfig
from .safety_provider import DriveAcceptance
from .supervisor import DriveSupervisor

APP_ROOT = Path(__file__).resolve().parents[1]
# 추종 거리·정지 반경은 현장 검수 전 자리표시값이다. 검수 기록이 없어 구동에 쓰이지 않는다.
PLACEHOLDER_FOLLOW = LiveFollowConfig(follow_distance_m=1.5, stop_distance_m=1.0, max_error_m=.5)


class VirtualBackend:
    """GPIO 없이 마지막 듀티만 보관하는 출력부."""
    hardware = False

    def __init__(self):
        self.output = (0.0, 0.0)

    def start(self, safety, now_ns):
        self.output = (0.0, 0.0)

    def write(self, left, right, safety, now_ns):
        self.output = (left, right)

    def stop(self):
        self.output = (0.0, 0.0)

    def close(self):
        self.output = (0.0, 0.0)


def build(run_config, *, stop_radius_m=1.0):
    """기존 실행 설정의 장착·UWB·추적 설정으로 단일 왼쪽 구성 감독기를 만든다."""
    from smartcart_runner.config import load_config as load_run_config
    from smartcart_tracking.config import load_tracking_config
    from smartcart_tracking.runtime import LiveTrackingRuntime
    from smartcart_uwb.config import load_config as load_mounting
    from smartcart_uwb.estimation_config import load_estimation_config
    run = load_run_config(run_config)
    runtime = LiveTrackingRuntime(load_mounting(run.mounting_config), load_estimation_config(run.uwb_config),
                                  load_tracking_config(run.tracking_config), single_left=True)
    controller = MotorController(MotorConfig(), VirtualBackend())
    supervisor = DriveSupervisor(controller, acceptance=DriveAcceptance(),
                                 clearance=ClearanceConfig(stop_radius_m), follow=PLACEHOLDER_FOLLOW,
                                 uwb_profile='single_left')
    return supervisor, DriveInputs(supervisor, runtime)


def _address(value):
    address = ipaddress.ip_address(value)
    if address.version != 4 or address.is_unspecified or address.is_multicast or not (
            address.is_loopback or address.is_private):
        raise argparse.ArgumentTypeError('루프백 또는 사설 IPv4 주소만 허용합니다')
    return address


def main(argv=None):
    parser = argparse.ArgumentParser(description='SmartCart 구동 감독기 · 가상 출력 · GPIO 미사용')
    parser.add_argument('--run-config', type=Path, default=APP_ROOT / 'local-config' / 'run.json')
    parser.add_argument('--address', type=_address, default=ipaddress.ip_address('127.0.0.1'))
    parser.add_argument('--port', type=int, default=8445)
    parser.add_argument('--cert', type=Path)
    parser.add_argument('--key', type=Path)
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--status-topic', default='/smartcart/uwb/dual/status')
    parser.add_argument('--stop-radius-m', type=float, default=1.0,
                        help='미검수 자리표시값. 장착 검수 전에는 항상 geometry_unverified로 차단')
    args = parser.parse_args(argv)
    if not 1024 <= args.port <= 65535 or args.port in (8765, 8444, 8080):
        parser.error('기존 센싱·중계·수동 서버와 다른 1024~65535 포트를 지정하십시오')
    if bool(args.cert) != bool(args.key):
        parser.error('HTTPS에는 --cert와 --key가 모두 필요합니다')
    if not args.address.is_loopback and not args.cert:
        parser.error('LAN 주소에는 HTTPS 인증서가 필요합니다. 루프백은 SSH 터널로 접속합니다')
    for path in (args.cert, args.key):
        if path is not None and (path.is_symlink() or not path.is_file()):
            parser.error('인증서 또는 개인키 파일을 확인하십시오')
    supervisor, inputs = build(args.run_config, stop_radius_m=args.stop_radius_m)
    key = getpass.getpass('이번 실행의 조종 연결키 (12자 이상, 입력 내용 숨김): ')
    if not 12 <= len(key) <= 128 or key != getpass.getpass('연결키 다시 입력: '):
        raise SystemExit('연결키가 12~128자가 아니거나 일치하지 않습니다')
    host = f'{args.address}:{args.port}'
    access = DriveAccess(host, key, secure=bool(args.cert))
    key = None
    stop = threading.Event()
    server = DriveServer((str(args.address), args.port), supervisor, access, stop=stop)
    if args.cert:
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.minimum_version = ssl.TLSVersion.TLSv1_2
        context.load_cert_chain(str(args.cert), str(args.key))
        server.socket = context.wrap_socket(server.socket, server_side=True)

    def ros_error(error):
        print('ROS 입력 오류로 종료합니다:', type(error).__name__, file=sys.stderr, flush=True)

    workers = [threading.Thread(target=supervisor.run, args=(stop,), name='drive-control'),
               threading.Thread(target=drive_ros_loop, args=(inputs, stop), name='drive-ros',
                                kwargs={'scan_topic': args.scan_topic, 'status_topic': args.status_topic,
                                        'on_error': ros_error}),
               threading.Thread(target=server.watch_operator, name='drive-operator'),
               threading.Thread(target=lambda: (stop.wait(), server.shutdown()), name='drive-shutdown')]
    print('SmartCart 구동 감독기 · 가상 출력(GPIO 미사용) · 비상정지 피드백 없음 → 구동 잠금', flush=True)
    print('조종 화면: ' + ('https://' if args.cert else 'http://') + host + '/', flush=True)
    print('정지는 PWM 0 요청이며 물리 비상정지를 대신하지 않습니다 · 종료: Ctrl+C', flush=True)
    for worker in workers:
        worker.start()
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for worker in workers:
            worker.join(timeout=3)
        server.server_close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
