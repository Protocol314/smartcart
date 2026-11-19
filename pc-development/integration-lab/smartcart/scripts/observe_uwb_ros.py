#!/usr/bin/env python3
"""기존 ROS UWB 토픽을 구독해 관측 유효율과 상태 수신을 집계한다."""
from collections import Counter
import argparse
import json
import math
import os
import sys
import time
import uuid


ROLES = ('uwb_left', 'uwb_right')
TOPICS = {'uwb_left': '/smartcart/uwb/left/observation',
          'uwb_right': '/smartcart/uwb/right/observation'}
STATUS_TOPIC = '/smartcart/uwb/dual/status'
MAX_STATUS_BYTES = 65536
VALID_STATES = {'live', 'waiting', 'stale', 'error'}
INVALID_REASONS = {'invalid_measurement', 'id_mismatch', 'duplicate_sequence',
                   'out_of_order_sequence', 'non_monotonic_time', 'invalid_frame',
                   'disconnected', 'stale', 'bad_frame'} | {
                       'bad_frame:' + code for code in
                       ('argument', 'size', 'header', 'length', 'footer', 'checksum', 'unknown')}


def percentage(value, total):
    return round(100.0 * value / total, 2) if total else None


class CounterState:
    """기기 주소·태그 주소·거리값을 저장하지 않고 상태와 관측 수만 보관한다."""
    def __init__(self):
        self.sensors = {role: {'observations': 0, 'valid': 0, 'invalid': 0,
                               'reasons': Counter(), 'missing_sequence_estimate': 0,
                               'repeated_or_reordered': 0, 'session_changes': 0,
                               'last_sequence': None, 'session': None,
                               'status': Counter(), 'status_connected': 0}
                        for role in ROLES}
        self.status_messages = 0
        self.malformed_status = 0
        self.ignored_observations = 0

    def observe(self, role, message):
        if role not in self.sensors or message.sensor_id != role or message.source_mode != 'live':
            self.ignored_observations += 1
            return
        seq = message.sample_sequence
        if type(seq) is not int or seq < 0:
            self.ignored_observations += 1
            return
        row = self.sensors[role]
        session = message.adapter_session
        if row['session'] is None:
            row['session'] = session
        elif row['session'] != session:
            row['session_changes'] += 1
            row['session'] = session
            row['last_sequence'] = None
        previous = row['last_sequence']
        if previous is not None and seq <= previous:
            row['repeated_or_reordered'] += 1
            return
        if previous is not None and seq > previous + 1:
            row['missing_sequence_estimate'] += seq - previous - 1
        row['last_sequence'] = seq
        row['observations'] += 1
        if message.valid is True:
            row['valid'] += 1
        else:
            row['invalid'] += 1
            reason = message.invalid_reason or 'unknown'
            if not isinstance(reason, str) or reason not in INVALID_REASONS:
                reason = 'other'
            row['reasons'][reason] += 1

    def status(self, raw):
        try:
            if not isinstance(raw, str) or len(raw.encode('utf-8')) > MAX_STATUS_BYTES:
                raise ValueError('상태 길이 오류')
            payload = json.loads(raw)
            if payload.get('source_mode') != 'live' or not isinstance(payload.get('sensors'), dict):
                raise ValueError('실측 상태 아님')
            values = payload['sensors']
            if not all(isinstance(values.get(role), dict) for role in ROLES):
                raise ValueError('센서 상태 누락')
        except (ValueError, TypeError, AttributeError, UnicodeError, RecursionError):
            self.malformed_status += 1
            return
        self.status_messages += 1
        for role in ROLES:
            row, current = self.sensors[role], values[role]
            state = current.get('state')
            row['status'][state if isinstance(state, str) and state in VALID_STATES else 'unknown'] += 1
            if current.get('connected') is True:
                row['status_connected'] += 1

    def summary(self, elapsed, complete):
        sensors = {}
        for role, row in self.sensors.items():
            observations, states = row['observations'], row['status']
            sensors[role] = {
                'observations_received': observations,
                'valid_observations': row['valid'], 'invalid_observations': row['invalid'],
                'valid_observation_pct': percentage(row['valid'], observations),
                'invalid_reasons': dict(row['reasons']),
                'missing_observation_sequence_estimate': row['missing_sequence_estimate'],
                'repeated_or_reordered': row['repeated_or_reordered'],
                'session_changes': row['session_changes'],
                'status_samples': sum(states.values()), 'status_counts': dict(states),
                'status_live_pct': percentage(states['live'], sum(states.values())),
                'status_stale_pct': percentage(states['stale'], sum(states.values())),
                'status_connected_pct': percentage(row['status_connected'], sum(states.values())),
            }
        return {'schema': 'smartcart.uwb_ros_monitor.v1', 'elapsed_seconds': round(elapsed, 2),
                'complete': complete, 'status_messages': self.status_messages,
                'malformed_status': self.malformed_status,
                'ignored_observations': self.ignored_observations, 'sensors': sensors,
                'port_opened': False, 'drive_authorized': False,
                'notice': 'ROS 구독 관측만 집계했습니다. 전체 USB 전송·RF 프레임이나 실측 정확도의 보증은 아닙니다.'}


def seconds(value):
    try:
        result = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError('시간은 숫자여야 합니다') from error
    if not math.isfinite(result) or not 1 <= result <= 600:
        raise argparse.ArgumentTypeError('시간은 1~600초여야 합니다')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=seconds, default=100.0)
    options = parser.parse_args()
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    try:
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from smartcart_uwb_interfaces.msg import UwbObservation
        from std_msgs.msg import String
    except ImportError as error:
        print('ROS Humble 및 빌드된 메시지 환경을 먼저 적용하십시오: ' + str(error), file=sys.stderr)
        return 2
    counter = CounterState()
    rclpy.init()
    node = rclpy.create_node('smartcart_uwb_observe_' + uuid.uuid4().hex[:8])
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=512,
                     reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    subscriptions = [node.create_subscription(UwbObservation, TOPICS[role],
                     lambda message, role=role: counter.observe(role, message), qos) for role in ROLES]
    subscriptions.append(node.create_subscription(String, STATUS_TOPIC, lambda message: counter.status(message.data), qos))
    started, complete = time.monotonic(), False
    next_progress = started + 30
    print(f'ROS UWB 구독 검사 시작: {options.seconds:g}초. 포트를 다시 열지 않습니다.', flush=True)
    try:
        while rclpy.ok() and time.monotonic() - started < options.seconds:
            elapsed = time.monotonic() - started
            rclpy.spin_once(node, timeout_sec=max(0.0, min(0.1, options.seconds - elapsed)))
            if time.monotonic() >= next_progress:
                print(f'진행: {int(time.monotonic()-started)}초', flush=True)
                next_progress += 30
        complete = time.monotonic() - started >= options.seconds
    except KeyboardInterrupt:
        pass
    finally:
        report = counter.summary(time.monotonic() - started, complete)
        print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not complete or not report['status_messages'] or any(
            report['sensors'][role]['observations_received'] == 0 for role in ROLES):
        return 3
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
