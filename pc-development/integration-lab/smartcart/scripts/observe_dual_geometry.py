#!/usr/bin/env python3
"""기존 Pi 웹 상태를 표본 조회해 좌우 거리·각도와 가정별 차이만 집계한다."""
from collections import Counter
import argparse
import json
import math
import statistics
import time
from urllib.error import URLError
from urllib.request import HTTPRedirectHandler, ProxyHandler, build_opener

URL = 'http://127.0.0.1:8765/api/dual-uwb'
ROLES = ('uwb_left', 'uwb_right')
MAX_BYTES = 65536


class NoRedirect(HTTPRedirectHandler):
    """로컬 상태 조회가 다른 주소로 전송되지 않게 한다."""
    def redirect_request(self, *_args, **_kwargs):
        return None


def number(value, low, high):
    return (type(value) in (int, float) and math.isfinite(value) and low <= value <= high)


def summarize(values):
    if not values:
        return None
    ordered = sorted(values)
    return {'min': round(ordered[0], 3), 'p10': round(ordered[round((len(ordered) - 1) * .1)], 3),
            'median': round(statistics.median(ordered), 3),
            'p90': round(ordered[round((len(ordered) - 1) * .9)], 3),
            'max': round(ordered[-1], 3)}


def residual(left, right, baseline_m, sign):
    """sign=-1은 현재 화면 규칙, +1은 반전 가정이다. 설정은 변경하지 않는다."""
    r_left, a_left = left
    r_right, a_right = right
    l_angle, r_angle = math.radians(sign * a_left), math.radians(sign * a_right)
    dx = r_left * math.cos(l_angle) - r_right * math.cos(r_angle)
    dy = baseline_m + r_left * math.sin(l_angle) - r_right * math.sin(r_angle)
    return math.hypot(dx, dy)


class GeometryMonitor:
    def __init__(self, baseline_m=.45):
        if not number(baseline_m, .05, 2):
            raise ValueError('센서 간격은 0.05~2m여야 합니다')
        self.baseline_m = float(baseline_m)
        self.states = {role: Counter() for role in ROLES}
        self.values = {role: {'range': [], 'angle': [], 'last_seq': None, 'resets': 0} for role in ROLES}
        self.last_pair = None
        self.residuals = {'current': [], 'reversed_sign_hypothesis': []}
        self.successful_polls = self.malformed_polls = 0

    @staticmethod
    def _observation(sensor):
        if (not isinstance(sensor, dict) or sensor.get('state') != 'live' or sensor.get('valid') is not True or
                sensor.get('connected') is not True or sensor.get('measurement_valid') is not True or
                sensor.get('id_valid') is not True):
            return None
        distance, angle, age, remaining, sequence = (sensor.get(name) for name in
            ('range_m', 'raw_angle_deg', 'age_ms', 'remaining_ms', 'sample_sequence'))
        if (not number(distance, .05, 20) or not number(angle, -180, 180) or not number(age, 0, 300) or
                not number(remaining, 0, 1000) or remaining == 0 or type(sequence) is not int or sequence < 1):
            return None
        return float(distance), float(angle), float(age), sequence

    def observe(self, payload):
        if (not isinstance(payload, dict) or payload.get('schema') != 'smartcart.dual_uwb_view.v1' or
                payload.get('source_mode') != 'live' or payload.get('state') != 'running' or
                payload.get('drive_authorized') is not False or not isinstance(payload.get('sensors'), dict) or
                set(payload['sensors']) != set(ROLES)):
            self.malformed_polls += 1
            self.last_pair = None
            return
        self.successful_polls += 1
        observations = {}
        for role in ROLES:
            sensor = payload['sensors'][role]
            state = sensor.get('state') if isinstance(sensor, dict) else None
            state = state if state in ('waiting', 'live', 'invalid', 'stale', 'disconnected', 'error') else 'unknown'
            self.states[role][state] += 1
            observation = self._observation(sensor)
            if observation is None:
                continue
            distance, angle, _age, sequence = observation
            entry = self.values[role]
            previous = entry['last_seq']
            if previous is not None and sequence < previous:
                entry['resets'] += 1
                self.last_pair = None
            if previous is None or sequence != previous:
                entry['range'].append(distance)
                entry['angle'].append(angle)
                entry['last_seq'] = sequence
            observations[role] = observation
        if len(observations) != 2:
            return
        left, right = observations['uwb_left'], observations['uwb_right']
        if abs(left[2] - right[2]) > 100:
            return
        pair = left[3], right[3]
        if self.last_pair is not None and not all(a > b for a, b in zip(pair, self.last_pair)):
            return
        self.last_pair = pair
        for key, sign in (('current', -1), ('reversed_sign_hypothesis', 1)):
            self.residuals[key].append(residual(left[:2], right[:2], self.baseline_m, sign))

    def summary(self, elapsed, http_errors):
        sensors = {}
        for role in ROLES:
            data = self.values[role]
            sensors[role] = {'state_samples': dict(self.states[role]), 'unique_live_observations': len(data['range']),
                             'range_m': summarize(data['range']), 'raw_angle_deg': summarize(data['angle']),
                             'sequence_resets': data['resets']}
        return {'schema': 'smartcart.dual_geometry_monitor.v1', 'elapsed_seconds': round(elapsed, 2),
                'successful_polls': self.successful_polls, 'malformed_polls': self.malformed_polls,
                'http_errors': http_errors, 'baseline_assumption_m': self.baseline_m, 'sensors': sensors,
                'paired_unique_samples': len(self.residuals['current']),
                'pair_residual_m': {key: summarize(value) for key, value in self.residuals.items()},
                'drive_authorized': False,
                'notice': '웹 표본·가정 비교이며 전체 무선 프레임, 각도 부호 확정 또는 거리 정확도 인증이 아닙니다.'}


def has_required_input(monitor, required):
    """단독 수신은 해당 쪽의 신규 관측만 확인하고 좌우 쌍으로 취급하지 않는다."""
    if required == 'both':
        return bool(monitor.residuals['current'])
    if required in ('left', 'right'):
        return bool(monitor.values['uwb_' + required]['range'])
    raise ValueError('알 수 없는 필수 수신 조건')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--require', choices=('both', 'left', 'right'), default='both',
                        help='결과 성공에 필요한 수신 장치. 단독 측정에서는 해당 센서만 지정합니다.')
    args = parser.parse_args()
    if not number(args.seconds, 1, 60):
        parser.error('시간은 1~60초의 유한한 수여야 합니다')
    opener = build_opener(ProxyHandler({}), NoRedirect())
    monitor, errors = GeometryMonitor(), 0
    started = time.monotonic()
    deadline = started + args.seconds
    while time.monotonic() < deadline:
        tick = time.monotonic()
        try:
            with opener.open(URL, timeout=1) as response:
                raw = response.read(MAX_BYTES + 1)
            if len(raw) > MAX_BYTES:
                raise ValueError('응답 크기 초과')
            value = json.loads(raw, parse_constant=lambda _: (_ for _ in ()).throw(ValueError('비유한 수')))
            monitor.observe(value)
        except (OSError, URLError, ValueError, TypeError, UnicodeError, KeyError, RecursionError):
            errors += 1
        time.sleep(max(0, min(.1 - (time.monotonic() - tick), deadline - time.monotonic())))
    report = monitor.summary(time.monotonic() - started, errors)
    report['required_input'] = args.require
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if has_required_input(monitor, args.require) else 3


if __name__ == '__main__':
    raise SystemExit(main())
