#!/usr/bin/env python3
"""Pi 빌드·센싱 상태를 읽기 전용으로 확인한다. 위치와 키를 출력하지 않는다."""
import argparse
import importlib.util
import json
import math
import platform
from pathlib import Path
import shutil
import sys
import time
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parent
MAX_BYTES = 524288


def get_json(path):
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open('http://127.0.0.1:8765' + path, timeout=2) as response:
        data = response.read(MAX_BYTES + 1)
    if len(data) > MAX_BYTES:
        raise ValueError('응답 크기 초과')
    value = json.loads(data)
    if not isinstance(value, dict):
        raise ValueError('응답 형식 오류')
    return value


def scan_summary(frame):
    scan = frame.get('scan')
    if not isinstance(scan, dict):
        return False, '스캔 없음'
    values = scan.get('ranges') or []
    low, high = scan.get('range_min'), scan.get('range_max')
    if not isinstance(values, list) or not all(type(v) in (int, float) and math.isfinite(v) for v in (low, high)):
        return False, '스캔 형식 오류'
    invalid = sum(type(v) not in (int, float) or not math.isfinite(v) or not low <= v <= high for v in values)
    near = sum(type(v) in (int, float) and math.isfinite(v) and low <= v <= high and v <= 1.0 for v in values)
    age = frame.get('age_ms')
    angles = [scan.get(key) for key in ('angle_min', 'angle_max', 'angle_increment')]
    geometry = (all(type(v) in (int, float) and math.isfinite(v) for v in angles) and
                0 < angles[2] <= .1 and abs((angles[1]-angles[0])-(len(values)-1)*angles[2]) <= 1.5*angles[2] and
                2*math.pi-1.5*angles[2] <= (len(values)-1)*angles[2] <= 2*math.pi+angles[2])
    ready = (frame.get('schema') == 'smartcart.envelope.v1' and frame.get('source_mode') == 'ros' and
             frame.get('state') == 'live' and type(age) in (int, float) and math.isfinite(age) and 0 <= age < 300 and
             3 <= len(values) <= 4096 and 0 <= low <= 1.0 <= high and not scan.get('truncated') and
             geometry and invalid == 0 and near == 0)
    return ready, f"상태={frame.get('state')} 출처={frame.get('source_mode')} 총={len(values)} 무효={invalid} 1m이내={near} 나이ms={age}"


def uwb_summary(uwb):
    sensors = uwb.get('sensors')
    left = sensors.get('uwb_left') if isinstance(sensors, dict) else None
    if not isinstance(left, dict):
        return False, 'BU04 상태 없음'
    ready = (uwb.get('schema') == 'smartcart.dual_uwb_view.v1' and uwb.get('source_mode') == 'live' and
             uwb.get('state') == 'running' and uwb.get('profile') == 'single_left' and
             left.get('state') == 'live' and
             all(left.get(k) is True for k in ('connected', 'valid', 'measurement_valid', 'id_valid')) and
             type(left.get('remaining_ms')) in (int, float) and math.isfinite(left['remaining_ms']) and 0 < left['remaining_ms'] <= 1000 and
             type(left.get('age_ms')) in (int, float) and math.isfinite(left['age_ms']) and 0 <= left['age_ms'] < 300)
    return ready, f"상태={left.get('state')} 연결={left.get('connected')} 측정={left.get('measurement_valid')} ID={left.get('id_valid')} 사유={left.get('invalid_reason')}"


def tracking_summary(tracking):
    result = tracking.get('result')
    track = result.get('track') if isinstance(result, dict) else None
    if not isinstance(track, dict):
        return False, '추적 결과 없음'
    ready = (tracking.get('schema') == 'smartcart.tracking_view.v1' and tracking.get('source_mode') == 'live' and
             tracking.get('state') == 'running' and tracking.get('valid') is True and
             track.get('state') == 'tracking' and track.get('valid') is True and track.get('candidate_count') == 1 and
             type(track.get('confirmation_count')) is int and track['confirmation_count'] >= 3 and
             type(tracking.get('remaining_ms')) in (int, float) and math.isfinite(tracking['remaining_ms']) and tracking['remaining_ms'] > 0)
    return ready, f"상태={track.get('state')} 유효={track.get('valid')} 후보={track.get('candidate_count')} 사유={track.get('invalid_reason')}"


def check_environment():
    required = [('ARM64', platform.machine() in ('aarch64', 'arm64')),
                ('ROS Humble', Path('/opt/ros/humble/setup.bash').is_file()),
                ('colcon', bool(shutil.which('colcon'))), ('gcc', bool(shutil.which('gcc'))),
                ('aiohttp', importlib.util.find_spec('aiohttp') is not None),
                ('rclpy', importlib.util.find_spec('rclpy') is not None),
                ('serial', importlib.util.find_spec('serial') is not None)]
    for name, present in required:
        print(f"{name}: {'확인' if present else '없음'}")
    for name, path in [('현장 실행 설정', ROOT/'SmartCart-D5-v2/smartcart/local-config/run.json'),
                       ('ROS 빌드 결과', ROOT/'.runtime/install/setup.bash'),
                       ('BU04 파서', ROOT/'SmartCart-D5-v2/smartcart/smartcart_uwb/lib/libsmartcart_uwb_parser.so')]:
        print(f"{name}: {'있음' if path.is_file() else '없음'}")
    print('확인 범위: 파일·패키지 존재 여부. 빌드 성공이나 장치 정확도 검수가 아님.')
    return all(present for _, present in required)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sensors', action='store_true', help='기존 8765 서버의 센서 상태도 5회 조회')
    args = parser.parse_args()
    environment_ok = check_environment()
    if not args.sensors:
        return 0 if environment_ok else 1
    recent = []
    for index in range(5):
        try:
            values = [scan_summary(get_json('/api/frame')), uwb_summary(get_json('/api/dual-uwb')),
                      tracking_summary(get_json('/api/tracking'))]
            print(f'표본 {index+1}:', ' | '.join(name + ' ' + value[1] for name, value in zip(('LiDAR','BU04','추적'), values)))
            recent.append(all(value[0] for value in values))
        except (OSError, ValueError, TypeError, urllib.error.URLError) as error:
            print(f'표본 {index+1}: 조회 실패 ({type(error).__name__})')
            recent.append(False)
        if index < 4:
            time.sleep(.25)
    print('판정: 최근 5회 모두 유효' if all(recent) else '판정: 유효하지 않은 표본 또는 조회 실패 있음')
    print('센서 연결과 가상 입력 진단이며, 모터 주행 승인이나 실제 거리 정확도 검수가 아닙니다.')
    return 0 if environment_ok and all(recent) else 1

if __name__ == '__main__':
    sys.exit(main())
