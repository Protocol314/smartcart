#!/usr/bin/env python3
"""물리 라벨로 확인한 세 USB를 로컬 실측 설정에 배정한다. 포트는 열지 않는다."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from smartcart_field.config import field_config_from_dict, load_field_config
from smartcart_field.identity import check_identity, enumerate_inventory
from smartcart_runner.config import initialize, load_config as load_run_config
from smartcart_uwb.config import config_from_dict, load_config as load_mount_config
from smartcart_uwb.serial_reader import canonical_port


def prepare(root, serials, tag, *, inventory=None):
    if type(tag) is not int or not 0 <= tag <= 65535:
        raise ValueError('태그 짧은 주소는 0~65535 정수여야 합니다')
    if len(set(serials)) != 3 or any(type(s) is not str or not 1 <= len(s) <= 128
                                  or any(ord(c) < 32 for c in s) for s in serials):
        raise ValueError('물리 확인한 서로 다른 세 USB 시리얼 번호가 필요합니다')
    inventory = enumerate_inventory() if inventory is None else inventory
    selected = []
    for serial, usb in zip(serials, ((0x0483, 0x5740), (0x0483, 0x5740), (0x10C4, 0xEA60))):
        matches = [item for item in inventory if item['serial_number'] == serial
                   and (item['vid'], item['pid']) == usb]
        if len(matches) != 1:
            raise ValueError('지정한 USB를 현재 목록에서 고유하게 확인하지 못했습니다')
        item = matches[0]
        aliases = [path for path in Path('/dev/serial/by-id').glob('*')
                   if canonical_port(str(path)) == item['canonical_port']]
        selected.append((str(aliases[0]) if len(aliases) == 1 else item['port'], item['identity']))
    directory = Path(root) / 'local-config'
    if directory.is_symlink():
        raise ValueError('로컬 설정 디렉터리는 심볼릭 링크가 아니어야 합니다')
    if not directory.exists():
        initialize(directory)
    paths = {name: directory / (name + '.json') for name in ('mounting', 'field', 'run')}
    if any(path.is_symlink() for path in paths.values()):
        raise ValueError('변경 대상 설정 파일은 심볼릭 링크가 아니어야 합니다')
    current_run = load_run_config(paths['run'])
    for name in ('mounting', 'field', 'uwb', 'tracking'):
        if getattr(current_run, name + '_config') != (directory / (name + '.json')).resolve():
            raise ValueError('이 도구는 local-config 안의 기본 파일 구성을 사용합니다')
    load_mount_config(paths['mounting'])
    load_field_config(paths['field'])
    originals = {name: path.read_bytes() for name, path in paths.items()}
    data = {name: json.loads(value) for name, value in originals.items()}
    for role, (port, identity) in zip(('uwb_left', 'uwb_right'), selected[:2]):
        data['mounting']['sensors'][role].update(port=port, expected_device_identity=identity,
                                               device_identity_verified=True)
    data['mounting']['allowed_tag_address'] = tag
    data['mounting']['lidar']['frame_id'] = 'lidar_scan'
    data['field']['lidar_device'].update(port=selected[2][0], expected_device_identity=selected[2][1],
                                        device_identity_verified=True)
    data['field']['expected_lidar_frame_id'] = 'lidar_scan'
    data['run'].update(lidar_port=selected[2][0], lidar_mode='managed', lidar_frame_id='lidar_scan')
    report = check_identity(config_from_dict(data['mounting']), field_config_from_dict(data['field']), inventory)
    if not report['ready']:
        raise ValueError('세 장치의 독립성과 현재 USB 식별 대조에 실패했습니다')
    fd, staged = tempfile.mkstemp(prefix='validate-live-', dir=directory)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as handle:
            json.dump(data['run'], handle, allow_nan=False)
        load_run_config(staged)
    finally:
        Path(staged).unlink()
    backup = Path(tempfile.mkdtemp(prefix='before-live-binding-', dir=directory))
    for name, raw in originals.items():
        fd = os.open(backup / (name + '.json'), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'wb') as handle:
            handle.write(raw)
    for name, path in paths.items():
        fd, temporary = tempfile.mkstemp(prefix='pending-live-', dir=directory)
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as handle:
                json.dump(data[name], handle, ensure_ascii=False, indent=2, allow_nan=False)
                handle.write('\n')
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
    return backup


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--left-serial', required=True)
    parser.add_argument('--right-serial', required=True)
    parser.add_argument('--lidar-serial', required=True)
    parser.add_argument('--tag-address', required=True, type=lambda value: int(value, 0))
    parser.add_argument('--confirmed-physical-labels', action='store_true')
    args = parser.parse_args()
    if not args.confirmed_physical_labels:
        parser.error('좌우·LiDAR의 물리 라벨을 대조한 뒤 --confirmed-physical-labels를 지정하십시오')
    try:
        backup = prepare(ROOT, (args.left_serial, args.right_serial, args.lidar_serial), args.tag_address)
    except (ValueError, OSError, ImportError) as error:
        print('실측 장치 배정 오류: ' + str(error), file=sys.stderr)
        return 2
    print('장치 배정 완료. 장착·거리 검수 상태는 기존 값을 유지했습니다.')
    print('변경 전 백업:', backup)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
