#!/usr/bin/env python3
"""Pi에서 확인한 화면 해시를 기준으로 표시 좌우반전 설치기를 만든다."""
import base64
import hashlib
import importlib.util
import json
from pathlib import Path
import zlib

ROOT = Path(__file__).resolve().parents[2]
PATHS = {
    'web/js/core.js': '42d175ce56f01813387764bf3c06e564095195fb9877db0262b827f84aa3b4a9',
    'web/js/radar.js': '4b54f544efec126ae3ec53e83a8ffd1f4788d67698317cc46c21a383e2c03c32',
    'web/index.html': '15f16cb2029c7c852841b6f340522732d4c8457da6ad223083f073ad739fd5d5',
}


def digest(value):
    return hashlib.sha256(value).hexdigest()


def main():
    payload = {}
    for path, previous in PATHS.items():
        data = (ROOT / 'smartcart' / path).read_bytes()
        payload[path] = {'old_sha256': previous, 'new_sha256': digest(data),
                         'text': data.decode('utf-8')}
    wire = json.dumps(payload, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
    template_path = ROOT / 'smartcart/scripts/build-dual-uwb-update.py'
    spec = importlib.util.spec_from_file_location('template', template_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    source = module.TEMPLATE.replace('좌우 BU04 실측 표시 업데이트', '레이더 화면 좌우반전 업데이트')
    source = source.replace('smartcart-ui-backup-', 'smartcart-mirrored-view-backup-')
    source = source.replace('__PAYLOAD__', base64.b64encode(zlib.compress(wire, 9)).decode('ascii'))
    source = source.replace('__DIGEST__', digest(wire)).replace('__PATHS__', repr(tuple(PATHS)))
    source = source.replace('시작: bash ~/SmartCart-D5-v2/smartcart/scripts/start-live.sh',
                            '기존 센싱 실행 명령으로 재시작하십시오.')
    target = ROOT / 'build/releases/smartcart-mirrored-view-20261117-install.py'
    content = source.encode('utf-8')
    if target.exists() and target.read_bytes() != content:
        raise FileExistsError('기존 설치 파일을 덮어쓰지 않습니다')
    target.write_bytes(content)
    print('설치기:', target)
    print('SHA256:', digest(content))


if __name__ == '__main__':
    main()
