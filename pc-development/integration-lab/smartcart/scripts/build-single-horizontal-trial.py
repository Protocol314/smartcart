#!/usr/bin/env python3
"""원본 설정을 보존하는 왼쪽 단독 수평거리 시험 실행기만 배포한다."""
import base64
import hashlib
import importlib.util
import json
from pathlib import Path
import zlib

ROOT = Path(__file__).resolve().parents[2]
RELEASE = ROOT / 'build/releases'
NAME = 'smartcart-single-horizontal-trial-20261117-install.py'
FILES = ('smartcart_runner/single_trial.py', 'scripts/start-single-left-trial.sh')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def build():
    payload = {}
    for path in FILES:
        data = (ROOT / 'smartcart' / path).read_bytes()
        payload[path] = {'old_sha256': None, 'new_sha256': digest(data), 'text': data.decode('utf-8')}
    wire = json.dumps(payload, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
    spec = importlib.util.spec_from_file_location('template', ROOT / 'smartcart/scripts/build-dual-uwb-update.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    source = module.TEMPLATE.replace('좌우 BU04 실측 표시 업데이트', '왼쪽 단독 수평거리 가정 시험 업데이트')
    source = source.replace('smartcart-ui-backup-', 'smartcart-horizontal-trial-backup-')
    source = source.replace('__PAYLOAD__', base64.b64encode(zlib.compress(wire, 9)).decode('ascii'))
    source = source.replace('__DIGEST__', digest(wire)).replace('__PATHS__', repr(FILES))
    source = source.replace('시작: bash ~/SmartCart-D5-v2/smartcart/scripts/start-live.sh',
                            '시작: bash ~/SmartCart-D5-v2/smartcart/scripts/start-single-left-trial.sh')
    data = source.encode('utf-8')
    target = RELEASE / NAME
    if target.exists() and target.read_bytes() != data:
        raise FileExistsError('기존 설치기를 덮어쓰지 않습니다')
    target.write_bytes(data)
    target.with_suffix('.py.sha256').write_text(digest(data) + '  ' + NAME + '\n', encoding='ascii')
    print(NAME + ' SHA256=' + digest(data))


if __name__ == '__main__':
    build()
