#!/usr/bin/env python3
"""기존 레이더 화면에 브라우저 전용 가상 조종을 추가하는 설치기를 만든다."""
import ast
import base64
import hashlib
import importlib.util
import json
from pathlib import Path
import zlib

ROOT = Path(__file__).resolve().parents[2]
RELEASE = ROOT / 'build/releases'
NAME = 'smartcart-controller-layout-20261117-install.py'
FILES = ('web/index.html', 'web/css/app.css', 'web/js/virtual_controls.js')
PREVIOUS = (('smartcart-field-preview-20261103-install.py', '74f54d152aa5f0fd029f08e93dbbcd20198e1e6fe726a4ed03dd0894a2dcf2ee'),
            ('smartcart-field-smoothing-20261103-install.py', 'f398f06f89429bfea24737d4bc00453270be4b3839512ea3c9e13d37bd70285f'),
            ('smartcart-single-left-preview-20261117-install.py', 'a2211b947d646261424ce2b70b0d060e8802149916eba0a29004610bab0c7805'),
            ('smartcart-integrated-virtual-20261117-install.py', '8439e436b52f934cfe950fe75f0fc5e246c666e4018afb6441a6d184e18ea505'))


def digest(data):
    return hashlib.sha256(data).hexdigest()


def build():
    import zipfile
    baseline = {}
    with zipfile.ZipFile(RELEASE / 'smartcart-d5-v2-pi4-arm64-v0.3.1.zip') as archive:
        for name in FILES:
            path = 'SmartCart-D5-v2/smartcart/' + name
            baseline[name] = archive.read(path) if path in archive.namelist() else None
    for name, expected in PREVIOUS:
        source = (RELEASE / name).read_bytes()
        if digest(source) != expected:
            raise ValueError('이전 설치기 해시가 일치하지 않습니다: ' + name)
        constants = {}
        for node in ast.parse(source.decode('utf-8')).body:
            if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) and node.targets[0].id in ('PAYLOAD', 'DIGEST'):
                constants[node.targets[0].id] = ast.literal_eval(node.value)
        raw = zlib.decompress(base64.b64decode(constants['PAYLOAD'], validate=True))
        if digest(raw) != constants['DIGEST']:
            raise ValueError('이전 설치기 내용이 일치하지 않습니다')
        for path, item in json.loads(raw).items():
            if path in baseline:
                data = item['text'].encode('utf-8')
                if digest(data) != item['new_sha256']:
                    raise ValueError('이전 파일 해시가 일치하지 않습니다')
                baseline[path] = data
    payload = {}
    for path in FILES:
        data = (ROOT / 'smartcart' / path).read_bytes()
        payload[path] = {'old_sha256': digest(baseline[path]) if baseline[path] is not None else None,
                         'new_sha256': digest(data), 'text': data.decode('utf-8')}
    wire = json.dumps(payload, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
    spec = importlib.util.spec_from_file_location('template', ROOT / 'smartcart/scripts/build-dual-uwb-update.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    source = module.TEMPLATE.replace('좌우 BU04 실측 표시 업데이트', '목업 기반 가상 조종 화면 업데이트')
    source = source.replace('smartcart-ui-backup-', 'smartcart-controller-layout-backup-')
    source = source.replace('__PAYLOAD__', base64.b64encode(zlib.compress(wire, 9)).decode('ascii'))
    source = source.replace('__DIGEST__', digest(wire)).replace('__PATHS__', repr(FILES))
    source = source.replace('시작: bash ~/SmartCart-D5-v2/smartcart/scripts/start-live.sh',
                            '기존 센싱 실행 명령으로 재시작하십시오.')
    data = source.encode('utf-8')
    target = RELEASE / NAME
    if target.exists() and target.read_bytes() != data:
        raise FileExistsError('기존 설치기를 덮어쓰지 않습니다')
    target.write_bytes(data)
    target.with_suffix('.py.sha256').write_text(digest(data) + '  ' + NAME + '\n', encoding='ascii')
    print(NAME + ' SHA256=' + digest(data))


if __name__ == '__main__':
    build()
