#!/usr/bin/env python3
"""Pi의 기존 실험 화면을 보존·백업하고 표시 필터만 갱신하는 설치기를 만든다."""
import ast
import base64
import hashlib
import importlib.util
import json
from pathlib import Path
import zlib

ROOT = Path(__file__).resolve().parents[2]
RELEASE = ROOT / 'build' / 'releases'
PREVIOUS = RELEASE / 'smartcart-field-preview-20261103-install.py'
PREVIOUS_SHA256 = '74f54d152aa5f0fd029f08e93dbbcd20198e1e6fe726a4ed03dd0894a2dcf2ee'
FILES = ('web/index.html', 'web/js/app.js', 'web/js/field_preview.js')
NAME = 'smartcart-field-smoothing-20261103-install.py'


def digest(data):
    return hashlib.sha256(data).hexdigest()


def baseline():
    source = PREVIOUS.read_bytes()
    if digest(source) != PREVIOUS_SHA256:
        raise ValueError('기준 실험 화면 설치기의 해시가 다릅니다')
    values = {}
    for item in ast.parse(source.decode('utf-8')).body:
        if (isinstance(item, ast.Assign) and len(item.targets) == 1 and
                isinstance(item.targets[0], ast.Name) and item.targets[0].id in ('PAYLOAD', 'DIGEST')):
            values[item.targets[0].id] = ast.literal_eval(item.value)
    raw = zlib.decompress(base64.b64decode(values['PAYLOAD'], validate=True))
    if len(raw) > 1024 * 1024 or digest(raw) != values['DIGEST']:
        raise ValueError('기준 실험 화면 자료의 검증에 실패했습니다')
    previous = json.loads(raw)
    result = {}
    for relative in FILES:
        data = previous[relative]['text'].encode('utf-8')
        if digest(data) != previous[relative]['new_sha256']:
            raise ValueError('기준 화면 파일의 해시가 다릅니다')
        result[relative] = data
    return result


def build(directory=RELEASE):
    spec = importlib.util.spec_from_file_location('smartcart_ui_template',
                                                  ROOT / 'smartcart/scripts/build-dual-uwb-update.py')
    template = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(template)
    payload = {}
    for relative, old in baseline().items():
        data = (ROOT / 'smartcart' / relative).read_bytes()
        payload[relative] = {'old_sha256': digest(old), 'new_sha256': digest(data),
                             'text': data.decode('utf-8')}
    wire = json.dumps(payload, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
    source = template.TEMPLATE.replace('좌우 BU04 실측 표시 업데이트', '좌우 UWB 실험 표시 필터 업데이트')
    source = source.replace('smartcart-ui-backup-', 'smartcart-smoothing-backup-')
    source = source.replace('__PAYLOAD__', base64.b64encode(zlib.compress(wire, 9)).decode('ascii'))
    source = source.replace('__DIGEST__', digest(wire)).replace('__PATHS__', repr(FILES))
    data = source.encode('utf-8')
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    target = directory / NAME
    if target.exists() and target.read_bytes() != data:
        raise FileExistsError('기존 표시 필터 설치기를 덮어쓰지 않습니다: ' + str(target))
    target.write_bytes(data)
    target.with_suffix('.py.sha256').write_text(digest(data) + '  ' + NAME + '\n', encoding='ascii')
    return target


if __name__ == '__main__':
    target = build()
    print(NAME + ' SHA256=' + digest(target.read_bytes()))
