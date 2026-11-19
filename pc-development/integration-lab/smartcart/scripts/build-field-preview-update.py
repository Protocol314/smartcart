#!/usr/bin/env python3
"""이전 Pi 화면 설치본을 기준으로 검증·백업 가능한 실험 표시 설치기를 만든다."""
import ast
import base64
import hashlib
import importlib.util
import json
from pathlib import Path
import zipfile
import zlib

ROOT = Path(__file__).resolve().parents[2]
RELEASE = ROOT / 'build' / 'releases'
PREVIOUS = RELEASE / 'smartcart-dual-uwb-ui-20261027-install.py'
PREVIOUS_SHA256 = '172dbdaf5c82f383b758ef35d7566c0cb4f07ac3e84601b6292452bd37acdbde'
BASE = RELEASE / 'smartcart-d5-v2-pi4-arm64-v0.3.1.zip'
FILES = ('web/index.html', 'web/css/app.css', 'web/js/app.js',
         'web/js/radar.js', 'web/js/field_preview.js')
NAME = 'smartcart-field-preview-20261103-install.py'


def digest(data):
    return hashlib.sha256(data).hexdigest()


def previous_payload():
    source = PREVIOUS.read_bytes()
    if digest(source) != PREVIOUS_SHA256:
        raise ValueError('이전 화면 설치기의 확인된 해시가 달라 기준 파일을 사용할 수 없습니다')
    values = {}
    for item in ast.parse(source.decode('utf-8')).body:
        if (isinstance(item, ast.Assign) and len(item.targets) == 1 and
                isinstance(item.targets[0], ast.Name) and item.targets[0].id in ('PAYLOAD', 'DIGEST')):
            values[item.targets[0].id] = ast.literal_eval(item.value)
    raw = zlib.decompress(base64.b64decode(values['PAYLOAD'], validate=True))
    if len(raw) > 1024 * 1024 or digest(raw) != values['DIGEST']:
        raise ValueError('이전 화면 설치기의 내부 자료 검증에 실패했습니다')
    return json.loads(raw)


def baseline():
    prior, result = previous_payload(), {}
    with zipfile.ZipFile(BASE) as archive:
        names = archive.namelist()
        for relative in FILES:
            if relative in prior:
                data = prior[relative]['text'].encode('utf-8')
                if digest(data) != prior[relative]['new_sha256']:
                    raise ValueError('이전 화면 파일의 해시가 다릅니다')
                result[relative] = data
                continue
            matches = [name for name in names if name.endswith('smartcart/' + relative)]
            if len(matches) > 1:
                raise ValueError('기준 묶음에 파일이 중복되었습니다')
            result[relative] = archive.read(matches[0]) if matches else None
    return result


def build(directory=RELEASE):
    spec = importlib.util.spec_from_file_location('smartcart_ui_update_template',
                                                  ROOT / 'smartcart/scripts/build-dual-uwb-update.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    payload = {}
    for relative, old in baseline().items():
        data = (ROOT / 'smartcart' / relative).read_bytes()
        payload[relative] = {'old_sha256': digest(old) if old is not None else None,
                             'new_sha256': digest(data), 'text': data.decode('utf-8')}
    wire = json.dumps(payload, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
    source = module.TEMPLATE.replace('좌우 BU04 실측 표시 업데이트', '좌우 UWB 위치 비교 실험 표시 업데이트')
    source = source.replace('smartcart-ui-backup-', 'smartcart-preview-backup-')
    source = source.replace('__PAYLOAD__', base64.b64encode(zlib.compress(wire, 9)).decode('ascii'))
    source = source.replace('__DIGEST__', digest(wire)).replace('__PATHS__', repr(FILES))
    data = source.encode('utf-8')
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    target = directory / NAME
    if target.exists() and target.read_bytes() != data:
        raise FileExistsError('기존 실험 표시 설치기를 덮어쓰지 않습니다: ' + str(target))
    target.write_bytes(data)
    checksum = target.with_suffix('.py.sha256')
    checksum.write_text(digest(data) + '  ' + NAME + '\n', encoding='ascii')
    return target


if __name__ == '__main__':
    target = build()
    print(NAME + ' SHA256=' + digest(target.read_bytes()))
