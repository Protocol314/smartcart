#!/usr/bin/env python3
"""0.3.1 설치본에 필요한 표시 파일만 넣은 단일 검증·백업 설치기를 생성한다."""
import base64
import hashlib
import json
from pathlib import Path
import zipfile
import zlib

ROOT = Path(__file__).resolve().parents[2]
FILES = (
    'smartcart_sensing/dual_uwb.py', 'smartcart_sensing/app.py',
    'smartcart_sensing/service.py', 'smartcart_sensing/server.py',
    'smartcart_tracking/ros_runtime.py', 'web/index.html', 'web/css/app.css',
    'web/js/app.js', 'web/js/dual_uwb.js',
    'smartcart_runner/supervisor.py',
)
NAME = 'smartcart-dual-uwb-ui-20261027-install.py'
TEMPLATE = r'''#!/usr/bin/env python3
"""좌우 BU04 실측 표시 업데이트. 기본은 검사만 하며 --apply에서 백업 후 반영한다."""
import argparse
import ast
import base64
import hashlib
import json
import os
from pathlib import Path
import tempfile
import zlib

PAYLOAD = '__PAYLOAD__'
DIGEST = '__DIGEST__'
PATHS = __PATHS__


def digest(data):
    return hashlib.sha256(data).hexdigest()


def atomic(path, data, mode=0o644):
    fd, name = tempfile.mkstemp(prefix='.smartcart-ui-', dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.chmod(mode)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path.home() / 'SmartCart-D5-v2')
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    root = args.root.expanduser().resolve(strict=True)
    app = root / 'smartcart'
    if not (app / 'cart.py').is_file():
        raise ValueError('기존 SmartCart-D5-v2 설치 폴더를 찾지 못했습니다')
    encoded = zlib.decompress(base64.b64decode(PAYLOAD, validate=True))
    if digest(encoded) != DIGEST or len(encoded) > 1024 * 1024:
        raise ValueError('내장 업데이트 검증에 실패했습니다')
    payload = json.loads(encoded)
    if set(payload) != set(PATHS):
        raise ValueError('업데이트 파일 목록이 일치하지 않습니다')
    pending = []
    for relative, entry in payload.items():
        path = app / relative
        if path.is_symlink() or any(parent.is_symlink() for parent in path.parents if parent != root):
            raise ValueError('링크를 통해 업데이트할 수 없습니다: ' + relative)
        if not path.parent.is_dir() or not path.resolve().is_relative_to(app.resolve()):
            raise ValueError('업데이트 경로가 잘못되었습니다: ' + relative)
        data = entry['text'].encode('utf-8')
        if digest(data) != entry['new_sha256']:
            raise ValueError('새 파일 해시가 일치하지 않습니다: ' + relative)
        if relative.endswith('.py'):
            ast.parse(entry['text'], feature_version=(3, 10))
        old = path.read_bytes() if path.exists() else None
        old_digest = digest(old) if old is not None else None
        if old_digest == entry['new_sha256']:
            continue
        if old_digest != entry['old_sha256']:
            raise ValueError('알 수 없는 현지 수정이 있어 덮어쓰지 않습니다: ' + relative)
        pending.append((path, relative, old, data, path.stat().st_mode & 0o777 if old is not None else 0o644))
    print('업데이트 검증 완료: %d개 파일, 변경 필요 %d개' % (len(payload), len(pending)))
    if not args.apply:
        print('검사만 완료했습니다. 통합 프로그램을 Ctrl+C로 종료한 뒤 --apply로 반영하십시오.')
        return 0
    if not pending:
        print('이미 같은 업데이트가 적용되어 있습니다.')
        return 0
    backup = Path(tempfile.mkdtemp(prefix='smartcart-ui-backup-', dir=root))
    backup.chmod(0o700)
    manifest = {}
    for path, relative, old, _data, mode in pending:
        manifest[relative] = {'existed': old is not None, 'sha256': digest(old) if old is not None else None}
        if old is not None:
            target = backup / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            atomic(target, old, mode)
    atomic(backup / 'backup.json', json.dumps(manifest, ensure_ascii=False, indent=2).encode('utf-8'), 0o600)
    written = []
    try:
        for item in pending:
            path, relative, old, data, mode = item
            # 검증 뒤 바뀐 파일은 덮어쓰지 않는다.
            if (path.read_bytes() if path.exists() else None) != old:
                raise ValueError('검증 뒤 파일이 변경되었습니다: ' + relative)
            atomic(path, data, mode)
            written.append(item)
    except Exception:
        for path, _relative, old, _data, mode in reversed(written):
            if old is None:
                path.unlink(missing_ok=True)
            else:
                atomic(path, old, mode)
        raise
    print('업데이트 완료. 변경 전 백업: ' + str(backup))
    print('장치 배정·보정·ROS 빌드·드라이버·센서 설정은 변경하지 않았습니다.')
    print('시작: bash ~/SmartCart-D5-v2/smartcart/scripts/start-live.sh')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError) as error:
        raise SystemExit('업데이트 중단: ' + str(error))
'''


def build(output_dir=None):
    release = ROOT / 'build/releases'
    original = release / 'smartcart-d5-v2-pi4-arm64-v0.3.1.zip'
    output = release if output_dir is None else Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    payload = {}
    with zipfile.ZipFile(original) as archive:
        names = archive.namelist()
        for relative in FILES:
            suffix = 'smartcart/' + relative
            matches = [name for name in names if name.endswith(suffix)]
            if len(matches) > 1:
                raise ValueError('원본 파일이 중복되었습니다: ' + relative)
            old = archive.read(matches[0]) if matches else None
            data = (ROOT / suffix).read_bytes()
            payload[relative] = {'old_sha256': hashlib.sha256(old).hexdigest() if old is not None else None,
                                 'new_sha256': hashlib.sha256(data).hexdigest(), 'text': data.decode('utf-8')}
    wire = json.dumps(payload, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
    source = TEMPLATE.replace('__PAYLOAD__', base64.b64encode(zlib.compress(wire, 9)).decode('ascii'))
    source = source.replace('__DIGEST__', hashlib.sha256(wire).hexdigest()).replace('__PATHS__', repr(FILES))
    target = output / NAME
    target.write_bytes(source.encode('utf-8'))
    sha = hashlib.sha256(target.read_bytes()).hexdigest()
    target.with_suffix('.py.sha256').write_text(sha + '  ' + NAME + '\n', encoding='ascii')
    print(NAME + ' SHA256=' + sha)
    return target


if __name__ == '__main__':
    build()
