#!/usr/bin/env python3
"""수동 조종 화면의 좌우 반전 설치기를 생성한다."""
import base64
import hashlib
import json
from pathlib import Path
import zlib

ROOT = Path(__file__).resolve().parents[2]
WEB = ROOT / 'SmartCart_ManualDrive_180/smartcart/web'
files = {}
reverts = {
    'js/core.js': [('  // 화면만 좌우 반전하며 ROS 좌표와 센서 측정값은 변경하지 않는다.\n', ''),
                   ('x: cx + point.y * scale', 'x: cx - point.y * scale')],
    'js/radar.js': [('// 각도는 ROS 좌표 기준이며 화면 위치만 좌우 반전한다.', '// ROS convention: +90 to the left, unlike clockwise compass bearings.'),
                    ("this.text(g, '−90°', cx - R - 7", "this.text(g, '90°', cx - R - 7"),
                    ("this.text(g, '90°', cx + R + 7", "this.text(g, '−90°', cx + R + 7")],
    'index.html': [('· 좌우 반전 표시 / ROS +Y 좌측', '· +X 전방 / +Y 좌측'),
                   ('ROS +X 전방 · +Y 좌측. 화면은 좌우 반전 표시합니다. 추적 서버 사용 중에는 배경·군집·대상이 같은 장착 좌표를 사용합니다. 로컬 LiDAR 보기의 장착 보정은 표시만 변경합니다.',
                    '+X 전방 · +Y 좌측. 추적 서버 사용 중에는 배경·군집·대상이 같은 장착 좌표를 사용합니다. 로컬 LiDAR 보기의 장착 보정은 표시만 변경합니다. 모터 제어 없음.')],
}
for name, changes in reverts.items():
    new = (WEB / name).read_text(encoding='utf-8')
    old = new
    for source, target in changes:
        if old.count(source) != 1:
            raise ValueError(f'기준 문구 확인 실패: {name}')
        old = old.replace(source, target, 1)
    files[name] = {'old': hashlib.sha256(old.encode()).hexdigest(),
                   'new': hashlib.sha256(new.encode()).hexdigest(), 'content': new}
raw = json.dumps(files, ensure_ascii=False).encode('utf-8')
payload = base64.b64encode(zlib.compress(raw, 9)).decode('ascii')
installer = r'''#!/usr/bin/env python3
import base64, hashlib, json, os, shutil, sys, tempfile, zlib
from pathlib import Path
WEB = Path.home() / 'SmartCart_ManualDrive_180/smartcart/web'
PAYLOAD = '__PAYLOAD__'
DIGEST = '__DIGEST__'
def h(data): return hashlib.sha256(data).hexdigest()
def main():
    if sys.argv[1:] not in ([], ['--apply']): raise ValueError('사용법: python3 설치기.py [--apply]')
    raw = zlib.decompress(base64.b64decode(PAYLOAD, validate=True))
    if h(raw) != DIGEST: raise ValueError('설치 데이터 오류')
    files = json.loads(raw)
    if set(files) != {'js/core.js','js/radar.js','index.html'}: raise ValueError('파일 목록 오류')
    changes = []
    for name, item in files.items():
        path = WEB / name
        if path.is_symlink() or not path.is_file(): raise ValueError('파일 확인 실패: ' + name)
        actual = h(path.read_bytes())
        if actual not in (item['old'], item['new']): raise ValueError('알 수 없는 현지 수정: ' + name)
        if h(item['content'].encode()) != item['new']: raise ValueError('내용 검증 실패: ' + name)
        if actual != item['new']: changes.append(name)
    print('화면 파일 검증 완료, 변경 필요:', len(changes), flush=True)
    if not sys.argv[1:] or not changes: return
    backup = Path(tempfile.mkdtemp(prefix='smartcart-manual-mirror-backup-', dir=Path.home()))
    for name in changes:
        target = backup / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(WEB / name, target)
    changed = []
    try:
        for name in changes:
            path = WEB / name
            fd, temp = tempfile.mkstemp(prefix='.screen-', dir=path.parent)
            try:
                with os.fdopen(fd, 'wb') as stream: stream.write(files[name]['content'].encode())
                os.chmod(temp, path.stat().st_mode & 0o777)
                os.replace(temp, path)
                changed.append(name)
            finally:
                if os.path.exists(temp): os.unlink(temp)
    except Exception:
        for name in changed: shutil.copy2(backup / name, WEB / name)
        raise
    print('적용 완료. 백업:', backup)
    print('센서 설정·모터 설정·구동은 변경하지 않았습니다.')
if __name__ == '__main__':
    try: main()
    except (OSError, ValueError, KeyError) as exc:
        print('업데이트 중단:', exc, file=sys.stderr)
        sys.exit(1)
'''.replace('__PAYLOAD__', payload).replace('__DIGEST__', hashlib.sha256(raw).hexdigest())
out = ROOT / 'build/releases/smartcart-manual-mirror-20261117-install.py'
out.write_text(installer, encoding='utf-8', newline='\n')
print(out)
print('SHA256:', hashlib.sha256(out.read_bytes()).hexdigest())
