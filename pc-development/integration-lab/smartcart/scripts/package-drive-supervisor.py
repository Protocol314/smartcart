#!/usr/bin/env python3
"""구동 감독기(가상 출력) Pi 배포 ZIP을 만든다. 기존 설치본과 별도 폴더에 풀어 쓴다.

포함: smartcart_motor 파이썬 파일, 조종 화면 web, 시작 스크립트, 안내. GPIO 실행 경로·
로컬 설정·비밀 값은 포함하지 않는다. 같은 이름의 ZIP이 있으면 덮어쓰지 않는다.
"""
import hashlib
from pathlib import Path
import sys
import zipfile

APP = Path(__file__).resolve().parents[1]
NAME = 'smartcart-drive-supervisor-20261117'
DEST = APP.parent / 'build' / 'releases' / (NAME + '.zip')
STAMP = (2026, 11, 17, 0, 0, 0)
README = f'''# SmartCart 구동 감독기 화면 (가상 출력)

기존 설치본(~/SmartCart-D5-v2/smartcart)을 바꾸지 않고 이 폴더의 감독기만 실행한다.
LiDAR·왼쪽 UWB ROS 토픽을 구독해 수동/추종 모드, 정지 우선순위, 잠금 사유를 화면에 보인다.
출력은 가상 전용이며 GPIO를 열지 않는다. 비상정지 피드백 회로와 현장 검수 기록이 없으므로
항상 구동이 잠긴다. 화면의 정지는 PWM 0 요청이며 물리 비상정지를 대신하지 않는다.

1. 다른 Pi 터미널에서 센싱을 먼저 실행한다(예: start-single-left-trial.sh).
2. LAN HTTPS(8444용 시험 인증서 재사용 가능):
   bash ~/{NAME}/scripts/start-drive-supervisor.sh --address <Pi IP> --cert <server.crt> --key <server.key>
   또는 루프백만: bash ~/{NAME}/scripts/start-drive-supervisor.sh  → PC에서 ssh -L 8445:127.0.0.1:8445
3. 터미널에 정한 연결키로 https://<Pi IP>:8445/ 에 로그인한다. 연결키·SSH 비밀번호를 채팅에 보내지 않는다.
'''


def entries():
    files = {f'{NAME}/README.md': README.encode('utf-8')}
    for path in sorted((APP / 'smartcart_motor').glob('*.py')):
        files[f'{NAME}/smartcart_motor/{path.name}'] = path.read_bytes()
    for path in sorted((APP / 'web').rglob('*')):
        if path.is_file() and not path.is_symlink():
            files[f'{NAME}/web/{path.relative_to(APP / "web").as_posix()}'] = path.read_bytes()
    files[f'{NAME}/scripts/start-drive-supervisor.sh'] = (APP / 'scripts' / 'start-drive-supervisor.sh').read_bytes()
    return files


def build(dest=DEST):
    dest = Path(dest)
    if dest.exists():
        raise FileExistsError('같은 이름의 배포 ZIP이 이미 있어 덮어쓰지 않습니다: ' + dest.name)
    dest.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(dest, 'x', zipfile.ZIP_DEFLATED) as archive:
        for name, data in entries().items():
            info = zipfile.ZipInfo(name, date_time=STAMP)
            info.external_attr = (0o755 if name.endswith('.sh') else 0o644) << 16
            info.compress_type = zipfile.ZIP_DEFLATED
            archive.writestr(info, data)
    digest = hashlib.sha256(dest.read_bytes()).hexdigest()
    dest.with_name(dest.name + '.sha256').write_text(f'{digest}  {dest.name}\n', encoding='ascii')
    return digest


if __name__ == '__main__':
    target = Path(sys.argv[1]) if len(sys.argv) > 1 else DEST
    print(target.name, build(target))
