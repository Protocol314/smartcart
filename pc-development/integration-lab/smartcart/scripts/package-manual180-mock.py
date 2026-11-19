#!/usr/bin/env python3
"""외부 수동 조종본에서 GPIO 경로를 제거한 별도 모의 시연 ZIP을 만든다."""
import hashlib
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'SmartCart_ManualDrive_180' / 'smartcart'
DEST = ROOT / 'build/releases/smartcart-manual180-mock-20261117.zip'


def build():
    drive = (SOURCE / 'drive.py').read_text(encoding='utf-8')
    motor = (SOURCE / 'smartcart_motor/manual_remote.py').read_text(encoding='utf-8')
    old_import = 'from smartcart_motor.manual_remote import ManualDrive, MockBackend, RPiBackend'
    if drive.count(old_import) != 1:
        raise ValueError('수입 코드 형식이 달라 중단했습니다')
    drive = drive.replace(old_import, 'from smartcart_motor.manual_remote import ManualDrive, MockBackend')
    old_choice = 'choices=("mock", "gpio"), default="mock"'
    if drive.count(old_choice) != 1:
        raise ValueError('백엔드 인수 형식이 달라 중단했습니다')
    drive = drive.replace(old_choice, 'choices=("mock",), default="mock"')
    start = drive.index('        if args.backend == "gpio":')
    end = drive.index('        app = make_app(backend, sensor_url=source)', start)
    drive = drive[:start] + '        backend = MockBackend()\n' + drive[end:]
    marker = 'def make_app(backend=None, *, key=None, sensor_url="http://127.0.0.1:8765", start_worker=True):\n'
    if drive.count(marker) != 1:
        raise ValueError('앱 생성 코드 형식이 달라 중단했습니다')
    drive = drive.replace(marker, marker + '    if backend is not None and type(backend) is not MockBackend:\n        raise ValueError("이 배포본은 모의 출력만 허용합니다")\n')
    start = motor.index('class RPiBackend:')
    end = motor.index('class ManualDrive:', start)
    motor = motor[:start] + motor[end:]
    marker = '    def __init__(self, backend, *, clock=time.monotonic):\n'
    if motor.count(marker) != 1:
        raise ValueError('모터 서비스 코드 형식이 달라 중단했습니다')
    motor = motor.replace(marker, marker + '        if type(backend) is not MockBackend:\n            raise ValueError("이 배포본은 모의 출력만 허용합니다")\n')
    entries = {
        'smartcart/drive.py': drive.encode('utf-8'),
        'smartcart/smartcart_motor/__init__.py': '"""모의 조종 전용."""\n'.encode('utf-8'),
        'smartcart/smartcart_motor/manual_remote.py': motor.encode('utf-8'),
    }
    for path in (SOURCE / 'web').rglob('*'):
        if path.is_file() and not path.is_symlink():
            entries['smartcart/' + path.relative_to(SOURCE).as_posix()] = path.read_bytes()
    for name in ('test_manual_controls.cjs', 'test_manual_protocol.py'):
        entries['smartcart/tests/' + name] = (SOURCE / 'tests' / name).read_bytes()
    entries['README.md'] = '''# SmartCart ManualDrive 180 모의 통합 시험

원본의 실제 GPIO 경로를 제거한 별도 복사본이다. `--backend gpio`는 거부되며
센서 서버에서 LiDAR·UWB를 읽기 전용으로 중계하고 가상 조종 명령만 계산한다.
장애물 자동 정지·사용자 추종 또는 실제 모터 구동이 아니다.

```bash
cd ~/smartcart-manual180-mock/smartcart
python3 drive.py --backend mock --host 0.0.0.0 --port 8080
```

콘솔의 일회성 연결키는 본인만 사용한다. 같은 네트워크에서 화면을 볼 수 있으며
HTTP에서는 키와 센서 정보가 암호화되지 않으므로 신뢰하는 네트워크에서만 쓴다.
기존 Pi 센싱 서버는 127.0.0.1:8765에서 별도로 실행해야 한다. 종료는 Ctrl+C다.
'''.encode('utf-8')
    if DEST.exists():
        raise FileExistsError('기존 배포 파일을 덮어쓰지 않습니다')
    with zipfile.ZipFile(DEST, 'x', compression=zipfile.ZIP_DEFLATED) as archive:
        for name, content in sorted(entries.items()):
            archive.writestr(name, content)
    with zipfile.ZipFile(DEST) as archive:
        if archive.testzip() is not None or set(archive.namelist()) != set(entries):
            raise ValueError('배포 ZIP 검증 실패')
    print('배포 파일:', DEST)
    print('SHA256:', hashlib.sha256(DEST.read_bytes()).hexdigest())
    print('파일 수:', len(entries))


if __name__ == '__main__':
    build()
