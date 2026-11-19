"""실측 센서 읽기 전용 중계와 MOCK 조종 화면만 묶는다."""
from pathlib import Path
import hashlib
import zipfile

root = Path(__file__).resolve().parents[1]
source = root.parent / 'SmartCart_ManualDrive_180' / 'smartcart'
output = root.parent / 'build' / 'releases' / 'smartcart-virtual-live-20261118.zip'
launcher = '''"""루프백·가상 출력 전용 시험 서버."""
from aiohttp import web
from drive import make_app
from smartcart_motor.manual_remote import MockBackend

if __name__ == '__main__':
    app = make_app(MockBackend(), sensor_url='http://127.0.0.1:8765')
    print('가상 출력 전용 · GPIO 미사용 · 센서 127.0.0.1:8765', flush=True)
    print('PC SSH 터널: http://127.0.0.1:18082/ · 종료 Ctrl+C', flush=True)
    print('가상 조이스틱 연결키:', app['key'], flush=True)
    web.run_app(app, host='127.0.0.1', port=18082, access_log=None)
'''
output.parent.mkdir(parents=True, exist_ok=True)
with zipfile.ZipFile(output, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
    archive.writestr('virtual_server.py', launcher)
    for path in [source/'drive.py', source/'smartcart_motor/__init__.py',
                 source/'smartcart_motor/control.py', source/'smartcart_motor/manual_remote.py',
                 *(p for p in (source/'web').rglob('*') if p.is_file())]:
        archive.write(path, path.relative_to(source).as_posix())
print(output)
print('SHA-256', hashlib.sha256(output.read_bytes()).hexdigest())
