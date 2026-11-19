"""루프백·가상 출력 전용 시험 서버."""
from aiohttp import web
from drive import make_app
from smartcart_motor.manual_remote import MockBackend

if __name__ == '__main__':
    app = make_app(MockBackend(), sensor_url='http://127.0.0.1:8765')
    print('가상 출력 전용 · GPIO 미사용 · 센서 127.0.0.1:8765', flush=True)
    print('PC SSH 터널: http://127.0.0.1:18082/ · 종료 Ctrl+C', flush=True)
    print('가상 조이스틱 연결키:', app['key'], flush=True)
    web.run_app(app, host='127.0.0.1', port=18082, access_log=None)
