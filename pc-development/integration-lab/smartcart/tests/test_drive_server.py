"""감독기 조종 서버의 인증·출처·조종권·명령·센서 중계를 루프백에서 검사한다.

GPIO·ROS·실제 센서를 쓰지 않는다. 'VIRTUAL-TEST-ONLY' 검수 기록은 시험용이다.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import http.client
import json
import math
from pathlib import Path
import tempfile
import threading
import unittest
from urllib.parse import urlencode

from smartcart_motor.control import MotorController
from smartcart_motor.drive_app import VirtualBackend, build
from smartcart_motor.drive_server import DriveAccess, DriveServer, OPERATOR_LEASE_NS
from smartcart_motor.supervisor import DriveSupervisor
from test_drive_supervisor import CLEAR, FOLLOW, VERIFIED, estop, uwb_status
from test_live_follow_integration import scan, track
from test_motor_control import MS, SECOND, known_config

KEY = 'virtual-test-key-0123'
ROOT = Path(__file__).resolve().parents[1]


class Upstream(BaseHTTPRequestHandler):
    """가짜 8765 센싱 서버. 받은 경로와 Host만 돌려준다."""
    def do_GET(self):
        body = json.dumps({'path': self.path, 'host': self.headers['Host']}).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_):
        pass


class DriveServerTests(unittest.TestCase):
    def setUp(self):
        self.now = SECOND
        self.backend = VirtualBackend()
        self.supervisor = DriveSupervisor(MotorController(known_config(), self.backend), acceptance=VERIFIED,
                                          clearance=CLEAR, follow=FOLLOW, clock=lambda: self.now)
        self.upstream = ThreadingHTTPServer(('127.0.0.1', 0), Upstream)
        self.access = DriveAccess('pending', KEY, secure=False, clock=lambda: self.now)
        self.server = DriveServer(('127.0.0.1', 0), self.supervisor, self.access,
                                  sensor_url='http://127.0.0.1:%d' % self.upstream.server_port)
        self.host = '127.0.0.1:%d' % self.server.server_port
        self.access.host = self.host
        self.threads = [threading.Thread(target=s.serve_forever, daemon=True) for s in (self.server, self.upstream)]
        for thread in self.threads:
            thread.start()

    def tearDown(self):
        self.server.stop.set()
        for server in (self.server, self.upstream):
            server.shutdown()
            server.server_close()

    def request(self, method, path, body=None, headers=None, cookie=None):
        client = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=3)
        try:
            headers = dict(headers or {})
            if cookie:
                headers['Cookie'] = 'smartcart_drive=' + cookie
            client.request(method, path, body=body, headers=headers)
            response = client.getresponse()
            return response.status, dict(response.getheaders()), response.read()
        finally:
            client.close()

    def login(self, key=KEY, origin=True):
        headers = {'Content-Type': 'application/x-www-form-urlencoded'}
        if origin:
            headers['Origin'] = 'http://' + self.host
        status, headers, _ = self.request('POST', '/login', urlencode({'key': key}), headers)
        if status != 303:
            return status, None
        return status, headers['Set-Cookie'].split(';')[0].split('=', 1)[1]

    def command(self, cookie, **data):
        status, _, body = self.request('POST', '/api/drive/command', json.dumps(data),
                                       {'Content-Type': 'application/json', 'Origin': 'http://' + self.host}, cookie)
        return status, json.loads(body)

    def feed(self, sequence=1):
        self.supervisor.update_inputs(lidar_observation=scan(self.now), uwb_status=uwb_status(self.now),
                                      estop=estop(self.now), track=track(self.now, sequence), now_ns=self.now)

    def test_login_required_headers_and_host_check(self):
        status, headers, body = self.request('GET', '/')
        self.assertEqual(status, 200)
        self.assertIn('연결키'.encode(), body)
        self.assertEqual(headers['Referrer-Policy'], 'same-origin')
        self.assertIn("frame-ancestors 'none'", headers['Content-Security-Policy'])
        self.assertEqual(self.request('GET', '/api/drive/status')[0], 401)
        self.assertEqual(self.request('GET', '/js/app.js')[0], 401)
        self.assertEqual(self.request('GET', '/', headers={'Host': 'evil.example:80'})[0], 403)

    def test_login_origin_key_cookie_and_rate_limit(self):
        self.assertEqual(self.login(origin=False)[0], 403)
        status, _, _ = self.request('POST', '/login', urlencode({'key': KEY}),
                                    {'Content-Type': 'application/x-www-form-urlencoded', 'Origin': 'null'})
        self.assertEqual(status, 403)
        self.assertEqual(self.login('wrong-key-0123456')[0], 403)
        status, headers, _ = self.request('POST', '/login', urlencode({'key': KEY}),
                                          {'Content-Type': 'application/x-www-form-urlencoded',
                                           'Origin': 'http://' + self.host})
        self.assertEqual(status, 303)
        cookie = headers['Set-Cookie']
        self.assertIn('HttpOnly', cookie)
        self.assertIn('SameSite=Strict', cookie)
        self.assertNotIn(KEY, cookie)
        for _ in range(3):
            self.login('wrong-key-0123456')
        self.assertEqual(self.login()[0], 429)

    def test_index_injects_drive_link_and_static_paths_are_confined(self):
        _, cookie = self.login()
        status, _, body = self.request('GET', '/', cookie=cookie)
        self.assertEqual(status, 200)
        self.assertIn(b'js/drive_link.js', body)
        self.assertIn(b'css/drive_link.css', body)
        status, headers, _ = self.request('GET', '/js/drive_link.js', cookie=cookie)
        self.assertEqual((status, headers['Content-Type']), (200, 'text/javascript; charset=utf-8'))
        for path in ('/js/../index.html', '/js/%2e%2e/app.js', '/config/x.js', '/js/sub/app.js',
                     '/smartcart_motor/drive_server.py'):
            self.assertEqual(self.request('GET', path, cookie=cookie)[0], 404, path)

    def test_sensor_proxy_allowlist(self):
        _, cookie = self.login()
        status, _, body = self.request('GET', '/api/health', cookie=cookie)
        self.assertEqual(status, 200)
        value = json.loads(body)
        self.assertEqual(value, {'path': '/api/health', 'host': '127.0.0.1:%d' % self.upstream.server_port})
        self.assertEqual(self.request('GET', '/api/frame?hz=10', cookie=cookie)[0], 200)
        for path in ('/api/motor', '/api/frame?x=1', '/api/drive/status?x=1'):
            self.assertEqual(self.request('GET', path, cookie=cookie)[0], 404, path)

    def test_operator_flow_reaches_virtual_output_and_anyone_can_stop(self):
        _, cookie = self.login()
        self.assertEqual(self.command(cookie, type='claim')[0], 200)
        self.assertEqual(self.command(cookie, type='mode', mode='manual')[1]['status']['mode'], 'manual')
        self.feed()
        self.assertTrue(self.command(cookie, type='arm')[1]['result']['accepted'])
        self.supervisor.cycle(self.now)
        for seq in range(1, 6):
            self.now += 20 * MS
            self.feed(seq + 1)
            self.assertTrue(self.command(cookie, type='drive', forward=1, turn=0, seq=seq)[1]['result']['accepted'])
            self.supervisor.cycle(self.now)
        self.assertGreater(self.backend.output[0], 0)
        status = json.loads(self.request('GET', '/api/drive/status', cookie=cookie)[2])
        self.assertTrue(status['active'])
        self.assertTrue(status['operator'])
        self.assertFalse(status['hardware_output'])
        _, other = self.login()
        self.assertEqual(self.command(other, type='claim')[0], 409)
        self.assertEqual(self.command(other, type='drive', forward=1, turn=0, seq=99)[0], 403)
        self.assertEqual(self.command(other, type='stop')[0], 200)
        self.assertEqual(self.backend.output, (0.0, 0.0))
        self.assertEqual(self.supervisor.status()['latched_reason'], 'operator_stop')

    def test_operator_signal_loss_stops_follow_and_frees_control(self):
        _, cookie = self.login()
        self.command(cookie, type='claim')
        self.command(cookie, type='mode', mode='follow')
        self.feed(1)
        self.assertTrue(self.command(cookie, type='arm')[1]['result']['accepted'])
        self.supervisor.cycle(self.now)
        self.assertTrue(self.supervisor.status()['active'])
        self.assertFalse(self.server.check_operator())
        self.now += OPERATOR_LEASE_NS
        self.assertTrue(self.server.check_operator())
        self.assertEqual(self.supervisor.status()['latched_reason'], 'operator_disconnected')
        self.assertEqual(self.command(cookie, type='heartbeat')[0], 403)
        _, other = self.login()
        self.assertEqual(self.command(other, type='claim')[0], 200)

    def test_malformed_and_unknown_commands(self):
        _, cookie = self.login()
        self.command(cookie, type='claim')
        headers = {'Content-Type': 'application/json', 'Origin': 'http://' + self.host}
        for body in ('{', '{"type":"drive","forward":NaN}', '{"type":"stop","type":"drive"}', '[1]',
                     '{"type":"drive","forward":1,"turn":0,"seq":1,"pad":"' + 'x' * 600 + '"}'):
            self.assertEqual(self.request('POST', '/api/drive/command', body, headers, cookie)[0], 400, body)
        self.assertEqual(self.request('POST', '/api/drive/command', '{"type":"stop"}',
                                      {'Content-Type': 'text/plain', 'Origin': 'http://' + self.host}, cookie)[0], 400)
        self.assertEqual(self.request('POST', '/api/drive/command', '{"type":"stop"}',
                                      {'Content-Type': 'application/json'}, cookie)[0], 403)
        self.assertEqual(self.command(cookie, type='fly')[0], 400)
        self.assertEqual(self.supervisor.status()['last_stop_reason'], 'operator_stop')
        # JSON true는 수치 명령이 아니므로 모드와 무관하게 정지 사유가 된다.
        status, value = self.command(cookie, type='drive', forward=True, turn=0, seq=1)
        self.assertEqual(value['result']['reason'], 'invalid_command')

    def test_status_event_stream(self):
        _, cookie = self.login()
        client = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=3)
        try:
            client.request('GET', '/api/drive/events', headers={'Cookie': 'smartcart_drive=' + cookie})
            response = client.getresponse()
            self.assertEqual(response.getheader('Content-Type'), 'text/event-stream')
            line = response.fp.readline().decode()
            self.assertTrue(line.startswith('data: '))
            value = json.loads(line[6:])
            self.assertEqual(value['schema'], 'smartcart.drive_supervisor.v1')
            self.assertNotIn('position_xy_m', line)
        finally:
            self.server.stop.set()
            client.close()


class BuildTests(unittest.TestCase):
    def test_built_supervisor_is_virtual_locked_single_left(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory) / 'run.json'
            run.write_text(json.dumps({
                'schema': 'smartcart.run_config.v1',
                'mounting_config': str(ROOT / 'config' / 'dual_uwb.d5.example.json'),
                'field_config': str(ROOT / 'config' / 'field_acceptance.example.json'),
                'uwb_config': str(ROOT / 'config' / 'uwb_estimation.example.json'),
                'tracking_config': str(ROOT / 'config' / 'target_tracking.example.json'),
                'http_port': 8765, 'lidar_mode': 'managed', 'lidar_port': None,
                'lidar_frame_id': 'lidar_scan'}), encoding='utf-8')
            supervisor, inputs = build(run)
        status = supervisor.status()
        self.assertEqual((status['mode'], status['uwb_profile'], status['hardware_output']),
                         ('locked', 'single_left', False))
        self.assertIn('independent_cutoff', status['acceptance_missing'])
        self.assertFalse(supervisor.clearance.geometry_verified)
        self.assertFalse(supervisor.controller.config.polarity_known)
        self.assertIs(inputs.supervisor, supervisor)
        self.assertTrue(math.isfinite(supervisor.clearance.stop_radius_m))


if __name__ == '__main__':
    unittest.main()
