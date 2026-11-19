"""인증 중계의 접근 제한을 실제 GPIO 없이 검증한다."""
import http.client
import importlib.util
import io
import secrets
import sys
import types
from unittest.mock import patch
from pathlib import Path
import threading
import unittest
from urllib.parse import urlencode

spec = importlib.util.spec_from_file_location('gateway', Path(__file__).resolve().parents[1] / 'scripts/radar_https_gateway.py')
gateway = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gateway)


class GatewayTest(unittest.TestCase):
    def setUp(self):
        self.calls = []
        self.example_password = secrets.token_urlsafe(24)
        def authenticate(user, password, host_key):
            self.calls.append((user, password, host_key))
            return password == self.example_password
        self.server = gateway.Server(('127.0.0.1', 0), gateway.Handler)
        self.port = self.server.server_port
        self.host = '127.0.0.1:' + str(self.port)
        self.server.state = gateway.State(self.host, 'capstone', ('key', 'fingerprint'), authenticate)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def request(self, method, path, body=None, headers=None):
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=3)
        conn.request(method, path, body=body, headers=headers or {})
        response = conn.getresponse()
        status, headers, body = response.status, dict(response.getheaders()), response.read()
        conn.close()
        return status, headers, body

    def login(self, password=None, origin=None):
        password = password if password is not None else self.example_password
        return self.request('POST', '/login', urlencode({'password': password}), {
            'Content-Type': 'application/x-www-form-urlencoded',
            'Origin': origin or 'https://' + self.host,
        })

    def test_auth_and_cookie(self):
        self.assertEqual(self.request('GET', '/api/health')[0], 401)
        status, headers, _ = self.login()
        self.assertEqual(status, 303)
        for attribute in ('Secure', 'HttpOnly', 'SameSite=Strict'):
            self.assertIn(attribute, headers['Set-Cookie'])
        self.assertEqual(len(self.calls), 1)

    def test_cross_origin_is_rejected(self):
        self.assertEqual(self.login(origin='https://attacker.invalid')[0], 403)
        self.assertEqual(self.calls, [])

    def test_attempt_limit_and_bad_password(self):
        for _ in range(5):
            self.assertEqual(self.login('wrong')[0], 403)
        self.assertEqual(self.login()[0], 429)
        self.assertEqual(len(self.calls), 5)

    def test_host_and_path_restrictions(self):
        self.assertEqual(self.request('GET', '/', headers={'Host': 'attacker.invalid'})[0], 403)
        for path in ('/../secret', '/js/../../secret.js', '/%2e%2e/a', '/login', '/api/motor'):
            self.assertFalse(gateway.allowed_path(path), path)
        for path in ('/', '/js/app.js', '/api/tracking/events'):
            self.assertTrue(gateway.allowed_path(path), path)

    def test_expired_session_is_rejected(self):
        token = self.server.state.new_session()
        self.assertTrue(self.server.state.valid(token))
        self.server.state.sessions[token] = 0
        self.assertFalse(self.server.state.valid(token))

    def test_changed_ssh_host_key_never_receives_password(self):
        calls = []
        class FakeKey:
            def get_name(self):
                return 'ssh-ed25519'
            def get_base64(self):
                return 'unexpected'
        class FakeTransport:
            def __init__(self, sock):
                pass
            def start_client(self, timeout):
                pass
            def get_remote_server_key(self):
                return FakeKey()
            def auth_password(self, *args, **kwargs):
                calls.append('password sent')
            def close(self):
                pass
        fake = types.SimpleNamespace(Transport=FakeTransport, SSHException=Exception)
        with patch.dict(sys.modules, {'paramiko': fake}), patch.object(gateway.socket, 'create_connection', return_value=object()):
            self.assertFalse(gateway.ssh_authenticate('capstone', self.example_password, ('ssh-ed25519', 'expected')))
        self.assertEqual(calls, [])

    def test_authenticated_radar_proxy(self):
        class Backend(io.BytesIO):
            headers = {'Content-Type': 'application/json'}
        token = self.server.state.new_session()
        with patch.object(gateway, 'urlopen', return_value=Backend(b'{"state":"live"}')) as upstream:
            status, _, body = self.request('GET', '/api/health', headers={'Cookie': 'smartcart_session=' + token})
        self.assertEqual(status, 200)
        self.assertEqual(body, b'{"state":"live"}')
        self.assertEqual(upstream.call_args.args[0].full_url, 'http://127.0.0.1:8765/api/health')


if __name__ == '__main__':
    unittest.main()
