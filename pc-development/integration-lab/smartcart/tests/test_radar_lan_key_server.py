"""같은 LAN 시험기의 연결키 및 읽기 전용 경계를 검사한다."""
import importlib.util
import http.client
import contextlib
import io
from pathlib import Path
import secrets
import sys
import threading
import unittest
from urllib.parse import urlencode

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location('radar_lan_key_server', SCRIPTS / 'radar_lan_key_server.py')
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


class RadarLanKeyTests(unittest.TestCase):
    def test_connection_key_and_session_expiry(self):
        token = secrets.token_urlsafe(24)
        state = server.make_state('192.168.43.212:8444', token)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertTrue(state.authenticate('', token, None))
            self.assertFalse(state.authenticate('', secrets.token_urlsafe(24), None))
        self.assertNotIn(token, output.getvalue())
        self.assertFalse(state.authenticate('', None, None))
        session = state.new_session()
        self.assertTrue(state.valid(session))
        state.sessions[session] = 0
        self.assertFalse(state.valid(session))

    def test_login_label_and_sensor_only_allowlist(self):
        original = server.gateway.LOGIN
        try:
            server.configure_login_page()
            text = server.gateway.LOGIN.decode('utf-8')
            self.assertIn('시연 연결키', text)
            self.assertNotIn('SSH 비밀번호', text)
            self.assertTrue(server.gateway.allowed_path('/api/health'))
            self.assertFalse(server.gateway.allowed_path('/api/motor'))
        finally:
            server.gateway.LOGIN = original

    def test_http_session_denies_before_auth_and_accepts_correct_key(self):
        token = secrets.token_urlsafe(24)
        httpd = server.gateway.Server(('127.0.0.1', 0), server.LoginDiagnosticsHandler)
        host = '127.0.0.1:' + str(httpd.server_port)
        httpd.state = server.make_state(host, token)
        thread = threading.Thread(target=httpd.serve_forever, daemon=True)
        thread.start()
        def request(method, path, body=None, headers=None):
            client = http.client.HTTPConnection('127.0.0.1', httpd.server_port, timeout=3)
            try:
                client.request(method, path, body=body, headers=headers or {})
                response = client.getresponse()
                body = response.read()
                return response.status, dict(response.getheaders()), body
            finally:
                client.close()
        try:
            status, response_headers, _ = request('GET', '/')
            self.assertEqual(status, 200)
            self.assertEqual(response_headers['Referrer-Policy'], 'same-origin')
            self.assertEqual(request('GET', '/api/health')[0], 401)
            headers = {'Content-Type': 'application/x-www-form-urlencoded', 'Origin': 'https://' + host}
            null_origin = {'Content-Type': 'application/x-www-form-urlencoded', 'Origin': 'null'}
            self.assertEqual(request('POST', '/login', urlencode({'password': token}), null_origin)[0], 403)
            self.assertEqual(request('POST', '/login', urlencode({'password': secrets.token_urlsafe(24)}), headers)[0], 403)
            status, response_headers, _ = request('POST', '/login', urlencode({'password': token}), headers)
            self.assertEqual(status, 303)
            self.assertIn('Secure', response_headers['Set-Cookie'])
            self.assertIn('HttpOnly', response_headers['Set-Cookie'])
            self.assertIn('SameSite=Strict', response_headers['Set-Cookie'])
        finally:
            httpd.shutdown()
            httpd.server_close()
            thread.join()

    def test_old_gateway_no_referrer_header_is_overridden(self):
        # 같은 폴더의 이전 게이트웨이가 no-referrer를 보내도 로그인 Origin이 유지돼야 한다.
        handler = server.LoginDiagnosticsHandler.__new__(server.LoginDiagnosticsHandler)
        handler.request_version = 'HTTP/1.1'
        handler._headers_buffer = []
        handler.send_header('Referrer-Policy', 'no-referrer')
        handler.send_header('X-Frame-Options', 'DENY')
        sent = b''.join(handler._headers_buffer)
        self.assertIn(b'Referrer-Policy: same-origin', sent)
        self.assertNotIn(b'no-referrer', sent)
        self.assertIn(b'X-Frame-Options: DENY', sent)
        path, digest = server.loaded_gateway()
        self.assertEqual(path.name, 'radar_https_gateway.py')
        self.assertRegex(digest, r'^[0-9a-f]{64}$')


if __name__ == '__main__':
    unittest.main()
