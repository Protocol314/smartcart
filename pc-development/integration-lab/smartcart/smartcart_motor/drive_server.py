"""구동 감독기의 조종 화면 서버. 표준 라이브러리 HTTP·SSE만 사용한다.

연결키 세션·같은 출처·단일 조종자로 명령을 제한한다. 정지는 로그인한 누구나 보낼 수
있다. 센싱 화면은 Pi 내부 127.0.0.1:8765의 읽기 전용 응답을 그대로 중계한다.
GPIO·ROS를 직접 열지 않으며 연결키·세션·위치를 기록하지 않는다.
"""
import collections
from http.cookies import CookieError, SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hmac
import json
from pathlib import Path
import re
import secrets
import threading
import time
from urllib.parse import parse_qs, urlsplit
from urllib.request import Request, urlopen

from smartcart_uwb.config import _reject_constant, _unique_object
from .supervisor import MODES, DriveSupervisor

WEB_ROOT = Path(__file__).resolve().parents[1] / 'web'
SESSION_SECONDS = 900
# 조종 화면은 50ms마다 명령 또는 heartbeat를 보낸다. 끊기면 추종 중에도 정지한다.
OPERATOR_LEASE_NS = 1_000_000_000
MAX_BODY = 512
SENSOR_API = {'/api/frame', '/api/health', '/api/sensing', '/api/events',
              '/api/uwb', '/api/uwb/events', '/api/dual-uwb', '/api/dual-uwb/events',
              '/api/tracking', '/api/tracking/events'}
STATIC = re.compile(r'/(?:js|css|assets)/[A-Za-z0-9_-]+\.(js|css|png|svg|ico)')
TYPES = {'js': 'text/javascript; charset=utf-8', 'css': 'text/css; charset=utf-8',
         'png': 'image/png', 'svg': 'image/svg+xml', 'ico': 'image/x-icon'}
APP_SCRIPT = '<script src="js/app.js" defer></script>'
CSP = ("default-src 'self'; style-src 'self' 'unsafe-inline'; script-src 'self'; img-src 'self' data:; "
       "connect-src 'self'; object-src 'none'; base-uri 'none'; frame-ancestors 'none'; form-action 'self'")
LOGIN = ('<!doctype html><html lang="ko"><meta charset="utf-8">'
         '<meta name="viewport" content="width=device-width,initial-scale=1"><title>SmartCart 조종 로그인</title>'
         '<style>body{background:#111;color:#fff;font:18px sans-serif;max-width:32rem;margin:3rem auto;padding:1rem}'
         'input,button{font:inherit;padding:.7rem;width:100%;box-sizing:border-box}button{margin-top:1rem}</style>'
         '<h1>SmartCart 조종</h1><p>Pi 터미널에서 정한 이번 실행의 연결키를 입력하세요.</p>'
         '<form action="/login" method="post"><label>연결키<input name="key" type="password" maxlength="128" '
         'autocomplete="off" required></label><button>로그인</button></form>'
         '<p>구동 감독기 화면 · 검수 전에는 구동이 잠깁니다 · 물리 비상정지 별도</p></html>').encode('utf-8')


def index_page(web_root=WEB_ROOT):
    """기존 센싱 화면에 감독기 연결 스크립트만 더한다. 형식이 다르면 시작하지 않는다."""
    page = (Path(web_root) / 'index.html').read_text(encoding='utf-8')
    if page.count(APP_SCRIPT) != 1:
        raise ValueError('기존 화면 형식이 일치하지 않습니다')
    return page.replace(APP_SCRIPT, APP_SCRIPT + '\n  <link rel="stylesheet" href="css/drive_link.css">'
                        '\n  <script src="js/drive_link.js" defer></script>').encode('utf-8')


class DriveAccess:
    """이번 실행의 연결키 세션과 단일 조종권. 연결키는 메모리에만 둔다."""

    def __init__(self, host, key, *, secure, clock=time.monotonic_ns):
        if type(key) is not str or not 12 <= len(key) <= 128:
            raise ValueError('연결키는 12~128자여야 합니다')
        if type(host) is not str or not host or len(host) > 128 or type(secure) is not bool:
            raise ValueError('호스트·보안 연결 설정이 잘못되었습니다')
        self.host, self.secure, self.clock = host, secure, clock
        self._key = key.encode('utf-8')
        self.lock = threading.Lock()
        self.slots = threading.BoundedSemaphore(24)
        self.streams = threading.BoundedSemaphore(4)
        self.attempts = collections.deque()
        self.sessions = {}
        self.operator = None
        self.operator_seen = 0

    def login(self, candidate):
        with self.lock:
            now = self.clock()
            while self.attempts and self.attempts[0] <= now - 60_000_000_000:
                self.attempts.popleft()
            if len(self.attempts) >= 5:
                return None, 429
            self.attempts.append(now)
            if type(candidate) is not str or not hmac.compare_digest(candidate.encode('utf-8'), self._key):
                return None, 403
            self.sessions = {k: v for k, v in self.sessions.items() if v > now}
            if len(self.sessions) >= 16:
                return None, 503
            token = secrets.token_urlsafe(32)
            self.sessions[token] = now + SESSION_SECONDS * 1_000_000_000
            return token, 303

    def valid(self, token):
        if type(token) is not str or not token or len(token) > 128:
            return False
        with self.lock:
            return self.sessions.get(token, 0) > self.clock()

    def claim(self, token):
        """반환: (성공, 이전 조종자의 조종권이 만료돼 교체됐는지)."""
        with self.lock:
            now = self.clock()
            expired = self.operator is not None and now - self.operator_seen >= OPERATOR_LEASE_NS
            if self.operator not in (None, token) and not expired:
                return False, False
            replaced = self.operator not in (None, token)
            self.operator, self.operator_seen = token, now
            return True, replaced

    def touch(self, token):
        with self.lock:
            now = self.clock()
            if self.operator != token or now - self.operator_seen >= OPERATOR_LEASE_NS:
                return False
            self.operator_seen = now
            return True

    def release(self, token):
        with self.lock:
            if self.operator != token:
                return False
            self.operator = None
            return True

    def is_operator(self, token):
        with self.lock:
            return self.operator == token and self.clock() - self.operator_seen < OPERATOR_LEASE_NS

    def expire_operator(self):
        """조종권이 만료돼 이번 호출에서 해제됐으면 참."""
        with self.lock:
            if self.operator is not None and self.clock() - self.operator_seen >= OPERATOR_LEASE_NS:
                self.operator = None
                return True
            return False


class DriveHandler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    server_version = 'SmartCartDrive'
    sys_version = ''

    def log_message(self, format, *args):
        pass  # 연결키·세션·위치를 기록하지 않는다.

    def reply(self, status, body=b'', content_type='text/plain; charset=utf-8', extra=()):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('X-Frame-Options', 'DENY')
        self.send_header('Referrer-Policy', 'same-origin')
        self.send_header('Content-Security-Policy', CSP)
        for name, value in extra:
            self.send_header(name, value)
        self.end_headers()
        if body and self.command != 'HEAD':
            self.wfile.write(body)

    def json_reply(self, status, value):
        self.reply(status, json.dumps(value, ensure_ascii=False, allow_nan=False).encode('utf-8'),
                   'application/json; charset=utf-8')

    @property
    def access(self):
        return self.server.access

    def host_ok(self):
        return self.headers.get_all('Host', []) == [self.access.host]

    def origin_ok(self):
        scheme = 'https://' if self.access.secure else 'http://'
        return self.headers.get_all('Origin', []) == [scheme + self.access.host]

    def token(self):
        try:
            cookie = SimpleCookie()
            cookie.load(self.headers.get('Cookie', ''))
            return cookie['smartcart_drive'].value if 'smartcart_drive' in cookie else ''
        except CookieError:
            return ''

    def status_body(self, token):
        value = self.server.supervisor.status()
        value['operator'] = self.access.is_operator(token)
        return value

    # 읽기 ---------------------------------------------------------------
    def do_GET(self):
        if not self.host_ok():
            self.close_connection = True
            return self.reply(403)
        parsed = urlsplit(self.path)
        token = self.token()
        if not self.access.valid(token):
            if parsed.path == '/' and not parsed.query:
                return self.reply(200, LOGIN, 'text/html; charset=utf-8')
            return self.reply(401)
        if '%' in self.path or '..' in self.path or '\\' in self.path or len(self.path) > 256:
            return self.reply(404)
        if parsed.path in ('/', '/index.html') and not parsed.query:
            return self.reply(200, self.server.index, 'text/html; charset=utf-8')
        static = STATIC.fullmatch(parsed.path)
        if static and not parsed.query:
            return self.static_file(parsed.path, static.group(1))
        if parsed.path == '/api/drive/status' and not parsed.query:
            return self.json_reply(200, self.status_body(token))
        if parsed.path == '/api/drive/events' and not parsed.query:
            return self.events(token)
        if parsed.path in SENSOR_API and (not parsed.query or re.fullmatch(r'hz=(?:2|5|10)', parsed.query)):
            return self.proxy()
        return self.reply(404)

    def static_file(self, path, extension):
        root = Path(self.server.web_root).resolve()
        target = (root / path.lstrip('/')).resolve()
        if root not in target.parents or target.is_symlink() or not target.is_file():
            return self.reply(404)
        body = target.read_bytes()
        if len(body) > 2 * 1024 * 1024:
            return self.reply(404)
        return self.reply(200, body, TYPES[extension])

    def events(self, token):
        if not self.access.streams.acquire(blocking=False):
            return self.reply(503)
        try:
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Connection', 'close')
            self.end_headers()
            self.close_connection = True
            while self.access.valid(token) and not self.server.stop.is_set():
                payload = json.dumps(self.status_body(token), ensure_ascii=False, allow_nan=False)
                self.wfile.write(('data: ' + payload + '\n\n').encode('utf-8'))
                self.wfile.flush()
                self.server.stop.wait(.1)
        except OSError:
            pass
        finally:
            self.access.streams.release()

    def proxy(self):
        target = self.server.sensor_url + self.path
        try:
            with urlopen(Request(target, headers={'Host': self.server.sensor_host}), timeout=3) as upstream:
                content_type = upstream.headers.get('Content-Type', 'application/octet-stream')
                if not self.path.split('?')[0].endswith('/events'):
                    body = upstream.read(2 * 1024 * 1024 + 1)
                    if len(body) > 2 * 1024 * 1024:
                        return self.reply(502)
                    return self.reply(200, body, content_type)
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.send_header('Connection', 'close')
                self.end_headers()
                self.close_connection = True
                token = self.token()
                while self.access.valid(token) and not self.server.stop.is_set():
                    chunk = upstream.read1(8192)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    self.wfile.flush()
        except (OSError, ValueError):
            self.close_connection = True
            try:
                self.json_reply(502, {'error': 'sensor_unavailable'})
            except OSError:
                pass

    # 쓰기 ---------------------------------------------------------------
    def body(self, content_type):
        if self.headers.get('Content-Type', '').split(';')[0].strip() != content_type or self.headers.get('Transfer-Encoding'):
            raise ValueError()
        length = int(self.headers.get('Content-Length', '-1'))
        if not 1 <= length <= MAX_BODY:
            raise ValueError()
        self._consumed = True
        return self.rfile.read(length).decode('utf-8')

    def refuse(self, status):
        """작은 미처리 본문은 읽고 닫아 거부 응답이 연결 리셋으로 사라지지 않게 한다."""
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if (not getattr(self, '_consumed', False) and 0 < length <= MAX_BODY
                    and not self.headers.get('Transfer-Encoding')):
                self._consumed = True
                self.rfile.read(length)
        except (ValueError, OSError):
            pass
        self.close_connection = True
        return self.reply(status)

    def do_POST(self):
        self._consumed = False
        if not self.host_ok() or not self.origin_ok():
            return self.refuse(403)
        path = urlsplit(self.path).path
        if path == '/login' and urlsplit(self.path).query == '':
            return self.login()
        if path == '/api/drive/command' and urlsplit(self.path).query == '':
            token = self.token()
            if not self.access.valid(token):
                return self.refuse(401)
            try:
                data = json.loads(self.body('application/json'), object_pairs_hook=_unique_object,
                                  parse_constant=_reject_constant)
                if type(data) is not dict or len(data) > 6:
                    raise ValueError()
            except (ValueError, UnicodeError, RecursionError):
                return self.refuse(400)
            status, value = self.drive_command(token, data)
            return self.json_reply(status, value)
        return self.refuse(404)

    def login(self):
        try:
            fields = parse_qs(self.body('application/x-www-form-urlencoded'), keep_blank_values=True,
                              strict_parsing=True, max_num_fields=1)
            if set(fields) != {'key'} or len(fields['key']) != 1:
                raise ValueError()
            candidate = fields['key'][0]
        except (ValueError, UnicodeError):
            return self.refuse(400)
        token, status = self.access.login(candidate)
        candidate = None
        if token is None:
            return self.reply(status, '연결키를 확인하지 못했습니다.'.encode('utf-8') if status == 403 else b'')
        cookie = ('smartcart_drive=' + token + '; Path=/; HttpOnly; SameSite=Strict; Max-Age='
                  + str(SESSION_SECONDS) + ('; Secure' if self.access.secure else ''))
        return self.reply(303, extra=(('Location', '/'), ('Set-Cookie', cookie)))

    def drive_command(self, token, data):
        supervisor, access = self.server.supervisor, self.access
        kind = data.get('type')

        def done(result=None, status=200):
            return status, {'result': result, 'status': self.status_body(token)}

        if kind == 'stop':
            # 조종권과 무관하게 로그인한 누구나 정지할 수 있다.
            supervisor.stop('operator_stop')
            return done({'accepted': True, 'reason': 'operator_stop'})
        if kind == 'claim':
            claimed, replaced = access.claim(token)
            if replaced:
                supervisor.stop('operator_disconnected')
            return done({'accepted': claimed, 'reason': None if claimed else 'operator_busy'},
                        200 if claimed else 409)
        if kind == 'release':
            if access.release(token):
                supervisor.stop('operator_disconnected')
            return done({'accepted': True, 'reason': None})
        if not access.touch(token):
            return done({'accepted': False, 'reason': 'not_operator'}, 403)
        if kind == 'heartbeat':
            return done({'accepted': True, 'reason': None})
        if kind == 'mode' and data.get('mode') in MODES:
            supervisor.set_mode(data['mode'])
            return done({'accepted': True, 'reason': None})
        if kind == 'arm':
            return done(supervisor.request_arm())
        if kind == 'drive':
            return done(supervisor.manual_command(data.get('forward'), data.get('turn'), data.get('seq')))
        supervisor.stop('operator_stop')
        return done({'accepted': False, 'reason': 'invalid_command'}, 400)


class DriveServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, supervisor, access, *, web_root=WEB_ROOT,
                 sensor_url='http://127.0.0.1:8765', stop=None):
        if not isinstance(supervisor, DriveSupervisor) or not isinstance(access, DriveAccess):
            raise ValueError('감독기와 접근 설정이 필요합니다')
        parsed = urlsplit(sensor_url)
        if parsed.scheme != 'http' or parsed.hostname not in ('127.0.0.1', 'localhost') or parsed.path not in ('', '/'):
            raise ValueError('센서 중계는 같은 Pi의 루프백 HTTP만 허용합니다')
        self.supervisor, self.access, self.web_root = supervisor, access, Path(web_root)
        self.sensor_url = sensor_url.rstrip('/')
        self.sensor_host = parsed.netloc
        self.index = index_page(web_root)
        self.stop = threading.Event() if stop is None else stop
        super().__init__(address, DriveHandler)

    def check_operator(self):
        """조종자 신호가 끊기면 모드와 무관하게 정지한다. 반환: 정지 여부."""
        if self.access.expire_operator():
            self.supervisor.stop('operator_disconnected')
            return True
        return False

    def watch_operator(self, period_s=.1):
        while not self.stop.wait(period_s):
            self.check_operator()

    def process_request(self, request, client_address):
        if not self.access.slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self.access.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.access.slots.release()
