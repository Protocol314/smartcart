#!/usr/bin/env python3
"""같은 네트워크의 인증된 브라우저에 Pi 내부 레이더를 중계한다.

실제 GPIO나 모터 API는 제공하지 않는다. SSH 서버에 대한 암호 인증을
사용하며, 서버 호스트키를 확인하기 전에는 암호를 전송하지 않는다.
"""
import argparse
import collections
import hmac
import ipaddress
import os
from pathlib import Path
import re
import secrets
import socket
import ssl
import stat
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit
from urllib.request import Request, urlopen

SESSION_SECONDS = 900
MAX_BODY = 1024
API = {'/api/frame', '/api/health', '/api/sensing', '/api/events',
       '/api/uwb', '/api/uwb/events', '/api/dual-uwb', '/api/dual-uwb/events',
       '/api/tracking', '/api/tracking/events'}
STATIC = re.compile(r'^/(?:$|index\.html$|(?:js|css|assets)/[A-Za-z0-9_./-]+\.(?:js|css|png|svg|ico))')
LOGIN = '''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SmartCart 로그인</title><style>body{background:#111;color:#fff;font:18px sans-serif;max-width:32rem;margin:3rem auto;padding:1rem}input,button{font:inherit;padding:.7rem;width:100%;box-sizing:border-box}button{margin-top:1rem}</style><h1>SmartCart</h1><p>Pi 계정의 SSH 비밀번호를 입력하세요.</p><form action="/login" method="post"><label>비밀번호<input name="password" type="password" maxlength="256" autocomplete="current-password" required></label><button>로그인</button></form><p>가상 조종만 제공하며 실제 모터는 작동하지 않습니다.</p></html>'''.encode()


def allowed_path(path):
    if not path.startswith('/') or '%' in path or '\\' in path or '..' in path or len(path) > 256:
        return False
    return path in API or bool(STATIC.fullmatch(path))


def read_host_key(path):
    p = Path(path)
    info = p.lstat()
    if not stat.S_ISREG(info.st_mode) or info.st_uid != 0 or info.st_mode & 0o022:
        raise ValueError('SSH 호스트 공개키 파일의 소유권 또는 권한이 안전하지 않습니다')
    parts = p.read_text(encoding='ascii').split()
    if len(parts) < 2 or parts[0] != 'ssh-ed25519':
        raise ValueError('Ed25519 SSH 호스트 공개키가 아닙니다')
    return parts[0], parts[1]


def ssh_authenticate(username, password, expected):
    import paramiko
    sock = socket.create_connection(('127.0.0.1', 22), timeout=4)
    transport = paramiko.Transport(sock)
    try:
        transport.banner_timeout = 4
        transport.auth_timeout = 4
        transport.start_client(timeout=4)
        key = transport.get_remote_server_key()
        if key.get_name() != expected[0] or not hmac.compare_digest(key.get_base64(), expected[1]):
            return False
        transport.auth_password(username, password, fallback=False)
        return transport.is_authenticated()
    except (OSError, paramiko.SSHException, EOFError):
        return False
    finally:
        transport.close()


class State:
    def __init__(self, host, username, host_key, authenticator=ssh_authenticate):
        self.host, self.username, self.host_key = host, username, host_key
        self.authenticate = authenticator
        self.lock = threading.Lock()
        self.slots = threading.BoundedSemaphore(16)
        self.auth_slot = threading.BoundedSemaphore(1)
        self.attempts = collections.deque()
        self.sessions = {}

    def attempt(self):
        with self.lock:
            now = time.monotonic()
            while self.attempts and self.attempts[0] <= now - 60:
                self.attempts.popleft()
            if len(self.attempts) >= 5:
                return False
            self.attempts.append(now)
            return True

    def new_session(self):
        with self.lock:
            now = time.monotonic()
            self.sessions = {k: v for k, v in self.sessions.items() if v > now}
            if len(self.sessions) >= 16:
                return None
            token = secrets.token_urlsafe(32)
            self.sessions[token] = now + SESSION_SECONDS
            return token

    def valid(self, token):
        if not token or len(token) > 128:
            return False
        with self.lock:
            return self.sessions.get(token, 0) > time.monotonic()


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    server_version = 'SmartCart'
    sys_version = ''

    def log_message(self, format, *args):
        pass  # 위치 정보, 세션, 암호를 기록하지 않는다.

    def reply(self, status, body=b'', content_type='text/plain; charset=utf-8', extra=()):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('X-Frame-Options', 'DENY')
        self.send_header('Referrer-Policy', 'no-referrer')
        self.send_header('Content-Security-Policy', "default-src 'self'; style-src 'self' 'unsafe-inline'; script-src 'self'; img-src 'self' data:; object-src 'none'; base-uri 'none'; frame-ancestors 'none'; form-action 'self'")
        for name, value in extra:
            self.send_header(name, value)
        self.end_headers()
        if body:
            self.wfile.write(body)

    def host_ok(self):
        return self.headers.get_all('Host', []) == [self.server.state.host]

    def token(self):
        from http.cookies import SimpleCookie, CookieError
        try:
            cookie = SimpleCookie()
            cookie.load(self.headers.get('Cookie', ''))
            return cookie['smartcart_session'].value if 'smartcart_session' in cookie else ''
        except CookieError:
            return ''

    def do_POST(self):
        state = self.server.state
        if not self.host_ok() or self.path != '/login' or self.headers.get('Origin') != 'https://' + state.host:
            self.close_connection = True
            return self.reply(403)
        try:
            if self.headers.get('Content-Type', '').split(';')[0] != 'application/x-www-form-urlencoded':
                raise ValueError()
            length = int(self.headers.get('Content-Length', '-1'))
            if length < 1 or length > MAX_BODY or self.headers.get('Transfer-Encoding'):
                raise ValueError()
            fields = parse_qs(self.rfile.read(length).decode('utf-8'), keep_blank_values=True, strict_parsing=True, max_num_fields=1)
            password = fields['password'][0]
            if set(fields) != {'password'} or len(fields['password']) != 1 or not 1 <= len(password) <= 256:
                raise ValueError()
        except (ValueError, KeyError, UnicodeError):
            self.close_connection = True
            return self.reply(400)
        if not state.attempt() or not state.auth_slot.acquire(blocking=False):
            return self.reply(429)
        try:
            accepted = state.authenticate(state.username, password, state.host_key)
        except Exception:
            accepted = False
        finally:
            password = None
            state.auth_slot.release()
        if not accepted:
            return self.reply(403, '인증하지 못했습니다.'.encode())
        token = state.new_session()
        if token is None:
            return self.reply(503)
        return self.reply(303, extra=(('Location', '/'), ('Set-Cookie', 'smartcart_session=' + token + '; Path=/; Secure; HttpOnly; SameSite=Strict; Max-Age=' + str(SESSION_SECONDS))))

    def do_GET(self):
        state = self.server.state
        if not self.host_ok():
            return self.reply(403)
        parsed = urlsplit(self.path)
        if not state.valid(self.token()):
            return self.reply(200, LOGIN, 'text/html; charset=utf-8') if parsed.path == '/' and not parsed.query else self.reply(401)
        if not allowed_path(parsed.path) or len(parsed.query) > 64 or (parsed.query and not re.fullmatch(r'hz=(?:1|2|5|10)', parsed.query)):
            return self.reply(404)
        target = 'http://127.0.0.1:8765' + self.path
        try:
            with urlopen(Request(target, headers={'Host': '127.0.0.1:8765'}), timeout=3) as upstream:
                content_type = upstream.headers.get('Content-Type', 'application/octet-stream')
                if parsed.path.endswith('/events'):
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/event-stream')
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('X-Content-Type-Options', 'nosniff')
                    self.send_header('Connection', 'close')
                    self.end_headers()
                    self.close_connection = True
                    token = self.token()
                    while state.valid(token):
                        chunk = upstream.read1(8192)
                        if not chunk:
                            break
                        self.wfile.write(chunk)
                        self.wfile.flush()
                else:
                    body = upstream.read(2 * 1024 * 1024 + 1)
                    if len(body) > 2 * 1024 * 1024:
                        return self.reply(502)
                    self.reply(200, body, content_type)
        except (OSError, ValueError):
            self.close_connection = True


class Server(ThreadingHTTPServer):
    daemon_threads = True

    def process_request(self, request, client_address):
        if not self.state.slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self.state.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.state.slots.release()


def main():
    parser = argparse.ArgumentParser(description='Pi 내부 레이더의 HTTPS 중계. 모터 제어 없음')
    parser.add_argument('--address', required=True)
    parser.add_argument('--cert', required=True)
    parser.add_argument('--key', required=True)
    args = parser.parse_args()
    address = ipaddress.ip_address(args.address)
    if address.version != 4 or not address.is_private or address.is_loopback or address.is_unspecified:
        raise SystemExit('Pi의 사설 IPv4 주소가 필요합니다')
    import getpass
    import paramiko  # 실행 전 의존성을 확인한다.
    del paramiko
    state = State(str(address) + ':8444', getpass.getuser(), read_host_key('/etc/ssh/ssh_host_ed25519_key.pub'))
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    context.load_cert_chain(args.cert, args.key)
    server = Server((str(address), 8444), Handler)
    server.state = state
    server.socket = context.wrap_socket(server.socket, server_side=True)
    print('인증 레이더: https://' + state.host + '/ · 가상 조종만 지원', flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
