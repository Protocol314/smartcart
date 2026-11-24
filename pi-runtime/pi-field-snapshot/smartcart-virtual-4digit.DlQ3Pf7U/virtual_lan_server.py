#!/usr/bin/env python3
"""사설망 HTTPS 인증을 사용하는 MOCK 전용 가상 조종 서버."""
import argparse
from collections import deque
import getpass
import ipaddress
import secrets
import ssl
import time

from aiohttp import web
from drive import make_app
from smartcart_motor.manual_remote import MockBackend

LOGIN = '''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>스마트카트 가상 시험</title><body><h1>스마트카트 가상 시험</h1><p>Pi에서 설정한 화면 접속키를 입력하세요.</p><form action="/login" method="post"><label>화면 접속키 <input name="key" type="password" maxlength="128" required autocomplete="off"></label><button type="submit">접속</button></form><p>가상 출력 전용 · 실제 모터 미연결</p></body></html>'''


def valid_pin(value):
    return isinstance(value, str) and len(value) == 4 and value.isascii() and value.isdigit()


def make_guard(address, port, password, clock=time.monotonic):
    host = f'{address}:{port}'
    expected = password.encode('utf-8')
    attempts = deque()
    sessions = {}

    @web.middleware
    async def guard(request, handler):
        now = clock()
        if request.host != host:
            raise web.HTTPForbidden(text='접속 주소가 일치하지 않습니다')
        origin = request.headers.get('Origin')
        if origin and origin != 'https://' + host:
            raise web.HTTPForbidden(text='요청 출처가 일치하지 않습니다')
        if request.path == '/login':
            if request.method == 'GET':
                return web.Response(text=LOGIN, content_type='text/html', headers={'Cache-Control':'no-store', 'Referrer-Policy':'same-origin'})
            if request.method != 'POST' or origin != 'https://' + host:
                raise web.HTTPForbidden(text='요청 출처를 확인하세요')
            while attempts and now - attempts[0] >= 900:
                attempts.popleft()
            if len(attempts) >= 5:
                raise web.HTTPTooManyRequests(text='접속 시도가 제한되었습니다. 15분 후 다시 시도하세요')
            try:
                form = await request.post()
                candidate = form.get('key', '')
                valid = isinstance(candidate, str) and len(candidate) <= 128 and secrets.compare_digest(candidate.encode('utf-8'), expected)
            except (ValueError, UnicodeError):
                valid = False
            if not valid:
                attempts.append(now)
                raise web.HTTPUnauthorized(text='접속키를 확인하세요')
            sessions.clear()
            token = secrets.token_urlsafe(32)
            sessions[token] = now + 3600
            response = web.HTTPSeeOther('/')
            response.set_cookie('sc_virtual_session', token, httponly=True, secure=True, samesite='Strict', max_age=3600, path='/')
            raise response
        session = request.cookies.get('sc_virtual_session', '')
        if not session or sessions.get(session, 0) <= now:
            if request.path in ('/', '/index.html'):
                raise web.HTTPFound('/login')
            raise web.HTTPUnauthorized(text='화면 접속이 필요합니다')
        return await handler(request)

    return guard


def main():
    parser = argparse.ArgumentParser(description='스마트카트 가상 조종 · HTTPS · GPIO 미사용')
    parser.add_argument('--address', required=True)
    parser.add_argument('--port', type=int, default=18083)
    parser.add_argument('--cert', required=True)
    parser.add_argument('--key', required=True)
    args = parser.parse_args()
    try:
        ip = ipaddress.IPv4Address(args.address)
        if ip not in ipaddress.IPv4Network('192.168.43.0/24') or ip.packed[-1] in (0,255):
            raise ValueError('192.168.43.1~254 주소가 필요합니다')
        if not 1024 <= args.port <= 65535 or args.port == 8765:
            raise ValueError('포트는 1024~65535이며 센서 포트와 달라야 합니다')
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.minimum_version = ssl.TLSVersion.TLSv1_2
        context.load_cert_chain(args.cert, args.key)
        password = getpass.getpass('화면 접속키 설정 (숫자 4자리, 입력 숨김): ')
        if not valid_pin(password) or password != getpass.getpass('화면 접속키 재입력: '):
            raise ValueError('접속키는 숫자 4자리이며 확인 값과 같아야 합니다')
        app = make_app(MockBackend(), sensor_url='http://127.0.0.1:8765')
        app.middlewares.insert(0, make_guard(str(ip), args.port, password))
        print('가상 조종 · GPIO 미사용 · 화면: https://' + str(ip) + ':' + str(args.port) + '/', flush=True)
        print('조이스틱 연결키:', app['key'], flush=True)
        print('종료: Ctrl+C', flush=True)
        web.run_app(app, host=str(ip), port=args.port, ssl_context=context, access_log=None)
    except (OSError, ValueError, ssl.SSLError) as exc:
        raise SystemExit('시작 실패: ' + str(exc)) from None

if __name__ == '__main__':
    main()
