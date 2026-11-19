#!/usr/bin/env python3
"""기존 로컬 센싱 화면을 일회성 연결키와 HTTPS로 같은 LAN에 중계한다.

센서 데이터는 기존 루프백 서버에서 읽으며 GPIO와 모터 API를 제공하지 않는다.
"""
import argparse
import getpass
import hashlib
import hmac
import ipaddress
from pathlib import Path
import ssl

import radar_https_gateway as gateway

# 일반 HTML 로그인 POST의 Origin이 null로 바뀌지 않도록 한다. 같은 폴더에 남은
# 이전 게이트웨이(no-referrer)를 불러와도 이 실행기는 이 값으로 응답한다.
REFERRER_POLICY = 'same-origin'


def make_state(host, token):
    if not isinstance(token, str) or not token:
        raise ValueError('연결키가 필요합니다')

    def authenticate(_username, candidate, _host_key):
        matches = isinstance(candidate, str) and hmac.compare_digest(candidate.encode('utf-8'), token.encode('utf-8'))
        print('연결키 인증:', '성공' if matches else '실패', flush=True)
        return matches

    return gateway.State(host, '시연', None, authenticator=authenticate)


class LoginDiagnosticsHandler(gateway.Handler):
    """비밀 값이나 주소를 남기지 않고 접속 차단 단계만 터미널에 표시한다."""

    def send_header(self, keyword, value):
        if keyword.lower() == 'referrer-policy':
            value = REFERRER_POLICY
        super().send_header(keyword, value)

    def do_POST(self):
        if self.path == '/login':
            origin = self.headers.get('Origin')
            print('로그인 요청 검사: 주소 일치 =', self.host_ok(),
                  '| 요청 출처 =', '없음' if origin is None else
                  ('일치' if origin == 'https://' + self.server.state.host else '불일치'),
                  flush=True)
        return super().do_POST()


def configure_login_page():
    page = gateway.LOGIN.decode('utf-8')
    old = 'Pi 계정의 SSH 비밀번호를 입력하세요.'
    if old not in page:
        raise ValueError('기존 로그인 화면 형식이 일치하지 않습니다')
    page = page.replace(old, 'Pi 터미널에서 정한 시연 연결키를 입력하세요.')
    page = page.replace('비밀번호<input', '연결키<input')
    page = page.replace('autocomplete="current-password"', 'autocomplete="off"')
    gateway.LOGIN = page.encode('utf-8')


def loaded_gateway():
    """실제로 불러온 게이트웨이 파일을 터미널에서 대조할 수 있게 한다."""
    path = Path(gateway.__file__).resolve()
    return path, hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--address', required=True, help='현재 Pi의 사설 IPv4 주소')
    parser.add_argument('--cert', required=True, type=Path)
    parser.add_argument('--key', required=True, type=Path)
    args = parser.parse_args()
    address = ipaddress.ip_address(args.address)
    if address.version != 4 or not address.is_private or address.is_loopback or address.is_unspecified:
        raise SystemExit('Pi의 사설 IPv4 주소를 지정하십시오')
    for path in (args.cert, args.key):
        if path.is_symlink() or not path.is_file():
            raise SystemExit('인증서 또는 개인키 파일을 확인하십시오')
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    context.load_cert_chain(str(args.cert), str(args.key))
    configure_login_page()
    token = getpass.getpass('이번 시연에 사용할 새 연결키 (12자 이상, 입력 내용 숨김): ')
    if not 12 <= len(token) <= 128:
        raise SystemExit('연결키는 12~128자여야 합니다')
    if token != getpass.getpass('연결키 다시 입력: '):
        raise SystemExit('연결키가 일치하지 않습니다')
    state = make_state(f'{address}:8444', token)
    server = gateway.Server((str(address), 8444), LoginDiagnosticsHandler)
    server.state = state
    server.socket = context.wrap_socket(server.socket, server_side=True)
    gateway_path, gateway_hash = loaded_gateway()
    print('게이트웨이 파일:', gateway_path, flush=True)
    print('게이트웨이 SHA-256:', gateway_hash, flush=True)
    print('Referrer-Policy:', REFERRER_POLICY, flush=True)
    print('센싱 화면: https://' + state.host + '/', flush=True)
    print('모터 제어 없음 · 종료: Ctrl+C', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
