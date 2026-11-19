#!/usr/bin/env python3
"""수동 조종의 상태와 정지 이유를 읽는다. GPIO와 조종 명령은 사용하지 않는다."""
import argparse
import ipaddress
import json
import ssl
import time
from urllib.error import URLError
from urllib.request import HTTPSHandler, ProxyHandler, build_opener


def main():
    parser = argparse.ArgumentParser(description="조종 서버 상태를 읽기만 하여 정지 이유를 확인합니다")
    parser.add_argument("--seconds", type=int, default=30)
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--https", action="store_true")
    parser.add_argument("--ca", help="HTTPS용 신뢰 CA 인증서 경로")
    args = parser.parse_args()
    if not 1 <= args.seconds <= 300 or not 1024 <= args.port <= 65535:
        parser.error("시간은 1~300초, 포트는 1024~65535여야 합니다")
    try:
        address = ipaddress.ip_address(args.host)
    except ValueError:
        parser.error("호스트는 Pi의 사설 IPv4 주소여야 합니다")
    if address.version != 4 or not (address.is_private or address.is_loopback) or address.is_unspecified:
        parser.error("호스트는 Pi의 사설 IPv4 주소여야 합니다")
    if args.https and not args.ca:
        parser.error("HTTPS 서버의 CA 인증서가 필요합니다")
    context = ssl.create_default_context(cafile=args.ca) if args.https else None
    url = f"{'https' if args.https else 'http'}://{address}:{args.port}/api/drive/health"
    opener = build_opener(ProxyHandler({}), HTTPSHandler(context=context))
    start = time.monotonic()
    previous = None
    print("조종 상태 확인 시작: GPIO와 조종 명령은 사용하지 않습니다.", flush=True)
    while time.monotonic() - start < args.seconds:
        try:
            with opener.open(url, timeout=0.4) as response:
                result = json.load(response)
            if result.get("schema") != "smartcart.manual.v1":
                raise ValueError("조종 상태 형식이 다릅니다")
            current = (result.get("reason"), result.get("armed"), bool(result.get("fault")))
            if current != previous:
                print(f"{time.monotonic()-start:.2f}초 이유={current[0]} 활성={current[1]} 오류={current[2]}", flush=True)
                previous = current
        except (OSError, URLError, ValueError, json.JSONDecodeError) as exc:
            current = (type(exc).__name__, False, True)
            if current != previous:
                print(f"{time.monotonic()-start:.2f}초 조회 실패={current[0]}", flush=True)
                previous = current
        time.sleep(0.1)
    print("확인 종료", flush=True)


if __name__ == "__main__":
    main()
