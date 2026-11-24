#!/usr/bin/env python3
"""Serve the existing SmartCart screen plus manual GPIO driving on one LAN port."""
from __future__ import annotations

import argparse
import asyncio
from collections import deque
import contextlib
import importlib.util
import ipaddress
from pathlib import Path
import secrets
import socket
import ssl
import sys
import time
from urllib.parse import urlsplit

try:
    from aiohttp import ClientError, ClientSession, ClientTimeout, WSMsgType, web
except ImportError:
    raise SystemExit("aiohttp 없음: sudo apt install python3-aiohttp")

from smartcart_motor.manual_remote import ManualDrive, MockBackend, RPiBackend

ROOT = Path(__file__).resolve().parent / "web"
SENSOR_PATHS = {
    "/api/frame", "/api/health", "/api/sensing", "/api/events",
    "/api/uwb", "/api/uwb/events", "/api/dual-uwb", "/api/dual-uwb/events",
    "/api/tracking", "/api/tracking/events",
}


def sensor_address(value):
    parsed = urlsplit(value)
    if (parsed.scheme != "http" or parsed.username or parsed.password
            or parsed.path not in ("", "/") or parsed.query or parsed.fragment):
        raise ValueError("센서 주소는 Pi 내부 http://127.0.0.1:8765 형식이어야 합니다")
    host = parsed.hostname
    if host != "localhost" and not ipaddress.ip_address(host).is_loopback:
        raise ValueError("센서 중계는 같은 Pi의 루프백 서버만 지원합니다")
    if not 1024 <= (parsed.port or 80) <= 65535:
        raise ValueError("센서 서버 포트는 1024~65535여야 합니다")
    return value.rstrip("/")


def valid_origin(request):
    host = urlsplit("http://" + request.host)
    if host.username or host.password or host.path or host.query or host.fragment:
        return False
    if host.hostname != "localhost":
        address = ipaddress.ip_address(host.hostname)
        if not (address.is_loopback or address.is_private) or address.is_unspecified or address.is_multicast:
            return False
    origin = request.headers.get("Origin")
    if origin:
        parsed = urlsplit(origin)
        if (parsed.scheme != request.scheme or parsed.netloc.lower() != request.host.lower()
                or parsed.path or parsed.query or parsed.fragment):
            return False
    return True


@web.middleware
async def boundary(request, handler):
    try:
        permitted = valid_origin(request)
    except (ValueError, TypeError):
        permitted = False
    if not permitted:
        raise web.HTTPForbidden(text="허용하지 않는 Host/Origin")
    return await handler(request)


async def page(request):
    return web.FileResponse(ROOT / "index.html", headers={"Cache-Control": "no-store"})


async def motor_health(request):
    return web.json_response(request.app["drive"].snapshot(), headers={"Cache-Control": "no-store"})


async def drive_socket(request):
    app = request.app
    if app["ws_count"] >= 4:
        raise web.HTTPServiceUnavailable(text="조종 연결 수 초과")
    app["ws_count"] += 1
    ws = web.WebSocketResponse(max_msg_size=2048, compress=False)
    owner = None
    sender = None
    try:
        await ws.prepare(request)
        app["sockets"].add(ws)
        now = time.monotonic()
        attempts = app["auth_failures"]
        while attempts and now - attempts[0] >= 60:
            attempts.popleft()
        if len(attempts) >= 5:
            await ws.send_json({"error": "인증 시도 횟수 초과 · 1분 후 다시 시도"})
            await ws.close(code=1008)
            return ws
        try:
            auth = await asyncio.wait_for(ws.receive_json(), timeout=5)
        except (ValueError, TypeError, asyncio.TimeoutError):
            await ws.close(code=1008)
            return ws
        if (not isinstance(auth, dict) or type(auth.get("key")) is not str
                or not auth["key"].isascii() or len(auth["key"]) > 256
                or not secrets.compare_digest(auth["key"], app["key"])):
            attempts.append(time.monotonic())
            await ws.send_json({"error": "연결키가 맞지 않습니다"})
            await ws.close(code=1008)
            return ws
        candidate = secrets.token_hex(12)
        drive = app["drive"]
        if not drive.acquire(candidate):
            await ws.send_json({"error": "다른 조종자가 연결 중이거나 GPIO 오류 상태입니다"})
            await ws.close(code=1008)
            return ws
        owner = candidate

        async def publish():
            try:
                while not ws.closed:
                    await asyncio.wait_for(ws.send_json(drive.snapshot()), timeout=0.15)
                    await asyncio.sleep(0.05)
            finally:
                drive.release(owner)
                await ws.close()

        sender = asyncio.create_task(publish())
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                break
            try:
                try:
                    command = msg.json()
                except (ValueError, TypeError):
                    command = None
                drive.command(owner, command)
            except (ValueError, TypeError):
                # 잘못된 명령에서 출력은 이미 0으로 잠겼다. 연결은 유지하고 재활성화한다.
                await ws.send_json({"error": "명령 오류·만료: 정지 후 다시 활성화하십시오"})
    finally:
        # An unauthenticated or rejected second connection must never stop the owner.
        if owner is not None:
            app["drive"].release(owner)
        if sender:
            sender.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await sender
        app["sockets"].discard(ws)
        app["ws_count"] -= 1
        await ws.close()
    return ws


async def sensor_proxy(request):
    if request.path not in SENSOR_PATHS:
        raise web.HTTPNotFound()
    url = request.app["sensor_url"] + request.path
    try:
        async with request.app["client"].get(
            url, params=request.query, allow_redirects=False,
        ) as upstream:
            if upstream.status != 200:
                return web.json_response(
                    {"error": "센서 서버 응답: " + str(upstream.status)},
                    status=upstream.status if 400 <= upstream.status <= 599 else 502)
            response = web.StreamResponse(
                status=200,
                headers={
                    "Content-Type": upstream.headers.get("Content-Type", "application/json"),
                    "Cache-Control": "no-store", "X-Accel-Buffering": "no",
                },
            )
            await response.prepare(request)
            try:
                async for chunk in upstream.content.iter_any():
                    await asyncio.wait_for(response.write(chunk), timeout=1)
            except (ConnectionError, ClientError, asyncio.TimeoutError):
                response.force_close()
            return response
    except (ClientError, OSError, asyncio.TimeoutError):
        return web.json_response(
            {"error": "센서 서버 연결 없음. 기존 센싱 서버를 127.0.0.1:8765에서 켜십시오. 가상 센서로 대체하지 않습니다."},
            status=502,
        )


def make_app(backend=None, *, key=None, sensor_url="http://127.0.0.1:8765", start_worker=True):
    app = web.Application(client_max_size=2048, middlewares=[boundary])
    app["drive"] = ManualDrive(backend if backend is not None else MockBackend())
    app["key"] = key or f"{secrets.randbelow(10000):04d}"
    app["auth_failures"] = deque()
    app["sensor_url"] = sensor_address(sensor_url)
    app["sockets"] = set()
    app["ws_count"] = 0

    async def resources(application):
        drive = application["drive"]
        if start_worker:
            drive.start()
        else:
            drive.backend.open()
        try:
            async with ClientSession(
                timeout=ClientTimeout(total=None, connect=1, sock_read=3),
                trust_env=False,
            ) as client:
                application["client"] = client
                yield
        finally:
            drive.close()

    async def shutdown(application):
        for ws in tuple(application["sockets"]):
            await ws.close(code=1001, message=b"Server shutting down")

    async def headers(request, response):
        response.headers["Cache-Control"] = "no-store"
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"

    app.cleanup_ctx.append(resources)
    app.on_shutdown.append(shutdown)
    app.on_response_prepare.append(headers)
    app.router.add_get("/", page)
    app.router.add_get("/index.html", page)
    app.router.add_get("/api/drive/health", motor_health)
    app.router.add_get("/ws/drive", drive_socket)
    app.router.add_get("/api/{tail:.*}", sensor_proxy)
    for folder in ("js", "css", "assets"):
        app.router.add_static("/" + folder + "/", ROOT / folder, show_index=False)
    return app


def main(argv=None):
    parser = argparse.ArgumentParser(description="SmartCart 화면 + 수동 주행, 고정 EN / Pi 4")
    parser.add_argument("--backend", choices=("mock", "gpio"), default="mock")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--sensor-url", default="http://127.0.0.1:8765")
    parser.add_argument("--left-direction", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--right-direction", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--cert", type=Path)
    parser.add_argument("--key", type=Path, help="선택적 HTTPS 인증서 개인키 파일")
    parser.add_argument("--doctor", action="store_true", help="GPIO를 열지 않고 실행 환경 확인")
    args = parser.parse_args(argv)
    try:
        if not 1024 <= args.port <= 65535:
            raise ValueError("포트는 1024~65535여야 합니다")
        if args.host != "localhost":
            address = ipaddress.IPv4Address(args.host)
            if address.is_multicast or (not address.is_private and not address.is_loopback and not address.is_unspecified):
                raise ValueError("이 서버는 로컬/사설 LAN용입니다")
        source = sensor_address(args.sensor_url)
        if urlsplit(source).port == args.port:
            raise ValueError("조종 포트와 센싱 포트는 달라야 합니다")
        if bool(args.cert) != bool(args.key):
            raise ValueError("HTTPS에는 --cert와 --key 둘 다 필요합니다")
        if args.backend == "gpio" and not args.cert and args.host not in ("localhost", "127.0.0.1"):
            raise ValueError("4자리 연결키를 사용하는 LAN 실제 조종은 HTTPS 인증서가 필요합니다")
        if args.doctor:
            model = Path("/proc/device-tree/model")
            print("장치:", model.read_text().strip("\0") if model.exists() else "Pi 모델 정보 없음")
            try:
                gpio_available = importlib.util.find_spec("RPi.GPIO") is not None
            except ModuleNotFoundError:
                gpio_available = False
            print("RPi.GPIO:", "설치됨" if gpio_available else "없음 (PC mock 모드는 정상)")
            print("핀: BCM 17/27/23/24, GND 물리 9번, EN/VCC=5V, GPIO22 미사용")
            print("기본 선택 PWM 50 / 최대 180; 센서:", source)
            return 0
        if args.backend == "gpio":
            model_file = Path("/proc/device-tree/model")
            if not model_file.exists() or "Raspberry Pi 4" not in model_file.read_text():
                raise ValueError("실제 GPIO 모드는 라즈베리파이 4에서만 실행하십시오")
            backend = RPiBackend((args.left_direction, args.right_direction))
        else:
            backend = MockBackend()
        app = make_app(backend, sensor_url=source)
        tls = None
        if args.cert:
            tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            tls.minimum_version = ssl.TLSVersion.TLSv1_2
            tls.load_cert_chain(str(args.cert), str(args.key))
        print("SmartCart 수동 조종 / " + ("실제 GPIO 출력" if backend.hardware else "모의 출력 (모터 미연결)"), flush=True)
        print("연결키:", app["key"], flush=True)
        print("아이폰: " + ("https" if tls else "http") + "://<Pi IP>:" + str(args.port), flush=True)
        print("EN=5V 고정: 화면 정지는 PWM 0, 물리 전원 차단 아님. 자율 추종/장애물 자동 정지 없음.", flush=True)
        web.run_app(app, host=args.host, port=args.port, ssl_context=tls, access_log=None)
        return 0
    except (OSError, ValueError, RuntimeError, ImportError) as exc:
        print("실행 실패:", exc, file=sys.stderr)
        print("설치: sudo apt install python3-aiohttp python3-rpi.gpio", file=sys.stderr)
        print("센싱은 8765, 이 조종 서버는 8080. 기존 모터 조종 프로그램과 동시 실행하지 마십시오.", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
