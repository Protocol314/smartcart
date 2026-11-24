"""같은 출처의 GUI와 센서별 SSE를 제공하는 제한된 읽기 전용 HTTP 서버."""
from __future__ import annotations

import ipaddress
import json
import threading
import time
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from smartcart_radar.server import RadarHandler, RadarHTTPServer


class SensingHTTPServer(RadarHTTPServer):
    """동시 요청과 스트림 수를 제한하고 localhost 호스트 검증을 적용한다."""

    def __init__(self, address, store, web_root: Path, *, allow_lan: bool = False):
        self.allow_lan = allow_lan
        self.request_slots = threading.BoundedSemaphore(32)
        self.uwb_stream_slots = threading.BoundedSemaphore(8)
        self.tracking_stream_slots = threading.BoundedSemaphore(8)
        super().__init__(address, store, web_root)
        self.RequestHandlerClass = SensingHandler

    def process_request(self, request, client_address):
        if not self.request_slots.acquire(blocking=False):
            try:
                request.sendall(b"HTTP/1.1 503 Service Unavailable\r\nContent-Length: 0\r\nConnection: close\r\n\r\n")
            except OSError:
                pass
            # 표준 TCP 종료로 503 응답을 전송한 뒤 소켓을 닫는다.
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except Exception:
            self.request_slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.request_slots.release()


class SensingHandler(RadarHandler):
    server_version = "SmartCartSensing/0.1.0"

    def parse_request(self):
        if not super().parse_request():
            return False
        if len(self.path) > 2048:
            self.close_connection = True
            self.json_response(414, {"error": "요청 경로가 너무 깁니다"})
            return False
        try:
            hosts = self.headers.get_all("Host", [])
            if len(hosts) != 1 or len(hosts[0]) > 256:
                raise ValueError()
            authority = urlsplit("http://" + hosts[0])
            if authority.username or authority.password or authority.path or authority.query or authority.fragment:
                raise ValueError()
            hostname = authority.hostname
            port = authority.port or 80
            if port != self.server.server_port:
                raise ValueError()
            if hostname not in {"localhost", "127.0.0.1", "::1"}:
                if not self.server.allow_lan:
                    raise ValueError()
                address = ipaddress.ip_address(hostname)
                if address.is_unspecified or address.is_multicast:
                    raise ValueError()
            origin = self.headers.get("Origin")
            if origin:
                parsed = urlsplit(origin)
                if (parsed.scheme != "http" or parsed.netloc.lower() != hosts[0].lower()
                        or parsed.path or parsed.query or parsed.fragment):
                    raise ValueError()
        except (ValueError, TypeError):
            self.close_connection = True
            self.json_response(403, {"error": "허용하지 않는 Host 또는 Origin입니다"})
            return False
        return True

    def write_headers(self, content_type: str, length=None):
        self.send_header("X-Frame-Options", "DENY")
        super().write_headers(content_type, length)

    def do_GET(self):
        parsed = urlsplit(self.path)
        if parsed.path in {'/api/dual-uwb', '/api/dual-uwb/events'}:
            dual = getattr(self.server.store, 'dual_uwb', None)
            if dual is None:
                self.json_response(404, {'error': '이 실행에는 좌우 UWB 표시가 없습니다'})
                return
            try:
                values = parse_qs(parsed.query, keep_blank_values=True)
                if parsed.path == '/api/dual-uwb':
                    if values:
                        raise ValueError()
                    self.json_response(200, dual.snapshot())
                    return
                if any(key != 'hz' for key in values) or len(values.get('hz', ['10'])) != 1:
                    raise ValueError()
                hz = int(values.get('hz', ['10'])[0])
                if hz not in {2, 5, 10}:
                    raise ValueError()
            except (ValueError, IndexError):
                self.json_response(400, {'error': '좌우 UWB 표시 주기는 2, 5, 10Hz만 허용합니다'})
                return
            self.stream_dual_uwb(hz)
            return
        if parsed.path in {"/api/tracking", "/api/tracking/events"}:
            if getattr(self.server.store, "tracking", None) is None:
                self.json_response(404, {"error": "이 실행에서 추적 표시를 사용하지 않습니다"})
                return
            try:
                values = parse_qs(parsed.query, keep_blank_values=True)
                if parsed.path == "/api/tracking":
                    if values:
                        raise ValueError()
                    self.json_response(200, self.server.store.tracking.snapshot())
                    return
                if any(key != "hz" for key in values) or len(values.get("hz", ["10"])) != 1:
                    raise ValueError()
                hz = int(values.get("hz", ["10"])[0])
                if hz not in {2, 5, 10}:
                    raise ValueError()
            except (ValueError, IndexError):
                self.json_response(400, {"error": "추적 표시 주기는 2, 5, 10Hz만 허용합니다"})
                return
            self.stream_tracking(hz)
            return
        if parsed.path == "/api/uwb/events":
            try:
                values = parse_qs(parsed.query, keep_blank_values=True)
                if any(key != "hz" for key in values) or len(values.get("hz", ["10"])) != 1:
                    raise ValueError()
                hz = int(values.get("hz", ["10"])[0])
                if hz not in {2, 5, 10}:
                    raise ValueError()
            except (ValueError, IndexError):
                self.json_response(400, {"error": "UWB 표시 주기는 2, 5, 10Hz만 허용합니다"})
                return
            self.stream_uwb(hz)
            return
        if parsed.path == "/api/uwb":
            self.json_response(200, self.server.store.uwb.snapshot())
            return
        if parsed.path == "/api/sensing":
            self.json_response(200, self.server.store.snapshot()["sensing"])
            return
        if parsed.path == "/api/health":
            snapshot = self.server.store.snapshot()
            uwb = snapshot["uwb"]
            health = {
                **{k: v for k, v in snapshot.items() if k not in {"scan", "uwb", "tracking", "dual_uwb"}},
                "uwb": {k: v for k, v in uwb.items() if k not in {"measurement", "transform"}},
            }
            if "tracking" in snapshot:
                health["tracking"] = {k: v for k, v in snapshot["tracking"].items()
                                      if k not in {"result", "lidar_transform"}}
            if 'dual_uwb' in snapshot:
                dual = snapshot['dual_uwb']
                health['dual_uwb'] = {key: value for key, value in dual.items() if key != 'sensors'}
                health['dual_uwb']['sensors'] = {role: {key: value for key, value in sensor.items()
                    if key in {'state', 'valid', 'age_ms', 'measurement_valid', 'id_valid'}}
                    for role, sensor in dual['sensors'].items()}
            self.json_response(200, health)
            return
        super().do_GET()

    def stream_dual_uwb(self, hz):
        if not self.server.uwb_stream_slots.acquire(blocking=False):
            self.json_response(503, {'error': 'UWB 스트림은 최대 8개입니다'})
            return
        try:
            self.send_response(200)
            self.send_header('X-Accel-Buffering', 'no')
            self.send_header('Connection', 'close')
            self.write_headers('text/event-stream; charset=utf-8')
            self.close_connection = True
            self.wfile.write(b'retry: 1000\n\n')
            self.wfile.flush()
            while not self.server.store.stop.is_set():
                value = self.server.store.dual_uwb.snapshot()
                wire = json.dumps(value, ensure_ascii=False, separators=(',', ':'), allow_nan=False)
                self.wfile.write(('event: dual_uwb\ndata: ' + wire + '\n\n').encode('utf-8'))
                self.wfile.flush()
                self.server.store.stop.wait(1 / hz)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            self.server.uwb_stream_slots.release()

    def stream_tracking(self, hz):
        if not self.server.tracking_stream_slots.acquire(blocking=False):
            self.json_response(503, {"error": "추적 스트림은 최대 8개입니다"})
            return
        try:
            self.send_response(200)
            self.send_header("X-Accel-Buffering", "no")
            self.send_header("Connection", "close")
            self.write_headers("text/event-stream; charset=utf-8")
            self.close_connection = True
            self.wfile.write(b"retry: 1000\n\n")
            self.wfile.flush()
            deadline = time.monotonic()
            while not self.server.store.stop.is_set():
                snapshot = self.server.store.tracking.snapshot()
                wire = json.dumps(snapshot, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
                self.wfile.write(("event: tracking\ndata: " + wire + "\n\n").encode("utf-8"))
                self.wfile.flush()
                deadline = max(deadline + 1 / hz, time.monotonic())
                self.server.store.stop.wait(max(0, deadline - time.monotonic()))
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, TimeoutError, OSError):
            pass
        finally:
            self.server.tracking_stream_slots.release()

    def stream_uwb(self, hz: int):
        if not self.server.uwb_stream_slots.acquire(blocking=False):
            self.json_response(503, {"error": "UWB 스트림은 최대 8개입니다"})
            return
        try:
            self.send_response(200)
            self.send_header("X-Accel-Buffering", "no")
            self.send_header("Connection", "close")
            self.write_headers("text/event-stream; charset=utf-8")
            self.close_connection = True
            self.wfile.write(b"retry: 1000\n\n")
            self.wfile.flush()
            deadline = time.monotonic()
            while not self.server.store.stop.is_set():
                snapshot = self.server.store.uwb.snapshot()
                wire = json.dumps(snapshot, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
                self.wfile.write(("event: uwb\ndata: " + wire + "\n\n").encode("utf-8"))
                self.wfile.flush()
                deadline = max(deadline + 1 / hz, time.monotonic())
                self.server.store.stop.wait(max(0, deadline - time.monotonic()))
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, TimeoutError, OSError):
            pass
        finally:
            self.server.uwb_stream_slots.release()
