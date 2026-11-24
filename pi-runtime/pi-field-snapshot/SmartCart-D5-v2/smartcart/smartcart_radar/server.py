"""Read-only, dependency-free HTTP + Server-Sent Events bridge.

Use only on a trusted LAN or via an SSH tunnel. Not an Internet-facing server.
No control endpoints, no serial access, no file uploads and no external CDNs.
"""
from __future__ import annotations
import json
import mimetypes
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlsplit
from .core import ScanStore

WEB_ROOT = Path(__file__).resolve().parent.parent / "web"


class RadarHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    request_queue_size = 16

    def __init__(self, address, store: ScanStore, web_root: Path = WEB_ROOT):
        self.store = store
        self.web_root = web_root.resolve()
        self.stream_slots = threading.BoundedSemaphore(8)
        super().__init__(address, RadarHandler)


class RadarHandler(BaseHTTPRequestHandler):
    server_version = "SmartCartRadar/1.0.1"
    protocol_version = "HTTP/1.1"

    def setup(self):
        super().setup()
        self.connection.settimeout(8.0)

    def log_message(self, fmt, *args):
        # No noisy per-frame logs. Startup and source errors are shown by main.
        pass

    def write_headers(self, content_type: str, length=None):
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' blob: data:; connect-src 'self'; object-src 'none'; base-uri 'none'; frame-ancestors 'none'")
        if length is not None:
            self.send_header("Content-Length", str(length))
        self.end_headers()

    def json_response(self, status: int, value: dict):
        payload = json.dumps(value, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.write_headers("application/json; charset=utf-8", len(payload))
        self.wfile.write(payload)

    def do_POST(self):
        self.close_connection = True
        self.json_response(405, {"error": "Read-only service; only GET is supported"})

    do_PUT = do_POST
    do_DELETE = do_POST
    do_PATCH = do_POST

    def do_GET(self):
        parsed = urlsplit(self.path)
        if parsed.path == "/api/frame":
            self.json_response(200, self.server.store.snapshot())
            return
        if parsed.path == "/api/health":
            snapshot = self.server.store.snapshot()
            self.json_response(200, {key: value for key, value in snapshot.items() if key != "scan"})
            return
        if parsed.path == "/api/events":
            params = parse_qs(parsed.query)
            try:
                hz = float(params.get("hz", ["2"])[0])
                if hz not in (1, 2):
                    raise ValueError()
            except (ValueError, IndexError):
                self.json_response(400, {"error": "hz must be 1 or 2"})
                return
            self.stream(hz)
            return
        if parsed.path.startswith("/api/"):
            self.json_response(404, {"error": "No such read-only endpoint"})
            return
        path = unquote(parsed.path)
        if "\x00" in path or "\\" in path or ".." in path.split("/"):
            self.json_response(403, {"error": "Path rejected"})
            return
        candidate = (self.server.web_root / (path.lstrip("/") or "index.html")).resolve()
        if not candidate.is_relative_to(self.server.web_root) or not candidate.is_file():
            self.json_response(404, {"error": "Not found"})
            return
        if candidate.suffix not in (".html", ".css", ".js", ".svg", ".png", ".ico", ".json"):
            self.json_response(403, {"error": "Not a web asset"})
            return
        payload = candidate.read_bytes()
        mime = {".js": "text/javascript", ".css": "text/css", ".html": "text/html", ".svg": "image/svg+xml"}.get(candidate.suffix, mimetypes.guess_type(candidate.name)[0] or "application/octet-stream")
        self.send_response(200)
        self.write_headers(mime + ("; charset=utf-8" if mime.startswith("text/") else ""), len(payload))
        self.wfile.write(payload)

    def stream(self, hz):
        if not self.server.stream_slots.acquire(blocking=False):
            self.json_response(503, {"error": "Maximum 8 simultaneous radar streams"})
            return
        try:
            self.send_response(200)
            self.send_header("X-Accel-Buffering", "no")
            self.send_header("Connection", "close")
            self.write_headers("text/event-stream; charset=utf-8")
            self.close_connection = True
            self.wfile.write(b"retry: 2000\n\n")
            self.wfile.flush()
            deadline = time.monotonic()
            while not self.server.store.stop.is_set():
                data = self.server.store.snapshot()
                data["display_hz"] = hz
                # No frame queue: a slow client receives the newest snapshot next.
                wire = json.dumps(data, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
                self.wfile.write(("event: frame\ndata: " + wire + "\n\n").encode("utf-8"))
                self.wfile.flush()
                deadline = max(deadline + 1 / hz, time.monotonic())
                self.server.store.stop.wait(max(0, deadline - time.monotonic()))
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, TimeoutError, OSError):
            pass
        finally:
            self.server.stream_slots.release()
