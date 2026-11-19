"""실제 루프백 HTTP·SSE와 읽기 전용 경계 검증."""
import http.client
import json
from pathlib import Path
import threading
import time
import unittest

from smartcart_radar.core import ScanStore
from smartcart_radar.sources import source_loop
from smartcart_sensing.server import SensingHTTPServer
from smartcart_sensing.service import SensingStore
from smartcart_sensing.uwb import UwbStore, demo_loop


class SensingHTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lidar = ScanStore("demo")
        cls.uwb = UwbStore("demo", stop=cls.lidar.stop)
        cls.store = SensingStore(cls.lidar, cls.uwb, "demo")
        cls.server = SensingHTTPServer(("127.0.0.1", 0), cls.store,
                                      Path(__file__).resolve().parents[1] / "web")
        cls.port = cls.server.server_port
        cls.workers = [threading.Thread(target=cls.server.serve_forever, daemon=True),
                       threading.Thread(target=source_loop, args=(cls.lidar,), daemon=True),
                       threading.Thread(target=demo_loop, args=(cls.uwb,), daemon=True)]
        for worker in cls.workers:
            worker.start()
        time.sleep(.15)

    @classmethod
    def tearDownClass(cls):
        cls.store.stop.set()
        cls.server.shutdown()
        cls.server.server_close()
        for worker in cls.workers:
            worker.join(timeout=3)

    def request(self, path, method="GET", headers=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=3)
        connection.request(method, path, headers=headers or {})
        response = connection.getresponse()
        status, metadata, body = response.status, dict(response.getheaders()), response.read()
        connection.close()
        return status, metadata, body

    def test_combined_contract_and_no_nonfinite_values(self):
        status, _, body = self.request("/api/frame")
        self.assertEqual(status, 200)
        data = json.loads(body, parse_constant=lambda _: self.fail("비유한 JSON"))
        self.assertEqual(data["schema"], "smartcart.envelope.v1")
        self.assertEqual(data["uwb"]["schema"], "smartcart.uwb.v1")
        self.assertEqual(data["sensing"]["schema"], "smartcart.sensing.v1")
        self.assertFalse(data["sensing"]["ready"])
        self.assertEqual(len(data["scan"]["ranges"]), 720)

    def test_health_has_no_raw_measurements(self):
        data = json.loads(self.request("/api/health")[2])
        self.assertNotIn("scan", data)
        self.assertNotIn("measurement", data["uwb"])
        self.assertIn("sensing", data)

    def test_independent_uwb_endpoint(self):
        data = json.loads(self.request("/api/uwb")[2])
        self.assertEqual(data["state"], "synthetic")
        self.assertTrue(data["synthetic"])

    def test_security_headers_and_same_origin(self):
        status, headers, body = self.request("/")
        self.assertEqual(status, 200)
        self.assertIn(b"js/sensing.js", body)
        self.assertEqual(headers["X-Frame-Options"], "DENY")
        self.assertIn("connect-src 'self'", headers["Content-Security-Policy"])
        self.assertNotIn("Access-Control-Allow-Origin", headers)

    def test_dns_rebinding_and_cross_origin_are_rejected(self):
        self.assertEqual(self.request("/api/frame", headers={"Host": f"evil.invalid:{self.port}"})[0], 403)
        self.assertEqual(self.request("/api/frame", headers={"Origin": "https://evil.invalid"})[0], 403)
        self.assertEqual(self.request("/api/frame", headers={"Origin": f"http://127.0.0.1:{self.port}"})[0], 200)

    def test_writes_traversal_and_control_endpoints_are_denied(self):
        for method in ("POST", "PUT", "DELETE", "PATCH"):
            self.assertEqual(self.request("/api/uwb", method)[0], 405)
        for path in ("/../sensing.py", "/%2e%2e/sensing.py"):
            self.assertEqual(self.request(path)[0], 403)
        self.assertEqual(self.request("/api/cmd_vel")[0], 404)

    def test_uwb_stream_rejects_unbounded_or_duplicate_rates(self):
        for query in ("hz=0", "hz=100000", "hz=nan", "hz=", "hz=10&hz=10", "x=10"):
            self.assertEqual(self.request("/api/uwb/events?" + query)[0], 400)

    def test_uwb_sse_is_independent_and_ten_hz(self):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=3)
        connection.request("GET", "/api/uwb/events?hz=10")
        response = connection.getresponse()
        self.assertEqual(response.status, 200)
        arrivals, sequences = [], []
        while len(arrivals) < 4:
            line = response.readline().decode("utf-8")
            if line.startswith("event:"):
                self.assertEqual(line.strip(), "event: uwb")
            if line.startswith("data:"):
                value = json.loads(line[5:])
                arrivals.append(time.monotonic())
                sequences.append(value["measurement"]["firmware_timestamp_ms"])
                self.assertTrue(value["synthetic"])
                self.assertNotIn("scan", value)
        response.close()
        connection.close()
        self.assertLess(arrivals[-1] - arrivals[0], .8)
        self.assertGreater(arrivals[-1] - arrivals[0], .15)
        self.assertGreater(sequences[-1], sequences[0])

    def test_request_thread_admission_is_bounded(self):
        server = SensingHTTPServer(("127.0.0.1", 0), self.store, self.server.web_root)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        acquired = 0
        connection = None
        try:
            # 다른 요청의 정리 시점과 무관하도록 전용 서버에서 포화시킨다.
            while server.request_slots.acquire(False):
                acquired += 1
            self.assertEqual(acquired, 32)
            worker.start()
            connection = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=3)
            try:
                connection.request("GET", "/api/health")
                self.assertEqual(connection.getresponse().status, 503)
            except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError,
                    http.client.RemoteDisconnected):
                # 요청을 읽기 전에 거절하면 Windows TCP는 503 대신 연결 종료를 전달할 수 있다.
                # 시간 초과나 정상 200 응답은 허용하지 않는다.
                pass
            self.assertFalse(server.request_slots.acquire(False))
        finally:
            if connection:
                connection.close()
            for _ in range(acquired):
                server.request_slots.release()
            if worker.is_alive():
                server.shutdown()
                worker.join(timeout=3)
            server.server_close()
