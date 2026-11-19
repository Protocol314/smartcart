"""실제 루프백 HTTP/SSE의 좌우 관측 전달과 읽기 전용 경계를 검사한다."""
import http.client
import json
from pathlib import Path
import threading
import time
import unittest

from smartcart_radar.core import ScanStore
from smartcart_sensing.dual_uwb import DualUwbStore
from smartcart_sensing.server import SensingHTTPServer
from smartcart_sensing.service import SensingStore
from smartcart_sensing.uwb import UwbStore
from smartcart_uwb.estimation_config import EstimationConfig
from test_tracking_runtime import mounting_config, status

ROOT = Path(__file__).resolve().parents[1]


class DualUwbHttpTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mounting = mounting_config()
        lidar = ScanStore('ros')
        cls.dual = DualUwbStore(cls.mounting, EstimationConfig(), stop=lidar.stop)
        cls.store = SensingStore(lidar, UwbStore('off', stop=lidar.stop), dual_uwb=cls.dual)
        cls.server = SensingHTTPServer(('127.0.0.1', 0), cls.store, ROOT / 'web')
        cls.port = cls.server.server_port
        cls.worker = threading.Thread(target=cls.server.serve_forever,
                                      kwargs={'poll_interval': .05}, daemon=True)
        cls.worker.start()

    @classmethod
    def tearDownClass(cls):
        cls.store.stop.set()
        cls.server.shutdown()
        cls.server.server_close()
        cls.worker.join(timeout=3)
        if cls.worker.is_alive():
            raise AssertionError('검사용 서버가 종료되지 않았습니다')

    def setUp(self):
        # 테스트마다 독립 세션을 써 앞 검사의 표본을 재사용하지 않는다.
        self.dual = DualUwbStore(self.mounting, EstimationConfig(), stop=self.store.stop)
        self.store.dual_uwb = self.dual
        now = time.monotonic_ns()
        self.dual.consume_status(status(self.mounting, at=now), now)

    def request(self, path, method='GET', headers=None):
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=3)
        try:
            conn.request(method, path, headers=headers or {})
            response = conn.getresponse()
            return response.status, dict(response.getheaders()), response.read()
        finally:
            conn.close()

    def test_snapshot_and_combined_frame_use_actual_dual_backend(self):
        code, headers, body = self.request('/api/dual-uwb')
        self.assertEqual(code, 200)
        result = json.loads(body)
        self.assertEqual(result['schema'], 'smartcart.dual_uwb_view.v1')
        self.assertEqual(result['sensors']['uwb_left']['range_m'], 2)
        self.assertFalse(result['drive_authorized'])
        self.assertEqual(headers['Cache-Control'], 'no-store')
        combined = json.loads(self.request('/api/frame')[2])
        self.assertEqual(combined['sensing']['uwb_backend'], 'dual')
        self.assertEqual(combined['uwb']['state'], 'off')
        self.assertIn('dual_uwb', combined)

    def test_health_contains_status_without_raw_distance_angle_or_identifiers(self):
        health = json.loads(self.request('/api/health')[2])
        self.assertTrue(health['dual_uwb']['sensors']['uwb_left']['valid'])
        wire = json.dumps(health)
        for forbidden in ('range_m', 'raw_angle_deg', 'tag_address', 'identity', 'position_xy_m'):
            self.assertNotIn(forbidden, wire)

    def test_stream_event_and_bounded_rate(self):
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=3)
        self.addCleanup(conn.close)
        conn.request('GET', '/api/dual-uwb/events?hz=10')
        response = conn.getresponse()
        self.addCleanup(response.close)
        self.assertEqual(response.status, 200)
        values, stamps = [], []
        while len(values) < 3:
            line = response.readline()
            self.assertTrue(line)
            if line.startswith(b'event:'):
                self.assertEqual(line.strip(), b'event: dual_uwb')
            if line.startswith(b'data:'):
                values.append(json.loads(line[5:]))
                stamps.append(time.monotonic())
        self.assertGreaterEqual(stamps[-1] - stamps[0], .17)
        self.assertEqual(values[0]['source_mode'], 'live')

    def test_queries_and_external_origin_are_rejected(self):
        for query in ('?hz=100', '?hz=0', '?hz=2&hz=10', '?hz=', '?extra=x'):
            self.assertEqual(self.request('/api/dual-uwb/events' + query)[0], 400)
        self.assertEqual(self.request('/api/dual-uwb?extra=x')[0], 400)
        self.assertEqual(self.request('/api/dual-uwb', headers={'Origin': 'https://example.com'})[0], 403)
        self.assertEqual(self.request('/api/dual-uwb', method='POST')[0], 405)

    def test_missing_backend_and_stream_capacity(self):
        self.store.dual_uwb = None
        try:
            self.assertEqual(self.request('/api/dual-uwb')[0], 404)
        finally:
            self.store.dual_uwb = self.dual
        for _ in range(8):
            self.assertTrue(self.server.uwb_stream_slots.acquire(blocking=False))
        try:
            self.assertEqual(self.request('/api/dual-uwb/events')[0], 503)
        finally:
            for _ in range(8):
                self.server.uwb_stream_slots.release()
