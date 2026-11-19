"""실제 루프백 소켓으로 추적 HTTP/SSE와 기존 읽기 전용 경계를 검증한다."""
from __future__ import annotations

from copy import deepcopy
import http.client
import json
from pathlib import Path
import threading
import time
import unittest

from smartcart_radar.core import ScanStore
from smartcart_sensing.server import SensingHTTPServer
from smartcart_sensing.uwb import UwbStore

ROOT = Path(__file__).resolve().parents[1]
FORBIDDEN_HEALTH_KEYS = {"result", "track", "uwb_estimate", "lidar_clusters", "lidar_transform", "tag_address",
                         "track_id", "position_xy_m", "predicted_position_xy_m", "velocity_xy_mps",
                         "ranges", "clusters"}


def assert_summary_only(test, value):
    """집계에 원시 위치나 대상 ID가 어느 깊이에도 끼어들지 않는지 검사한다."""
    if isinstance(value, dict):
        test.assertFalse(FORBIDDEN_HEALTH_KEYS.intersection(value))
        for item in value.values():
            assert_summary_only(test, item)
    elif isinstance(value, list):
        for item in value:
            assert_summary_only(test, item)


class FixedTrackingView:
    """HTTP 전달 계약 대역이다. 실제 수명 계산은 TrackingStore 검사에서 다룬다."""
    def __init__(self, stop):
        self.stop = stop
        self.lock = threading.Lock()
        self.reset()

    def reset(self):
        with self.lock:
            self.mode = "replay"
            self.value = {
                "schema": "smartcart.tracking_view.v1", "mode": "replay", "source_mode": "replay",
                "state": "running", "sequence": 1, "age_ms": 0.0, "remaining_ms": 1000.0,
                "cluster_remaining_ms": 1000.0, "prediction_remaining_ms": 0.0,
                "valid": True, "drive_authorized": False, "provisional": True, "frame_id": "base_link",
                "lidar_transform": {"yaw_deg": 0.0, "translation_m": [.817, 0.0, .2678]},
                "result": {
                    "uwb_estimate": {"valid": True, "position_xy_m": [2.8, 0.0], "tag_address": 28185},
                    "lidar_clusters": {"valid": True, "clusters": [{"centroid_xy_m": [2.72, 0.0]}]},
                    "track": {"schema": "smartcart.target_track.v1", "state": "tracking", "valid": True,
                              "track_id": 17, "position_xy_m": [2.72, 0.0], "velocity_xy_mps": [0.0, 0.0],
                              "predicted_position_xy_m": None, "drive_authorized": False},
                },
            }

    def snapshot(self, **_kwargs):
        with self.lock:
            return deepcopy(self.value)

    def change(self, **changes):
        with self.lock:
            self.value.update(changes)
            self.mode = self.value["mode"]


class FixedSensingStore:
    """입력 생산·ROS·장치를 시작하지 않는 기존 센싱 저장소의 전달 대역."""
    def __init__(self):
        self.lidar = ScanStore("replay")
        self.stop = self.lidar.stop
        self.uwb = UwbStore("off", stop=self.stop)
        self.tracking = FixedTrackingView(self.stop)
        self.mode = "replay"

    def snapshot(self, **kwargs):
        return {**self.lidar.snapshot(**kwargs), "uwb": self.uwb.snapshot(),
                "tracking": self.tracking.snapshot(),
                "sensing": {"schema": "smartcart.sensing.v1", "state": "waiting", "mode": "replay",
                            "ready": False, "synthetic": True, "frame_id": "base_link"}}


class TrackingHTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.store = FixedSensingStore()
        cls.server = SensingHTTPServer(("127.0.0.1", 0), cls.store, ROOT / "web")
        cls.port = cls.server.server_port
        cls.worker = threading.Thread(target=cls.server.serve_forever,
                                      kwargs={"poll_interval": .05}, daemon=True)
        cls.worker.start()

    @classmethod
    def tearDownClass(cls):
        cls.store.stop.set()
        cls.server.shutdown()
        cls.server.server_close()
        cls.worker.join(timeout=3)
        if cls.worker.is_alive():
            raise AssertionError("검사용 HTTP 서버가 종료되지 않았습니다")

    def setUp(self):
        self.store.tracking.reset()

    def request(self, path, method="GET", headers=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=3)
        try:
            connection.request(method, path, headers=headers or {})
            response = connection.getresponse()
            return response.status, dict(response.getheaders()), response.read()
        finally:
            connection.close()

    def open_stream(self, hz=10):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=3)
        connection.request("GET", f"/api/tracking/events?hz={hz}")
        response = connection.getresponse()
        self.addCleanup(connection.close)
        self.addCleanup(response.close)
        self.assertEqual(response.status, 200)
        self.assertIn("text/event-stream", response.getheader("Content-Type"))
        return response

    def read_tracking(self, response):
        event = None
        for _ in range(30):
            line = response.readline()
            self.assertTrue(line, "SSE가 관측 이벤트 없이 끝났습니다")
            if line.startswith(b"event: "):
                event = line[7:].strip()
            elif line.startswith(b"data: "):
                self.assertEqual(event, b"tracking")
                return json.loads(line[6:], parse_constant=lambda _: self.fail("비유한 JSON"))
        self.fail("SSE 프레임을 찾지 못했습니다")

    def test_tracking_snapshot_and_combined_frame_keep_contracts(self):
        status, _, body = self.request("/api/tracking")
        self.assertEqual(status, 200)
        tracking = json.loads(body)
        self.assertEqual(tracking["schema"], "smartcart.tracking_view.v1")
        self.assertEqual(tracking["mode"], "replay")
        self.assertEqual(tracking["source_mode"], "replay")
        self.assertFalse(tracking["drive_authorized"])
        self.assertTrue(tracking["provisional"])
        self.assertEqual(tracking["result"]["track"]["track_id"], 17)
        combined = json.loads(self.request("/api/frame")[2])
        self.assertEqual(combined["schema"], "smartcart.envelope.v1")
        self.assertEqual(combined["uwb"]["schema"], "smartcart.uwb.v1")
        self.assertEqual(combined["sensing"]["schema"], "smartcart.sensing.v1")
        self.assertEqual(combined["tracking"], tracking)

    def test_health_omits_tracking_positions_ids_and_raw_arrays(self):
        status, _, body = self.request("/api/health")
        self.assertEqual(status, 200)
        summary = json.loads(body)["tracking"]
        self.assertEqual(summary["mode"], "replay")
        self.assertFalse(summary["drive_authorized"])
        assert_summary_only(self, summary)

    def test_off_and_error_states_remain_read_only(self):
        self.store.tracking.change(mode="off", source_mode="off", state="off", result=None,
                                   valid=False, remaining_ms=0, cluster_remaining_ms=0)
        off = json.loads(self.request("/api/tracking")[2])
        self.assertEqual(off["state"], "off")
        self.assertIsNone(off["result"])
        self.assertFalse(off["valid"])
        self.store.tracking.change(mode="replay", source_mode="replay", state="error")
        self.assertEqual(json.loads(self.request("/api/tracking")[2])["state"], "error")

    def test_security_headers_no_cors_and_gui_assets(self):
        status, headers, body = self.request("/")
        self.assertEqual(status, 200)
        self.assertEqual(headers["X-Frame-Options"], "DENY")
        self.assertIn("connect-src 'self'", headers["Content-Security-Policy"])
        self.assertNotIn("Access-Control-Allow-Origin", headers)
        self.assertIn(b"trackingPanel", body)
        status, headers, body = self.request("/js/tracking.js")
        self.assertEqual(status, 200)
        self.assertIn("javascript", headers["Content-Type"])

    def test_host_and_origin_validation_covers_tracking_and_streams(self):
        for path in ("/api/tracking", "/api/tracking/events?hz=10"):
            with self.subTest(path=path):
                self.assertEqual(self.request(path, headers={"Host": f"evil.invalid:{self.port}"})[0], 403)
                self.assertEqual(self.request(path, headers={"Origin": "https://evil.invalid"})[0], 403)
        valid = {"Origin": f"http://127.0.0.1:{self.port}"}
        self.assertEqual(self.request("/api/tracking", headers=valid)[0], 200)

    def test_all_write_methods_and_control_routes_are_rejected(self):
        before = self.store.tracking.snapshot()
        for method in ("POST", "PUT", "PATCH", "DELETE"):
            with self.subTest(method=method):
                self.assertEqual(self.request("/api/tracking", method=method)[0], 405)
        for path in ("/api/tracking/start", "/api/tracking/stop", "/api/tracking/control"):
            self.assertEqual(self.request(path)[0], 404)
        self.assertEqual(self.store.tracking.snapshot(), before)

    def test_stream_hz_unknown_duplicate_and_nonfinite_values_rejected(self):
        for query in ("hz=0", "hz=1", "hz=4", "hz=20", "hz=nan", "hz=inf", "hz=2.5",
                      "hz=", "hz=-1", "hz=2&hz=5", "hz=10&control=1"):
            with self.subTest(query=query):
                self.assertEqual(self.request("/api/tracking/events?" + query)[0], 400)

    def test_snapshot_does_not_accept_control_or_stream_parameters(self):
        for query in ("control=1", "hz=10", "mode=ros", "token=unused"):
            with self.subTest(query=query):
                self.assertEqual(self.request("/api/tracking?" + query)[0], 400)

    def test_stream_event_name_latest_snapshot_and_finite_json(self):
        response = self.open_stream(10)
        first = self.read_tracking(response)
        self.assertEqual(first["sequence"], 1)
        self.store.tracking.change(sequence=27, state="stale", valid=False, result=None, remaining_ms=0)
        latest = self.read_tracking(response)
        self.assertEqual(latest["sequence"], 27)
        self.assertEqual(latest["state"], "stale")
        self.assertIsNone(latest["result"])

    def test_two_hz_stream_does_not_send_at_ten_hz(self):
        response = self.open_stream(2)
        self.read_tracking(response)
        begin = time.monotonic()
        self.read_tracking(response)
        self.assertGreaterEqual(time.monotonic() - begin, .35)

    def test_tracking_stream_limit_is_eight(self):
        slots = self.server.tracking_stream_slots
        acquired = 0
        deadline = time.monotonic() + 2
        try:
            # 앞 검사의 닫힌 SSE 처리기가 토큰을 반납할 때까지 짧게 기다린다.
            while acquired < 8 and time.monotonic() < deadline:
                if slots.acquire(False):
                    acquired += 1
                else:
                    time.sleep(.01)
            self.assertEqual(acquired, 8)
        finally:
            for _ in range(acquired):
                slots.release()
        streams = []
        try:
            # 실제 소켓 여덟 개를 연결해야 개별 SSE 처리기의 제한도 검증된다.
            for _ in range(8):
                response = self.open_stream(10)
                streams.append(response)
                self.read_tracking(response)
            self.assertEqual(self.request("/api/tracking/events?hz=10")[0], 503)
        finally:
            for response in streams:
                response.close()
        acquired = 0
        deadline = time.monotonic() + 2
        try:
            while acquired < 8 and time.monotonic() < deadline:
                if slots.acquire(False):
                    acquired += 1
                else:
                    time.sleep(.01)
            self.assertEqual(acquired, 8, "닫힌 실제 SSE의 자원이 반환되지 않았습니다")
        finally:
            for _ in range(acquired):
                slots.release()


if __name__ == "__main__":
    unittest.main()
