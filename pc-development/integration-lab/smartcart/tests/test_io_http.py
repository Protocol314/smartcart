import http.client
import json
import math
from pathlib import Path
import tempfile
import threading
import time
import unittest
from smartcart_radar.core import ScanStore, project_scan
from smartcart_radar.scan_io import parse_echo, read_frames, write_scan
from smartcart_radar.server import RadarHTTPServer
from smartcart_radar.sources import synthetic_scan, source_loop
from smartcart_radar.terminal import render_ascii

ROOT = Path(__file__).resolve().parents[1]


class IOTests(unittest.TestCase):
    def test_actual_sample(self):
        scan = next(read_frames(ROOT/'samples/user_scan_truncated.txt'))
        s = scan['stats']
        self.assertEqual((s['provided_rays'], s['expected_rays'], s['valid_sensor_rays'], s['no_return_rays'], s['below_min_rays']), (128,720,56,40,32))
        self.assertTrue(scan['truncated'])
        self.assertEqual(len(project_scan(scan, radius=.25)['points']), 56)

    def test_sample_roundtrip(self):
        text = next(read_frames(ROOT/'samples/user_scan_truncated.txt'))
        obj = next(read_frames(ROOT/'samples/user_scan_truncated.json'))
        self.assertEqual(text['ranges'], obj['ranges'])

    def test_echo_multiple_docs_reads_first(self):
        text = 'angle_min: 0\nranges:\n- 1.0\n---\nranges:\n- 2.0\n'
        self.assertEqual(parse_echo(text)['ranges'], [1])

    def test_reject_flow_yaml(self):
        with self.assertRaises(ValueError): parse_echo('ranges: [1, 2]')

    def test_reject_yaml_tags(self):
        with self.assertRaises(ValueError): parse_echo('ranges:\n- !!python/object/apply:os.system ["false"]')

    def test_reject_terminal_prompt(self):
        with self.assertRaises(ValueError): parse_echo('capstone@SUDO$ ros2 topic echo /scan')

    def test_jsonl_roundtrip(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder)/'record.jsonl'
            with p.open('w') as f:
                write_scan(f, synthetic_scan(0)); write_scan(f, synthetic_scan(1))
            scans = list(read_frames(p))
            self.assertEqual(len(scans), 2)
            self.assertEqual(scans[0]['source']['kind'], 'replay')
            self.assertEqual(scans[0]['source']['original_kind'], 'synthetic')

    def test_synthetic_is_labeled(self):
        scan = synthetic_scan(0)
        self.assertEqual(scan['source']['kind'], 'synthetic')
        self.assertEqual(scan['stats']['provided_rays'], 720)
        self.assertTrue(project_scan(scan)['points'])

    def test_cli_stale_hides_scan(self):
        scan = synthetic_scan(0)
        text = render_ascii({'state':'stale','scan':scan})
        self.assertIn('NO FRESH SCAN', text)
        self.assertNotIn('*', text)

    def test_cli_sample_warning(self):
        scan = next(read_frames(ROOT/'samples/user_scan_truncated.txt'))
        text = render_ascii({'state':'ended','scan':scan}, radius=.25, rows=40)
        self.assertIn('TRUNCATED_SCAN', text)
        self.assertIn('NOT LIVE HARDWARE DATA', text)
        self.assertIn('128/720', text)

    def test_server_replay_single_ends(self):
        store = ScanStore('replay')
        source_loop(store, replay_file=ROOT/'samples/user_scan_truncated.json')
        self.assertEqual(store.snapshot()['state'], 'ended')


class HTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.store = ScanStore('demo')
        cls.server = RadarHTTPServer(('127.0.0.1',0), cls.store)
        cls.port = cls.server.server_port
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True); cls.thread.start()
        cls.producer = threading.Thread(target=source_loop, args=(cls.store,), daemon=True); cls.producer.start()
        time.sleep(.2)

    @classmethod
    def tearDownClass(cls):
        cls.store.stop.set(); cls.server.shutdown(); cls.server.server_close()
        cls.thread.join(); cls.producer.join()

    def get(self, path, method='GET'):
        connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=4)
        connection.request(method, path)
        response = connection.getresponse()
        status, headers, data = response.status, dict(response.getheaders()), response.read()
        connection.close()
        return status, headers, data

    def test_real_http_page(self):
        status, _, body = self.get('/')
        self.assertEqual(status,200); self.assertIn(b'SmartCart',body)

    def test_static_all_assets(self):
        for path in ['/js/core.js','/js/sample.js','/js/radar.js','/js/app.js','/css/app.css','/assets/icon.svg']:
            self.assertEqual(self.get(path)[0],200)

    def test_csp_and_no_cors(self):
        _, h, _ = self.get('/')
        self.assertIn("connect-src 'self'",h['Content-Security-Policy'])
        self.assertNotIn('Access-Control-Allow-Origin',h)

    def test_api_no_nonfinite_json(self):
        status, _, body = self.get('/api/frame')
        self.assertEqual(status,200)
        data=json.loads(body,parse_constant=lambda value: self.fail(value))
        self.assertEqual(data['source_mode'],'demo')
        self.assertEqual(len(data['scan']['ranges']),720)

    def test_health_omits_big_arrays(self):
        self.assertNotIn('scan',json.loads(self.get('/api/health')[2]))

    def test_traversal_denied(self):
        for path in ['/../radar.py','/%2e%2e/radar.py','/js/../../radar.py','/..%5cradar.py']:
            self.assertEqual(self.get(path)[0],403)

    def test_python_source_not_exposed(self):
        self.assertEqual(self.get('/radar.py')[0],404)

    def test_no_motor_endpoint(self):
        self.assertEqual(self.get('/api/cmd_vel')[0],404)

    def test_writes_rejected(self):
        for method in ['POST','PUT','DELETE','PATCH']:
            self.assertEqual(self.get('/api/frame', method)[0],405)

    def test_hz_validation(self):
        for hz in ['0','10','nan','inf','abc']:
            self.assertEqual(self.get('/api/events?hz='+hz)[0],400)

    def sample_stream(self, hz, count=3):
        connection = http.client.HTTPConnection('127.0.0.1',self.port,timeout=4)
        connection.request('GET', f'/api/events?hz={hz}')
        response = connection.getresponse()
        self.assertEqual(response.status,200)
        arrivals, seqs = [], []
        while len(arrivals)<count:
            line = response.readline().decode()
            if line.startswith('data: '):
                data = json.loads(line[6:]); arrivals.append(time.monotonic()); seqs.append(data['scan']['sequence'])
                self.assertEqual(data['display_hz'],hz)
        response.close(); connection.close()
        return arrivals, seqs

    def test_real_sse_two_hz_latest_only(self):
        arrivals, seqs = self.sample_stream(2)
        self.assertTrue(all(.35 < b-a < .8 for a,b in zip(arrivals,arrivals[1:])))
        self.assertTrue(all(b-a >= 3 for a,b in zip(seqs,seqs[1:])))

    def test_real_sse_one_hz(self):
        arrivals, seqs = self.sample_stream(1)
        self.assertTrue(all(.8 < b-a < 1.3 for a,b in zip(arrivals,arrivals[1:])))
        self.assertTrue(all(b-a >= 7 for a,b in zip(seqs,seqs[1:])))
