"""조종 화면에 센서 요소와 가상 입력이 함께 유지되는지 검사한다."""
from html.parser import HTMLParser
from pathlib import Path
import unittest


class Layout(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = set()
        self.virtual = set()
        self.details_closed = False

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if attrs.get('id'):
            if attrs['id'] in self.ids:
                raise AssertionError('중복 ID: ' + attrs['id'])
            self.ids.add(attrs['id'])
        if attrs.get('data-virtual'):
            self.virtual.add(attrs['data-virtual'])
        if tag == 'details' and 'sensor-details' in attrs.get('class', '').split():
            self.details_closed = 'open' not in attrs


class LayoutTest(unittest.TestCase):
    def test_layout_and_sensor_elements(self):
        root = Path(__file__).resolve().parents[1] / 'web'
        parser = Layout()
        parser.feed((root / 'index.html').read_text(encoding='utf-8'))
        self.assertTrue(parser.details_closed)
        self.assertEqual(parser.virtual, {'forward', 'reverse', 'left', 'right', 'stop'})
        self.assertTrue({'radar', 'radarUwb', 'radarTracking', 'dualUwbPanel',
                         'trackingPanel', 'source', 'virtualRpmLimit', 'virtualRpmOutput'} <= parser.ids)
        code = (root / 'js/virtual_controls.js').read_text(encoding='utf-8')
        self.assertNotIn('fetch(', code)
        self.assertNotIn('WebSocket', code)


if __name__ == '__main__':
    unittest.main()
