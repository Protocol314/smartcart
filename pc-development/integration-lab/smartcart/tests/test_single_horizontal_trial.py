"""수평거리 가정 시험은 원본을 덮지 않고 미검증 상태를 보존한다."""
import json
from pathlib import Path
import tempfile
import unittest
import subprocess
import sys

from smartcart_runner.config import initialize, load_config
from smartcart_runner.single_trial import prepare_trial
from smartcart_uwb.estimation_config import load_estimation_config
from smartcart_uwb.single_estimator import SingleLeftEstimator
from test_tracking_runtime import mounting_config, status, SECOND


class HorizontalTrialTests(unittest.TestCase):
    def test_original_and_legacy_remain_unchanged(self):
        with tempfile.TemporaryDirectory() as temp:
            source = initialize(Path(temp) / 'config')
            before = {p.name: p.read_bytes() for p in source.parent.iterdir()}
            first = prepare_trial(source)
            second = prepare_trial(source)
            self.assertNotEqual(first, second)
            self.assertEqual(before, {p.name: p.read_bytes() for p in source.parent.iterdir() if p.is_file()})
            run = load_config(first)
            settings = load_estimation_config(run.uwb_config)
            self.assertEqual(settings.range_model, 'horizontal')
            self.assertIs(settings.assumptions_verified, False)
            self.assertEqual(load_estimation_config(load_config(source).uwb_config).range_model, 'unknown')
            result = SingleLeftEstimator(mounting_config(), settings, source_mode='live').update(
                status(mounting_config()), SECOND)
            self.assertTrue(result['valid'])
            self.assertTrue(result['provisional'])
            self.assertFalse(result['drive_authorized'])

    def test_missing_config_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaises(ValueError):
                prepare_trial(Path(temp) / 'missing.json')

    def test_installer_keeps_existing_config_and_is_idempotent(self):
        root_source = Path(__file__).resolve().parents[2]
        installer = root_source / 'build/releases/smartcart-single-horizontal-trial-20261117-install.py'
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            app = root / 'smartcart'
            for directory in ('smartcart_runner', 'scripts', 'local-config'):
                (app / directory).mkdir(parents=True, exist_ok=True)
            (app / 'cart.py').write_text('# 기존 실행기\n', encoding='utf-8')
            original = app / 'local-config/uwb.json'
            original.write_text('{"range_model":"unknown"}', encoding='utf-8')
            for _ in range(2):
                run = subprocess.run([sys.executable, '-X', 'utf8', str(installer), '--root', str(root), '--apply'],
                                     capture_output=True, text=True, encoding='utf-8')
                self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            self.assertIn('이미 같은 업데이트', run.stdout)
            self.assertEqual(original.read_text(encoding='utf-8'), '{"range_model":"unknown"}')
