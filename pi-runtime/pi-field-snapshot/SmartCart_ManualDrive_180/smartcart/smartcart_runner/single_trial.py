"""원본을 보존한 왼쪽 UWB 수평거리 가정 시험 설정을 만든다."""
from dataclasses import asdict, replace
import json
import os
from pathlib import Path
import sys
import tempfile

from .config import SCHEMA, load_config
from smartcart_uwb.estimation_config import load_estimation_config


def _write(path, payload):
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, 'w', encoding='utf-8', newline='\n') as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def prepare_trial(run_path):
    """검증되지 않은 수평거리 가정의 설정만 별도 디렉터리에 생성한다."""
    config = load_config(run_path)
    settings = load_estimation_config(config.uwb_config)
    trial = replace(settings, range_model='horizontal', tag_height_m=None,
                    assumptions_verified=False)
    directory = Path(tempfile.mkdtemp(prefix='single-left-horizontal-trial-', dir=config.path.parent))
    directory.chmod(0o700)
    trial_uwb = directory / 'uwb.json'
    trial_run = directory / 'run.json'
    _write(trial_uwb, asdict(trial))
    _write(trial_run, {'schema': SCHEMA, 'mounting_config': str(config.mounting_config),
        'field_config': str(config.field_config), 'uwb_config': str(trial_uwb),
        'tracking_config': str(config.tracking_config), 'http_port': config.http_port,
        'lidar_mode': config.lidar_mode, 'lidar_port': config.lidar_port,
        'lidar_frame_id': config.lidar_frame_id})
    load_config(trial_run)
    load_estimation_config(trial_uwb)
    return trial_run


def main():
    if len(sys.argv) != 2:
        raise SystemExit('사용법: python3 -m smartcart_runner.single_trial 기존-run.json')
    print(prepare_trial(sys.argv[1]))


if __name__ == '__main__':
    main()
