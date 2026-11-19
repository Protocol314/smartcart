#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# 합성 자료만 처리하며 실제 장치·ROS·모터를 열지 않는다.
python3 scripts/build_uwb_native.py --cc "${CC:-cc}"
python3 -m unittest discover -s tests -p 'test_target*.py' -v
python3 -m unittest discover -s tests -p 'test_tracking*.py' -v
if command -v node >/dev/null; then
  node tests/test_tracking_frontend.cjs
  node tests/test_field_preview.cjs
fi
python3 -m smartcart_tracking --config config/dual_uwb.d5.example.json --uwb-estimation-config config/uwb_estimation.synthetic.json --tracking-config config/target_tracking.synthetic.json --input samples/target_tracking.synthetic.jsonl --summary
