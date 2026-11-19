#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# 기존 순수 C 파서를 명시적으로 빌드하며 실제 장치는 열지 않는다.
python3 scripts/build_uwb_native.py --cc "${CC:-cc}"
python3 -m unittest discover -s tests -p 'test_dual_uwb*.py' -v
python3 -m smartcart_uwb replay --config config/dual_uwb.d5.example.json --input samples/dual_uwb.synthetic.jsonl --summary
python3 -m smartcart_uwb estimate-replay --config config/dual_uwb.d5.example.json --estimation-config config/uwb_estimation.synthetic.json --input samples/dual_uwb.position.synthetic.jsonl --summary
