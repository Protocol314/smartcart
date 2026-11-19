#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
python3 -m unittest discover -s tests -v
if command -v node >/dev/null; then
  node tests/test_frontend.cjs
  node tests/test_contours.cjs
  node tests/test_sensing_frontend.cjs
  node tests/test_tracking_frontend.cjs
fi
