#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"

if curl -fsS --max-time 2 http://127.0.0.1:8765/api/health >/dev/null 2>&1; then
  echo "센싱 서버가 이미 실행 중입니다."
  exit 0
fi
exec bash "$ROOT/SmartCart-D5-v2/smartcart/scripts/start-single-left-trial.sh"
