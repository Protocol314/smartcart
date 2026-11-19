#!/usr/bin/env bash
# 현재 플랫폼 C 파서가 준비된 환경에서 센서 없는 통합 검사를 실행한다.
set -euo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$TASK_ROOT"
exec python3 -X utf8 scripts/verify-offline.py "$@"
