#!/usr/bin/env bash
# ROS 환경은 실측 전에 운영자가 source하며, 기본 실행은 센서 없는 합성 재생이다.
set -euo pipefail
task_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
export PYTHONDONTWRITEBYTECODE=1
if [[ $# -eq 0 ]]; then
  exec python3 "$task_script_dir/../cart.py" start --mode replay
fi
exec python3 "$task_script_dir/../cart.py" "$@"
