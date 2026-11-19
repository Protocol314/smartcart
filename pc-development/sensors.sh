#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"

if curl -fsS --max-time 2 http://127.0.0.1:8765/api/health >/dev/null 2>&1; then
  echo "센싱 서버가 이미 실행 중입니다."
  exit 0
fi
if [[ ! -r /opt/ros/humble/setup.bash || ! -r "$ROOT/.runtime/install/setup.bash" || ! -r "$ROOT/.runtime/root.txt" || -e "$ROOT/.runtime/build-in-progress" ]] || [[ "$(cat "$ROOT/.runtime/root.txt")" != "$ROOT" ]]; then
  echo '현재 폴더용 ROS 작업 환경이 없습니다. 먼저 bash prepare_ros.sh를 실행하세요.' >&2
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/.runtime/install/setup.bash"
set -u
export ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
task_app="$ROOT/SmartCart-D5-v2/smartcart"
cd -- "$task_app"
task_trial_run="$(python3 -m smartcart_runner.single_trial "$task_app/local-config/run.json")"
echo '단일 BU04 수평거리 가정 센싱 · GPIO 미사용 · 정확도 미검증'
exec python3 "$task_app/cart.py" --config "$task_trial_run" start --mode live --single-left
