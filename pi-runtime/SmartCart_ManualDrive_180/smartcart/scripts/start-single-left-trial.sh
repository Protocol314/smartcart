#!/usr/bin/env bash
# 수평거리 가정 시험이며 검증·구동 권한을 만들지 않는다.
set -eo pipefail
if [[ $# -ne 0 ]]; then
  printf '%s\n' '이 시작 스크립트는 인수를 받지 않습니다.' >&2
  exit 2
fi
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
task_app="$(cd -- "$task_dir/.." && pwd -P)"
task_root="$(cd -- "$task_app/.." && pwd -P)"
if [[ ! -r /opt/ros/humble/setup.bash || ! -r "$task_root/workspace/install/setup.bash" || ! -r "$task_app/local-config/run.json" ]]; then
  printf '%s\n' 'ROS 환경 또는 로컬 실행 설정이 없습니다.' >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
if [[ -r "$HOME/smartcart_ws/install/setup.bash" ]]; then source "$HOME/smartcart_ws/install/setup.bash"; fi
if [[ -r "$HOME/smartcart_lidar_ws/install/setup.bash" ]]; then source "$HOME/smartcart_lidar_ws/install/setup.bash"; fi
source "$task_root/workspace/install/setup.bash"
export ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
cd -- "$task_app"
task_trial_run="$(python3 -m smartcart_runner.single_trial "$task_app/local-config/run.json")"
printf '%s\n' '왼쪽 단독 수평거리 가정 시험 · 장착과 정확도 미검증 · 모터 구동 없음'
printf '시험 설정: %s\n' "$task_trial_run"
exec python3 "$task_app/cart.py" --config "$task_trial_run" start --mode live --single-left
