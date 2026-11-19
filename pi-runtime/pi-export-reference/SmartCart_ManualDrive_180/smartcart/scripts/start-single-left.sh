#!/usr/bin/env bash
# 왼쪽 BU04와 LiDAR만 수집하며 구동을 시작하지 않는다.
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
printf '%s\n' '왼쪽 BU04 단독 실측 실행 · 모터 구동 없음 · 종료: Ctrl+C'
exec python3 "$task_app/cart.py" --config "$task_app/local-config/run.json" start --mode live --single-left
