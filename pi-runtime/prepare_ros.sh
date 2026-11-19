#!/usr/bin/env bash
# 현재 위치에 ROS 패키지를 구성하며 장치나 모터에 연결하지 않는다.
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
if [[ ! -r /opt/ros/humble/setup.bash ]] || ! command -v colcon >/dev/null; then
  printf '%s\n' 'Ubuntu 22.04용 ROS 2 Humble과 colcon이 필요합니다.' >&2
  exit 2
fi
if [[ "$(uname -m)" != "aarch64" ]] || ! command -v gcc >/dev/null || ! command -v python3 >/dev/null; then
  printf '%s\n' 'Pi ARM64용 gcc와 python3가 필요합니다.' >&2
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
set -u
mkdir -p "$task_root/.runtime"
: > "$task_root/.runtime/build-in-progress"
python3 "$task_root/SmartCart-D5-v2/smartcart/scripts/build_uwb_native.py" --cc gcc \
  --output "$task_root/SmartCart-D5-v2/smartcart/smartcart_uwb/lib/libsmartcart_uwb_parser.so"
colcon --log-base "$task_root/.runtime/log" build \
  --base-paths "$task_root/smartcart_ws/src" "$task_root/SmartCart-D5-v2/ros2" "$task_root/SmartCart-D5-v2/smartcart" \
  --build-base "$task_root/.runtime/build" --install-base "$task_root/.runtime/install"
printf '%s\n' "$task_root" > "$task_root/.runtime/root.txt"
rm -f -- "$task_root/.runtime/build-in-progress"
printf '%s\n' '현재 위치에 ROS 작업 환경을 구성했습니다.'
