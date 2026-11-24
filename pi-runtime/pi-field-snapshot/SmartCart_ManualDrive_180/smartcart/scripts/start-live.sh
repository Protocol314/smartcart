#!/usr/bin/env bash
# Pi의 ROS 환경을 불러와 확인된 실측 구성만 한 번에 실행한다.
set -eo pipefail

if [[ $# -ne 0 ]]; then
  printf '%s\n' '이 시작 스크립트는 인수를 받지 않습니다.' >&2
  exit 2
fi

task_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
task_smartcart_dir="$(cd -- "$task_script_dir/.." && pwd -P)"
task_install_dir="$(cd -- "$task_smartcart_dir/.." && pwd -P)"
task_ros_setup=/opt/ros/humble/setup.bash
task_workspace_setup="$task_install_dir/workspace/install/setup.bash"
task_config="$task_smartcart_dir/local-config/run.json"

if [[ ! -r "$task_ros_setup" ]]; then
  printf '%s\n' 'ROS 2 Humble 환경이 없습니다: /opt/ros/humble/setup.bash' >&2
  exit 2
fi
if [[ ! -r "$task_workspace_setup" ]]; then
  printf '%s\n' '통합 ROS workspace가 없습니다. 설치와 빌드 결과를 확인하십시오.' >&2
  exit 2
fi
if [[ ! -f "$task_config" || ! -r "$task_config" ]]; then
  printf '%s\n' '실측 설정이 없습니다. 장치 배정 절차를 먼저 완료하십시오.' >&2
  exit 2
fi

# ROS 설정 스크립트가 미정의 변수를 참조할 수 있어 여기서는 nounset을 사용하지 않는다.
source "$task_ros_setup"
if [[ -r "$HOME/smartcart_ws/install/setup.bash" ]]; then
  source "$HOME/smartcart_ws/install/setup.bash"
fi
if [[ -r "$HOME/smartcart_lidar_ws/install/setup.bash" ]]; then
  source "$HOME/smartcart_lidar_ws/install/setup.bash"
fi
source "$task_workspace_setup"

export ROS_LOCALHOST_ONLY=1
export PYTHONDONTWRITEBYTECODE=1
export PYTHONUNBUFFERED=1
cd -- "$task_smartcart_dir"
printf '%s\n' 'SmartCart 실측 실행: 장치와 설정을 검사합니다. 종료는 Ctrl+C입니다.'
exec python3 "$task_smartcart_dir/cart.py" --config "$task_config" start --mode live
