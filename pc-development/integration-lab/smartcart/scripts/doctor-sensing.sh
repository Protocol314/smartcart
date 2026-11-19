#!/usr/bin/env bash
# 장치 설정이나 시리얼 출력을 변경하지 않는 진단만 수행한다.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SENSING_ROS_DISTRO="${SENSING_ROS_DISTRO:-humble}"
case "$SENSING_ROS_DISTRO" in
  humble|jazzy) ;;
  *) printf '%s\n' 'SENSING_ROS_DISTRO는 humble 또는 jazzy여야 합니다.' >&2; exit 2 ;;
esac
if [ -f "/opt/ros/$SENSING_ROS_DISTRO/setup.bash" ]; then
  source "/opt/ros/$SENSING_ROS_DISTRO/setup.bash"
fi
SENSING_ROS_WORKSPACE="${SENSING_ROS_WORKSPACE:-$HOME/smartcart_ws}"
if [[ "$SENSING_ROS_WORKSPACE" == /* ]] && [ -f "$SENSING_ROS_WORKSPACE/install/setup.bash" ]; then
  source "$SENSING_ROS_WORKSPACE/install/setup.bash"
fi
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
printf 'ROS_DISTRO=%s\nROS_LOCALHOST_ONLY=%s\nROS_DOMAIN_ID=%s\nRMW_IMPLEMENTATION=%s\n' \
  "${ROS_DISTRO:-미설정}" "$ROS_LOCALHOST_ONLY" "${ROS_DOMAIN_ID:-0 (기본)}" "${RMW_IMPLEMENTATION:-기본}"
printf '\n%s\n' 'USB 장치 식별 링크:'
if [ -d /dev/serial/by-id ]; then
  ls -l /dev/serial/by-id/
else
  printf '%s\n' '/dev/serial/by-id가 없습니다. ESP32와 LiDAR USB 연결을 확인하십시오.'
fi
printf '\n%s\n' '패키지 진단:'
cd "$ROOT"
exec /usr/bin/python3 "$ROOT/sensing.py" doctor "$@"
