#!/usr/bin/env bash
# 기존 LiDAR 드라이버와 별개로 통합 센싱 서버 한 개만 실행한다.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [ "$#" -ne 0 ]; then
  printf '%s\n' '이 실행기는 CLI 인수를 받지 않습니다. README-SENSING.md의 SENSING_* 환경변수를 사용하십시오.' >&2
  exit 2
fi
SENSING_ROS_DISTRO="${SENSING_ROS_DISTRO:-humble}"
case "$SENSING_ROS_DISTRO" in
  humble|jazzy) ;;
  *) printf '%s\n' 'SENSING_ROS_DISTRO는 humble 또는 jazzy여야 합니다.' >&2; exit 2 ;;
esac
if [ ! -f "/opt/ros/$SENSING_ROS_DISTRO/setup.bash" ]; then
  printf '%s\n' "설치된 ROS 환경이 없습니다: /opt/ros/$SENSING_ROS_DISTRO/setup.bash" >&2
  exit 2
fi
if [[ ! "${SENSING_SERIAL_PORT:-}" =~ ^/dev/serial/by-id/[A-Za-z0-9_.:+-]{1,200}$ ]]; then
  printf '%s\n' 'ESP32의 /dev/serial/by-id/... 경로를 SENSING_SERIAL_PORT에 명시하십시오.' >&2
  exit 2
fi
if [ ! -c "$SENSING_SERIAL_PORT" ] || [ ! -r "$SENSING_SERIAL_PORT" ] || [ ! -w "$SENSING_SERIAL_PORT" ]; then
  printf '%s\n' 'ESP32 시리얼 장치가 없거나 현재 계정에 읽기/쓰기 권한이 없습니다.' >&2
  exit 2
fi
SENSING_HTTP_PORT="${SENSING_HTTP_PORT:-8765}"
if [[ ! "$SENSING_HTTP_PORT" =~ ^[0-9]{4,5}$ ]] || (( 10#$SENSING_HTTP_PORT < 1024 || 10#$SENSING_HTTP_PORT > 65535 )); then
  printf '%s\n' 'SENSING_HTTP_PORT는 1024~65535여야 합니다.' >&2
  exit 2
fi
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
if [[ ! "$ROS_LOCALHOST_ONLY" =~ ^[01]$ ]]; then
  printf '%s\n' 'ROS_LOCALHOST_ONLY는 0 또는 1이어야 합니다.' >&2
  exit 2
fi
# ROS_DOMAIN_ID와 RMW_IMPLEMENTATION은 기존 운용 환경을 보존한다.
source "/opt/ros/$SENSING_ROS_DISTRO/setup.bash"
SENSING_ROS_WORKSPACE="${SENSING_ROS_WORKSPACE:-$HOME/smartcart_ws}"
if [[ "$SENSING_ROS_WORKSPACE" != /* ]] || [ ! -f "$SENSING_ROS_WORKSPACE/install/setup.bash" ]; then
  printf '%s\n' '기존 LiDAR 워크스페이스의 절대 경로와 install/setup.bash를 확인하십시오.' >&2
  exit 2
fi
source "$SENSING_ROS_WORKSPACE/install/setup.bash"
cd "$ROOT"
exec /usr/bin/python3 "$ROOT/sensing.py" serve \
  --mode ros --uwb-mode serial --serial-port "$SENSING_SERIAL_PORT" --baud 115200 \
  --topic "${SENSING_SCAN_TOPIC:-/scan}" --uwb-topic "${SENSING_UWB_TOPIC:-/smartcart/uwb}" \
  --host 127.0.0.1 --port "$SENSING_HTTP_PORT" \
  --uwb-yaw "${SENSING_UWB_YAW:-0}" --uwb-x "${SENSING_UWB_X:-0}" --uwb-y "${SENSING_UWB_Y:-0}"
