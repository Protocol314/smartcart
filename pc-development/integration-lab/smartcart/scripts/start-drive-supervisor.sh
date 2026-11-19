#!/usr/bin/env bash
# 구동 감독기 화면: 기존 ROS 센서 토픽 구독 + 가상 출력. GPIO를 열지 않으며 구동 승인이 아니다.
# 센싱(start-single-left-trial.sh 등)을 다른 터미널에서 먼저 실행한다.
# 기존 설치본(SMARTCART_APP, 기본 ~/SmartCart-D5-v2/smartcart)의 추적·UWB 패키지와 실행 설정을 읽고,
# 이 폴더의 smartcart_motor·web만 앞에 둔다. 설치본 파일은 바꾸지 않는다.
# 예) LAN:  bash start-drive-supervisor.sh --address 192.168.x.x --cert server.crt --key server.key
#     터널: bash start-drive-supervisor.sh            (127.0.0.1:8445, PC에서 ssh -L 8445:127.0.0.1:8445)
set -eo pipefail
drive_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
app="${SMARTCART_APP:-$HOME/SmartCart-D5-v2/smartcart}"
if [[ ! -d "$drive_dir/smartcart_motor" || ! -r "$drive_dir/web/index.html" ]]; then
  printf '%s\n' '감독기 파일(smartcart_motor, web)이 이 폴더에 없습니다.' >&2
  exit 2
fi
if [[ ! -d "$app/smartcart_tracking" || ! -r "$app/local-config/run.json" ]]; then
  printf '%s\n' "기존 설치본 또는 실행 설정이 없습니다: $app" >&2
  exit 2
fi
root="$(cd -- "$app/.." && pwd -P)"
if [[ ! -r /opt/ros/humble/setup.bash || ! -r "$root/workspace/install/setup.bash" ]]; then
  printf '%s\n' 'ROS 환경 또는 통합 workspace가 없습니다.' >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
if [[ -r "$HOME/smartcart_ws/install/setup.bash" ]]; then source "$HOME/smartcart_ws/install/setup.bash"; fi
source "$root/workspace/install/setup.bash"
export ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
export PYTHONPATH="$drive_dir:$app${PYTHONPATH:+:$PYTHONPATH}"
cd -- "$app"
printf '%s\n' '구동 감독기 · 가상 출력 · 비상정지 피드백 없음 → 구동 잠금 · 물리 비상정지 별도'
exec python3 -m smartcart_motor.drive_app --run-config "$app/local-config/run.json" "$@"
