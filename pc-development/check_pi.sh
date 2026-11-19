#!/usr/bin/env bash
# 장치와 모터를 제어하지 않고 준비 상태만 조회한다.
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
if [[ -r /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
if [[ -r "$task_root/.runtime/root.txt" && -r "$task_root/.runtime/install/setup.bash" && ! -e "$task_root/.runtime/build-in-progress" ]] && [[ "$(cat "$task_root/.runtime/root.txt")" == "$task_root" ]]; then
  source "$task_root/.runtime/install/setup.bash"
fi
exec python3 "$task_root/diagnose_pi.py" "$@"
