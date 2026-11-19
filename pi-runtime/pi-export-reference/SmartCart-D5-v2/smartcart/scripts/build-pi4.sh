#!/usr/bin/env bash
# 검증 설치본에서 명시적으로 실행할 때만 새 워크스페이스를 빌드한다.
set -euo pipefail
workspace_override=""
if [[ $# -eq 2 && "$1" == --workspace && "$2" == /* && "$2" != / ]]; then
  workspace_override="$2"
elif [[ $# -ne 0 ]]; then
  printf '%s\n' '인수 없이 실행하거나 --workspace 다음에 새 절대 경로를 지정하십시오.' >&2
  exit 2
fi
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
package_root="$(cd -- "$script_dir/../.." && pwd -P)"
workspace="${workspace_override:-$package_root/workspace}"
if [[ -e "$workspace" || -L "$workspace" ]]; then
  printf '%s\n' 'workspace가 이미 있습니다. 기존 작업공간은 덮어쓰지 않습니다.' >&2
  exit 2
fi
python3 - "$package_root" "$workspace" <<'PY'
import importlib.util
import hashlib
from pathlib import Path
import sys

root = Path(sys.argv[1])
workspace = Path(sys.argv[2])
if (not workspace.is_absolute() or workspace == Path("/") or workspace.exists() or workspace.is_symlink()
        or any(ord(char) < 32 for char in str(workspace))
        or not workspace.parent.is_dir() or workspace.parent.resolve() != workspace.parent):
    raise ValueError("워크스페이스는 링크가 아닌 기존 부모 아래 새 절대 경로여야 합니다")
spec = importlib.util.spec_from_file_location("pi4_build_validator", root / "smartcart/deploy/install-pi4.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.check_target_environment()
metadata = module.validate_metadata(module.parse_json((root / module.TARGET_FILE).read_bytes()))
expected = {**metadata["source_sha256"], module.NATIVE: metadata["native"]["sha256"]}
for name, digest in expected.items():
    path = root / name
    if path.is_symlink() or not path.is_file() or path.stat().st_size > module.MAX_FILE:
        raise ValueError("원본 파서·ARM64 라이브러리의 파일 형식이 잘못되었습니다")
    data = path.read_bytes()
    if hashlib.sha256(data).hexdigest() != digest:
        raise ValueError("원본 파서·ARM64 라이브러리가 배포 검증 이후 바뀌었습니다")
    if name == module.NATIVE:
        module.validate_elf(data)
print("Ubuntu 22.04 aarch64·배포 대상·ARM64 ELF 확인 완료")
PY
if [[ ! -r /opt/ros/humble/setup.bash ]] || ! command -v colcon >/dev/null; then
  printf '%s\n' 'ROS 2 Humble와 colcon이 필요합니다. 자동 apt 설치는 수행하지 않습니다.' >&2
  exit 2
fi
# setup.bash가 선택적으로 읽는 미설정 환경변수를 허용한다.
set +u
source /opt/ros/humble/setup.bash
set -u
if [[ "${ROS_DISTRO:-}" != humble ]]; then
  printf '%s\n' 'ROS 2 Humble 환경을 확인하지 못했습니다.' >&2
  exit 2
fi
export ROS_LOCALHOST_ONLY=1
mkdir -m 700 -- "$workspace"
colcon --log-base "$workspace/log" build \
  --base-paths "$package_root/ros2/smartcart_uwb_interfaces" "$package_root/smartcart" \
  --build-base "$workspace/build" --install-base "$workspace/install" \
  --packages-select smartcart_uwb_interfaces smartcart_sensing --executor sequential
printf '%s\n' '빌드 완료. 드라이버·장치·서비스는 시작하지 않았습니다.'
printf '다음 명령으로 사용자가 명시적으로 환경을 적용하십시오: source %q\n' "$workspace/install/setup.bash"
