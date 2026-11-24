#!/usr/bin/env bash
set -eo pipefail
task_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
task_project_dir="$(cd -- "$task_script_dir/.." && pwd -P)"
cd -- "$task_project_dir"
exec python3 "$task_project_dir/drive.py" --backend gpio --host 0.0.0.0 --port 8080 "$@"
