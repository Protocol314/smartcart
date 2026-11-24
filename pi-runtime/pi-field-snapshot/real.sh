#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ADDRESS="${1:-}"
BACKEND="${2:-gpio}"
case "$BACKEND" in gpio|mock) ;; *) echo 'Use gpio or mock'; exit 2;; esac
if [[ -n "$ADDRESS" ]]; then
  exec /usr/bin/python3 -u "$ROOT/releases/real-uwb-v1/server.py" --address "$ADDRESS" --backend "$BACKEND"
else
  exec /usr/bin/python3 -u "$ROOT/releases/real-uwb-v1/server.py" --backend "$BACKEND"
fi
