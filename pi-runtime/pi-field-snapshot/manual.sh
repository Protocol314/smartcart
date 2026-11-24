#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"

source "$ROOT/cert.sh" "$@"
echo "실제 수동 조종: https://$IP:8081/"
exec /usr/bin/python3 -u "$ROOT/SmartCart_ManualDrive_180/smartcart/drive.py" --backend gpio --host 0.0.0.0 --port 8081 --sensor-url http://127.0.0.1:8765 --cert "$CERT/server.crt" --key "$CERT/server.key"
