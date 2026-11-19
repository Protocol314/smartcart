#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"

source "$ROOT/cert.sh" "$@"
echo "센서 감시 가상 조종 · GPIO 잠금: https://$IP:8081/"
exec /usr/bin/python3 -u "$ROOT/SmartCart_ManualDrive_180/smartcart/virtual_lan_server.py" --address "$IP" --port 8081 --cert "$CERT/server.crt" --key "$CERT/server.key"
