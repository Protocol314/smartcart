#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"

source "$ROOT/cert.sh" "$@"
exec /usr/bin/python3 -u "$ROOT/smartcart-virtual-4digit.DlQ3Pf7U/virtual_lan_server.py" --address "$IP" --port 18084 --cert "$CERT/server.crt" --key "$CERT/server.key"
