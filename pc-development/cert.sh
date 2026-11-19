#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"

IP="${1:?실행 명령 뒤에 현재 Pi IP를 입력하세요}"
CERT="${SMARTCART_CERT_DIR:-}"
if [[ -z "$CERT" ]]; then
  for d in "$ROOT/smartcart-virtual-lan-cert" "$ROOT/smartcart-radar-https-check-PyDCFCjn/cert"; do
    if [[ -r "$d/server.key" ]] &&
       openssl verify -CAfile "$d/ca.crt" -verify_ip "$IP" "$d/server.crt" >/dev/null 2>&1; then
      CERT="$d"
      break
    fi
  done
fi
[[ -n "$CERT" ]] || {
  echo "현재 IP($IP)에 맞는 인증서가 없습니다."
  exit 2
}
openssl verify -CAfile "$CERT/ca.crt" -verify_ip "$IP" "$CERT/server.crt"
