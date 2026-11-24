#!/usr/bin/env python3
"""Pi 사설 IP에만 유효한 7일 시험용 레이더 HTTPS 인증서를 생성한다."""
import argparse
import hashlib
import ipaddress
import os
from pathlib import Path
import ssl
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--address', required=True)
    parser.add_argument('--directory', required=True)
    args = parser.parse_args()
    address = ipaddress.ip_address(args.address)
    if address.version != 4 or not address.is_private or address.is_loopback or address.is_unspecified:
        raise SystemExit('Pi의 사설 IPv4 주소가 필요합니다')
    address = str(address)
    directory = Path(args.directory).expanduser()
    directory.mkdir(mode=0o700, parents=True, exist_ok=False)
    files = {n: directory / n for n in ('ca.key', 'ca.crt', 'server.key', 'server.csr', 'server.crt', 'server.ext')}
    old = os.umask(0o077)
    try:
        files['server.ext'].write_text('basicConstraints=critical,CA:FALSE\nkeyUsage=critical,digitalSignature,keyEncipherment\nextendedKeyUsage=serverAuth\nsubjectAltName=IP:' + address + '\n', encoding='ascii')
        commands = [
            ['openssl', 'req', '-x509', '-newkey', 'rsa:3072', '-nodes', '-sha256', '-days', '7', '-subj', '/CN=SmartCart Radar Trial CA', '-addext', 'basicConstraints=critical,CA:TRUE', '-addext', 'keyUsage=critical,keyCertSign,cRLSign', '-addext', 'nameConstraints=critical,permitted;IP:' + address + '/255.255.255.255', '-keyout', str(files['ca.key']), '-out', str(files['ca.crt'])],
            ['openssl', 'req', '-new', '-newkey', 'rsa:2048', '-nodes', '-sha256', '-subj', '/CN=' + address, '-keyout', str(files['server.key']), '-out', str(files['server.csr'])],
            ['openssl', 'x509', '-req', '-in', str(files['server.csr']), '-CA', str(files['ca.crt']), '-CAkey', str(files['ca.key']), '-CAcreateserial', '-out', str(files['server.crt']), '-days', '7', '-sha256', '-extfile', str(files['server.ext'])],
        ]
        for command in commands:
            subprocess.run(command, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60)
    finally:
        os.umask(old)
    der = ssl.PEM_cert_to_DER_cert(files['ca.crt'].read_text(encoding='ascii'))
    print('CA SHA256:', hashlib.sha256(der).hexdigest())
    print('공개 인증서:', files['ca.crt'])
    print('개인 키는 Pi에만 보관하십시오. 7일 후 재생성이 필요합니다.')


if __name__ == '__main__':
    main()
