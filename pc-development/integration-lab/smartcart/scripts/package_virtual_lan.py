"""기존 가상 시험 묶음에 LAN 인증 실행기를 추가한다."""
from pathlib import Path
import hashlib
import zipfile

root = Path(__file__).resolve().parents[2]
old = root / 'build/releases/smartcart-virtual-live-20261118.zip'
new = root / 'build/releases/smartcart-virtual-lan-4digit-20261118.zip'
expected = '0a5a6f04be064f3a76437a8c07726574123140f748b0ef181260ace4002fe85d'
if hashlib.sha256(old.read_bytes()).hexdigest() != expected:
    raise SystemExit('기존 시험 묶음의 해시가 달라 중단합니다')
with zipfile.ZipFile(old) as source, zipfile.ZipFile(new, 'w', zipfile.ZIP_DEFLATED) as target:
    for item in source.infolist():
        target.writestr(item, source.read(item))
    target.write(root / 'smartcart/scripts/virtual_lan_server.py', 'virtual_lan_server.py')
print('SHA256:', hashlib.sha256(new.read_bytes()).hexdigest())
print('파일:', new)
