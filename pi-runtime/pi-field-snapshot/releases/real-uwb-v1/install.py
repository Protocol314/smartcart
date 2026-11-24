#!/usr/bin/env python3
"""Install a separate release; back up launchers and never start GPIO automatically."""
import ast
from datetime import datetime
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import socket
import sys

LAUNCHER='''#!/usr/bin/env bash
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
'''

def main():
    if sys.platform!='linux':
        raise SystemExit('Run this installer on the Pi SSH terminal, not Windows.')
    root=Path.home()/'smartcart'
    package=Path(__file__).resolve().parent
    release=root/'releases/real-uwb-v1'
    if root.is_symlink() or (root/'releases').is_symlink() or (root/'archive').is_symlink():
        raise SystemExit('Refusing symlink installation roots')
    for port in (8081,18084):
        with socket.socket() as sock:
            try:sock.bind(('0.0.0.0',port))
            except OSError:raise SystemExit(f'{port} server still running. Stop its own terminal with Ctrl+C first.')
    for path in (root/'SmartCart_ManualDrive_180/smartcart/drive.py',root/'SmartCart_ManualDrive_180/smartcart/smartcart_motor/manual_remote.py',root/'smartcart-virtual-4digit.DlQ3Pf7U/virtual_lan_server.py'):
        if not path.is_file():raise SystemExit('Missing dependency: '+str(path))
        ast.parse(path.read_text(encoding='utf-8'))
    if release.exists() or release.is_symlink():
        raise SystemExit('Release already exists; refusing to overwrite: '+str(release))
    expected=json.loads((package/'SHA256.json').read_text())
    actual={p.relative_to(package).as_posix() for p in package.rglob('*')
            if p.is_file() and p.name!='SHA256.json' and '__pycache__' not in p.parts and p.suffix!='.pyc'}
    if not expected or set(expected)!=actual:
        raise SystemExit('Package file list does not match its manifest')
    for relative,digest in expected.items():
        path=package/relative
        if not path.resolve().is_relative_to(package) or path.is_symlink() or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise SystemExit('Package verification failed: '+relative)
        if path.suffix=='.py':ast.parse(path.read_text(encoding='utf-8'))
    for name in ('real.sh','virtual.sh'):
        target=root/name
        if target.is_symlink() or (target.exists() and not target.is_file()):
            raise SystemExit('Refusing non-regular launcher: '+name)
        if (root/(name+'.real-v1-new')).exists() or (root/(name+'.real-v1-new')).is_symlink():
            raise SystemExit('Previous installation staging file exists: '+name)
    # Importing the installed implementation does not open GPIO. Check the
    # interface before replacing either entry point.
    sys.path.insert(0,str(package))
    import server
    base=server.load_base()
    for name in ('ManualDrive','MockBackend','RPiBackend','make_app'):
        if not hasattr(base,name):raise SystemExit('Installed drive interface missing: '+name)
    if 'make_guard' not in server.load_login():
        raise SystemExit('Existing screen login is incompatible')
    backup=root/'archive'/('before-real-uwb-v1-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    backup.mkdir(parents=True)
    for name in ('real.sh','virtual.sh'):
        if (root/name).exists():shutil.copy2(root/name,backup/name)
    changed=[]
    staging=[]
    try:
        (backup/'install.json').write_text(json.dumps({'release':str(release),'version':'1.0.0','launchers':['real.sh','virtual.sh'],'previous_real_sh_existed':(backup/'real.sh').exists()},indent=2))
        release.parent.mkdir(exist_ok=True)
        shutil.copytree(package,release,ignore=shutil.ignore_patterns('__pycache__','*.pyc','.git'))
        for name in ('real.sh','virtual.sh'):
            temp=root/(name+'.real-v1-new')
            with temp.open('x',encoding='utf-8',newline='\n') as stream:stream.write(LAUNCHER)
            staging.append(temp)
            temp.chmod(0o700)
        for name in ('real.sh','virtual.sh'):
            (root/(name+'.real-v1-new')).replace(root/name)
            changed.append(name)
    except Exception:
        # Restore entry points and retain even failed new files in the archive.
        for name in changed:
            (root/name).replace(backup/('failed-'+name))
            if (backup/name).exists():shutil.copy2(backup/name,root/name)
        for temp in staging:
            if temp.exists():temp.replace(backup/temp.name)
        if release.exists():release.rename(backup/'failed-release')
        raise
    print('Installed separate real GPIO release:',release)
    print('Previous launcher backup:',backup)
    print('No server or motor was started. Sensor, network and existing motor implementation were not modified.')
    print('Run: bash ~/smartcart/real.sh 172.20.10.4 gpio')
    print('Test without physical output: bash ~/smartcart/real.sh 172.20.10.4 mock')

if __name__=='__main__':main()
