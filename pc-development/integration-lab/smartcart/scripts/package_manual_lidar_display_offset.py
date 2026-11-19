#!/usr/bin/env python3
"""LiDAR 화면 표시를 이동하는 시험 설치기를 생성한다."""
import ast
import base64
import hashlib
import json
from pathlib import Path
import zlib

root = Path(__file__).resolve().parents[2]
previous = root / 'build/releases/smartcart-manual-coordinate-view-20261118-install.py'
source = previous.read_text(encoding='utf-8')
values = {}
for node in ast.parse(source).body:
    if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) and node.targets[0].id in ('PAYLOAD', 'DIGEST'):
        values[node.targets[0].id] = ast.literal_eval(node.value)
prior = json.loads(zlib.decompress(base64.b64decode(values['PAYLOAD'])))
web = root / 'SmartCart_ManualDrive_180/smartcart/web'
files = {}
for name in ('js/radar.js', 'index.html'):
    content = (web / name).read_text(encoding='utf-8')
    files[name] = {'old': [prior[name]['new']], 'new': hashlib.sha256(content.encode()).hexdigest(), 'content': content}
raw = json.dumps(files, ensure_ascii=False).encode()
payload = base64.b64encode(zlib.compress(raw, 9)).decode('ascii')
source = source.replace(values['PAYLOAD'], payload).replace(values['DIGEST'], hashlib.sha256(raw).hexdigest())
source = source.replace("{'js/core.js','js/radar.js','js/app.js','index.html'}", "{'js/radar.js','index.html'}")
out = root / 'build/releases/smartcart-lidar-display-offset-20261118-install.py'
out.write_text(source, encoding='utf-8', newline='\n')
print(out)
print('SHA256:', hashlib.sha256(out.read_bytes()).hexdigest())
