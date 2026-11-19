#!/usr/bin/env python3
"""수동 화면 앞뒤 반전 시험 설치기를 생성한다."""
import ast
import base64
import hashlib
import json
from pathlib import Path
import zlib

root = Path(__file__).resolve().parents[2]
old = root / 'build/releases/smartcart-manual-mirror-20261117-install.py'
source = old.read_text(encoding='utf-8')
values = {}
for node in ast.parse(source).body:
    if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
        if node.targets[0].id in ('PAYLOAD', 'DIGEST'):
            values[node.targets[0].id] = ast.literal_eval(node.value)
data = json.loads(zlib.decompress(base64.b64decode(values['PAYLOAD'])))
web = root / 'SmartCart_ManualDrive_180/smartcart/web'
files = {}
for name, item in data.items():
    content = (web / name).read_text(encoding='utf-8')
    files[name] = {'old': item['new'], 'new': hashlib.sha256(content.encode()).hexdigest(), 'content': content}
raw = json.dumps(files, ensure_ascii=False).encode()
new_payload = base64.b64encode(zlib.compress(raw, 9)).decode('ascii')
output = source.replace(values['PAYLOAD'], new_payload).replace(values['DIGEST'], hashlib.sha256(raw).hexdigest())
path = root / 'build/releases/smartcart-manual-frontback-20261117-install.py'
path.write_text(output, encoding='utf-8', newline='\n')
print(path)
print('SHA256:', hashlib.sha256(path.read_bytes()).hexdigest())
