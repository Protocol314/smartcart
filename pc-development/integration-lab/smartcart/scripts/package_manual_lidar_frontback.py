#!/usr/bin/env python3
"""LiDAR 표시만 앞뒤 반전하는 시험 설치기를 만든다."""
import ast
import base64
import hashlib
import json
from pathlib import Path
import zlib

root = Path(__file__).resolve().parents[2]
base = root / 'build/releases/smartcart-manual-mirror-20261117-install.py'
frontback = root / 'build/releases/smartcart-manual-frontback-20261117-install.py'
def load(path):
    text = path.read_text(encoding='utf-8')
    values = {}
    for node in ast.parse(text).body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) and node.targets[0].id in ('PAYLOAD', 'DIGEST'):
            values[node.targets[0].id] = ast.literal_eval(node.value)
    return text, values, json.loads(zlib.decompress(base64.b64decode(values['PAYLOAD'])))
source, values, first = load(base)
_, _, second = load(frontback)
web = root / 'SmartCart_ManualDrive_180/smartcart/web'
files = {}
for name in first:
    content = (web / name).read_text(encoding='utf-8')
    files[name] = {'old': [first[name]['new'], second[name]['new']], 'new': hashlib.sha256(content.encode()).hexdigest(), 'content': content}
raw = json.dumps(files, ensure_ascii=False).encode()
payload = base64.b64encode(zlib.compress(raw, 9)).decode('ascii')
source = source.replace(values['PAYLOAD'], payload).replace(values['DIGEST'], hashlib.sha256(raw).hexdigest())
source = source.replace("actual not in (item['old'], item['new'])", "actual not in (*item['old'], item['new'])")
out = root / 'build/releases/smartcart-manual-lidar-frontback-20261117-install.py'
out.write_text(source, encoding='utf-8', newline='\n')
print(out)
print('SHA256:', hashlib.sha256(out.read_bytes()).hexdigest())
