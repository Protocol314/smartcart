#!/usr/bin/env python3
"""측정·제어 값과 분리된 X 좌표 표시 시험 설치기를 만든다."""
import ast
import base64
import hashlib
import json
from pathlib import Path
import zlib

root = Path(__file__).resolve().parents[2]
previous = root / 'build/releases/smartcart-manual-lidar-frontback-20261117-install.py'
source = previous.read_text(encoding='utf-8')
values = {}
for node in ast.parse(source).body:
    if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) and node.targets[0].id in ('PAYLOAD', 'DIGEST'):
        values[node.targets[0].id] = ast.literal_eval(node.value)
old_files = json.loads(zlib.decompress(base64.b64decode(values['PAYLOAD'])))
web = root / 'SmartCart_ManualDrive_180/smartcart/web'
files = {}
for name, item in old_files.items():
    content = (web / name).read_text(encoding='utf-8')
    files[name] = {'old': [item['new']], 'new': hashlib.sha256(content.encode()).hexdigest(), 'content': content}
app = (web / 'js/app.js').read_text(encoding='utf-8')
prior = app
for now, old in [
    ('C.displayX(previewPosition.x).toFixed(2)', 'previewPosition.x.toFixed(2)'),
    ('C.displayX(um.position.x).toFixed(2)', 'um.position.x.toFixed(2)'),
    ('ESP 방위 +는 BU04 오른쪽 · X는 부호 및 원점 변경 표시값입니다.', 'ESP 방위 +는 BU04 오른쪽 · X/Y는 ROS 전방/좌측입니다.'),
]:
    if prior.count(now) != 1:
        raise ValueError('화면 기준 파일이 예상과 다릅니다')
    prior = prior.replace(now, old, 1)
files['js/app.js'] = {'old': [hashlib.sha256(prior.encode()).hexdigest()], 'new': hashlib.sha256(app.encode()).hexdigest(), 'content': app}
raw = json.dumps(files, ensure_ascii=False).encode()
payload = base64.b64encode(zlib.compress(raw, 9)).decode('ascii')
source = source.replace(values['PAYLOAD'], payload).replace(values['DIGEST'], hashlib.sha256(raw).hexdigest())
source = source.replace("{'js/core.js','js/radar.js','index.html'}", "{'js/core.js','js/radar.js','js/app.js','index.html'}")
out = root / 'build/releases/smartcart-manual-coordinate-view-20261118-install.py'
out.write_text(source, encoding='utf-8', newline='\n')
print(out)
print('SHA256:', hashlib.sha256(out.read_bytes()).hexdigest())
