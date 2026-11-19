#!/usr/bin/env python3
"""수동 조종 PWM 출력 응답 감시의 기준 확인형 설치기를 만든다."""
import ast
import base64
import hashlib
import io
from pathlib import Path
import zipfile

root = Path(__file__).resolve().parents[2]
webapp = root / 'SmartCart_ManualDrive_180/smartcart'
previous = root / 'build/releases/smartcart-manual-pivot-gpio-update-20261117.py'
source = previous.read_text(encoding='utf-8')
values = {}
for node in ast.parse(source).body:
    if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) and node.targets[0].id in ('PAYLOAD', 'DIGEST'):
        values[node.targets[0].id] = ast.literal_eval(node.value)
names = ('smartcart_motor/manual_remote.py', 'web/js/manual_controls.js',
         'tests/test_manual_protocol.py', 'tests/test_manual_controls.cjs', 'tests/test_manual_remote.py')
buffer = io.BytesIO()
with zipfile.ZipFile(buffer, 'w', zipfile.ZIP_DEFLATED) as archive:
    for name in names:
        archive.writestr(name, (webapp / name).read_bytes())
raw = buffer.getvalue()
source = source.replace(values['PAYLOAD'], base64.b64encode(raw).decode('ascii')).replace(values['DIGEST'], hashlib.sha256(raw).hexdigest())
replacements = [
    ('4223ee540c6839f9359a88a6b720b86fae6d2bc2a90073fec19fea1497391be7', '3e4271c4f2bb7252ca41af63fd306abae1a12726dd001e23b8d35919491cccbe'),
    ('938a3efd594f3079e8ef866272853778571a02f97687fcdd2858e2130bd0c00b', '5ef6e5c0ebe118a86750f6b56a07403c69aa6611eccb587daf1f361687bb60f2'),
    ('FILES = ("drive.py", "web/js/manual_controls.js")', 'FILES = ("smartcart_motor/manual_remote.py", "web/js/manual_controls.js")'),
    ('{"drive.py", "web/js/manual_controls.js", "tests/test_manual_protocol.py", "tests/test_manual_controls.cjs", "tests/test_manual_remote.py"}',
     '{"smartcart_motor/manual_remote.py", "web/js/manual_controls.js", "tests/test_manual_protocol.py", "tests/test_manual_controls.cjs", "tests/test_manual_remote.py"}'),
]
for before, after in replacements:
    if source.count(before) != 1:
        raise ValueError('설치기 기준 확인 실패')
    source = source.replace(before, after, 1)
out = root / 'build/releases/smartcart-manual-output-guard-20261118-install.py'
out.write_text(source, encoding='utf-8', newline='\n')
print(out)
print('SHA256:', hashlib.sha256(out.read_bytes()).hexdigest())
