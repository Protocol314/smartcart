#!/usr/bin/env python3
"""기존 Pi 묶음과 순차 적용 화면 패치를 기준으로 단일 구성 설치기를 만든다."""
import ast
import base64
import hashlib
import importlib.util
import json
from pathlib import Path
import zipfile
import zlib

ROOT = Path(__file__).resolve().parents[2]
RELEASE = ROOT / "build" / "releases"
NAME = "smartcart-single-left-pi-identity-20261117-install.py"
FILES = ("smartcart_uwb/serial_reader.py", "smartcart_uwb/ros_adapter.py",
         "smartcart_uwb/single_estimator.py", "smartcart_field/identity.py",
         "smartcart_runner/plan.py", "smartcart_runner/app.py",
         "smartcart_tracking/runtime.py", "smartcart_tracking/tracker.py",
         "smartcart_tracking/ros_runtime.py", "smartcart_sensing/app.py",
         "smartcart_sensing/dual_uwb.py", "web/js/tracking.js", "web/js/dual_uwb.js",
         "web/js/app.js", "scripts/start-single-left.sh")
PATCHES = (("smartcart-dual-uwb-ui-20261027-install.py", "172dbdaf5c82f383b758ef35d7566c0cb4f07ac3e84601b6292452bd37acdbde"),
           ("smartcart-field-preview-20261103-install.py", "74f54d152aa5f0fd029f08e93dbbcd20198e1e6fe726a4ed03dd0894a2dcf2ee"),
           ("smartcart-field-smoothing-20261103-install.py", "f398f06f89429bfea24737d4bc00453270be4b3839512ea3c9e13d37bd70285f"))
# 이전 설치기의 원본과 세 패치가 정상일 때 확인한 기준 파일 해시다.
BASELINE = {"smartcart_uwb/serial_reader.py": "41ed4f7a8d5d2fce77208b3019319361f03a416335b9d25d728bebdc647dfc37",
 "smartcart_uwb/ros_adapter.py": "0f64f822ed93fd1fe1dc5ca611a420444e98c53dd2c79f466640c02081097805",
 "smartcart_uwb/single_estimator.py": None,
 "smartcart_field/identity.py": "605893ea4562f50a9ea46c518cc1472253f576af308170b2d69d061fea1f5f75",
 "smartcart_runner/plan.py": "ae2e818b4f3c77ae280d8d525db925d6bd0fea652ef5961333da7305c1c6a561",
 "smartcart_runner/app.py": "148362ea25ebc7cc631ce660e51a3efdaa8ae1ee45c7a1624e3adfc3f0d39987",
 "smartcart_tracking/runtime.py": "4a51c4527b58f983a5ab99db48e4fd45e5eaa72c6d63f48195a0e200013b0de9",
 "smartcart_tracking/tracker.py": "8c1d3d31d083c00e3cb7344727d44ce947b7bde2e816e2c12935a21ba70fa219",
 "smartcart_tracking/ros_runtime.py": "50486f508325a0e3802badafe1ec248197e4868c9cf232c26b1a1cb7dce22f49",
 "smartcart_sensing/app.py": "b598210e51c0dff64e08a273126cd65430739fa231ea3c285cfa404237eee27e",
 "smartcart_sensing/dual_uwb.py": "c05f5e4494ab9de9c9edb37e1381656b0f65aceed4e920375f3070c4fbb408e2",
 "web/js/tracking.js": "c09c57cb30c7109bdb7e2a35a3e147bba92a28cf4250a0ed5d43682e07ddc921",
 "web/js/dual_uwb.js": "f1d02afcc541d79d6b75e5a0be893e0a0e20b1fceca0cfd0dd9b305657cf8b43",
 "web/js/app.js": "467875f96e903474ec89c43fdbd91066b2df5e8f03d3757a123a461566516ca2",
 "scripts/start-single-left.sh": None}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def build():
    if set(BASELINE) != set(FILES):
        raise ValueError("기준 파일 목록이 다릅니다")
    payload = {}
    for path in FILES:
        data = (ROOT / "smartcart" / path).read_bytes()
        payload[path] = {"old_sha256": BASELINE[path],
                         "new_sha256": digest(data), "text": data.decode("utf-8")}
    wire = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    spec = importlib.util.spec_from_file_location("update_template", ROOT / "smartcart/scripts/build-dual-uwb-update.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    source = module.TEMPLATE.replace("좌우 BU04 실측 표시 업데이트", "왼쪽 BU04 단독 센싱 업데이트")
    source = source.replace("smartcart-ui-backup-", "smartcart-single-left-backup-")
    source = source.replace("__PAYLOAD__", base64.b64encode(zlib.compress(wire, 9)).decode("ascii"))
    source = source.replace("__DIGEST__", digest(wire)).replace("__PATHS__", repr(FILES))
    source = source.replace("시작: bash ~/SmartCart-D5-v2/smartcart/scripts/start-live.sh", "시작: bash ~/SmartCart-D5-v2/smartcart/scripts/start-single-left.sh")
    target = RELEASE / NAME
    data = source.encode("utf-8")
    if target.exists() and target.read_bytes() != data:
        raise FileExistsError("기존 설치기를 덮어쓰지 않습니다")
    target.write_bytes(data)
    target.with_suffix(".py.sha256").write_text(digest(data) + "  " + NAME + "\n", encoding="ascii")
    print(NAME + " SHA256=" + digest(data))


if __name__ == "__main__":
    build()
