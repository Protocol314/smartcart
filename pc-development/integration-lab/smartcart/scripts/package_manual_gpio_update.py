#!/usr/bin/env python3
"""현장 파일의 기준 해시와 모의 시험을 확인하는 단일 설치기를 만든다."""
import base64
import hashlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
archive = ROOT / "build/releases/smartcart-manual-pivot-connection-test-20261117.zip"
output = ROOT / "build/releases/smartcart-manual-pivot-gpio-update-20261117.py"
payload = base64.b64encode(archive.read_bytes()).decode("ascii")
template = r'''#!/usr/bin/env python3
import base64, hashlib, io, os, shutil, subprocess, sys, tempfile, zipfile
from pathlib import Path

ROOT = Path.home() / "SmartCart_ManualDrive_180" / "smartcart"
EXPECTED = {
    "drive.py": "4223ee540c6839f9359a88a6b720b86fae6d2bc2a90073fec19fea1497391be7",
    "web/js/manual_controls.js": "938a3efd594f3079e8ef866272853778571a02f97687fcdd2858e2130bd0c00b",
    "smartcart_motor/manual_remote.py": "06a0a6f9a1da04d37dd895f65135850262fc1752fecb7303f85092e244f1c276",
}
PAYLOAD = "__PAYLOAD__"
DIGEST = "__DIGEST__"
FILES = ("drive.py", "web/js/manual_controls.js")

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    if sys.argv[1:] not in ([], ["--apply"]):
        raise RuntimeError("사용법: python3 설치기.py [--apply]")
    raw = base64.b64decode(PAYLOAD, validate=True)
    if hashlib.sha256(raw).hexdigest() != DIGEST:
        raise RuntimeError("설치 데이터 검증 실패")
    with zipfile.ZipFile(io.BytesIO(raw)) as archive:
        names = set(archive.namelist())
        required = {"drive.py", "web/js/manual_controls.js", "tests/test_manual_protocol.py", "tests/test_manual_controls.cjs", "tests/test_manual_remote.py"}
        if names != required:
            raise RuntimeError("설치 데이터 파일 목록이 다릅니다")
        data = {name: archive.read(name) for name in names}
    for name, expected in EXPECTED.items():
        path = ROOT / name
        if path.is_symlink() or not path.is_file() or digest(path) != expected:
            raise RuntimeError("기준과 다른 파일: " + name)
    print("기존 파일 3개 및 설치 데이터 확인 완료", flush=True)
    if not sys.argv[1:]:
        print("검사만 완료했습니다. --apply로 가상 시험 후 적용합니다.")
        return
    sys.path.insert(0, str(ROOT))
    from smartcart_motor.gpio_backend import MotorLock
    with MotorLock():
        with tempfile.TemporaryDirectory(prefix="smartcart-manual-check-", dir=Path.home()) as directory:
            trial = Path(directory) / "smartcart"
            shutil.copytree(ROOT, trial, symlinks=True)
            for name, content in data.items():
                path = trial / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(content)
            for pattern in ("test_manual_protocol.py", "test_manual_remote.py"):
                print("가상 시험:", pattern, flush=True)
                subprocess.run([sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", pattern], cwd=trial, check=True)
        for name, expected in EXPECTED.items():
            if (ROOT / name).is_symlink() or digest(ROOT / name) != expected:
                raise RuntimeError("시험 중 원본 파일이 변경되었습니다: " + name)
        backup = Path(tempfile.mkdtemp(prefix="smartcart-manual-backup-", dir=Path.home()))
        for name in FILES:
            target = backup / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / name, target)
        replaced = []
        try:
            for name in FILES:
                target = ROOT / name
                fd, temporary = tempfile.mkstemp(prefix=".smartcart-update-", dir=target.parent)
                try:
                    with os.fdopen(fd, "wb") as stream:
                        stream.write(data[name])
                    os.chmod(temporary, target.stat().st_mode & 0o777)
                    os.replace(temporary, target)
                    replaced.append(name)
                finally:
                    if os.path.exists(temporary): os.unlink(temporary)
        except Exception:
            for name in replaced: shutil.copy2(backup / name, ROOT / name)
            raise
        print("적용 완료. 백업:", backup, flush=True)
        print("서버·GPIO·모터는 시작하지 않았습니다.", flush=True)

if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, ValueError, subprocess.CalledProcessError) as exc:
        print("업데이트 중단:", exc, file=sys.stderr)
        sys.exit(1)
'''
content = template.replace("__PAYLOAD__", payload).replace("__DIGEST__", hashlib.sha256(archive.read_bytes()).hexdigest())
output.write_text(content, encoding="utf-8", newline="\n")
print(output)
print("SHA256:", hashlib.sha256(output.read_bytes()).hexdigest())
