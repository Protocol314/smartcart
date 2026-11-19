"""실행 설정을 제한된 JSON으로 읽으며 임의 명령이나 환경 코드를 허용하지 않는다."""
from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import re

SCHEMA = "smartcart.run_config.v1"
ROOT = Path(__file__).resolve().parents[1]
KEYS = {"schema", "mounting_config", "field_config", "uwb_config", "tracking_config",
        "http_port", "lidar_mode", "lidar_port", "lidar_frame_id"}
CONFIG_FILES = {"mounting_config": ("mounting.json", "dual_uwb.d5.example.json"),
                "field_config": ("field.json", "field_acceptance.example.json"),
                "uwb_config": ("uwb.json", "uwb_estimation.example.json"),
                "tracking_config": ("tracking.json", "target_tracking.example.json")}
PORT = re.compile(r"/dev/(?:serial/by-id/[A-Za-z0-9_.:+-]{1,200}|tty(?:USB|ACM)[0-9]{1,3})\Z")
FRAME = re.compile(r"[A-Za-z][A-Za-z0-9_]{0,63}\Z")


def asset_root():
    """현재 디렉터리 대신 소스 또는 설치된 패키지의 자료만 사용한다."""
    import sys
    candidates = (ROOT, ROOT / "share/smartcart_sensing", Path(sys.prefix) / "share/smartcart_sensing")
    for directory in candidates:
        if (directory / "config/dual_uwb.d5.example.json").is_file():
            return directory
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("smartcart_sensing"))
    except (ImportError, LookupError):
        return ROOT


def local_path(value, *, base=None):
    """로컬 경로만 허용하고 설정의 상대 경로는 설정 파일 위치에서 해석한다."""
    value = os.fspath(value)
    if (not isinstance(value, str) or not value or len(value) > 4096
            or any(ord(char) < 32 for char in value) or value.startswith(("\\\\", "//"))
            or "://" in value):
        raise ValueError("길이가 제한된 로컬 파일 경로가 필요합니다")
    path = Path(value)
    return ((base / path) if base is not None and not path.is_absolute() else path).resolve()


def _pairs(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError("실행 설정에 중복된 JSON 키가 있습니다")
        value[key] = item
    return value


@dataclass(frozen=True)
class RunConfig:
    path: Path
    mounting_config: Path
    field_config: Path
    uwb_config: Path
    tracking_config: Path
    http_port: int
    lidar_mode: str
    lidar_port: str | None
    lidar_frame_id: str


def load_config(path):
    path = local_path(path)
    if not path.is_file() or path.stat().st_size > 65536:
        raise ValueError("실행 설정은 64 KiB 이하의 일반 JSON 파일이어야 합니다")
    with path.open("rb") as stream:
        raw = stream.read(65537)
    if len(raw) > 65536:
        raise ValueError("실행 설정의 크기 상한을 넘었습니다")
    value = json.loads(raw.decode("utf-8"), object_pairs_hook=_pairs,
                       parse_constant=lambda _: (_ for _ in ()).throw(ValueError("유한 JSON 값만 허용합니다")))
    if type(value) is not dict or set(value) != KEYS or value["schema"] != SCHEMA:
        raise ValueError("실행 설정의 스키마·필수 키·허용 키를 확인하십시오")
    if type(value["http_port"]) is not int or not 1024 <= value["http_port"] <= 65535:
        raise ValueError("HTTP 포트는 1024~65535 정수여야 합니다")
    if value["lidar_mode"] not in ("managed", "external"):
        raise ValueError("LiDAR 모드는 managed 또는 external이어야 합니다")
    if value["lidar_port"] is not None and (type(value["lidar_port"]) is not str or not PORT.fullmatch(value["lidar_port"])):
        raise ValueError("LiDAR 포트는 확인한 /dev/serial/by-id 또는 ttyUSB/ttyACM 경로여야 합니다")
    if type(value["lidar_frame_id"]) is not str or not FRAME.fullmatch(value["lidar_frame_id"]):
        raise ValueError("LiDAR 프레임은 영문으로 시작하는 64자 이하 이름이어야 합니다")
    paths = {}
    for name in CONFIG_FILES:
        if type(value[name]) is not str:
            raise ValueError("센서 설정 경로는 문자열이어야 합니다")
        paths[name] = local_path(value[name], base=path.parent)
    return RunConfig(path=path, **paths, **{name: value[name] for name in
                     ("http_port", "lidar_mode", "lidar_port", "lidar_frame_id")})


def default_config():
    """설정 파일 없이도 합성 재생이 가능하며 실측 기본값은 미배정 상태다."""
    assets = asset_root()
    paths = {name: assets / "config" / source for name, (_, source) in CONFIG_FILES.items()}
    return RunConfig(path=assets / "config/run.example.json", **paths, http_port=8765,
                     lidar_mode="managed", lidar_port=None, lidar_frame_id="lidar_scan")


def initialize(directory):
    """새 디렉터리에만 미검증 기본 설정을 만들고 기존 값은 덮어쓰지 않는다."""
    directory = local_path(directory)
    payloads = {target: (asset_root() / "config" / source).read_bytes()
                for target, source in CONFIG_FILES.values()}
    run = {"schema": SCHEMA, **{name: target for name, (target, _) in CONFIG_FILES.items()},
           "http_port": 8765, "lidar_mode": "managed", "lidar_port": None,
           "lidar_frame_id": "lidar_scan"}
    payloads["run.json"] = (json.dumps(run, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
    directory.mkdir(mode=0o700)
    for name, content in payloads.items():
        descriptor = os.open(directory / name, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(content)
    return directory / "run.json"
