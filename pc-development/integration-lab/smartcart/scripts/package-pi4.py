#!/usr/bin/env python3
"""허용한 원본 소스·런타임과 명시한 ARM64 라이브러리만 새 Pi 4 ZIP으로 묶는다."""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import stat
import sys
import zipfile


def load_installer(repository):
    path = repository / "smartcart/deploy/install-pi4.py"
    spec = importlib.util.spec_from_file_location("pi4_release_installer", path)
    if spec is None or spec.loader is None:
        raise ValueError("Pi 4 설치 검증기를 찾을 수 없습니다")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _read_source(path, repository, limit):
    if not path.is_relative_to(repository):
        raise ValueError("저장소 밖 런타임 파일을 포함할 수 없습니다")
    for parent in (path, *path.parents):
        if parent.is_symlink():
            raise ValueError("심볼릭 링크 런타임 파일·디렉터리는 배포하지 않습니다")
        if parent == repository:
            break
    if not path.is_file() or path.stat().st_size > limit or not path.resolve().is_relative_to(repository):
        raise ValueError("일반 파일·크기·저장소 경계 검사를 통과하지 못했습니다: " + path.name)
    with path.open("rb") as source:
        data = source.read(limit + 1)
    if len(data) > limit:
        raise ValueError("읽는 도중 배포 파일이 크기 상한을 넘었습니다")
    return data


def _check_examples(files, installer):
    """현장 장치 식별값을 예제 파일명으로 덮어쓴 경우에도 배포를 거부한다."""
    mounting = installer.parse_json(files["smartcart/config/dual_uwb.d5.example.json"])
    sensors = mounting.get("sensors") if type(mounting) is dict else None
    if type(sensors) is not dict or sensors.keys() != {"uwb_left", "uwb_right"}:
        raise ValueError("좌우 UWB 기본 예제 구조를 확인할 수 없습니다")
    for sensor in sensors.values():
        if (type(sensor) is not dict or sensor.get("port") is not None
                or sensor.get("expected_device_identity") != "" or sensor.get("device_identity_verified") is not False
                or sensor.get("mounting_verified") is not False):
            raise ValueError("현장 포트·장치 식별값·검수 완료 설정은 Pi 예제로 배포하지 않습니다")
    lidar = mounting.get("lidar")
    if type(lidar) is not dict or lidar.get("mounting_verified") is not False:
        raise ValueError("LiDAR 장착 검수 완료 설정은 기본 예제로 배포하지 않습니다")
    field = installer.parse_json(files["smartcart/config/field_acceptance.example.json"])
    coordinate = {"frame_id": "base_link", "origin": "rear_axle_center", "x_axis": "forward",
                  "y_axis": "left", "z_axis": "up", "translation_unit": "m", "yaw_unit": "deg"}
    evidence = {"translation_verified": False, "yaw_verified": False, "height_verified": False,
                "level_verified": False, "reference_point": "unknown", "evidence_note": ""}
    expected_field = {"schema": "smartcart.field_acceptance_config.v1", "coordinate_contract": coordinate,
        "origin_verified": False, "axes_verified": False,
        "lidar_device": {"port": None, "expected_device_identity": "", "device_identity_verified": False},
        "expected_lidar_frame_id": "lidar_scan",
        "mounting_evidence": {role: evidence for role in ("uwb_left", "uwb_right", "lidar_center")},
        "range_model_verified": False, "tag_height_verified": False}
    limits = {"min_uwb_baseline_m": (.01, 2), "max_lidar_lateral_offset_m": (0, 1)}
    if (type(field) is not dict or field.keys() != expected_field.keys() | limits.keys()
            or not installer._exact({key: field[key] for key in expected_field}, expected_field)):
        raise ValueError("현장 LiDAR 식별값·실측 증빙·메모·확인 완료 표시는 기본 예제로 배포하지 않습니다")
    for name, (minimum, maximum) in limits.items():
        value = field[name]
        if type(value) not in (int, float) or not minimum <= value <= maximum or not math.isfinite(value):
            raise ValueError("현장 검수 예제의 기하 문턱이 잘못되었습니다")
    for name in ("uwb_estimation", "target_tracking"):
        value = installer.parse_json(files["smartcart/config/" + name + ".example.json"])
        if type(value) is not dict or value.get("assumptions_verified") is not False:
            raise ValueError("검수 완료 가정이 설정된 파일은 기본 예제로 배포하지 않습니다")
        if name == "uwb_estimation" and value.get("range_model") != "unknown":
            raise ValueError("미확정 거리 모델을 기본 예제에서 확정할 수 없습니다")
    for name in installer.SYNTHETIC_CONFIGS:
        value = installer.parse_json(files[name])
        if type(value) is not dict or value.get("assumptions_verified") is not False:
            raise ValueError("합성 설정은 실물 가정 검수 완료로 표시할 수 없습니다")
    for name in (installer.FIELD_SAMPLE, installer.TRACKING_SAMPLE):
        count = 0
        for line in files[name].splitlines():
            count += 1
            if count > 5000 or len(line) > 512 * 1024:
                raise ValueError("합성 예제 크기·레코드 제한 초과")
            record = json.loads(line, object_pairs_hook=installer._unique, parse_constant=installer._constant)
            if (type(record) is not dict or record.get("schema") != "smartcart.target_capture.v1"
                    or record.get("source") != "synthetic"):
                raise ValueError("명시한 합성 예제 외 실측 캡처는 배포하지 않습니다")
        if not count:
            raise ValueError("합성 예제가 비어 있습니다")
    _check_tracking_truth(files, installer)


def _check_tracking_truth(files, installer):
    """배포하는 합성 입력 범위에 속한 제한된 기대값만 허용한다."""
    truth = installer.parse_json(files[installer.TRACKING_TRUTH])
    keys = {"schema", "source", "frame_id", "tag_height_base_link_m", "notes", "checkpoints"}
    height = installer.parse_json(files["smartcart/config/uwb_estimation.synthetic.json"])["tag_height_m"]
    if (type(truth) is not dict or truth.keys() != keys
            or truth["schema"] != "smartcart.target_tracking_truth.v1" or truth["source"] != "synthetic"
            or truth["frame_id"] != "base_link" or type(truth["tag_height_base_link_m"]) not in (int, float)
            or truth["tag_height_base_link_m"] != height
            or type(truth["notes"]) is not str or len(truth["notes"]) > 1024
            or type(truth["checkpoints"]) is not list or not 1 <= len(truth["checkpoints"]) <= 256):
        raise ValueError("합성 추적 기대값의 출처·좌표·높이·구조가 잘못되었습니다")
    count = len(files[installer.TRACKING_SAMPLE].splitlines())
    fields = {"label", "event_index", "expected_valid", "same_track_as", "different_track_from",
              "candidate_min", "track_id_null", "expected_position_xy_m", "max_target_error_m"}
    labels, indices = set(), set()
    for checkpoint in truth["checkpoints"]:
        if (type(checkpoint) is not dict or checkpoint.keys() != fields
                or type(checkpoint["label"]) is not str
                or not re.fullmatch(r"[a-z][a-z0-9_]{0,79}", checkpoint["label"])
                or checkpoint["label"] in labels or type(checkpoint["event_index"]) is not int
                or not 0 <= checkpoint["event_index"] < count or checkpoint["event_index"] in indices
                or type(checkpoint["expected_valid"]) is not bool or type(checkpoint["track_id_null"]) is not bool):
            raise ValueError("합성 추적 기대값의 체크포인트가 잘못되었습니다")
        for key in ("same_track_as", "different_track_from"):
            if checkpoint[key] is not None and (type(checkpoint[key]) is not str or checkpoint[key] not in labels):
                raise ValueError("합성 체크포인트 참조가 앞선 기대값에 없습니다")
        minimum = checkpoint["candidate_min"]
        if minimum is not None and (type(minimum) is not int or not 0 <= minimum <= 128):
            raise ValueError("합성 체크포인트 후보 개수가 범위를 벗어납니다")
        position, error = checkpoint["expected_position_xy_m"], checkpoint["max_target_error_m"]
        if (position is None) != (error is None):
            raise ValueError("합성 위치와 오차 한도를 함께 지정하십시오")
        if position is not None and (type(position) is not list or len(position) != 2
                or any(type(value) not in (int, float) or not math.isfinite(value) or abs(value) > 10 for value in position)
                or type(error) not in (int, float) or not math.isfinite(error) or not 0 < error <= 2):
            raise ValueError("합성 기대 위치·오차가 허용 범위를 벗어납니다")
        labels.add(checkpoint["label"])
        indices.add(checkpoint["event_index"])


def build_release(repository, output, native_library, release_version):
    repository, output, native_library = Path(repository).resolve(strict=True), Path(output), Path(native_library)
    if type(release_version) is not str or not re.fullmatch(r"[0-9]{1,4}\.[0-9]{1,4}\.[0-9]{1,4}", release_version):
        raise ValueError("묶음 버전은 숫자 세 부분이어야 합니다")
    installer = load_installer(repository)
    if native_library.is_symlink() or not native_library.is_file() or native_library.stat().st_size > installer.MAX_FILE:
        raise ValueError("명시한 ARM64 라이브러리가 일반 파일이 아니거나 너무 큽니다")
    with native_library.open("rb") as source:
        native = source.read(installer.MAX_FILE + 1)
    installer.validate_elf(native)
    sources = {name: repository / name for name in installer.FIXED}
    sources["README-PI4.md"] = repository / "smartcart/README-PI4.md"
    for package in installer.PYTHON_PACKAGES:
        for path in (repository / "smartcart" / package).glob("*.py"):
            sources[path.relative_to(repository).as_posix()] = path
    for path in (repository / "smartcart/launch").glob("*.launch.py"):
        sources[path.relative_to(repository).as_posix()] = path
    for path in (repository / "smartcart/web").rglob("*"):
        name = path.relative_to(repository).as_posix()
        if installer.allowed_path(name):
            sources[name] = path
    files = {name: _read_source(path, repository, installer.MAX_FILE) for name, path in sorted(sources.items())}
    files[installer.NATIVE] = native
    _check_examples(files, installer)
    metadata = {"schema": "smartcart.pi4_package.v1", "release_version": release_version,
                "target": installer.TARGET, "safety": installer.SAFETY,
                "native": {"path": installer.NATIVE, "elf_class": 64, "endianness": "little",
                           "machine": 183, "abi_version": 1, "sha256": hashlib.sha256(native).hexdigest()},
                "source_sha256": {name: hashlib.sha256(files[name]).hexdigest() for name in sorted(installer.NATIVE_SOURCES)}}
    installer.validate_metadata(metadata)
    files[installer.TARGET_FILE] = (json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode("utf-8")
    if not installer.REQUIRED <= files.keys():
        raise ValueError("필수 Pi 4 런타임 파일이 없습니다: " + ", ".join(sorted(installer.REQUIRED - files.keys())))
    if len(files) + 1 > installer.MAX_FILES or sum(map(len, files.values())) > installer.MAX_EXPANDED - installer.MAX_MANIFEST:
        raise ValueError("Pi 4 묶음의 파일 수·크기 제한 초과")
    manifest = "".join(hashlib.sha256(data).hexdigest() + "  " + name + "\n" for name, data in sorted(files.items())).encode("ascii")
    if len(manifest) > installer.MAX_MANIFEST:
        raise ValueError("Pi 4 해시 목록 크기 제한 초과")
    files[installer.MANIFEST] = manifest
    output = output.absolute()
    checksum = output.with_name(output.name + ".sha256")
    bootstrap = output.with_name(output.stem + "-install.py")
    if output.suffix != ".zip" or any(path.exists() or path.is_symlink() for path in (output, checksum, bootstrap)):
        raise ValueError("새 .zip·체크섬·설치기 파일명을 지정하십시오. 기존 파일은 덮어쓰지 않습니다")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("xb") as destination:
        with zipfile.ZipFile(destination, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
            for name, data in sorted(files.items()):
                item = zipfile.ZipInfo(installer.PREFIX + name, date_time=(2026, 1, 1, 0, 0, 0))
                item.create_system = 3
                item.external_attr = (stat.S_IFREG | (0o700 if name.endswith(".sh") else 0o600)) << 16
                item.compress_type = zipfile.ZIP_DEFLATED
                archive.writestr(item, data)
    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    installer.validate_archive(output, digest)
    with bootstrap.open("xb") as destination:
        destination.write(files["smartcart/deploy/install-pi4.py"])
    with checksum.open("x", encoding="ascii", newline="\n") as destination:
        destination.write(digest + "  " + output.name + "\n")
        destination.write(hashlib.sha256(bootstrap.read_bytes()).hexdigest() + "  " + bootstrap.name + "\n")
    return digest, len(files) - 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--native-library", type=Path, required=True, help="이미 빌드·검증한 Linux aarch64 .so")
    parser.add_argument("--release-version", required=True)
    args = parser.parse_args()
    try:
        digest, count = build_release(Path(__file__).resolve().parents[2], args.output, args.native_library, args.release_version)
    except (ValueError, OSError, zipfile.BadZipFile, UnicodeError) as error:
        print("Pi 4 패키지 생성 실패: " + str(error), file=sys.stderr)
        return 2
    print(f"새 Pi 4 묶음: {args.output.resolve()} | 파일 {count}개 | SHA-256 {digest}")
    print("장치·서비스·드라이버·설치는 실행하지 않았습니다")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
