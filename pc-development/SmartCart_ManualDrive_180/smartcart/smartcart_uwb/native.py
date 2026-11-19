"""원본 순수 C BU04 파서의 명시적 ctypes 연결. 장치·파일·기록을 자동 생성하지 않는다."""
from __future__ import annotations

import ctypes
import os
from pathlib import Path
import sys
import threading
import weakref


ABI_VERSION = 1
MAX_FEED_BYTES = 65536
LIBRARY_FILENAME = ("smartcart_uwb_parser.dll" if sys.platform == "win32" else
                    "libsmartcart_uwb_parser.dylib" if sys.platform == "darwin" else
                    "libsmartcart_uwb_parser.so")
_ERROR_REASONS = {0: "none", 1: "argument", 2: "size", 3: "header",
                  4: "length", 5: "footer", 6: "checksum"}


class _Frame(ctypes.Structure):
    _fields_ = [("sequence", ctypes.c_uint32), ("tag_address", ctypes.c_uint32),
                ("angle_deg", ctypes.c_int32), ("distance_cm", ctypes.c_uint32),
                ("user_command", ctypes.c_uint32), ("first_path_power", ctypes.c_int32),
                ("rx_level", ctypes.c_int32), ("acceleration_x", ctypes.c_int32),
                ("acceleration_y", ctypes.c_int32), ("acceleration_z", ctypes.c_int32)]


def default_library_candidates() -> tuple[Path, ...]:
    """환경변수나 현재 작업 디렉터리를 탐색하지 않고 패키지·저장소 위치만 반환한다."""
    package = Path(__file__).resolve().parent
    return (package / "lib" / LIBRARY_FILENAME,
            package.parent.parent / "tests" / "build" / "dual_uwb" / LIBRARY_FILENAME)


def _library_path(path: Path | str | None) -> Path:
    if path is None:
        for candidate in default_library_candidates():
            if candidate.is_file() and candidate.resolve().is_relative_to(candidate.parent.resolve()):
                return candidate.resolve()
        raise FileNotFoundError("UWB 네이티브 라이브러리가 없습니다. build_uwb_native.py를 명시적으로 실행하십시오")
    if not isinstance(path, (Path, str)):
        raise TypeError("네이티브 라이브러리는 Path 또는 문자열 경로여야 합니다")
    value = os.fspath(path)
    if (not value or len(value) > 4096 or any(ord(char) < 32 for char in value)
            or value.startswith(("\\\\", "//")) or "://" in value):
        raise ValueError("로컬 네이티브 라이브러리 경로를 지정하십시오")
    resolved = Path(value).resolve(strict=True)
    if not resolved.is_file() or resolved.suffix.lower() != Path(LIBRARY_FILENAME).suffix:
        raise ValueError("현재 플랫폼의 일반 네이티브 라이브러리 파일이 필요합니다")
    return resolved


def _bind(library):
    """ctypes 기본 int 포인터 잘림을 막고 ABI 버전·프레임 크기를 먼저 검사한다."""
    signatures = {
        "sc_uwb_abi_version": ([], ctypes.c_uint32),
        "sc_uwb_frame_size": ([], ctypes.c_uint32),
        "sc_uwb_create": ([], ctypes.c_void_p),
        "sc_uwb_destroy": ([ctypes.c_void_p], None),
        "sc_uwb_reset": ([ctypes.c_void_p], None),
        "sc_uwb_feed": ([ctypes.c_void_p, ctypes.c_uint8, ctypes.POINTER(_Frame)], ctypes.c_uint32),
        "sc_uwb_accepted": ([ctypes.c_void_p], ctypes.c_uint32),
        "sc_uwb_rejected": ([ctypes.c_void_p], ctypes.c_uint32),
        "sc_uwb_last_error": ([ctypes.c_void_p], ctypes.c_uint32),
    }
    try:
        for name, (arguments, result) in signatures.items():
            function = getattr(library, name)
            function.argtypes, function.restype = arguments, result
    except AttributeError as exc:
        raise ValueError("UWB 네이티브 라이브러리에 필수 ABI 함수가 없습니다") from exc
    if library.sc_uwb_abi_version() != ABI_VERSION or library.sc_uwb_frame_size() != ctypes.sizeof(_Frame):
        raise ValueError("UWB 네이티브 라이브러리의 ABI 버전 또는 프레임 크기가 일치하지 않습니다")


class NativeParser:
    """인스턴스마다 독립 C 스트림을 소유한다. 호출자는 close 또는 with로 자원을 반환한다."""

    def __init__(self, library_path: Path | str | None = None):
        self._lock = threading.RLock()
        self.library_path = _library_path(library_path)
        self._library = ctypes.CDLL(str(self.library_path))
        _bind(self._library)
        handle = self._library.sc_uwb_create()
        if not handle:
            raise MemoryError("UWB C 파서 메모리를 할당하지 못했습니다")
        self._handle = ctypes.c_void_p(handle)
        self._finalizer = weakref.finalize(self, self._library.sc_uwb_destroy, self._handle)

    def _require_open(self) -> None:
        if not self._finalizer.alive:
            raise RuntimeError("이미 닫힌 UWB 파서입니다")

    def feed_events(self, data: bytes) -> list[dict]:
        """불량 프레임과 정상 프레임의 순서를 보존하여 안전 무효화 판단에 전달한다."""
        if not isinstance(data, bytes):
            raise TypeError("UWB 입력은 bytes여야 합니다")
        if len(data) > MAX_FEED_BYTES:
            raise ValueError("UWB 입력 묶음이 65536바이트 상한을 초과했습니다")
        with self._lock:
            self._require_open()
            events = []
            frame = _Frame()
            for byte in data:
                result = self._library.sc_uwb_feed(self._handle, byte, ctypes.byref(frame))
                if result == 1:
                    events.append({"kind": "frame", "frame": {
                        name: int(getattr(frame, name)) for name, _ctype in _Frame._fields_}})
                elif result == 2:
                    code = self._library.sc_uwb_last_error(self._handle)
                    events.append({"kind": "bad_frame", "reason": _ERROR_REASONS.get(code, "unknown")})
                elif result != 0:
                    raise RuntimeError("UWB 네이티브 파서가 알 수 없는 이벤트를 반환했습니다")
            return events

    def feed(self, data: bytes) -> list[dict[str, int]]:
        return [event["frame"] for event in self.feed_events(data) if event["kind"] == "frame"]

    @property
    def accepted_frames(self) -> int:
        with self._lock:
            self._require_open()
            return int(self._library.sc_uwb_accepted(self._handle))

    @property
    def rejected_frames(self) -> int:
        with self._lock:
            self._require_open()
            return int(self._library.sc_uwb_rejected(self._handle))

    def reset(self) -> None:
        with self._lock:
            self._require_open()
            self._library.sc_uwb_reset(self._handle)

    def close(self) -> None:
        with self._lock:
            self._finalizer()

    def __enter__(self):
        with self._lock:
            self._require_open()
        return self

    def __exit__(self, _type, _value, _traceback):
        self.close()
