"""자신이 시작한 프로세스만 감독·정리하며 센서 설정이나 구동 출력을 쓰지 않는다."""
from __future__ import annotations

import hashlib
import http.client
import json
import os
from pathlib import Path
import signal
import socket
import stat
import subprocess
import tempfile
import threading
import time


class InstanceLock:
    """동일 사용자·HTTP 포트의 이중 실행을 OS 잠금으로 막는다. PID로 타 프로세스를 종료하지 않는다."""
    def __init__(self, port, *, scope="http"):
        if scope not in ("http", "live"):
            raise ValueError("실행기 잠금 범위는 http 또는 live여야 합니다")
        if type(port) is not int or not 1024 <= port <= 65535:
            raise ValueError("실행기 잠금 포트는 1024~65535 정수여야 합니다")
        user = str(os.getuid()) if hasattr(os, "getuid") else str(Path.home())
        key = hashlib.sha256(user.encode("utf-8")).hexdigest()[:16]
        self.directory = Path(tempfile.gettempdir()) / ("smartcart-runner-" + key)
        # 실측 입력·출력 토픽은 고정이므로 HTTP 포트가 달라도 실측은 사용자당 한 묶음만 허용한다.
        self.path = self.directory / ("live.lock" if scope == "live" else str(port) + ".lock")
        self.stream = None

    def __enter__(self):
        self.directory.mkdir(mode=0o700, exist_ok=True)
        directory_stat = self.directory.lstat()
        self._validate(directory_stat, directory=True)
        directory_descriptor = None
        descriptor = None
        try:
            flags = os.O_RDWR | os.O_CREAT | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0)
            flags |= getattr(os, "O_CLOEXEC", 0)
            if os.name != "nt":
                directory_descriptor = os.open(self.directory, os.O_RDONLY | os.O_DIRECTORY | getattr(os, "O_NOFOLLOW", 0))
                self._validate(os.fstat(directory_descriptor), directory=True)
                descriptor = os.open(self.path.name, flags, 0o600, dir_fd=directory_descriptor)
            else:
                if self.path.exists() or self.path.is_symlink():
                    self._validate(self.path.lstat())
                descriptor = os.open(self.path, flags, 0o600)
            self._validate(os.fstat(descriptor))
            self.stream = os.fdopen(descriptor, "r+b", buffering=0)
            descriptor = None
            if os.name == "nt":
                import msvcrt
                if os.fstat(self.stream.fileno()).st_size == 0:
                    self.stream.write(b"\0")
                self.stream.seek(0)
                msvcrt.locking(self.stream.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(self.stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (OSError, ValueError, RuntimeError) as error:
            if self.stream is not None:
                self.stream.close()
                self.stream = None
            if descriptor is not None:
                os.close(descriptor)
            raise RuntimeError("실행기 잠금을 얻지 못했습니다. 중복 실행 또는 잠금 경로 소유권·종류·권한을 확인하십시오") from error
        finally:
            if directory_descriptor is not None:
                os.close(directory_descriptor)
        return self

    @staticmethod
    def _validate(value, *, directory=False):
        expected_kind = stat.S_ISDIR if directory else stat.S_ISREG
        if (not expected_kind(value.st_mode)
                or getattr(value, "st_file_attributes", 0) & getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400)
                or (not directory and value.st_nlink != 1)):
            raise RuntimeError("실행기 잠금은 재분석 지점이 아닌 전용 디렉터리와 단일 일반 파일이어야 합니다")
        if os.name != "nt" and (value.st_uid != os.getuid() or value.st_mode & 0o077):
            raise RuntimeError("실행기 잠금은 현재 사용자 소유이며 다른 사용자 접근 권한이 없어야 합니다")

    def __exit__(self, *_):
        if self.stream is not None:
            if os.name == "nt":
                import msvcrt
                self.stream.seek(0)
                msvcrt.locking(self.stream.fileno(), msvcrt.LK_UNLCK, 1)
            self.stream.close()
            self.stream = None
        # 잠금 경로를 삭제하면 대기 중인 다른 실행기와 경합하므로 작은 잠금 파일만 유지한다.


def ensure_http_available(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        if os.name == "nt":
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        else:
            # 실제 HTTP 서버와 같이 종료 뒤 TCP 대기 상태의 주소 재사용을 허용한다.
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(("127.0.0.1", port))
        except OSError as error:
            raise RuntimeError("HTTP 포트가 이미 사용 중입니다. 기존 프로세스를 확인하십시오") from error


def http_running(port):
    connection = http.client.HTTPConnection("127.0.0.1", port, timeout=.3)
    try:
        connection.request("GET", "/api/health")
        response = connection.getresponse()
        response.read(65537)
        return response.status == 200
    except (OSError, http.client.HTTPException):
        return False
    finally:
        connection.close()


def stop_children(children, grace=3.0):
    """Linux에서는 새 세션의 그룹, Windows에서는 새 그룹·필요시 해당 트리만 정리한다."""
    for process in reversed(children):
        if process.poll() is None:
            try:
                if os.name == "nt":
                    process.send_signal(signal.CTRL_BREAK_EVENT)
                else:
                    os.killpg(process.pid, signal.SIGINT)
            except (ProcessLookupError, OSError):
                pass
    deadline = time.monotonic() + grace
    for process in reversed(children):
        try:
            process.wait(timeout=max(0, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            pass
    for process in reversed(children):
        # 부모가 먼저 종료해도 POSIX 그룹에는 하위 프로세스가 남을 수 있다.
        if os.name != "nt":
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except (ProcessLookupError, OSError):
                pass
        elif process.poll() is None:
            taskkill = Path(os.environ.get("SystemRoot", "C:/Windows")) / "System32/taskkill.exe"
            try:
                subprocess.run([str(taskkill), "/PID", str(process.pid), "/T", "/F"],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5,
                               check=False, creationflags=subprocess.CREATE_NO_WINDOW)
            except (OSError, subprocess.TimeoutExpired):
                process.kill()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            raise RuntimeError("자식 프로세스 정리가 시간 안에 완료되지 않았습니다")


def supervise(commands, *, port, duration=None, startup_timeout=20.0, stop=None, popen=subprocess.Popen,
              readiness=http_running, announce=True):
    """어느 자식이든 예기치 않게 끝나면 나머지도 종료한다. 자동 재시작은 하지 않는다."""
    if not commands or not 0 < startup_timeout <= 60:
        raise ValueError("실행 명령과 60초 이하 기동 대기 시간이 필요합니다")
    stop = stop or threading.Event()
    children = []
    started, ready_at = time.monotonic(), None
    environment = os.environ.copy()
    environment.update(ROS_LOCALHOST_ONLY="1", PYTHONUNBUFFERED="1", PYTHONUTF8="1")
    try:
        for command in commands:
            if stop.is_set():
                return 130
            options = {"cwd": command["cwd"], "env": environment, "shell": False}
            if os.name == "nt":
                options["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
            else:
                options["start_new_session"] = True
            children.append(popen(command["argv"], **options))
        while not stop.is_set():
            if any(process.poll() is not None for process in children):
                raise RuntimeError("구성 프로세스가 종료되어 전체 실행을 중단했습니다")
            now = time.monotonic()
            if ready_at is None:
                if readiness(port):
                    ready_at = now
                    if announce:
                        print(json.dumps({"schema": "smartcart.run_state.v1", "service_running": True,
                            "http_url": "http://127.0.0.1:" + str(port), "sensor_freshness_verified": False,
                            "physical_acceptance": False, "drive_authorized": False,
                            "notice": "웹 서버가 기동했습니다. 센서 수신 상태와 실물 검수는 별도입니다"},
                            ensure_ascii=False), flush=True)
                elif now - started >= startup_timeout:
                    raise RuntimeError("웹 서버 기동 대기 시간이 초과되어 전체 실행을 중단했습니다")
            elif duration is not None and now - ready_at >= duration:
                return 0
            stop.wait(.1)
        return 130
    finally:
        stop_children(children)
