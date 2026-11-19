"""실물 없이 실제 자식 프로세스의 실패·기한·시작 오류와 중복 실행 정리를 검사한다."""
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import tempfile
import unittest
from unittest.mock import patch

from smartcart_runner.supervisor import InstanceLock, ensure_http_available, supervise

ROOT = Path(__file__).resolve().parents[1]


class RunnerSupervisorTests(unittest.TestCase):
    def setUp(self):
        self.children = []
        self.options = []
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            self.port = probe.getsockname()[1]

    def command(self, body="import time; time.sleep(60)"):
        return {"name": "test", "argv": [sys.executable, "-c", body], "cwd": str(ROOT)}

    def spawn(self, argv, **options):
        self.options.append(options)
        child = subprocess.Popen(argv, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, **options)
        self.children.append(child)
        return child

    def assert_all_closed(self):
        self.assertTrue(self.children)
        self.assertTrue(all(child.poll() is not None for child in self.children))

    def test_duration_stops_every_owned_process(self):
        code = supervise([self.command(), self.command()], port=self.port, duration=.15,
                         readiness=lambda _: True, popen=self.spawn, announce=False)
        self.assertEqual(code, 0)
        self.assert_all_closed()
        self.assertTrue(all(not options["shell"] for options in self.options))
        self.assertTrue(all(options["env"]["ROS_LOCALHOST_ONLY"] == "1" for options in self.options))

    def test_single_child_failure_stops_sibling(self):
        with self.assertRaisesRegex(RuntimeError, "프로세스가 종료"):
            supervise([self.command("import time; time.sleep(.2); raise SystemExit(7)"), self.command()],
                      port=self.port, readiness=lambda _: True, popen=self.spawn, announce=False)
        self.assert_all_closed()

    def test_failed_second_spawn_cleans_first_child(self):
        counter = 0
        def spawn(argv, **options):
            nonlocal counter
            counter += 1
            if counter == 2:
                raise OSError("시험용 시작 실패")
            return self.spawn(argv, **options)
        with self.assertRaises(OSError):
            supervise([self.command(), self.command()], port=self.port, popen=spawn, announce=False)
        self.assert_all_closed()

    def test_startup_deadline_cleans_children(self):
        with self.assertRaisesRegex(RuntimeError, "기동 대기"):
            supervise([self.command()], port=self.port, startup_timeout=.15,
                      readiness=lambda _: False, popen=self.spawn, announce=False)
        self.assert_all_closed()

    def test_stop_event_cleans_children(self):
        stop = threading.Event()
        timer = threading.Timer(.15, stop.set)
        timer.start()
        try:
            result = supervise([self.command()], port=self.port, stop=stop,
                               readiness=lambda _: True, popen=self.spawn, announce=False)
        finally:
            timer.cancel()
        self.assertEqual(result, 130)
        self.assert_all_closed()

    def test_instance_lock_denies_duplicate_then_releases(self):
        with InstanceLock(self.port):
            with self.assertRaises(RuntimeError):
                with InstanceLock(self.port):
                    pass
        with InstanceLock(self.port):
            pass

    def test_live_lock_denies_another_http_port_and_releases(self):
        other_port = 1024 if self.port != 1024 else 1025
        with InstanceLock(self.port, scope="live"):
            with self.assertRaises(RuntimeError):
                with InstanceLock(other_port, scope="live"):
                    pass
        with InstanceLock(other_port, scope="live"):
            pass

    def test_replay_http_locks_are_independent_for_distinct_ports(self):
        other_port = 1024 if self.port != 1024 else 1025
        with InstanceLock(self.port), InstanceLock(other_port):
            pass

    def test_lock_scope_is_allowlisted(self):
        for scope in ("../live", "other", None):
            with self.subTest(scope=scope), self.assertRaises(ValueError):
                InstanceLock(self.port, scope=scope)

    def test_used_http_port_is_refused_without_killing_owner(self):
        with socket.socket() as server:
            server.bind(("127.0.0.1", self.port))
            server.listen()
            with self.assertRaisesRegex(RuntimeError, "이미 사용"):
                ensure_http_available(self.port)
            self.assertEqual(server.getsockname()[1], self.port)

    @unittest.skipUnless(sys.platform.startswith('linux'), 'Linux TCP TIME_WAIT 재시작 검사')
    def test_closed_reusable_server_port_is_available_during_time_wait(self):
        with socket.socket() as server, socket.socket() as client:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('127.0.0.1', self.port))
            server.listen()
            client.connect(('127.0.0.1', self.port))
            connection, _address = server.accept()
            with connection:
                connection.shutdown(socket.SHUT_WR)
                self.assertEqual(client.recv(1), b'')
                client.close()
        # 서버 쪽에 남은 종료 대기 때문에 재사용 없는 probe는 실패하는 재현 조건이다.
        with socket.socket() as raw_probe, self.assertRaises(OSError):
            raw_probe.bind(('127.0.0.1', self.port))
        ensure_http_available(self.port)

    @unittest.skipUnless(os.name != 'nt', 'POSIX 주소 재사용과 실제 리스너 충돌 검사')
    def test_active_reusable_listener_is_still_refused(self):
        with socket.socket() as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('127.0.0.1', self.port))
            server.listen()
            with self.assertRaisesRegex(RuntimeError, '이미 사용'):
                ensure_http_available(self.port)

    @unittest.skipUnless(os.name != "nt", "POSIX 심볼릭 링크와 파일 권한 검사")
    def test_symbolic_link_lock_is_refused_without_changing_target(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "tests") as directory, patch("tempfile.gettempdir", return_value=directory):
            lock = InstanceLock(self.port)
            lock.directory.mkdir(mode=0o700)
            target = Path(directory) / "target"
            target.write_bytes(b"preserved")
            lock.path.symlink_to(target)
            with self.assertRaises(RuntimeError):
                with lock:
                    pass
            self.assertEqual(target.read_bytes(), b"preserved")

    @unittest.skipUnless(hasattr(os, "mkfifo"), "POSIX FIFO 검사")
    def test_fifo_lock_is_refused_without_waiting(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "tests") as directory, patch("tempfile.gettempdir", return_value=directory):
            lock = InstanceLock(self.port)
            lock.directory.mkdir(mode=0o700)
            os.mkfifo(lock.path, 0o600)
            started = time.monotonic()
            with self.assertRaises(RuntimeError):
                with lock:
                    pass
            self.assertLess(time.monotonic() - started, 1)

    @unittest.skipUnless(os.name != "nt", "POSIX 하드 링크·권한 검사")
    def test_hard_link_and_shared_directory_are_refused(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "tests") as directory, patch("tempfile.gettempdir", return_value=directory):
            lock = InstanceLock(self.port)
            lock.directory.mkdir(mode=0o700)
            target = Path(directory) / "target"
            target.write_bytes(b"unchanged")
            target.chmod(0o600)
            os.link(target, lock.path)
            with self.assertRaises(RuntimeError):
                with lock:
                    pass
            self.assertEqual(target.read_bytes(), b"unchanged")
            lock.path.unlink()
            lock.directory.chmod(0o755)
            with self.assertRaises(RuntimeError):
                with lock:
                    pass


if __name__ == "__main__":
    unittest.main()
