"""장치·ROS·소켓 없이 통합 실행 옵션과 작업자 소유·재생 사전 검사를 확인한다."""

from contextlib import ExitStack, contextmanager, redirect_stderr, redirect_stdout
import io
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from smartcart_sensing import app
from smartcart_tracking.ros_runtime import tracking_replay_ros_loop, tracking_ros_loop
from smartcart_tracking.service_runtime import replay_loop


ROOT = Path(__file__).resolve().parents[1]
DEPENDENCIES = ("rclpy", "sensor_msgs", "std_msgs", "geometry_msgs", "diagnostic_msgs", "serial")


class TrackingAppTests(unittest.TestCase):
    def setUp(self):
        # 샌드박스에서도 임시 입력은 허용된 저장소의 무시되는 빌드 경로만 사용한다.
        temporary_root = ROOT.parent / "build" / "verification" / "tracking-app" / "tmp"
        temporary_root.mkdir(parents=True, exist_ok=True)
        directory = tempfile.TemporaryDirectory(dir=temporary_root)
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)

    def args(self, *options):
        return app.build_parser().parse_args(["serve", *options])

    def capture(self, text=None):
        path = self.directory / "capture.jsonl"
        if text is None:
            text = json.dumps({"schema": "smartcart.target_capture.v1", "event": "tick",
                               "at_ns": 0, "source": "synthetic"}) + "\n"
        path.write_text(text, encoding="utf-8")
        return path

    @contextmanager
    def harness(self, *, missing=(), native_error=None, server_error=None):
        """생성된 작업자의 대상만 수집하며 스레드나 서버를 실제 실행하지 않는다."""
        state = SimpleNamespace(threads=[], servers=[], native=[], events=[],
                                stdout=io.StringIO(), stderr=io.StringIO())

        class FakeNativeParser:
            def __init__(self, library_path=None):
                state.events.append("native_created")
                if native_error is not None:
                    raise native_error
                self.library_path = library_path
                self.closed = False
                state.native.append(self)

            def __enter__(self):
                state.events.append("native_entered")
                return self

            def __exit__(self, *_error):
                self.closed = True
                state.events.append("native_closed")

        class FakeThread:
            def __init__(self, *, target, args=(), kwargs=None, daemon=None):
                self.target, self.args = target, args
                self.kwargs = kwargs or {}
                self.daemon, self.started, self.join_timeout = daemon, False, None
                state.threads.append(self)

            def start(self):
                self.started = True
                state.events.append("worker_started")

            def join(self, timeout=None):
                self.join_timeout = timeout

        class FakeServer:
            def __init__(self, address, store, root, *, allow_lan=False):
                self.address, self.store, self.root = address, store, root
                self.allow_lan, self.server_port = allow_lan, address[1]
                self.closed, self.poll_interval = False, None
                state.events.append("server_created")
                state.servers.append(self)

            def serve_forever(self, *, poll_interval):
                self.poll_interval = poll_interval
                if server_error is not None:
                    raise server_error

            def server_close(self):
                self.closed = True
                state.events.append("server_closed")

        prerequisites = {"dependencies": {name: name not in missing for name in DEPENDENCIES}}
        with ExitStack() as stack:
            stack.enter_context(patch.object(app, "doctor", return_value=prerequisites))
            stack.enter_context(patch.object(app, "SensingHTTPServer", FakeServer))
            stack.enter_context(patch.object(app.threading, "Thread", FakeThread))
            # 테스트 도중 프로세스 전체의 SIGTERM 핸들러를 바꾸지 않는다.
            stack.enter_context(patch.object(app.threading, "current_thread", return_value=object()))
            stack.enter_context(patch("smartcart_uwb.native.NativeParser", FakeNativeParser))
            stack.enter_context(patch.dict(os.environ, {}, clear=False))
            stack.enter_context(redirect_stdout(state.stdout))
            stack.enter_context(redirect_stderr(state.stderr))
            yield state

    def assert_lifecycle(self, state):
        self.assertEqual(len(state.servers), 1)
        server = state.servers[0]
        self.assertTrue(server.closed)
        self.assertEqual(server.poll_interval, .2)
        self.assertTrue(server.store.stop.is_set())
        self.assertIs(server.store.stop, server.store.uwb.stop)
        self.assertIs(server.store.stop, server.store.tracking.stop)
        for worker in state.threads:
            self.assertTrue(worker.daemon)
            self.assertTrue(worker.started)
            self.assertEqual(worker.join_timeout, 3)

    def test_tracking_defaults_preserve_legacy_off(self):
        args = self.args()
        self.assertEqual((args.mode, args.uwb_mode, args.tracking_mode), ("ros", "serial", "off"))
        self.assertEqual(args.tracking_speed, .25)
        self.assertEqual(args.tracking_status_topic, "/smartcart/uwb/dual/status")
        self.assertIsNone(args.tracking_input)
        self.assertIsNone(args.tracking_native_library)
        self.assertFalse(args.tracking_loop)
        self.assertFalse(args.tracking_ros_replay)

    def test_replay_forces_explicit_replay_source_and_disables_legacy_serial(self):
        path = self.capture()
        args = app.validate_args(self.args("--mode", "replay", "--tracking-input", str(path)))
        self.assertEqual((args.mode, args.tracking_mode, args.uwb_mode), ("replay", "replay", "off"))
        self.assertIsInstance(args.tracking_input, Path)
        self.assertIsNone(args.serial_port)

    def test_replay_options_cannot_mix_with_demo_or_live(self):
        path = self.capture()
        for mode in ("demo", "ros"):
            for flags in (("--tracking-mode", "replay"), ("--tracking-input", str(path)),
                          ("--tracking-loop",), ("--tracking-ros-replay",)):
                with self.subTest(mode=mode, flags=flags), self.assertRaises(ValueError):
                    app.validate_args(self.args("--mode", mode, "--uwb-mode", "off", *flags))
        with self.assertRaises(ValueError):
            app.validate_args(self.args("--mode", "demo", "--tracking-mode", "ros"))

    def test_replay_requires_input_and_refuses_live_tracking(self):
        with self.assertRaises(ValueError):
            app.validate_args(self.args("--mode", "replay"))
        with self.assertRaises(ValueError):
            app.validate_args(self.args("--mode", "replay", "--tracking-mode", "ros",
                                        "--tracking-input", str(self.capture())))

    def test_live_tracking_without_legacy_uwb_does_not_require_a_serial_port(self):
        args = app.validate_args(self.args("--tracking-mode", "ros", "--uwb-mode", "off"))
        self.assertEqual(args.tracking_mode, "ros")
        self.assertIsNone(args.serial_port)

    def test_tracking_status_topic_rejects_input_collisions_and_bad_names(self):
        for topic in ("/scan", "/smartcart/uwb", "../../status", "/status bad"):
            with self.subTest(topic=topic), self.assertRaises(ValueError):
                app.validate_args(self.args("--tracking-mode", "ros", "--uwb-mode", "off",
                                            "--tracking-status-topic", topic))

    def test_replay_speed_is_bounded_and_nonfinite_rejected_during_parsing(self):
        for speed in (".049", "4.001", "0", "-1"):
            with self.subTest(speed=speed), self.assertRaises(ValueError):
                app.validate_args(self.args("--mode", "demo", "--tracking-speed", speed))
        for speed in ("nan", "inf", "-inf"):
            with self.subTest(speed=speed), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.args("--mode", "demo", "--tracking-speed=" + speed)
        for speed in (".05", "4"):
            self.assertEqual(app.validate_args(self.args("--mode", "demo", "--tracking-speed", speed)).tracking_speed,
                             float(speed))

    def test_demo_keeps_only_two_existing_synthetic_workers(self):
        with self.harness() as state:
            self.assertEqual(app.serve(self.args("--mode", "demo")), 0)
        self.assertEqual([worker.target for worker in state.threads], [app.source_loop, app.demo_loop])
        store = state.servers[0].store
        self.assertEqual(store.tracking.mode, "off")
        self.assertEqual(store.mode, "demo")
        self.assertEqual(state.native, [])
        self.assert_lifecycle(state)

    def test_legacy_serial_keeps_scan_ros_owner_and_only_one_serial_worker(self):
        with self.harness() as state:
            self.assertEqual(app.serve(self.args("--serial-port", "COM25", "--baud", "230400")), 0)
        self.assertEqual([worker.target for worker in state.threads], [app.ros_loop, app.serial_loop])
        ros, serial = state.threads
        self.assertTrue(ros.kwargs["subscribe_scan"])
        self.assertFalse(ros.kwargs["subscribe_uwb"])
        self.assertTrue(ros.kwargs["publish_uwb"])
        self.assertEqual(serial.args[1:], ("COM25", 230400))
        self.assertEqual(state.native, [])
        self.assert_lifecycle(state)

    def test_tracking_ros_owns_scan_without_legacy_workers_when_uwb_off(self):
        ros_args = ["--ros-args", "-r", "__ns:=/test"]
        with self.harness(missing=("serial",)) as state:
            self.assertEqual(app.serve(self.args("--tracking-mode", "ros", "--uwb-mode", "off",
                "--topic", "/front_scan", "--tracking-status-topic", "/dual_status"), ros_args=ros_args), 0)
        self.assertEqual([worker.target for worker in state.threads], [tracking_ros_loop])
        worker = state.threads[0]
        self.assertEqual(worker.kwargs, {"scan_topic": "/front_scan", "status_topic": "/dual_status",
                                         "ros_args": ros_args, "dual_uwb_store": state.servers[0].store.dual_uwb})
        self.assertIsNotNone(state.servers[0].store.dual_uwb)
        self.assertEqual(state.servers[0].store.tracking.mode, "ros")
        self.assertIs(worker.args[0], state.servers[0].store.lidar)
        self.assertIs(worker.args[1], state.servers[0].store.tracking)
        self.assertEqual(state.native, [])
        self.assert_lifecycle(state)

    def test_tracking_ros_and_legacy_ros_share_scan_with_one_subscription_owner(self):
        with self.harness(missing=("serial",)) as state:
            self.assertEqual(app.serve(self.args("--tracking-mode", "ros", "--uwb-mode", "ros")), 0)
        self.assertEqual([worker.target for worker in state.threads], [tracking_ros_loop, app.ros_loop])
        legacy = state.threads[1]
        self.assertFalse(legacy.kwargs["subscribe_scan"])
        self.assertTrue(legacy.kwargs["subscribe_uwb"])
        self.assertFalse(legacy.kwargs["publish_uwb"])
        self.assertIs(legacy.args[0], state.threads[0].args[0])
        self.assert_lifecycle(state)

    def test_tracking_ros_and_legacy_serial_do_not_duplicate_scan_subscription(self):
        with self.harness() as state:
            self.assertEqual(app.serve(self.args("--tracking-mode", "ros", "--serial-port", "COM25")), 0)
        self.assertEqual([worker.target for worker in state.threads],
                         [tracking_ros_loop, app.ros_loop, app.serial_loop])
        self.assertFalse(state.threads[1].kwargs["subscribe_scan"])
        self.assertFalse(state.threads[1].kwargs["subscribe_uwb"])
        self.assertTrue(state.threads[1].kwargs["publish_uwb"])
        self.assert_lifecycle(state)

    def test_legacy_off_still_keeps_scan_ros_worker_when_tracking_off(self):
        with self.harness(missing=("serial",)) as state:
            self.assertEqual(app.serve(self.args("--uwb-mode", "off")), 0)
        self.assertEqual([worker.target for worker in state.threads], [app.ros_loop])
        self.assertTrue(state.threads[0].kwargs["subscribe_scan"])
        self.assertFalse(state.threads[0].kwargs["subscribe_uwb"])
        self.assertFalse(state.threads[0].kwargs["publish_uwb"])
        self.assert_lifecycle(state)

    def test_replay_preflight_closes_parser_before_server_and_worker_start(self):
        path = self.capture()
        library = self.directory / "explicit-parser.dll"
        with self.harness(missing=DEPENDENCIES) as state:
            result = app.serve(self.args("--mode", "replay", "--tracking-input", str(path),
                "--tracking-native-library", str(library), "--tracking-speed", ".5", "--tracking-loop"))
        self.assertEqual(result, 0)
        self.assertEqual([worker.target for worker in state.threads], [replay_loop])
        self.assertEqual(len(state.native), 1)
        self.assertTrue(state.native[0].closed)
        self.assertEqual(state.native[0].library_path, library)
        self.assertLess(state.events.index("native_closed"), state.events.index("server_created"))
        self.assertLess(state.events.index("server_created"), state.events.index("worker_started"))
        worker = state.threads[0]
        self.assertEqual(worker.args[-1], path)
        self.assertEqual(worker.kwargs["speed"], .5)
        self.assertTrue(worker.kwargs["loop"])
        self.assertTrue(callable(worker.kwargs["parser_factory"]))
        store = state.servers[0].store
        self.assertEqual((store.mode, store.tracking.mode), ("replay", "replay"))
        self.assertEqual(store.uwb.snapshot()["source_mode"], "off")
        self.assertFalse(store.tracking.snapshot()["drive_authorized"])
        self.assert_lifecycle(state)

    def test_explicit_configuration_files_reach_the_replay_worker_unchanged(self):
        names = ("dual_uwb.d5.example.json", "uwb_estimation.synthetic.json", "target_tracking.synthetic.json")
        paths = []
        for name in names:
            destination = self.directory / name
            destination.write_bytes((ROOT / "config" / name).read_bytes())
            paths.append(destination)
        options = ["--mode", "replay", "--tracking-input", str(self.capture())]
        for flag, path in zip(("--tracking-mounting-config", "--tracking-uwb-config", "--tracking-config"), paths):
            options.extend((flag, str(path)))
        with self.harness() as state:
            self.assertEqual(app.serve(self.args(*options)), 0)
            worker = state.threads[0]
            # 기본 unknown 거리 모델 대신 명시한 합성 경사 거리 설정이 전달되어야 한다.
            self.assertEqual(worker.args[3].range_model, "slant_known_height")
            self.assertEqual(worker.args[3].tag_height_m, .9)
            self.assertIs(worker.args[2], state.servers[0].store.tracking.config)
            self.assertIs(worker.args[4], state.servers[0].store.tracking.settings)
            with worker.kwargs["parser_factory"]() as parser:
                self.assertIsNone(parser.library_path)
            self.assertTrue(parser.closed)
        self.assert_lifecycle(state)

    def test_replay_optional_ros_publisher_gets_only_tracking_store_and_ros_arguments(self):
        ros_args = ["--ros-args", "-p", "use_sim_time:=false"]
        with self.harness(missing=("serial",)) as state:
            self.assertEqual(app.serve(self.args("--mode", "replay", "--tracking-input", str(self.capture()),
                "--tracking-ros-replay"), ros_args=ros_args), 0)
        self.assertEqual([worker.target for worker in state.threads], [replay_loop, tracking_replay_ros_loop])
        publisher = state.threads[1]
        self.assertEqual(publisher.args, (state.servers[0].store.tracking,))
        self.assertEqual(publisher.kwargs, {"ros_args": ros_args})
        self.assert_lifecycle(state)

    def test_empty_and_malformed_capture_fail_before_server_or_worker_creation(self):
        for text in ("", "not-json\n", '{"schema":"smartcart.uwb_capture.v1"}\n'):
            with self.subTest(text=text), self.harness(missing=DEPENDENCIES) as state:
                result = app.main(["serve", "--mode", "replay", "--tracking-input", str(self.capture(text))])
                self.assertEqual(result, 2)
                self.assertEqual(state.servers, [])
                self.assertEqual(state.threads, [])
                self.assertEqual(len(state.native), 1)
                self.assertTrue(state.native[0].closed)
                self.assertIn("센싱 실행 오류:", state.stderr.getvalue())
                self.assertNotIn("Traceback", state.stderr.getvalue())

    def test_exhausted_capture_iterator_becomes_clear_value_error_and_is_closed(self):
        class EmptyCapture:
            closed = False

            def __next__(self):
                raise StopIteration

            def close(self):
                self.closed = True

        capture = EmptyCapture()
        with self.harness() as state, patch("smartcart_tracking.replay.read_records", return_value=capture):
            with self.assertRaisesRegex(ValueError, "재생 입력에 레코드가 없습니다"):
                app.serve(self.args("--mode", "replay", "--tracking-input", str(self.capture())))
        self.assertTrue(capture.closed)
        self.assertTrue(state.native[0].closed)
        self.assertEqual(state.servers, [])
        self.assertEqual(state.threads, [])

    def test_native_library_error_fails_before_capture_server_and_workers(self):
        with self.harness(native_error=OSError("명시한 파서 라이브러리를 열 수 없습니다")) as state:
            with patch("smartcart_tracking.replay.read_records") as read_records:
                self.assertEqual(app.main(["serve", "--mode", "replay", "--tracking-input", str(self.capture())]), 2)
                read_records.assert_not_called()
        self.assertEqual(state.servers, [])
        self.assertEqual(state.threads, [])
        self.assertIn("파서 라이브러리", state.stderr.getvalue())

    def test_bad_explicit_tracking_configuration_fails_before_parser_or_server(self):
        invalid = self.directory / "bad-tracking.json"
        invalid.write_text('{"unknown":true}', encoding="utf-8")
        with self.harness() as state:
            result = app.main(["serve", "--mode", "replay", "--tracking-input", str(self.capture()),
                               "--tracking-config", str(invalid)])
        self.assertEqual(result, 2)
        self.assertEqual(state.native, [])
        self.assertEqual(state.servers, [])
        self.assertEqual(state.threads, [])

    def test_missing_ros_dependency_fails_before_any_workers_or_server(self):
        for options in (("--tracking-mode", "ros", "--uwb-mode", "off"),
                        ("--mode", "replay", "--tracking-input", str(self.capture()), "--tracking-ros-replay")):
            with self.subTest(options=options), self.harness(missing=("diagnostic_msgs",)) as state:
                self.assertEqual(app.main(["serve", *options]), 2)
                self.assertEqual(state.threads, [])
                self.assertEqual(state.servers, [])
                self.assertEqual(state.native, [])
                self.assertIn("ROS 2 환경", state.stderr.getvalue())

    def test_missing_serial_dependency_only_blocks_explicit_serial_mode(self):
        with self.harness(missing=("serial",)) as state:
            self.assertEqual(app.main(["serve", "--serial-port", "COM25"]), 2)
        self.assertEqual(state.servers, [])
        self.assertEqual(state.threads, [])
        self.assertIn("python3-serial", state.stderr.getvalue())

    def test_keyboard_interrupt_closes_server_and_joins_all_owned_workers(self):
        with self.harness(server_error=KeyboardInterrupt()) as state:
            self.assertEqual(app.serve(self.args("--mode", "demo")), 0)
        self.assert_lifecycle(state)

    def test_server_runtime_error_stops_owned_workers_before_main_returns_failure(self):
        with self.harness(server_error=OSError("모의 HTTP 실행 오류")) as state:
            self.assertEqual(app.main(["serve", "--mode", "demo"]), 2)
        self.assert_lifecycle(state)
        self.assertIn("모의 HTTP 실행 오류", state.stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
