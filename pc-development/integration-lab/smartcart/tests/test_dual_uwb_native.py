"""원본 실측 프레임과 손상·분할·중첩 헤더로 C 연결 ABI를 검증한다."""
from __future__ import annotations

import ctypes
import importlib.util
from pathlib import Path
import struct
import unittest
from unittest.mock import patch

from smartcart_uwb import native
from smartcart_uwb.native import NativeParser


REAL_FRAME = bytes.fromhex("2a 1b 0c 19 6e eb ff ff ff 39 00 00 00 00 c0 "
                           "00 00 00 00 00 00 00 00 00 00 00 00 00 00 96 23")


def make_frame(*, sequence=12, tag_address=0x6E19, angle_deg=-21, distance_cm=57,
               user_command=0xC000, first_path_power=0, rx_level=0,
               acceleration_x=0, acceleration_y=0, acceleration_z=0) -> bytes:
    payload = struct.pack("<BHiIHii3h", sequence, tag_address, angle_deg, distance_cm,
                          user_command, first_path_power, rx_level,
                          acceleration_x, acceleration_y, acceleration_z)
    checksum = 0
    for byte in payload:
        checksum ^= byte
    return b"\x2a\x1b" + payload + bytes((checksum, 0x23))


class NativeBoundaryTests(unittest.TestCase):
    def test_import_does_not_load_or_build_library(self):
        with patch("ctypes.CDLL", side_effect=AssertionError("자동 로딩 금지")):
            spec = importlib.util.spec_from_file_location("native_import_check", native.__file__)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            self.assertEqual(module.ABI_VERSION, 1)

    def test_explicit_network_and_control_paths_rejected(self):
        for path in ("", "socket://host/library.dll", "//host/library.dll", "\\\\host\\library.dll", "x\n.dll"):
            with self.subTest(path=path), self.assertRaises(ValueError):
                NativeParser(path)

    def test_default_candidates_are_fixed_without_environment_search(self):
        with patch.dict("os.environ", {"SMARTCART_UWB_LIBRARY": "untrusted.dll"}):
            self.assertEqual(len(native.default_library_candidates()), 2)
            self.assertTrue(all(path.is_absolute() for path in native.default_library_candidates()))
            self.assertTrue(all(path.name == native.LIBRARY_FILENAME for path in native.default_library_candidates()))

    def test_abi_version_and_frame_size_mismatch_rejected(self):
        class Function:
            def __init__(self, result):
                self.result = result
            def __call__(self):
                return self.result

        class Library:
            def __getattr__(self, name):
                result = 2 if name == "sc_uwb_abi_version" else 40
                function = Function(result)
                setattr(self, name, function)
                return function

        with self.assertRaises(ValueError):
            native._bind(Library())
        library = Library()
        library.sc_uwb_abi_version = Function(1)
        library.sc_uwb_frame_size = Function(44)
        with self.assertRaises(ValueError):
            native._bind(library)


class NativeParserTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.library_path = native._library_path(None)
        except FileNotFoundError:
            raise unittest.SkipTest("명시적으로 빌드한 UWB 네이티브 라이브러리가 없습니다")

    def test_real_frame_and_fixed_abi(self):
        self.assertEqual(make_frame(), REAL_FRAME)
        with NativeParser(self.library_path) as parser:
            self.assertEqual(parser._library.sc_uwb_frame_size(), 40)
            frames = parser.feed(REAL_FRAME)
            self.assertEqual(len(frames), 1)
            self.assertEqual(frames[0], {"sequence": 12, "tag_address": 0x6E19, "angle_deg": -21,
                "distance_cm": 57, "user_command": 0xC000, "first_path_power": 0, "rx_level": 0,
                "acceleration_x": 0, "acceleration_y": 0, "acceleration_z": 0})
            self.assertEqual((parser.accepted_frames, parser.rejected_frames), (1, 0))

    def test_all_signed_and_unsigned_fields_preserved(self):
        values = dict(sequence=255, tag_address=65535, angle_deg=-2147483648, distance_cm=4294967295,
                      user_command=65535, first_path_power=-2147483648, rx_level=2147483647,
                      acceleration_x=-32768, acceleration_y=32767, acceleration_z=-1)
        with NativeParser() as parser:
            self.assertEqual(parser.feed(make_frame(**values)), [values])

    def test_fragmented_reads_and_independent_sensor_streams(self):
        with NativeParser() as left, NativeParser() as right:
            self.assertEqual(left.feed(REAL_FRAME[:17]), [])
            self.assertEqual(right.feed(make_frame(sequence=9))[0]["sequence"], 9)
            self.assertEqual(left.feed(REAL_FRAME[17:])[0]["sequence"], 12)
            self.assertEqual((left.accepted_frames, right.accepted_frames), (1, 1))

    def test_bad_then_good_and_good_then_bad_events_remain_ordered(self):
        corrupt = bytearray(REAL_FRAME)
        corrupt[9] ^= 1
        with NativeParser() as parser:
            events = parser.feed_events(bytes(corrupt) + REAL_FRAME + bytes(corrupt))
            self.assertEqual([event["kind"] for event in events], ["bad_frame", "frame", "bad_frame"])
            self.assertEqual(events[0]["reason"], "checksum")
            self.assertEqual((parser.accepted_frames, parser.rejected_frames), (1, 2))

    def test_noise_bad_length_footer_and_reset_partial_state(self):
        corrupt = REAL_FRAME[:-1] + b"\x00"
        with NativeParser() as parser:
            events = parser.feed_events(b"\x99\x2a\x00" + corrupt + REAL_FRAME)
            self.assertEqual([event.get("reason") for event in events], ["length", "footer", None])
            parser.feed(REAL_FRAME[:5])
            parser.reset()
            self.assertEqual((parser.accepted_frames, parser.rejected_frames), (0, 0))
            self.assertEqual(parser.feed(REAL_FRAME[5:]), [])
            self.assertEqual(parser.feed(REAL_FRAME)[0]["tag_address"], 0x6E19)

    def test_nested_header_resynchronization_preserves_following_frame(self):
        # 불량 프레임 내부에 새 프레임 머리가 들어온 경우 다음 읽기로 완성되어야 한다.
        with NativeParser() as parser:
            events = parser.feed_events(b"\x2a\x1b" + b"\x00" * 8 + REAL_FRAME)
            self.assertEqual([event["kind"] for event in events], ["bad_frame", "frame"])
            self.assertEqual(events[-1]["frame"]["distance_cm"], 57)

    def test_many_corruptions_do_not_overflow_stream_state(self):
        with NativeParser() as parser:
            events = parser.feed_events((b"\x2a\x00" * 1000) + REAL_FRAME)
            self.assertEqual(parser.rejected_frames, 1000)
            self.assertEqual(parser.accepted_frames, 1)
            self.assertEqual(events[-1]["kind"], "frame")

    def test_closed_and_bounded_inputs_are_rejected_before_native_call(self):
        parser = NativeParser()
        with self.assertRaises(TypeError):
            parser.feed(bytearray(REAL_FRAME))
        with self.assertRaises(ValueError):
            parser.feed(b"x" * (native.MAX_FEED_BYTES + 1))
        self.assertEqual(parser.accepted_frames, 0)
        parser.close()
        parser.close()
        for operation in (lambda: parser.feed(b""), parser.reset,
                          lambda: parser.accepted_frames, lambda: parser.rejected_frames):
            with self.assertRaises(RuntimeError):
                operation()

    def test_c_null_boundary_is_safe(self):
        with NativeParser() as parser:
            frame = native._Frame()
            library = parser._library
            self.assertEqual(library.sc_uwb_feed(None, 0, ctypes.byref(frame)), 2)
            self.assertEqual(library.sc_uwb_feed(parser._handle, 0, None), 2)
            self.assertEqual(library.sc_uwb_last_error(None), 1)
            self.assertEqual(library.sc_uwb_accepted(None), 0)
            self.assertEqual(library.sc_uwb_rejected(None), 0)
            library.sc_uwb_reset(None)
            library.sc_uwb_destroy(None)


if __name__ == "__main__":
    unittest.main()
