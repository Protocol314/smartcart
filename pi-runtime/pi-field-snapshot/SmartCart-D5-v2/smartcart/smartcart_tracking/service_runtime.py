"""실물 없이 원시 통합 캡처를 벽시계 속도에 맞춰 기존 GUI에 공급한다."""
import math
import time

from smartcart_radar.core import normalise_scan
from smartcart_uwb.replay import REPLAY_CLOCK_ORIGIN_NS, MAX_RECORDS
from .pipeline import TrackingPipeline
from .replay import read_records


def replay_loop(lidar_store, tracking_store, mounting, uwb_settings, settings, path, *,
                speed=.25, loop=False, parser_factory=None):
    """스캔과 추적은 같은 캡처를 사용한다. 재생을 실측 토픽으로 바꾸지 않는다."""
    if (type(speed) not in (int, float) or not math.isfinite(speed) or not .05 <= speed <= 4
            or type(loop) is not bool or tracking_store.mode != "replay"
            or lidar_store.stop is not tracking_store.stop):
        raise ValueError("재생 속도·표시 모드·종료 이벤트가 잘못되었습니다")
    iteration = 0
    try:
        while not tracking_store.stop.is_set():
            iteration += 1
            metadata = {"iteration": iteration, "speed": float(speed), "event_index": 0}
            tracking_store.begin(replay=metadata)
            with TrackingPipeline(mounting, uwb_settings, settings,
                                  parser_factory=parser_factory) as pipeline:
                start_wall = time.monotonic_ns()
                previous = -1
                original_source = None
                count = 0
                for count, record in enumerate(read_records(path), 1):
                    if count > MAX_RECORDS or record["at_ns"] < previous:
                        raise ValueError("재생 레코드 수·시각이 잘못되었습니다")
                    if original_source is not None and original_source != record["source"]:
                        raise ValueError("재생 출처가 혼합되었습니다")
                    previous, original_source = record["at_ns"], record["source"]
                    while not tracking_store.stop.is_set():
                        elapsed = max(0, int((time.monotonic_ns() - start_wall) * speed))
                        if elapsed >= record["at_ns"]:
                            break
                        # 다음 사건의 시각을 넘겨 평가하지 않아 원본 시계의 역행을 막는다.
                        value = pipeline.refresh(REPLAY_CLOCK_ORIGIN_NS + elapsed)
                        tracking_store.publish(value, now_ns=REPLAY_CLOCK_ORIGIN_NS + elapsed,
                                               clock_rate=speed, replay=metadata)
                        tracking_store.stop.wait(min(.05, (record["at_ns"] - elapsed) / speed / 1e9))
                    if tracking_store.stop.is_set():
                        return
                    now_ns = REPLAY_CLOCK_ORIGIN_NS + record["at_ns"]
                    value = pipeline.process(record, now_ns)
                    metadata = {"iteration": iteration, "speed": float(speed), "event_index": count - 1}
                    if record["event"] == "lidar_scan":
                        scan = normalise_scan(record["scan"], kind="replay", label="통합 센서 캡처 재생")
                        scan["header"]["frame_id"] = mounting.lidar.frame_id
                        lidar_store.publish(scan)
                    elif record["event"] == "lidar_disconnect":
                        lidar_store.set_error("재생: LiDAR 연결 끊김")
                    tracking_store.publish(value, now_ns=now_ns, clock_rate=speed, replay=metadata)
                if not count:
                    raise ValueError("통합 캡처가 비어 있습니다")
            tracking_store.finish()
            if not loop:
                lidar_store.finish()
                return
            if tracking_store.stop.wait(.75):
                return
    except Exception:
        tracking_store.set_error("tracking_replay_error")
        lidar_store.set_error("통합 캡처 재생 오류")
