"""원시 합성·기록 캡처의 수신 메타데이터만 검수 모니터로 재생한다."""
from smartcart_tracking.replay import read_records
from smartcart_uwb.collector import DualCollector
from smartcart_uwb.replay import REPLAY_CLOCK_ORIGIN_NS, MAX_RECORDS


def inspect_replay(mounting, monitor, path, *, parser_factory=None):
    if monitor.source_mode != "replay":
        raise ValueError("재생 검수는 replay 출처 모니터에만 전달합니다")
    previous, source, count = -1, None, 0
    with DualCollector(mounting, source_mode="replay", parser_factory=parser_factory) as collector:
        for count, record in enumerate(read_records(path), 1):
            if count > MAX_RECORDS or record["at_ns"] < previous or (source is not None and source != record["source"]):
                raise ValueError("재생 개수·시각·출처가 일관되지 않습니다")
            previous, source = record["at_ns"], record["source"]
            now = REPLAY_CLOCK_ORIGIN_NS + previous
            event = record["event"]
            if event == "uwb_bytes":
                collector.feed(record["sensor_id"], bytes.fromhex(record["data_hex"]), now_ns=now)
            elif event == "uwb_disconnect":
                collector.disconnected(record["sensor_id"])
            elif event == "uwb_reconnect":
                collector.reconnect(record["sensor_id"])
            elif event == "lidar_scan":
                monitor.observe_scan(record["scan"], now, frame_id=mounting.lidar.frame_id)
            # 실제 ROS LaserScan도 단절 메시지가 없으므로 도착 중단과 tick의 TTL을 사용한다.
            monitor.observe_status(collector.snapshot(now_ns=now), now)
            monitor.tick(now)
    if count == 0:
        raise ValueError("재생 입력이 비어 있습니다")
    return {"records": count, "original_source": source, "report": monitor.report()}
