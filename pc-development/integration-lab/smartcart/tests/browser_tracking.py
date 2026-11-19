#!/usr/bin/env python3
"""CUA에서 열 실제 합성 HTTP 서버와 재현 가능한 브라우저 검사 계획.

이 스크립트는 브라우저를 자동 조작하지 않는다. 기존 C 파서와 실제 통합
재생 작업자를 사용하고 루프백 HTTP/SSE만 검사한다. 시각 검사는 출력한
URL을 CUA로 열어 별도로 수행해야 하며 보고서에서도 이를 구분한다.
"""
from __future__ import annotations

import argparse
import http.client
import json
import math
from pathlib import Path
import sys
import threading
import time
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

BROWSER_CHECKS = [
    "데스크톱에서 재생·검수 전·구동 불가 표시와 한국어 추적 상태를 확인한다.",
    "신선한 tracking에서 군집·선택 대상이 같은 카트 좌표에 표시되는지 확인한다.",
    "LiDAR 장착 X=.817m와 화면 장착 보정의 차이를 알리거나 같은 변환을 적용하는지 확인한다.",
    "가림·모호성에서는 측정 마커를 숨기고 제한된 예측만 구별해서 표시한다.",
    "센서 단절·만료·오류·가림 한도 초과에서 측정·예측 마커가 모두 사라지는지 확인한다.",
    "화면 정지 중에도 남은 벽시계 수명이 지나면 마커가 사라지는지 확인한다.",
    "첨부 스캔·로컬 데모 소스로 전환하면 서버 추적 데이터를 섞지 않는지 확인한다.",
    "서버 연결을 끊으면 오래된 선택·군집·예측이 남지 않는지 확인한다.",
    "모바일 390px와 태블릿 768px에서 패널·레이더·설정에 가로 넘침이 없는지 확인한다.",
    "실제 DOM과 Canvas를 검사하고 데스크톱·모바일·가림 상태의 CUA 스크린샷을 보존한다.",
]
PRIVATE_KEYS = {"result", "track", "uwb_estimate", "lidar_clusters", "lidar_transform", "track_id", "tag_address",
                "position_xy_m", "velocity_xy_mps", "predicted_position_xy_m", "ranges", "clusters"}


def summary_only(value):
    if isinstance(value, dict):
        return not PRIVATE_KEYS.intersection(value) and all(summary_only(item) for item in value.values())
    if isinstance(value, list):
        return all(summary_only(item) for item in value)
    return True


def finite_number(text, minimum, maximum):
    result = float(text)
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise argparse.ArgumentTypeError("허용 범위의 유한한 수가 필요합니다")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--seconds", type=lambda value: finite_number(value, .5, 3600), default=120)
    parser.add_argument("--speed", type=lambda value: finite_number(value, .05, 4), default=.25)
    parser.add_argument("--loop", action="store_true", help="명시적으로 선택한 경우에만 합성 재생 반복")
    parser.add_argument("--input", type=Path, default=ROOT / "samples" / "target_tracking.synthetic.jsonl")
    parser.add_argument("--mounting-config", type=Path, default=ROOT / "config" / "dual_uwb.d5.example.json")
    parser.add_argument("--uwb-config", type=Path, default=ROOT / "config" / "uwb_estimation.synthetic.json")
    parser.add_argument("--tracking-config", type=Path, default=ROOT / "config" / "target_tracking.synthetic.json")
    parser.add_argument("--output", type=Path, help="선택한 새 JSON 파일에만 좌표 없는 검사 집계 저장")
    parser.add_argument("--plan-only", action="store_true", help="서버 없이 CUA 검사 계획만 출력")
    args = parser.parse_args(argv)
    if not 0 <= args.port <= 65535:
        parser.error("포트 범위는 0~65535입니다")
    if args.output is not None and args.output.exists():
        parser.error("기존 검사 보고서는 덮어쓰지 않습니다")
    if args.plan_only:
        print(json.dumps({"browser_checks_run": False, "browser_checks": BROWSER_CHECKS}, ensure_ascii=False))
        return 0

    from smartcart_radar.core import ScanStore
    from smartcart_sensing.server import SensingHTTPServer
    from smartcart_sensing.service import SensingStore
    from smartcart_sensing.uwb import UwbStore
    from smartcart_tracking.config import load_tracking_config
    from smartcart_tracking.service_runtime import replay_loop
    from smartcart_tracking.view import TrackingStore
    from smartcart_uwb.config import load_config
    from smartcart_uwb.estimation_config import load_estimation_config

    mounting = load_config(args.mounting_config)
    uwb_settings = load_estimation_config(args.uwb_config)
    settings = load_tracking_config(args.tracking_config)
    lidar = ScanStore("replay")
    uwb = UwbStore("off", stop=lidar.stop)
    tracking = TrackingStore(mounting, settings, mode="replay", stop=lidar.stop)
    store = SensingStore(lidar, uwb, mode="replay", tracking=tracking)
    server = SensingHTTPServer(("127.0.0.1", args.port), store, ROOT / "web")
    url = f"http://127.0.0.1:{server.server_port}/"
    workers = [threading.Thread(target=server.serve_forever, kwargs={"poll_interval": .05}, daemon=True),
               threading.Thread(target=replay_loop,
                                args=(lidar, tracking, mounting, uwb_settings, settings, args.input),
                                kwargs={"speed": args.speed, "loop": args.loop}, daemon=True)]
    checks, states, track_states = [], set(), set()
    errors = []

    def check(label, condition):
        if not condition:
            raise AssertionError(label)
        checks.append(label)

    def get_json(path):
        with urlopen(url.rstrip("/") + path, timeout=3) as response:
            return json.load(response, parse_constant=lambda _: (_ for _ in ()).throw(ValueError("비유한 JSON")))

    def stream_check():
        connection = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=3)
        try:
            connection.request("GET", "/api/tracking/events?hz=10")
            response = connection.getresponse()
            check("실제 추적 SSE 연결", response.status == 200)
            event = None
            for _ in range(20):
                line = response.readline()
                if line.startswith(b"event: "):
                    event = line[7:].strip()
                elif line.startswith(b"data: "):
                    value = json.loads(line[6:])
                    check("추적 SSE 이벤트·재생 출처", event == b"tracking" and value["source_mode"] == "replay")
                    return
                elif not line:
                    break
            raise AssertionError("추적 SSE 데이터가 없습니다")
        finally:
            connection.close()

    report = None
    try:
        for worker in workers:
            worker.start()
        with urlopen(url, timeout=3) as response:
            check("실제 동일 출처 HTTP GUI", response.status == 200)
            check("인라인 스크립트 없는 CSP", "script-src 'self'" in response.headers.get("Content-Security-Policy", ""))
        stream_check()
        frame = get_json("/api/frame")
        check("기존 LiDAR 봉투와 추적 상태 공존", frame["schema"] == "smartcart.envelope.v1" and "tracking" in frame)
        health = get_json("/api/health")
        check("health에서 위치·태그·군집 제외", summary_only(health["tracking"]))
        print(f"CUA 브라우저 검사 URL: {url}", flush=True)
        print("브라우저 조작·시각 판정은 아직 수행하지 않았습니다.", flush=True)
        print(json.dumps({"browser_checks": BROWSER_CHECKS}, ensure_ascii=False), flush=True)
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            value = get_json("/api/tracking")
            if (value["schema"] != "smartcart.tracking_view.v1" or value["source_mode"] != "replay"
                    or value["drive_authorized"] is not False or value["provisional"] is not True):
                raise AssertionError("재생·읽기 전용 추적 봉투가 바뀌었습니다")
            states.add(value["state"])
            result = value.get("result")
            if result is not None:
                track_states.add(result["track"]["state"])
            if value["state"] == "error":
                raise AssertionError("통합 합성 재생 작업자가 오류 상태입니다")
            lidar.stop.wait(.1)
        checks.append("실제 재생 작업자·HTTP 출처와 읽기 전용 상태 유지")
    except KeyboardInterrupt:
        checks.append("사용자가 검사 서버 종료 요청")
    except (AssertionError, ValueError, OSError) as error:
        # 위치·태그·원시 입력을 예외 로그에 복사하지 않는다.
        errors.append(type(error).__name__)
    finally:
        lidar.stop.set()
        server.shutdown()
        server.server_close()
        for worker in workers:
            if worker.ident is not None:
                worker.join(timeout=4)
        if any(worker.is_alive() for worker in workers):
            errors.append("worker_not_stopped")
        report = {"schema": "smartcart.tracking_http_harness.v1", "source_mode": "replay",
                  "drive_authorized": False, "http_checks": checks, "http_checks_passed": len(checks),
                  "view_states_seen": sorted(states), "track_states_seen": sorted(track_states),
                  "errors": errors, "browser_checks_run": False, "browser_checks": BROWSER_CHECKS,
                  "not_tested": ["실제 브라우저 DOM·Canvas·반응형", "실물 센서·모터", "Pi ARM64 성능"]}
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as handle:
            json.dump(report, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
    print(json.dumps(report, ensure_ascii=False), flush=True)
    return int(bool(errors))


if __name__ == "__main__":
    raise SystemExit(main())
