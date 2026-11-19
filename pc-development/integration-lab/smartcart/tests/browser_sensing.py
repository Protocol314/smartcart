#!/usr/bin/env python3
"""실제 HTTP/SSE와 Edge Canvas로 통합 GUI를 검증한다. 실제 센서 검수가 아니다."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from smartcart_radar.core import ScanStore
from smartcart_radar.sources import synthetic_scan
from smartcart_sensing.server import SensingHTTPServer
from smartcart_sensing.service import SensingStore
from smartcart_sensing.uwb import UwbStore, synthetic_line


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--chromium", default=shutil.which("chromium") or
                        r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe")
    parser.add_argument("--output", default=str(ROOT / "validation-output" / "sensing"))
    args = parser.parse_args()
    if not Path(args.chromium).is_file():
        raise SystemExit("--chromium으로 Chromium 또는 Edge 실행 파일을 지정하십시오")
    from playwright.sync_api import sync_playwright

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    lidar = ScanStore("demo", 1.5)
    uwb = UwbStore("demo", .5, lidar.stop)
    store = SensingStore(lidar, uwb, "demo")
    server = SensingHTTPServer(("127.0.0.1", 0), store, ROOT / "web")
    url = f"http://127.0.0.1:{server.server_port}/"
    lidar_enabled, uwb_enabled = threading.Event(), threading.Event()
    lidar_enabled.set()
    uwb_enabled.set()
    producer_errors = []

    def produce():
        start, index = time.monotonic(), 0
        try:
            while not store.stop.is_set():
                elapsed = time.monotonic() - start
                if lidar_enabled.is_set():
                    lidar.publish(synthetic_scan(elapsed))
                if uwb_enabled.is_set():
                    uwb.publish_line(synthetic_line(elapsed, index))
                index += 1
                store.stop.wait(.1)
        except Exception as error:
            producer_errors.append(str(error))

    workers = [threading.Thread(target=server.serve_forever, kwargs={"poll_interval": .1}, daemon=True),
               threading.Thread(target=produce, daemon=True)]
    for worker in workers:
        worker.start()
    results, requests, viewports, page_errors = [], [], [], []

    def check(label, condition):
        if not condition:
            raise AssertionError(label)
        results.append(label)
        print("통과", label, flush=True)

    def wait(page, predicate):
        page.wait_for_function("() => (" + predicate + ")", timeout=7000)

    try:
        with sync_playwright() as pw:
            browser = pw.chromium.launch(executable_path=args.chromium, headless=True,
                                         args=["--no-sandbox", "--disable-dev-shm-usage"])
            page = browser.new_page(viewport={"width": 1440, "height": 1000}, device_scale_factor=1)
            page.on("pageerror", lambda error: page_errors.append(str(error)))
            page.on("request", lambda request: requests.append(request.url))
            response = page.goto(url, wait_until="domcontentloaded")
            wait(page, "window.SmartCartDiagnostics && SmartCartDiagnostics.uwb.state==='synthetic' && SmartCartDiagnostics.points>500")
            check("실제 동일 출처 HTTP로 GUI 로드", response.status == 200)
            check("인라인 스크립트 없는 CSP 적용", "script-src 'self'" in response.headers.get("content-security-policy", "") and "unsafe-inline" not in response.headers.get("content-security-policy", ""))
            check("기존 LiDAR SSE 실제 요청", any("/api/events?hz=2" in request for request in requests))
            check("별도 UWB 10Hz SSE 실제 요청", any("/api/uwb/events?hz=10" in request for request in requests))
            check("한국어 통합 상태와 비실측 배지", "합성" in page.locator("#sensingState").inner_text() and page.locator("#statusBadge").inner_text() == "DEMO")
            check("합성으로 구동 허가를 표시하지 않음", page.evaluate("SmartCartDiagnostics.sensing.ready===false"))
            check("LiDAR 720개 원본 위치와 보수적 윤곽 유지", page.evaluate("SmartCartDiagnostics.scan.ranges.length===720 && SmartCartDiagnostics.links>450"))
            check("UWB 식별과 ESP 상태 수신", page.locator("#uwbAddress").inner_text() == "0x6E19" and page.locator("#uwbFirmware").inner_text() == "WELCOME")
            check("LiDAR와 UWB 수신률·경과 시간 표시", "Hz" in page.locator("#rxHz").inner_text() and "Hz" in page.locator("#uwbRxAge").inner_text() and "ms" in page.locator("#lidarAge").inner_text())
            check("UWB 청록 마름모를 실제 Canvas 픽셀로 표시", page.evaluate("""() => {
                const c=document.querySelector('#radarUwb'),d=c.getContext('2d').getImageData(0,0,c.width,c.height).data;
                let cyan=0;for(let i=0;i<d.length;i+=4)if(d[i]<130&&d[i+1]>180&&d[i+2]>190&&d[i+3]>170)cyan++;
                return cyan>15;
            }"""))
            check("마커는 ROS 전방·좌측 좌표와 동일", page.evaluate("""() => {
                const v=SmartCartDiagnostics.uwb,m=SmartCartDiagnostics.renderer.uwb;
                return m && v.measurement.position.x===m.x && v.measurement.position.y===m.y;
            }"""))
            page.screenshot(path=str(output / "sensing-desktop.png"), full_page=True)

            page.select_option("#displayHz", "1")
            wait(page, "SmartCartDiagnostics.uwb.marker!==null")
            builds_before = page.evaluate("({lidar:SmartCartDiagnostics.renderer.scanBuilds,uwb:SmartCartDiagnostics.renderer.uwbBuilds})")
            for _ in range(8):
                page.wait_for_timeout(150)
                check("LiDAR 1Hz 중 UWB 10Hz 위치 유효 유지", page.evaluate("SmartCartDiagnostics.uwb.marker!==null"))
            builds_after = page.evaluate("({lidar:SmartCartDiagnostics.renderer.scanBuilds,uwb:SmartCartDiagnostics.renderer.uwbBuilds})")
            check("UWB 고속 갱신은 LiDAR 레이어를 재계산하지 않음", builds_after["lidar"] - builds_before["lidar"] <= 3)
            check("UWB 전용 레이어는 독립 갱신", builds_after["uwb"] - builds_before["uwb"] >= 5)
            page.select_option("#displayHz", "2")
            wait(page, "SmartCartDiagnostics.uwb.marker!==null")

            uwb_enabled.clear()
            wait(page, "SmartCartDiagnostics.uwb.state==='stale'")
            check("UWB 수신 중단 500ms 후 마커 제거", page.evaluate("SmartCartDiagnostics.renderer.uwb===null"))
            check("UWB 중단 중 LiDAR 점·윤곽 유지", page.evaluate("SmartCartDiagnostics.active && SmartCartDiagnostics.points>500"))
            check("UWB 만료 좌표·거리 숨김", page.locator("#uwbPosition").inner_text() == "—" and page.locator("#uwbDistance").inner_text() == "—")
            check("만료 UWB 청록 픽셀 제거", page.evaluate("""() => {
                const c=document.querySelector('#radarUwb'),d=c.getContext('2d').getImageData(0,0,c.width,c.height).data;
                for(let i=0;i<d.length;i+=4)if(d[i]<130&&d[i+1]>180&&d[i+2]>190&&d[i+3]>170)return false;return true;
            }"""))
            page.screenshot(path=str(output / "sensing-uwb-stale.png"), full_page=True)
            uwb_enabled.set()
            wait(page, "SmartCartDiagnostics.uwb.state==='synthetic' && SmartCartDiagnostics.renderer.uwb!==null")
            check("UWB 입력 복구로 최신 위치 복귀", page.locator("#uwbDistance").inner_text() != "—")

            lidar_enabled.clear()
            wait(page, "SmartCartDiagnostics.state==='STALE'")
            check("LiDAR 중단 시 점·방향 거리 숨김", page.locator("#visibleCount").inner_text() == "—" and page.locator("#sectorFront").inner_text() == "—")
            check("LiDAR 중단 중 UWB 최신 위치 독립 유지", page.evaluate("SmartCartDiagnostics.uwb.state==='synthetic' && SmartCartDiagnostics.renderer.uwb!==null"))
            page.screenshot(path=str(output / "sensing-lidar-stale.png"), full_page=True)
            lidar_enabled.set()
            wait(page, "SmartCartDiagnostics.active")

            page.click("#pauseView")
            frozen = page.evaluate("SmartCartDiagnostics.renderer.uwb.x")
            page.wait_for_timeout(150)
            check("화면 정지는 UWB도 같은 snapshot 유지", page.evaluate("SmartCartDiagnostics.renderer.uwb.x") == frozen)
            check("화면 정지 안내와 준비 해제", page.locator("#pauseNotice").is_visible() and page.evaluate("SmartCartDiagnostics.sensing.ready===false"))
            wait(page, "SmartCartDiagnostics.uwb.marker===null")
            check("정지 화면의 UWB도 유효 시간 만료", page.evaluate("SmartCartDiagnostics.uwb.state==='stale'"))
            page.click("#pauseView")
            wait(page, "SmartCartDiagnostics.uwb.marker!==null")
            check("화면 재개는 대기 중 최신 UWB 사용", page.evaluate("!SmartCartDiagnostics.paused"))

            page.select_option("#source", "sample")
            check("첨부 샘플에 서버 UWB를 섞지 않음", page.evaluate("SmartCartDiagnostics.uwb.state==='disabled' && SmartCartDiagnostics.renderer.uwb===null"))
            page.wait_for_timeout(250)
            check("샘플 실측점 56개 보존", page.locator("#visibleCount").inner_text() == "56")
            page.select_option("#source", "demo")
            wait(page, "SmartCartDiagnostics.uwb.state==='synthetic'")
            check("로컬 데모 태그는 명확한 합성 표식", page.evaluate("SmartCartDiagnostics.renderer.uwb.synthetic===true") and "비실측" in page.locator("#plotLabel").inner_text())
            page.select_option("#source", "live")
            wait(page, "SmartCartDiagnostics.uwb.marker!==null")
            check("서버 소스 재접속 후 실제 SSE 복구", page.locator("#uwbFirmware").inner_text() == "WELCOME")

            for width, height in [(320, 568), (390, 844), (768, 1024), (844, 390), (1440, 1000)]:
                page.set_viewport_size({"width": width, "height": height})
                page.evaluate("window.scrollTo(0,0)")
                page.wait_for_timeout(200)
                box = page.locator("#radar").bounding_box()
                check(f"{width}×{height} 가로 넘침 없음", page.evaluate("document.documentElement.scrollWidth<=innerWidth"))
                check(f"{width}×{height} 정사각 레이더의 가로폭이 화면 안에 있음", abs(box["width"] - box["height"]) < 1 and box["x"] >= 0 and box["x"] + box["width"] <= width + 1)
                check(f"{width}×{height} 한글 UWB 패널 표시", page.locator("#uwbNotice").inner_text() != "")
                viewports.append({"width": width, "height": height, "canvas_css": box})
                if width in (390, 768):
                    page.screenshot(path=str(output / f"sensing-{width}.png"), full_page=True)
            page.set_viewport_size({"width": 320, "height": 568})
            page.click("#openSettings")
            check("320px 설정 대화상자 가로 넘침 없음", page.evaluate("document.querySelector('#settingsDialog').scrollWidth<=document.querySelector('#settingsDialog').clientWidth"))
            page.screenshot(path=str(output / "sensing-settings-320.png"), full_page=True)
            page.click("#closeSettings")
            check("외부 CDN·분석 서버 요청 없음", all(request.startswith(url) for request in requests))
            check("브라우저 JavaScript 오류 없음", page_errors == [])
            check("합성 생산 스레드 오류 없음", producer_errors == [])
            report = {"version": "0.1.0", "harness": "실제 HTTP와 네이티브 EventSource/Canvas를 사용하는 headless Edge; 입력은 테스트 합성 스레드",
                      "passed": len(results), "checks": results, "viewports": viewports, "page_errors": page_errors,
                      "not_tested": ["실제 Raspberry Pi 4 성능", "ROS 2 DDS 통신", "실제 LiDAR·UWB·ESP32", "실제 모바일 장치", "SSH 실배포"]}
            (output / "browser-sensing-results.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
            browser.close()
    finally:
        store.stop.set()
        server.shutdown()
        server.server_close()
        for worker in workers:
            worker.join(timeout=3)
    print(f"통합 브라우저 검증 {len(results)}개 통과", flush=True)


if __name__ == "__main__":
    main()
