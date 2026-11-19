#!/usr/bin/env python3
"""외부 접속·실물 센서 없이 합성 이벤트와 실제 브라우저/Canvas를 검사한다."""
from pathlib import Path
import argparse
import json
from urllib.parse import urlparse

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]
ASSETS = {
    '/': ('index.html', 'text/html'), '/index.html': ('index.html', 'text/html'),
    '/css/app.css': ('css/app.css', 'text/css'), '/assets/icon.svg': ('assets/icon.svg', 'image/svg+xml'),
}
for name in ('core', 'sample', 'sensing', 'dual_uwb', 'tracking', 'field_preview', 'radar', 'app'):
    ASSETS['/js/' + name + '.js'] = ('js/' + name + '.js', 'text/javascript')

MOCK = """
window.__sources = [];
window.EventSource = class {
  constructor(url) { this.url = url; this.listeners = {}; window.__sources.push(this); }
  addEventListener(name, fn) { this.listeners[name] = fn; }
  close() { this.closed = true; }
  emit(name, data) { if (!this.closed && this.listeners[name]) this.listeners[name]({data: JSON.stringify(data)}); }
};
"""

SYNTHETIC = """
() => {
  const common = {frame_id:'base_link',source_mode:'live',drive_authorized:false,provisional:true};
  const cluster = {cluster_id:0,centroid_xy_m:[2.2,0],radius_m:0.1,width_m:0.2,point_count:8,
    first_ray:1,last_ray:8,error_radius_m:0.15};
  const tracking = {schema:'smartcart.tracking_view.v1',mode:'ros',source_mode:'live',state:'running',sequence:1,
    age_ms:0,remaining_ms:0,cluster_remaining_ms:300,prediction_remaining_ms:0,uwb_remaining_ms:0,
    valid:false,drive_authorized:false,provisional:true,frame_id:'base_link',
    lidar_transform:{yaw_deg:0,translation_m:[0,0,0]},result:{
      uwb_estimate:{...common,schema:'smartcart.uwb_tag_estimate.v1',valid:false,invalid_reason:'range_model_unknown',position_xy_m:null},
      lidar_clusters:{...common,schema:'smartcart.lidar_clusters.v1',valid:true,invalid_reason:null,clusters:[cluster]},
      track:{...common,schema:'smartcart.target_track.v1',state:'waiting',valid:false,invalid_reason:'uwb_invalid',
        track_id:null,position_xy_m:null,velocity_xy_mps:null,predicted_position_xy_m:null,error_radius_m:null,
        confirmation_count:0,candidate_count:0,associated_cluster_id:null}}};
  const d=Math.hypot(2,.225), a=Math.atan2(.225,2)*180/Math.PI;
  const sensor={state:'live',connected:true,valid:true,invalid_reason:null,measurement_valid:true,id_valid:true,
    range_m:d,raw_angle_deg:a,range_out_of_bounds:false,angle_out_of_bounds:false,age_ms:10,remaining_ms:490,sample_sequence:1};
  const dual={schema:'smartcart.dual_uwb_view.v1',source_mode:'live',state:'running',sequence:1,age_ms:0,
    remaining_ms:750,valid:true,received:true,drive_authorized:false,
    sensors:{uwb_left:{...sensor},uwb_right:{...sensor,raw_angle_deg:-a}}};
  const scan={...RadarCore.syntheticScan(0),sequence:1,source:{kind:'ros'}};
  const frame={schema:'smartcart.envelope.v1',source_mode:'ros',state:'live',age_ms:0,stale_after_ms:1500,
    receive_hz:10,scan,tracking,sensing:{schema:'smartcart.sensing.v1',uwb_backend:'dual'},dual_uwb:dual};
  window.__synthetic = {frame,dual,tracking};
  window.__sources.find(s => s.url.startsWith('/api/events')).emit('frame',frame);
  window.__tick = setInterval(() => {
    frame.scan.sequence++; tracking.sequence++; dual.sequence++;
    dual.sensors.uwb_left.sample_sequence++; dual.sensors.uwb_right.sample_sequence++;
    window.__sources.find(s => s.url.startsWith('/api/events') && !s.closed).emit('frame',frame);
    window.__sources.find(s => s.url.startsWith('/api/tracking/events') && !s.closed).emit('tracking',tracking);
    window.__sources.find(s => s.url.startsWith('/api/dual-uwb/events') && !s.closed).emit('dual_uwb',dual);
  },80);
}
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--chromium', default=r'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results, errors, requests = [], [], []
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(executable_path=args.chromium, headless=True)
        page = browser.new_page(viewport={'width': 1440, 'height': 1000}, device_scale_factor=1)
        page.on('pageerror', lambda error: errors.append(str(error)))

        def route(request):
            url = urlparse(request.request.url)
            requests.append(request.request.url)
            asset = ASSETS.get(url.path) if url.scheme == 'http' and url.netloc == 'smartcart.test' else None
            if asset is None:
                request.abort()
                return
            name, mime = asset
            request.fulfill(status=200, content_type=mime, body=(ROOT / 'web' / name).read_bytes())

        page.route('**/*', route)
        page.add_init_script(MOCK)
        page.goto('http://smartcart.test/', wait_until='domcontentloaded')
        page.evaluate(SYNTHETIC)
        page.wait_for_function("document.querySelector('#fieldPreviewState').textContent.includes('인접 군집 1개')")

        def check(label, condition):
            if not condition:
                raise AssertionError(label)
            results.append(label)
            print('통과', label, flush=True)

        check('실험 패널과 불확실성 표시', page.locator('#fieldPreviewPanel').is_visible() and
              '검수 전' in page.locator('#fieldPreviewPanel').inner_text())
        check('좌우 가정 평균과 유일 인접 군집',
              '2.10' in page.locator('#fieldPreviewPosition').inner_text() and
              page.locator('#fieldPreviewCandidates').inner_text() == '1개')
        check('화면 후보와 공식 추적·구동 분리', page.evaluate("""() => {
          const d=SmartCartDiagnostics,r=d.renderer.tracking.fieldPreview;
          return r.average && r.candidate && !r.driveAuthorized && d.tracking.selected===null && d.tracking.driveAuthorized===false;
        }"""))
        check('실제 Canvas에 가정 표시 픽셀 출력', page.evaluate("""() => {
          const c=document.querySelector('#radarTracking'), p=c.getContext('2d').getImageData(0,0,c.width,c.height).data;
          let n=0; for(let i=0;i<p.length;i+=4) if(p[i]>180&&p[i+1]>120&&p[i+1]<230&&p[i+2]<180&&p[i+3]>100)n++;
          return n>10;
        }"""))
        page.screenshot(path=str(args.output / 'field-preview-desktop.png'), full_page=True)
        page.wait_for_timeout(300)
        page.evaluate("""() => {__synthetic.dual.sensors.uwb_left.range_m=4;
          __synthetic.dual.sensors.uwb_right.range_m=4;}""")
        page.wait_for_timeout(100)
        check('실제 앱의 짧은 거리 이상치는 표시 평균에 크게 반영하지 않음',
              page.evaluate("""() => {
                const p=SmartCartDiagnostics.renderer.tracking.fieldPreview;
                return p && p.average && p.average.x<3 && p.displaySmoothed===true;
              }"""))
        page.evaluate("""() => {Object.assign(__synthetic.dual.sensors.uwb_left,{range_m:1.44,raw_angle_deg:-15});
          Object.assign(__synthetic.dual.sensors.uwb_right,{range_m:1.45,raw_angle_deg:12}); }""")
        page.wait_for_function("document.querySelector('#fieldPreviewState').textContent.includes('불일치')")
        check('과거에 관측된 형태의 불일치는 평균 후보 연관을 거부', page.evaluate("""() => {
          const r=SmartCartDiagnostics.renderer.tracking.fieldPreview;
          return r.disagreementAverage && r.average===null && r.candidate===null;
        }"""))
        page.screenshot(path=str(args.output / 'field-preview-disagreement.png'), full_page=True)
        page.uncheck('#fieldPreviewEnabled')
        check('실험 표시 끄기는 마커 제거', page.evaluate("""() => {
          const r=SmartCartDiagnostics.renderer.tracking.fieldPreview;
          return r.state==='off' && r.left===null && r.disagreementAverage===null;
        }"""))
        page.check('#fieldPreviewEnabled')
        page.set_viewport_size({'width': 390, 'height': 844})
        check('모바일 폭 가로 넘침 없음', page.evaluate('document.documentElement.scrollWidth<=innerWidth'))
        page.screenshot(path=str(args.output / 'field-preview-mobile.png'), full_page=True)
        check('외부 서비스 요청 없음', all(url.startswith('http://smartcart.test/') for url in requests))
        check('브라우저 스크립트 오류 없음', not errors)
        page.evaluate('clearInterval(window.__tick)')
        browser.close()
    (args.output / 'results.json').write_text(json.dumps({'checks': results, 'page_errors': errors,
        'source': '합성 이벤트', 'physical_test': False}, ensure_ascii=False, indent=2), encoding='utf-8')
    print('브라우저 검사 %d개 통과' % len(results))


if __name__ == '__main__':
    main()
