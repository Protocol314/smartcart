#!/usr/bin/env python3
"""Native DOM/Canvas responsive tests in an isolated document.

EventSource and download navigation are intercepted. This deliberately does NOT
claim browser HTTP, completed downloads, storage persistence or hardware tests.
Requires development-only `playwright` and a Chromium binary.
"""
import argparse
import base64
import json
from pathlib import Path
import re
import shutil
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from smartcart_radar.core import ScanStore
from smartcart_radar.sources import synthetic_scan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--chromium', default=shutil.which('chromium'))
    parser.add_argument('--output', default=str(ROOT/'validation-output'))
    args = parser.parse_args()
    if not args.chromium:
        raise SystemExit('Specify --chromium /path/to/chromium')
    from playwright.sync_api import sync_playwright
    output = Path(args.output); output.mkdir(parents=True, exist_ok=True)
    web = ROOT/'web'
    html = (web/'index.html').read_text()
    html = re.sub(r'<script[^>]+src=[^>]+></script>', '', html)
    html = re.sub(r'<link[^>]+>', '', html)
    icon = base64.b64encode((web/'assets/icon.svg').read_bytes()).decode()
    html = html.replace('src="assets/icon.svg"', f'src="data:image/svg+xml;base64,{icon}"')
    results, viewports = [], []
    def check(label, value):
        if not value:
            raise AssertionError(label)
        results.append(label); print('PASS', label, flush=True)
    with sync_playwright() as pw:
        browser = pw.chromium.launch(executable_path=args.chromium, headless=True, args=['--no-sandbox','--disable-dev-shm-usage'])
        page = browser.new_page(viewport={'width':1440, 'height':900}, device_scale_factor=1)
        errors = []; page.on('pageerror', lambda e: errors.append(str(e)))
        page.set_content(html); page.add_style_tag(path=str(web/'css/app.css'))
        page.evaluate('''() => {
          window.__sources=[]; window.__downloads=[]; window.__blobs=[];
          window.EventSource=class {
            constructor(url){this.url=url;this.handlers={};this.closed=false;window.__sources.push(this);}
            addEventListener(name,handler){this.handlers[name]=handler;}
            close(){this.closed=true;}
            emit(value){if(!this.closed&&this.handlers.frame)this.handlers.frame({data:JSON.stringify(value)});}
            fail(){if(!this.closed&&this.onerror)this.onerror();}
          };
          const create=URL.createObjectURL.bind(URL);
          URL.createObjectURL=blob=>{window.__blobs.push(blob);return create(blob);};
          const click=HTMLAnchorElement.prototype.click;
          HTMLAnchorElement.prototype.click=function(){
            if(this.download)window.__downloads.push({name:this.download,url:this.href});else click.call(this);
          };
        }''')
        for name in ['core.js','sample.js','sensing.js','radar.js','app.js']:
            page.add_script_tag(path=str(web/'js'/name))
        check('no fabricated data at startup',page.locator('#visibleCount').inner_text()=='—' and page.evaluate('SmartCartDiagnostics.points')==0)
        check('pure black background',page.evaluate('getComputedStyle(document.body).backgroundColor')=='rgb(0, 0, 0)')
        check('only actual RETURNS count, no OBJECTS estimate', 'OBJECTS' not in page.locator('body').inner_text())
        check('old instruction banners and help cards removed',page.locator('.help,.summary-grid,.source-switch').count()==0)
        check('settings dialog initially closed',not page.locator('#settingsDialog').is_visible())
        page.select_option('#source','sample'); page.wait_for_timeout(250)
        check('sample automatically zooms to .25m',page.locator('#radius').input_value()=='0.25')
        check('partial sample label is always visible','PARTIAL 128 / 720' in page.locator('#coverage').inner_text())
        check('sample keeps 56 real returns',page.locator('#visibleCount').inner_text()=='56')
        check('sample retains only adjacency guides',0<page.evaluate('SmartCartDiagnostics.links')<55)
        check('sample cannot be LIVE',page.locator('#statusBadge').inner_text()=='SAMPLE')
        page.click('#openSettings')
        page.locator('#diagnostics summary').click()
        check('diagnostics show original 128 / 720',page.locator('#rayCount').inner_text()=='128 / 720')
        check('diagnostics show 40 missing, 32 below minimum',page.locator('#invalidCount').inner_text()=='40' and page.locator('#belowCount').inner_text()=='32')
        page.locator('#diagnostics summary').click()
        page.click('#saveJson'); page.wait_for_timeout(100)
        capture=page.evaluate('async()=>JSON.parse(await window.__blobs.at(-1).text())')
        check('export retains all original raw positions',len(capture['scan']['ranges'])==128 and capture['scan']['ranges'][9] is None)
        check('export has version and sample provenance',capture['version']=='1.0.1' and capture['scan']['source']['kind']=='sample')
        page.click('#savePng'); page.wait_for_timeout(150)
        check('merged PNG blob contains all layers',page.evaluate('window.__blobs.at(-1).type==="image/png" && window.__blobs.at(-1).size>1000'))
        check('two functional download requests',page.evaluate('window.__downloads.length')==2)
        page.click('#closeSettings');page.screenshot(path=str(output/'sample-desktop.png'),full_page=True)
        page.select_option('#source','demo');page.wait_for_timeout(650)
        check('demo resets display to 5m',page.locator('#radius').input_value()=='5')
        check('demo visibly nonmeasurement','비실측' in page.locator('#plotLabel').inner_text() and page.locator('#statusBadge').inner_text()=='DEMO')
        check('demo renders dense contour, not object centroids',page.evaluate('SmartCartDiagnostics.points>500&&SmartCartDiagnostics.links>450'))
        check('sector values remain white',page.evaluate("getComputedStyle(document.querySelector('#sectorFront')).color")=='rgb(243, 243, 243)')
        check('all three range colors actually rasterized',page.evaluate('''()=>{
          const c=document.querySelector('#radar'),d=c.getContext('2d').getImageData(0,0,c.width,c.height).data;
          const found=[false,false,false];
          for(let i=0;i<d.length;i+=4){if(d[i+3]<180)continue;
            if(d[i]>220&&d[i+1]<120&&d[i+2]<120)found[0]=true;
            if(d[i]>220&&d[i+1]>180&&d[i+2]<120)found[1]=true;
            if(d[i]<100&&d[i+1]>170&&d[i+2]>80)found[2]=true;
          }return found.every(Boolean);
        }'''))
        before=page.evaluate('({a:SmartCartDiagnostics.renderedFrames,g:SmartCartDiagnostics.renderer.gridBuilds,s:SmartCartDiagnostics.renderer.scanBuilds})')
        page.wait_for_timeout(800)
        after=page.evaluate('({a:SmartCartDiagnostics.renderedFrames,g:SmartCartDiagnostics.renderer.gridBuilds,s:SmartCartDiagnostics.renderer.scanBuilds})')
        check('animation independent of 2Hz data refresh',after['a']-before['a']>=3)
        check('static grid not rebuilt during animation',before['g']==after['g'])
        check('return layer rebuilds on scan only',after['s']-before['s']<=3)
        page.click('#pauseView'); old=page.evaluate('SmartCartDiagnostics.scan.ranges')
        page.wait_for_timeout(700)
        check('pause freezes data rather than scanner',old==page.evaluate('SmartCartDiagnostics.scan.ranges') and page.locator('#statusBadge').inner_text()=='PAUSED')
        page.click('#pauseView');page.wait_for_timeout(200)
        check('resume displays newest pending frame',old!=page.evaluate('SmartCartDiagnostics.scan.ranges'))
        page.select_option('#displayHz','1')
        check('one Hz is a functional setting',page.evaluate('SmartCartDiagnostics.settings.displayHz')==1)
        page.select_option('#displayHz','2')
        page.emulate_media(reduced_motion='reduce');page.wait_for_timeout(150)
        frames=page.evaluate('SmartCartDiagnostics.renderedFrames');page.wait_for_timeout(500)
        check('reduced motion stops sweep only',page.evaluate('SmartCartDiagnostics.renderedFrames')==frames and page.locator('#visibleCount').inner_text()!='—')
        page.emulate_media(reduced_motion='no-preference');page.wait_for_timeout(100)
        page.click('#openSettings');page.fill('#yaw','90');page.fill('#offsetX','0.2');page.fill('#maxGap','0.1');page.click('#applySettings')
        check('mount and contour settings applied',page.evaluate('SmartCartDiagnostics.settings.yaw===90 && SmartCartDiagnostics.settings.offsetX===.2 && SmartCartDiagnostics.settings.maxGap===.1'))
        page.click('#openSettings');page.fill('#nearM','4');page.fill('#farM','2');page.click('#applySettings')
        check('invalid color thresholds are rejected',page.locator('#settingsDialog').is_visible() and page.evaluate('SmartCartDiagnostics.settings.nearM')==1)
        page.click('#resetSettings');page.click('#closeSettings')
        check('reset restores new v1.0.1 settings',page.evaluate('SmartCartDiagnostics.settings.yaw===0 && SmartCartDiagnostics.settings.maxGap===.2 && SmartCartDiagnostics.settings.nearM===1'))
        # Feed software fixtures through the actual EventSource handler, never claim hardware.
        page.select_option('#source','live');store=ScanStore('ros');fixture=synthetic_scan(0)
        fixture['source']={'kind':'ros','label':'SOFTWARE TEST FIXTURE'}
        def emit(state=None):
            store.publish(fixture); e=store.snapshot()
            if state:e.update(state)
            page.evaluate('value=>window.__sources.at(-1).emit(value)',e)
        emit()
        check('ROS handler displays incoming geometry',page.locator('#statusBadge').inner_text()=='LIVE' and page.evaluate('SmartCartDiagnostics.points>500'))
        page.select_option('#displayHz','1')
        check('Hz selection changes real SSE request path',page.evaluate('window.__sources.at(-1).url.endsWith("hz=1")'))
        check('old SSE subscriptions are closed',page.evaluate('window.__sources.slice(0,-1).every(s=>s.closed)'))
        page.select_option('#displayHz','2');emit();page.wait_for_timeout(1750)
        check('1.5s stale timeout without another event',page.locator('#statusBadge').inner_text()=='STALE')
        check('stale distances and counts hidden',page.locator('#visibleCount').inner_text()=='—' and page.locator('#sectorFront').inner_text()=='—')
        check('stale scan canvas is physically cleared',page.evaluate('''()=>{const c=document.querySelector('#radar'),d=c.getContext('2d').getImageData(0,0,c.width,c.height).data;for(let i=3;i<d.length;i+=4)if(d[i])return false;return true;}'''))
        emit();check('fresh recovery works',page.evaluate('SmartCartDiagnostics.active'))
        page.evaluate('window.__sources.at(-1).fail()')
        check('disconnect never falls back to demo',page.locator('#statusBadge').inner_text()=='OFFLINE' and page.evaluate('SmartCartDiagnostics.mode')=='live')
        check('disconnect hides contours too',not page.evaluate('SmartCartDiagnostics.renderer.view.fresh'))
        emit({'source_mode':'replay','state':'replay'})
        check('server replay cannot claim LIVE',page.locator('#statusBadge').inner_text()=='REPLAY')
        page.evaluate('window.__sources.at(-1).emit({schema:"bad"})')
        check('malformed input disables displayed measurements',page.locator('#statusBadge').inner_text()=='ERROR' and not page.evaluate('SmartCartDiagnostics.active'))
        emit();check('input error recovers with valid envelope',page.locator('#statusBadge').inner_text()=='LIVE')
        page.locator('#fileInput').set_input_files(str(ROOT/'samples/user_scan_truncated.json'));page.wait_for_timeout(200)
        check('native File API works for saved JSON',page.evaluate('SmartCartDiagnostics.mode')=='file' and '128 / 720' in page.locator('#coverage').inner_text())
        check('file loading does not label data LIVE',page.locator('#statusBadge').inner_text()=='REPLAY STOP')
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'fixture.jsonl';p.write_text('\n'.join(json.dumps(synthetic_scan(i*.1)) for i in range(12)))
            page.locator('#fileInput').set_input_files(str(p));page.wait_for_timeout(200)
            check('JSONL file has twelve real frames',page.locator('#replayPosition').get_attribute('max')=='11')
            page.click('#replayPlay');page.wait_for_timeout(1750)
            check('JSONL playback reaches last frame',page.locator('#replayPosition').input_value()=='11')
            page.locator('#replayPosition').fill('3');page.wait_for_timeout(100)
            check('scrubbing selects exactly one recorded frame',page.locator('#replayLabel').inner_text()=='4 / 12')
        page.select_option('#source','demo');page.wait_for_timeout(600)
        for width,height in [(320,568),(360,800),(390,844),(414,896),(768,1024),(820,1180),(844,390),(1024,768),(1280,800),(1440,900),(1920,1080)]:
            page.set_viewport_size({'width':width,'height':height});page.wait_for_timeout(130)
            box=page.locator('#radar').bounding_box()
            check(f'{width}x{height} no horizontal overflow',page.evaluate('document.documentElement.scrollWidth<=innerWidth'))
            check(f'{width}x{height} square radar fits viewport',abs(box['width']-box['height'])<1 and box['x']>=0 and box['x']+box['width']<=width+1)
            viewports.append({'width':width,'height':height,'canvas_css':box})
            if width in (390,768,1440):page.screenshot(path=str(output/f'demo-{width}.png'),full_page=True)
        page.set_viewport_size({'width':320,'height':568});page.click('#openSettings');page.wait_for_timeout(150)
        d=page.locator('#settingsDialog').bounding_box()
        check('320px settings modal fits and scrolls',d['x']>=0 and d['x']+d['width']<=320 and d['height']<=568)
        check('no horizontal overflow inside mobile dialog',page.evaluate('document.querySelector("#settingsDialog").scrollWidth<=document.querySelector("#settingsDialog").clientWidth'))
        page.screenshot(path=str(output/'settings-320.png'),full_page=True);page.click('#closeSettings')
        check('no JavaScript page exceptions',errors==[])
        report={'version':'1.0.1','harness':'native Chromium DOM/Canvas; injected EventSource; native File/Blob APIs; download navigation intercepted',
                'passed':len(results),'checks':results,'viewports':viewports,'page_errors':errors,
                'not_tested':['real browser HTTP/SSE or file:// navigation','native localStorage persistence','native completed downloads','actual mobile/tablet devices','Raspberry Pi ROS/DDS/C1 hardware']}
        (output/'browser-results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        browser.close()
    print(f'Browser checks: {len(results)} passed')


if __name__=='__main__':main()
