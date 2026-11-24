#!/usr/bin/env python3
"""Use the existing virtual UI with the existing GPIO drive backend."""
import argparse
import getpass
import importlib.util
import ipaddress
from pathlib import Path
import auto8081 as integration

ROOT = Path(__file__).resolve().parent
VIRTUAL = ROOT/'smartcart-virtual-4digit.DlQ3Pf7U'

UI = r'''<script>
(()=>{'use strict';let auto=false,follow=false,avoid=false,button,status,latest;
const Native=window.WebSocket;
function off(){auto=false;paint()}
function stop(){off();window.SmartCartManual?.stop()}
function paint(){for(const [id,value] of [['simAuto',auto],['simFollow',follow],['simAvoid',avoid]]){
const b=document.getElementById(id);if(b){b.setAttribute('aria-pressed',String(value));b.querySelector('span').textContent=value?'ON':'OFF'}}}
window.WebSocket=class extends Native{
send(raw){let d;try{d=JSON.parse(raw)}catch{}
if(d?.type==='stop')off();
if(d?.type==='drive'&&auto){if(window.SmartCartManual?.pressed.length)off();
else{d.type='auto';raw=JSON.stringify(d)}}
return super.send(raw)}
constructor(...args){super(...args);this.addEventListener('message',e=>{try{
const d=JSON.parse(e.data);if(!d.autonomy)return;latest=d;
if(!d.armed)off();if(status)status.textContent=(d.hardware?'GPIO':'MOCK')+' / '+d.autonomy.reason;
const canvas=document.getElementById('simCanvas');if(canvas){const c=canvas.getContext('2d');
c.fillStyle='#101a23';c.fillRect(0,0,canvas.width,canvas.height);c.strokeStyle='#70e6c8';c.beginPath();
c.moveTo(canvas.width/2,canvas.height-15);for(const [x,y] of d.autonomy.path||[])c.lineTo(canvas.width/2-y*35,canvas.height-15-x*35);c.stroke();
c.fillStyle='white';c.fillText('Local path / body frame',8,15)}}catch{}})}};
window.addEventListener('DOMContentLoaded',()=>{
const $=id=>document.getElementById(id);
button=$('simAuto');status=$('simStatus');
if(!button||!$('simFollow')||!$('simAvoid')){window.SmartCartManual?.stop();return}
$('simSource').value='live';$('simSource').disabled=true;
$('simAvoid').disabled=true;$('simAvoid').firstChild.textContent='LiDAR 미사용 ';
for(const id of ['simDistance','simSpeed','simRadius'])$(id).disabled=true;
$('simDistance').value=2;$('simDistanceValue').textContent='2.00';
$('simRadius').value=1;$('simRadiusValue').textContent='1.00';
$('simSpeedValue').textContent='uncalibrated';
$('autonomyHelp').textContent='UWB 단독 시험 · LiDAR 장애물 회피 없음 · 차체 기준 추종거리 2m / PWM 상한 30 · 실측 속도 미보정. 먼저 바퀴를 띄워 방향 확인.';
button.onclick=()=>{if(auto){stop();return}
if(!window.SmartCartManual?.enabled){status.textContent='먼저 조이스틱 연결 → Pi 센싱 SOURCE → 주행 활성화';return}
if(!follow){status.textContent='사용자 추종을 먼저 켜세요';return}
auto=true;paint()};
$('simFollow').onclick=()=>{follow=!follow;if(!follow)stop();paint()};
$('simAvoid').onclick=()=>{avoid=!avoid;if(!avoid)stop();paint()};
$('simReset').onclick=()=>{follow=avoid=false;stop();paint()};
window.addEventListener('blur',off);window.addEventListener('pagehide',off);
document.addEventListener('visibilitychange',()=>{if(document.hidden)off()});paint();
});})();</script>'''

def configure_ui():
    html = (VIRTUAL/'web/index.html').read_text(encoding='utf-8')
    expected = '<script src="js/virtual_autonomy.js" defer></script>'
    if html.count(expected) != 1 or '<head>' not in html:
        raise ValueError('Virtual UI version differs; refusing to replace its controls')
    for name in ('simAuto','simFollow','simAvoid','simStatus','simSource','simCanvas','autonomyHelp'):
        if f'id="{name}"' not in html:
            raise ValueError('Missing UI control: '+name)
    # Remove the browser simulation controller. Keep the original visual layout.
    html = html.replace(expected,'').replace('<head>','<head>'+UI,1)
    integration.original.ROOT = VIRTUAL/'web'
    async def page(request):
        return integration.web.Response(text=html,content_type='text/html')
    integration.original.page = page
    return html

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--address',required=True)
    parser.add_argument('--backend',choices=('gpio','mock'),default='gpio')
    parser.add_argument('--cert',required=True)
    parser.add_argument('--key',required=True)
    args=parser.parse_args()
    address=ipaddress.IPv4Address(args.address)
    if address not in ipaddress.IPv4Network('192.168.43.0/24') or address.packed[-1] in (0,255):
        parser.error('Existing screen access is restricted to 192.168.43.1~254')
    configure_ui()
    spec=importlib.util.spec_from_file_location('virtual_login',VIRTUAL/'virtual_lan_server.py')
    login=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(login)
    password=getpass.getpass('화면 접속키 (숫자 4자리): ')
    if not login.valid_pin(password) or password!=getpass.getpass('화면 접속키 재입력: '):
        raise SystemExit('접속키 불일치')
    make=integration.original.make_app
    def guarded(*a,**kw):
        app=make(*a,**kw)
        app.middlewares.insert(0,login.make_guard(str(address),8081,password))
        return app
    integration.original.make_app=guarded
    return integration.original.main(['--backend',args.backend,'--host',str(address),'--port','8081','--cert',args.cert,'--key',args.key])

if __name__=='__main__':
    raise SystemExit(main())
