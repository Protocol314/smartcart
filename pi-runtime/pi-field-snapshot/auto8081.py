#!/usr/bin/env python3
"""Experimental local A* + curvature tracking; existing motor guard is retained."""
import asyncio
import contextlib
import heapq
import math
from pathlib import Path
import sys
import time
from aiohttp import ClientTimeout, web

APP = Path.home() / 'smartcart/SmartCart_ManualDrive_180/smartcart'
sys.path.insert(0, str(APP))
import drive as original

def finite(x):
    return type(x) in (int, float) and math.isfinite(x)

def plan(view, frame):
    # UWB-only trial: no LiDAR association or obstacle avoidance.
    if view.get('source_mode') != 'live' or view.get('state') != 'running':
        raise ValueError('uwb_transport_not_live')

    dual = frame.get('dual_uwb') or {}
    left = (dual.get('sensors') or {}).get('uwb_left') or {}
    if (dual.get('profile') != 'single_left'
            or dual.get('source_mode') != 'live'
            or left.get('connected') is not True
            or left.get('valid') is not True
            or left.get('measurement_valid') is not True
            or left.get('id_valid') is not True):
        raise ValueError('uwb_measurement_or_id_invalid')

    u = (view.get('result') or {}).get('uwb_estimate') or {}
    if u.get('valid') is not True or u.get('frame_id') != 'base_link':
        raise ValueError('uwb_estimate_invalid')

    for age in (left.get('age_ms'), view.get('age_ms')):
        if not finite(age) or not 0 <= age < 200:
            raise ValueError('uwb_stale')

    for remaining in (left.get('remaining_ms'), view.get('uwb_remaining_ms')):
        if not finite(remaining) or remaining <= 0:
            raise ValueError('uwb_expired')

    distance = left.get('range_m')
    angle = left.get('raw_angle_deg')
    if (not finite(distance) or not .05 <= distance <= 10
            or not finite(angle) or not -90 <= angle <= 90):
        raise ValueError('uwb_range_or_angle_invalid')

    xy = u.get('position_xy_m')
    if not isinstance(xy, list) or len(xy) != 2 or not all(finite(v) for v in xy):
        raise ValueError('uwb_position_invalid')

    x, y = xy
    gap = math.hypot(x, y)

    # Distance is measured from the configured base_link origin.
    if gap <= 2.0:
        return 0., 0., []

    heading = math.atan2(y, x)
    if x <= 0 or abs(heading) > math.radians(70):
        raise ValueError('target_outside_forward_trial')

    turn = max(-.5, min(.5, heading * 1.3))
    forward = (
        0. if abs(heading) > math.radians(35)
        else min(.5, (gap - 2.0) * .5)
    )
    return forward, turn, [[x * (gap-2)/gap, y * (gap-2)/gap]]


class Bridge:
    def __init__(self, drive):
        self.drive = drive
        self.enabled = False
        self.reason = 'off'
        self.path = []
        self.sample = None

    def __getattr__(self,name):
        return getattr(self.drive,name)

    def snapshot(self):
        return dict(self.drive.snapshot(),autonomy={'enabled':self.enabled,'reason':self.reason,'path':self.path})

    def release(self,owner):
        if self.drive.owner == owner:
            self.enabled = False
        self.drive.release(owner)

    def command(self,owner,data):
        if not isinstance(data,dict) or data.get('type') != 'auto':
            self.drive.command(owner,data)
            self.enabled = False
            self.reason = 'off'
            return
        if self.drive.owner != owner:
            raise ValueError('not_owner')
        try:
            if self.sample is None:
                raise ValueError('waiting_for_sensors')
            deadline,view,frame = self.sample
            if time.monotonic() >= deadline:
                raise ValueError('sensor_stale')
            # The current sensing stack is explicitly observational. Never rewrite its flags.
            # Separate UWB-only trial: keep observational flags unchanged.
            # Existing owner, arming and motor watchdog remain required.
            if not self.drive.snapshot()['armed']:
                raise ValueError('rearm_required')
            forward,turn,self.path = plan(view,frame)
            command = dict(data,type='drive',forward=forward,turn=turn,pwm=min(float(data.get('pwm',0)),30.))
            self.drive.command(owner,command)
            if not self.drive.snapshot()['armed']:
                raise ValueError('rearm_required')
            self.enabled,self.reason = True,'following' if forward else 'distance_hold'
        except Exception as error:
            self.enabled,self.reason,self.path = False,str(error),[]
            self.drive.command(owner,{'type':'stop'})
            raise ValueError(self.reason) from None

JS = '''<script>
(()=>{let active=false,button,status;
const Native=window.WebSocket;
window.WebSocket=class extends Native {
send(raw){let d;try{d=JSON.parse(raw)}catch{};
if(d?.type==='stop')active=false;
if(d?.type==='drive'&&active){
if(window.SmartCartManual?.pressed.length){active=false}else{d.type='auto';raw=JSON.stringify(d)}
}return super.send(raw)}
constructor(...args){super(...args);this.addEventListener('message',e=>{
try{const d=JSON.parse(e.data);if(d.autonomy&&status){status.textContent=d.autonomy.reason;
if(!d.armed)active=false;button.textContent=active?'자율주행 OFF':'자율주행 ON'}}catch{}
})}};
window.addEventListener('DOMContentLoaded',()=>{
const panel=document.createElement('div');
panel.style='position:fixed;top:75px;left:12px;z-index:10000;background:#18232d;color:white;padding:10px;border:1px solid #7594a8';
button=document.createElement('button');button.textContent='자율주행 ON';
status=document.createElement('span');status.textContent='연결 후 주행 활성화 필요';
button.onclick=()=>{if(active){active=false;window.SmartCartManual?.stop();return}
if(!window.SmartCartManual?.enabled){status.textContent='먼저 연결하고 주행 활성화';return}
active=true;button.textContent='자율주행 OFF'};
panel.append(button,status);document.body.append(panel);
});
})();</script>'''

old_page = original.page
async def page(request):
    html = (original.ROOT/'index.html').read_text(encoding='utf-8')
    return web.Response(text=html.replace('<head>','<head>'+JS,1),content_type='text/html')
original.page = page
old_make = original.make_app
def make_app(*args,**kwargs):
    app = old_make(*args,**kwargs)
    bridge = Bridge(app['drive'])
    app['drive'] = bridge
    async def resources(application):
        async def poll():
            while True:
                started = time.monotonic()
                try:
                    data = []
                    for endpoint in ('/api/frame',):
                        async with application['client'].get(application['sensor_url']+endpoint,timeout=ClientTimeout(total=.18),allow_redirects=False) as r:
                            r.raise_for_status()
                            data.append(await r.json())
                    frame = data[0]
                    view = frame.get('tracking') or {}
                    remaining = min(view.get('uwb_remaining_ms',0), ((frame.get('dual_uwb') or {}).get('sensors') or {}).get('uwb_left',{}).get('remaining_ms',0))/1000
                    bridge.sample = (started+min(.20,remaining),view,frame)
                except Exception:
                    bridge.sample = None
                await asyncio.sleep(.05)
        task = asyncio.create_task(poll())
        try:
            yield
        finally:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task
    app.cleanup_ctx.append(resources)
    return app
original.make_app = make_app

if __name__ == '__main__':
    raise SystemExit(original.main())
