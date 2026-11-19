"""스마트폰 조종 화면의 가상 시험 전용. 실제 모터를 구동하지 않는다."""
import asyncio
import contextlib
import math
import secrets
import time

from aiohttp import web


MAX_PWM = 43
DIRECTION = (1, 1)  # 가상 차체 기준이며 실제 모터의 극성이 아니다.
TOKEN = secrets.token_urlsafe(18)

MOTOR = None
OWNER = None


class Drive:
    def __init__(self):
        self.value = [0.0, 0.0]
        self.sign = [0, 0]
        self.zero_at = -1.0

    def stop(self, now):
        if any(self.value):
            self.zero_at = now
        self.value = [0.0, 0.0]

    def step(self, target, dt, now):
        target = [
            max(-MAX_PWM, min(MAX_PWM, v))
            for v in target
        ]

        reversing = any(
            target[i] * self.sign[i] < 0
            for i in (0, 1)
        )

        # 역방향 요청: 양쪽 감속 후 0.5초 대기
        if reversing:
            if any(self.value) or now - self.zero_at < 0.5:
                target = [0.0, 0.0]

        delta = [
            target[i] - self.value[i]
            for i in (0, 1)
        ]

        increasing = any(
            abs(target[i]) > abs(self.value[i])
            for i in (0, 1)
        )
        rate = 30 if increasing else 90

        biggest = max(map(abs, delta))
        factor = min(1, rate * dt / biggest) if biggest else 1

        moving = any(self.value)
        self.value = [
            self.value[i] + delta[i] * factor
            for i in (0, 1)
        ]
        self.value = [
            0.0 if abs(v) < 0.0001 else v
            for v in self.value
        ]

        if moving and not any(self.value):
            self.zero_at = now

        for i, v in enumerate(self.value):
            if v:
                self.sign[i] = 1 if v > 0 else -1

        return [round(v) for v in self.value]


def target(y, x, speed):
    """차체 기준 혼합: 전진 양수, 좌회전 양수. 모터 출력은 없다."""
    left, right = y - x, y + x
    scale = max(1.0, abs(left), abs(right))
    return [max(-MAX_PWM, min(MAX_PWM, speed * left / scale)),
            max(-MAX_PWM, min(MAX_PWM, speed * right / scale))]


class Motors:
    """실제 GPIO를 가져오거나 열지 않는 가상 출력 장치."""
    pins = ((17, 27), (23, 24))  # 좌우 RPWM/LPWM의 BCM 번호, 관찰용

    def __init__(self):
        self.values = [0, 0]

    def write(self, values):
        self.values = [max(-MAX_PWM, min(MAX_PWM, int(v))) for v in values]

    def stop(self):
        self.values = [0, 0]

    def close(self):
        self.stop()


async def index(request):
    return web.Response(
        text=PAGE,
        content_type="text/html",
        headers={"Cache-Control": "no-store"},
    )


async def websocket(request):
    global OWNER
    origin = request.headers.get("Origin", "")
    if origin not in ("http://127.0.0.1:8080", "http://localhost:8080"):
        raise web.HTTPForbidden()

    ws = web.WebSocketResponse(max_msg_size=1024)
    await ws.prepare(request)

    if OWNER is not None:
        await ws.close(code=1008, message=b"Controller busy")
        return ws

    OWNER = ws
    state = {
        "armed": False,
        "last": 0.0,
        "seq": -1,
        "y": 0.0,
        "x": 0.0,
        "speed": 0,
        "out": [0, 0],
    }

    drive = Drive()
    task = None

    def stop():
        state["armed"] = False
        state["y"] = state["x"] = 0.0
        state["out"] = [0, 0]
        drive.stop(time.monotonic())
        MOTOR.stop()

    async def loop():
        previous = time.monotonic()

        try:
            while not ws.closed:
                now = time.monotonic()
                dt = now - previous
                previous = now

                # 통신 단절 또는 제어 루프 지연 시 출력 차단
                if dt > 0.2 or now - state["last"] > 0.35:
                    stop()

                if state["armed"]:
                    state["out"] = drive.step(
                        target(
                            state["y"],
                            state["x"],
                            state["speed"],
                        ),
                        min(dt, 0.04),
                        now,
                    )
                    MOTOR.write(state["out"])

                a, b = state["out"]

                # 6인치·160RPM·PWM 선형 비례 가정의 추정값
                estimate = (
                    abs((a + b) / 2)
                    / 255
                    * 160
                    * math.pi
                    * 0.1524
                    * 60
                    / 1000
                )

                await asyncio.wait_for(
                    ws.send_json({
                        "stamp": now,
                        "armed": state["armed"],
                        "left": a,
                        "right": b,
                        "speed": round(estimate, 2),
                    }),
                    timeout=0.1,
                )
                await asyncio.sleep(0.02)

        finally:
            stop()

    try:
        first = await asyncio.wait_for(
            ws.receive_json(),
            timeout=5,
        )

        if not secrets.compare_digest(
            str(first.get("token", "")),
            TOKEN,
        ):
            await ws.close(code=1008)
            return ws

        task = asyncio.create_task(loop())

        async for message in ws:
            if message.type != web.WSMsgType.TEXT:
                break

            data = message.json()

            if data.get("kind") == "stop":
                stop()
                continue

            now = time.monotonic()
            stamp = float(data["stamp"])
            seq = int(data["seq"])

            # 오래 밀린 명령과 중복 명령은 거부
            if (
                not math.isfinite(stamp)
                or not 0 <= now - stamp <= 0.35
                or seq <= state["seq"]
            ):
                stop()
                continue

            y = float(data["y"])
            x = float(data["x"])
            speed = float(data["speed"])

            if not all(map(math.isfinite, (y, x, speed))):
                raise ValueError("Invalid command")

            if not (
                -1 <= y <= 1
                and -1 <= x <= 1
                and 0 <= speed <= MAX_PWM
            ):
                raise ValueError("Out of range")

            state.update(last=now, seq=seq)

            if data.get("kind") == "arm":
                if abs(y) < 0.001 and abs(x) < 0.001:
                    drive.stop(now)
                    state.update(armed=True, y=0.0, x=0.0)

            elif data.get("kind") == "drive":
                if state["armed"]:
                    state.update(y=y, x=x, speed=speed)

    except (
        ValueError,
        TypeError,
        KeyError,
        asyncio.TimeoutError,
    ):
        pass

    finally:
        if task:
            task.cancel()
            with contextlib.suppress(
                asyncio.CancelledError,
                Exception,
            ):
                await task

        stop()
        OWNER = None
        await ws.close()

    return ws


async def hardware(app):
    global MOTOR

    MOTOR = Motors()
    try:
        yield
    finally:
        MOTOR.close()


PAGE = r"""<!doctype html>
<html lang="ko">
<meta charset="utf-8">
<meta name="viewport"
 content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta name="apple-mobile-web-app-capable" content="yes">
<title>SUDO 카트</title>
<style>
body{
 background:#121820;color:#eef4fa;
 font:18px sans-serif;text-align:center;margin:18px
}
input,button{
 font:inherit;padding:12px;border-radius:12px;max-width:90%
}
button{border:0;margin:8px;background:#5bd3b4}
#pad{
 width:260px;height:260px;border-radius:50%;
 background:#263443;margin:18px auto;
 position:relative;touch-action:none
}
#knob{
 width:64px;height:64px;border-radius:50%;
 background:#5bd3b4;position:absolute;
 left:98px;top:98px;pointer-events:none
}
#stop{
 background:#ed5555;color:white;width:90%;
 font-weight:bold;position:sticky;bottom:12px
}
small{display:block;color:#aab8c8;margin:12px}
</style>

<h2>가상 조종 시험 · 모터 구동 없음</h2>
<div>
 <input id="token" type="password"
  placeholder="라즈베리 화면의 연결키">
 <button id="connect">연결</button>
</div>

<p id="status">연결 안 됨</p>
<h2><span id="velocity">0.00</span> km/h</h2>
<small>가상 값 · 실제 속도 아님</small>

<div>
 좌 <span id="left">0</span> /
 우 <span id="right">0</span> PWM
</div>

<p>가상 출력 <output id="chosen">0</output> / 43</p>
<input id="limit" type="range"
 min="0" max="43" step="1" value="0">

<div id="pad"><div id="knob"></div></div>
<small>
 위: 전진 · 아래: 후진 · 좌우: 제자리 회전<br>
 가상 조종 시험 · 실제 GPIO에 접근하지 않음
</small>

<button id="arm">주행 활성화</button><br>
<button id="stop">STOP · 출력 차단</button>

<script>
const el = id => document.getElementById(id);
let ws=null, stamp=0, seq=0, x=0, y=0;
let active=false, held=null, last=0;

function center(){
 x=y=0;
 held=null;
 el('knob').style.transform='';
}

function send(kind){
 if(ws?.readyState!==1) return;
 ws.send(JSON.stringify({
  kind, stamp, seq:++seq, x, y,
  speed:+el('limit').value
 }));
}

function stop(){
 active=false;
 center();
 send('stop');
}

el('limit').oninput=()=>{
 el('chosen').value=el('limit').value;
};

el('connect').onclick=()=>{
 if(ws) ws.close();

 ws=new WebSocket(
  (location.protocol==='https:'?'wss://':'ws://')
  +location.host+'/ws'
 );

 const current=ws;

 ws.onopen=()=>{
  current.send(JSON.stringify({
   token:el('token').value
  }));
 };

 ws.onmessage=e=>{
  const d=JSON.parse(e.data);
  stamp=d.stamp;
  last=performance.now();
  active=d.armed;

  el('status').textContent=active
   ?'주행 활성화'
   :'정지 · 활성화 버튼을 누르세요';

  el('velocity').textContent=d.speed.toFixed(2);
  el('left').textContent=d.left;
  el('right').textContent=d.right;
 };

 ws.onclose=()=>{
  active=false;
  center();
  el('status').textContent='연결 끊김 · 다시 연결';
 };

 ws.onerror=()=>stop();
};

el('arm').onclick=()=>{
 if(held===null){
  center();
  send('arm');
 }
};

el('stop').onclick=stop;

function move(e){
 const r=el('pad').getBoundingClientRect();
 let dx=(e.clientX-r.left-r.width/2)/98;
 let dy=(e.clientY-r.top-r.height/2)/98;
 const n=Math.max(1,Math.hypot(dx,dy));

 x=dx/n;
 y=-dy/n;

 el('knob').style.transform=
  'translate('+x*98+'px,'+-y*98+'px)';
}

el('pad').onpointerdown=e=>{
 if(!active || held!==null) return;
 held=e.pointerId;
 el('pad').setPointerCapture(held);
 move(e);
};

el('pad').onpointermove=e=>{
 if(e.pointerId===held) move(e);
};

el('pad').onpointerup=e=>{
 if(e.pointerId===held) center();
};

el('pad').onpointercancel=stop;

el('pad').onlostpointercapture=()=>{
 if(held!==null) stop();
};

document.addEventListener('visibilitychange',()=>{
 if(document.hidden) stop();
});

window.addEventListener('pagehide',stop);
window.addEventListener('blur',stop);

setInterval(()=>{
 if(document.hidden) return;

 if(performance.now()-last>350){
  stop();
  el('status').textContent='통신 대기';
  return;
 }

 send('drive');
},100);
</script>
</html>
"""


if __name__ == "__main__":
    print("연결키:", TOKEN, flush=True)

    app = web.Application(client_max_size=1024)
    app.cleanup_ctx.append(hardware)
    app.router.add_get("/", index)
    app.router.add_get("/ws", websocket)

    web.run_app(
        app,
        host="127.0.0.1",
        port=8080,
        access_log=None,
    )
