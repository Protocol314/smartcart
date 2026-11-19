"""스마트폰 조종 화면의 가상 시험 전용. 실제 모터를 구동하지 않는다."""
import asyncio
import argparse
import collections
import hashlib
import ipaddress
import os
from pathlib import Path
import ssl
import subprocess
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
BIND_IP = None
ATTEMPTS = {}


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
    if request.host != f"{BIND_IP}:8443":
        raise web.HTTPForbidden()
    return web.Response(
        text=PAGE,
        content_type="text/html",
        headers={"Cache-Control": "no-store"},
    )


async def websocket(request):
    global OWNER
    origin = request.headers.get("Origin", "")
    if request.host != f"{BIND_IP}:8443" or origin != f"https://{BIND_IP}:8443":
        raise web.HTTPForbidden()
    now = time.monotonic()
    for peer in list(ATTEMPTS):
        ATTEMPTS[peer] = [t for t in ATTEMPTS[peer] if now - t < 60]
        if not ATTEMPTS[peer]:
            del ATTEMPTS[peer]
    peer = request.remote or "unknown"
    if peer not in ATTEMPTS and len(ATTEMPTS) >= 128:
        raise web.HTTPTooManyRequests()
    attempts = ATTEMPTS.setdefault(peer, [])
    if len(attempts) >= 10:
        raise web.HTTPTooManyRequests()
    attempts.append(now)
    ws = web.WebSocketResponse(max_msg_size=1024)
    await ws.prepare(request)

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

        if OWNER is not None:
            await ws.close(code=1008, message=b"Controller busy")
            return ws
        OWNER = ws
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

        if OWNER is ws:
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


def validated_address(value):
    """명시적인 사설 IPv4에만 바인딩한다."""
    addr = ipaddress.IPv4Address(value)
    networks = (ipaddress.IPv4Network("10.0.0.0/8"), ipaddress.IPv4Network("172.16.0.0/12"), ipaddress.IPv4Network("192.168.0.0/16"))
    if not any(addr in network for network in networks):
        raise ValueError("사설 IPv4 주소를 지정해야 합니다")
    return str(addr)


def make_certificates(address, directory):
    """Pi에서만 생성하는 시험용 CA·서버 인증서다. 기존 파일을 덮어쓰지 않는다."""
    address = validated_address(address)
    directory = Path(directory)
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    if directory.is_symlink() or not directory.is_dir():
        raise ValueError("인증서 디렉터리를 확인하십시오")
    files = {name: directory / name for name in ("ca.key", "ca.crt", "server.key", "server.csr", "server.crt", "server.ext")}
    if any(path.exists() or path.is_symlink() for path in files.values()):
        raise FileExistsError("기존 인증서가 있어 덮어쓰지 않습니다")
    extension = ("basicConstraints=critical,CA:FALSE\nkeyUsage=critical,digitalSignature,keyEncipherment\n"
                 "extendedKeyUsage=serverAuth\nsubjectAltName=IP:" + address + "\n")
    old = os.umask(0o077)
    try:
        with files["server.ext"].open("x", encoding="ascii") as f:
            f.write(extension)
        commands = [
            ["openssl", "req", "-x509", "-newkey", "rsa:3072", "-nodes", "-sha256", "-days", "7", "-subj", "/CN=SmartCart Virtual Trial CA", "-addext", "basicConstraints=critical,CA:TRUE", "-addext", "keyUsage=critical,keyCertSign,cRLSign", "-addext", "nameConstraints=critical,permitted;IP:" + address + "/255.255.255.255", "-keyout", str(files["ca.key"]), "-out", str(files["ca.crt"])],
            ["openssl", "req", "-new", "-newkey", "rsa:2048", "-nodes", "-sha256", "-subj", "/CN=" + address, "-keyout", str(files["server.key"]), "-out", str(files["server.csr"])],
            ["openssl", "x509", "-req", "-in", str(files["server.csr"]), "-CA", str(files["ca.crt"]), "-CAkey", str(files["ca.key"]), "-CAcreateserial", "-out", str(files["server.crt"]), "-days", "7", "-sha256", "-extfile", str(files["server.ext"])],
        ]
        for command in commands:
            subprocess.run(command, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60)
    finally:
        os.umask(old)
    return files


def fingerprint(cert_path):
    text = Path(cert_path).read_text(encoding="ascii")
    return hashlib.sha256(ssl.PEM_cert_to_DER_cert(text)).hexdigest()


def main():
    global BIND_IP
    parser = argparse.ArgumentParser(description="실제 모터를 열지 않는 LAN 가상 조종 HTTPS 시험")
    parser.add_argument("--address", required=True, help="Pi의 사설 IPv4 주소")
    parser.add_argument("--cert-dir", required=True, help="미리 준비한 CA와 서버 인증서 디렉터리")
    parser.add_argument("--init-cert", action="store_true", help="새 디렉터리에 시험용 인증서 생성")
    options = parser.parse_args()
    BIND_IP = validated_address(options.address)
    directory = Path(options.cert_dir).expanduser()
    if options.init_cert:
        make_certificates(BIND_IP, directory)
    ca, cert, key = (directory / name for name in ("ca.crt", "server.crt", "server.key"))
    if any(path.is_symlink() or not path.is_file() for path in (ca, cert, key)):
        raise SystemExit("필요한 일반 인증서 파일이 없습니다")
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    context.load_cert_chain(str(cert), str(key))
    print("시험 CA SHA256:", fingerprint(ca), flush=True)
    print("연결키:", TOKEN, flush=True)
    print("가상 시험 주소: https://" + BIND_IP + ":8443/", flush=True)
    print("가상 화면 전용 · 실제 GPIO 및 모터 제어 없음", flush=True)
    app = web.Application(client_max_size=1024)
    app.cleanup_ctx.append(hardware)
    app.router.add_get("/", index)
    app.router.add_get("/ws", websocket)
    web.run_app(app, host=BIND_IP, port=8443, ssl_context=context, access_log=None)


if __name__ == "__main__":
    main()
