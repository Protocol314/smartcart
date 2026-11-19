import asyncio
import contextlib
import json
import math
import os
import time
import unittest

from aiohttp import WSServerHandshakeError, web
from aiohttp.test_utils import TestClient, TestServer

from smartcart_motor.manual_remote import (
    COMMAND_TIMEOUT, ManualDrive, MockBackend, Ramp, RPiBackend, PINS,
)
from drive import make_app


class CoreTests(unittest.TestCase):
    def test_full_ramp_and_release(self):
        ramp = Ramp()
        out = (0, 0)
        for i in range(1, 301):
            new = ramp.step(1, 0, 180, .02, i*.02)
            self.assertTrue(all(0 <= n <= 180 for n in new))
            self.assertLessEqual(max(abs(a-b) for a,b in zip(new,out)), 1)
            out = new
        self.assertEqual(out, (180,180))
        out = ramp.step(0, 0, 180, .02, 6.02)
        self.assertTrue(0 < out[0] < 180)
        for i in range(1, 102):
            out = ramp.step(0, 0, 180, .02, 6.02+i*.02)
        self.assertEqual(out, (0,0))

    def test_reversal_stops_both_and_waits(self):
        ramp=Ramp()
        now=0
        for _ in range(300):
            now+=.02; ramp.step(1,0,180,.02,now)
        zero=None
        for _ in range(250):
            now+=.02
            out=ramp.step(-1,0,180,.02,now)
            if not any(ramp.value) and zero is None: zero=now
            if any(v<0 for v in out):
                self.assertIsNotNone(zero)
                self.assertGreaterEqual(now-zero,.5)
                break
        else: self.fail("Reverse never started")

    def test_turn_caps_pivot_and_slow_before_turn(self):
        r=Ramp()
        self.assertEqual(r.targets(0,1,180),[-70,70])
        self.assertEqual(r.targets(0,-1,180),[70,-70])
        self.assertEqual(r.targets(1,1,180),[20,80])
        self.assertEqual(r.targets(-1,1,180),[-70,-17.5])
        r.value=[180,180]
        self.assertEqual(r.targets(1,1,180),[80,80])

    def make_controller(self):
        now=[100.0]
        b=MockBackend(); d=ManualDrive(b,clock=lambda:now[0])
        self.assertTrue(d.acquire("one"))
        return d,b,now

    def submit(self,d,**changes):
        s=d.snapshot()
        msg=dict(type="arm",epoch=s["epoch"],seq=s["ack_seq"]+1,
                 stamp=s["stamp"],forward=0,turn=0,pwm=50)
        msg.update(changes)
        d.command("one",msg)
        return msg

    def test_timeout_latches_off_and_rejects_stale_epoch(self):
        d,b,now=self.make_controller()
        self.submit(d)
        msg=self.submit(d,type="drive",forward=1)
        for _ in range(5):
            now[0]+=.02; d.tick()
        self.assertGreater(b.values[0],0)
        now[0]+=.36; d.tick()
        self.assertFalse(d.armed)
        self.assertEqual(b.values,(0,0))
        msg.update(seq=msg["seq"]+1, stamp=now[0])
        with self.assertRaises(ValueError): d.command("one",msg)
        self.assertFalse(d.armed)

    def test_stop_and_rearm_require_zero(self):
        d,b,now=self.make_controller()
        self.submit(d)
        self.submit(d,type="drive",forward=1)
        now[0]+=.02; d.tick()
        d.command("one",{"type":"stop"})
        self.assertEqual(b.values,(0,0))
        with self.assertRaises(ValueError): self.submit(d,forward=1)
        self.submit(d)
        self.assertTrue(d.armed)
        now[0]+=.02; d.tick()
        self.assertEqual(b.values,(0,0))

    def test_invalid_numbers_replays_and_loop_delay_stop(self):
        for changes in ({"pwm":181},{"forward":float("nan")},{"forward":True},{"seq":-1},{"stamp":0}):
            d,b,now=self.make_controller()
            self.submit(d)
            with self.assertRaises(ValueError): self.submit(d,type="drive",**changes)
            self.assertFalse(d.armed)
            self.assertEqual(b.values,(0,0))
        d,b,now=self.make_controller()
        self.submit(d)
        now[0]+=.02; d.tick()
        now[0]+=.21; d.tick()
        self.assertFalse(d.armed)
        self.assertEqual(d.reason,"loop_delay")

    def test_second_owner_cannot_stop_first(self):
        d,b,now=self.make_controller()
        self.submit(d)
        self.assertFalse(d.acquire("two"))
        d.release("two")
        self.assertTrue(d.armed)

    def test_gpio_fault_latches(self):
        class Faulty(MockBackend):
            def write(self,*args): raise OSError("test failure")
        d=ManualDrive(Faulty())
        self.assertTrue(d.acquire("one"))
        self.submit(d)
        d.tick()
        self.assertFalse(d.armed)
        self.assertIsNotNone(d.fault)
        self.assertFalse(d.acquire("two"))


class FakeLock:
    def __enter__(self): return self
    def __exit__(self,*args): pass


class FakeGPIO:
    BCM="BCM"; OUT="OUT"; LOW=0
    def __init__(self):
        self.pins=[]; self.pwms={}; self.cleaned=None
    def setmode(self,mode): self.mode=mode
    def setup(self,pin,*args,**kwargs):
        self.pins.append(pin)
    def PWM(self,pin,hz):
        outer=self
        class PWM:
            value=None
            def start(self,v): self.value=v
            def ChangeDutyCycle(self,v): self.value=v
            def stop(self): self.value=0
        pwm=PWM(); outer.pwms[pin]=pwm; return pwm
    def cleanup(self,pins): self.cleaned=list(pins)


class GpioTests(unittest.TestCase):
    def test_fixed_enable_only_four_pwm_pins_and_polarity(self):
        gpio=FakeGPIO()
        b=RPiBackend(gpio=gpio,ownership=FakeLock())
        b.open()
        self.assertEqual(tuple(gpio.pins),PINS)
        self.assertNotIn(22,gpio.pins)
        b.write(100,100)
        self.assertEqual(gpio.pwms[17].value,0)
        self.assertEqual(gpio.pwms[23].value,0)
        self.assertAlmostEqual(gpio.pwms[27].value,100*100/255)
        self.assertAlmostEqual(gpio.pwms[24].value,100*100/255)
        with self.assertRaises(RuntimeError): b.write(-100,-100)
        self.assertTrue(all(pwm.value==0 for pwm in gpio.pwms.values()))
        b.close()
        self.assertEqual(tuple(gpio.cleaned),PINS)


@unittest.skipUnless(os.environ.get('SMARTCART_NETWORK_TESTS') == '1', '실제 소켓 검사는 SMARTCART_NETWORK_TESTS=1에서 실행')
class ServerTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.backend=MockBackend()
        upstream=web.Application()
        async def unavailable(request):
            return web.json_response({"error":"sensor unavailable"}, status=503)
        upstream.router.add_get('/api/{tail:.*}', unavailable)
        self.sensor=TestServer(upstream)
        await self.sensor.start_server()
        self.app=make_app(self.backend,key="test-key", sensor_url=str(self.sensor.make_url('')).rstrip('/'))
        self.client=TestClient(TestServer(self.app))
        await self.client.start_server()

    async def asyncTearDown(self):
        await self.client.close()
        await self.sensor.close()

    async def get_state(self,ws,predicate=lambda s:True,timeout=2):
        async def read():
            while True:
                s=await ws.receive_json()
                if predicate(s): return s
        return await asyncio.wait_for(read(),timeout)

    async def owner(self):
        ws=await self.client.ws_connect("/ws/drive")
        await ws.send_json({"key":"test-key"})
        return ws,await self.get_state(ws)

    async def arm(self,ws,s):
        await ws.send_json(dict(type="arm",epoch=s["epoch"],seq=1,stamp=s["stamp"],
                                forward=0,turn=0,pwm=180))
        return await self.get_state(ws,lambda s:s["armed"])

    async def test_page_assets_health_sensor_offline(self):
        r=await self.client.get("/")
        html=await r.text()
        self.assertEqual(r.status,200)
        self.assertIn('js/manual_controls.js',html)
        self.assertNotIn('src="js/virtual_controls.js"',html)
        self.assertIn('id="radarTracking"',html)
        self.assertEqual((await self.client.get("/js/manual_controls.js")).status,200)
        s=await (await self.client.get("/api/drive/health")).json()
        self.assertFalse(s["hardware"])
        self.assertFalse(s["armed"])
        r=await self.client.get("/api/events?hz=2")
        self.assertEqual(r.status,503)
        self.assertIn("error",await r.json())

    async def test_real_socket_input_to_backend_stop_and_no_auto_resume(self):
        ws,s=await self.owner()
        s=await self.arm(ws,s)
        seq=1
        for _ in range(6):
            seq+=1
            await ws.send_json(dict(type="drive",epoch=s["epoch"],seq=seq,stamp=s["stamp"],
                                    forward=1,turn=0,pwm=180))
            await asyncio.sleep(.04)
            s=await self.get_state(ws,lambda s:s["ack_seq"]>=seq)
        self.assertGreater(self.backend.values[0],0)
        await ws.send_json({"type":"stop"})
        s=await self.get_state(ws,lambda s:not s["armed"])
        self.assertEqual(self.backend.values,(0,0))
        seq+=1
        await ws.send_json(dict(type="drive",epoch=s["epoch"],seq=seq,stamp=s["stamp"],
                                forward=1,turn=0,pwm=180))
        await asyncio.sleep(.07)
        self.assertFalse(self.app["drive"].armed)
        self.assertEqual(self.backend.values,(0,0))
        await ws.close()

    async def test_disconnect_and_command_timeout(self):
        ws,s=await self.owner()
        await self.arm(ws,s)
        await asyncio.sleep(.45)
        self.assertFalse(self.app["drive"].armed)
        self.assertEqual(self.backend.values,(0,0))
        await ws.close()
        ws,s=await self.owner()
        await self.arm(ws,s)
        await ws.close()
        await asyncio.sleep(.05)
        self.assertIsNone(self.app["drive"].owner)
        self.assertFalse(self.app["drive"].armed)

    async def test_invalid_key_and_second_controller_do_not_stop_owner(self):
        ws,s=await self.owner()
        s=await self.arm(ws,s)
        for key in ("wrong-key","test-key"):
            other=await self.client.ws_connect("/ws/drive")
            await other.send_json({"key":key})
            result=await other.receive_json()
            self.assertIn("error",result)
            await other.close()
            self.assertTrue(self.app["drive"].armed)
        await ws.close()

    async def test_other_origin_rejected(self):
        with self.assertRaises(WSServerHandshakeError):
            await self.client.ws_connect("/ws/drive",headers={"Origin":"http://example.com"})


@unittest.skipUnless(os.environ.get('SMARTCART_NETWORK_TESTS') == '1', '실제 소켓 검사는 SMARTCART_NETWORK_TESTS=1에서 실행')
class ProxyTests(unittest.IsolatedAsyncioTestCase):
    async def test_existing_sensor_stream_is_preserved(self):
        async def stream(request):
            response=web.StreamResponse(headers={"Content-Type":"text/event-stream"})
            await response.prepare(request)
            await response.write(b'event: frame\ndata: {"source_mode":"ros","scan":null}\n\n')
            return response
        upstream=web.Application()
        upstream.router.add_get("/api/events",stream)
        async with TestServer(upstream) as sensor:
            app=make_app(sensor_url=str(sensor.make_url("")).rstrip("/"))
            async with TestClient(TestServer(app)) as client:
                r=await client.get("/api/events?hz=2")
                self.assertEqual(r.status,200)
                self.assertEqual(r.headers["Content-Type"],"text/event-stream")
                data=await r.text()
                self.assertIn('event: frame',data)
                self.assertIn('"source_mode":"ros"',data)


if __name__ == "__main__":
    unittest.main()
