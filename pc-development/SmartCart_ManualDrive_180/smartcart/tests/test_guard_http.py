"""합성 센서 HTTP와 WebSocket을 통한 가상 모터 차단 시험."""
import asyncio
import math
import unittest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
from drive import make_app
from smartcart_motor.manual_remote import MockBackend

class GuardIntegration(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.bad=False;self.seq=0;self.delay=0
        async def frame(_):
            await asyncio.sleep(self.delay)
            self.seq+=1; step=2*math.pi/360
            return web.json_response({'schema':'smartcart.envelope.v1','source_mode':'ros','state':'live','age_ms':0,
              'scan':{'sequence':self.seq,'angle_min':-math.pi,'angle_max':math.pi-step,'angle_increment':step,
                      'range_min':.05,'range_max':10.,'ranges':[2.]*360}})
        async def uwb(_):
            await asyncio.sleep(self.delay)
            return web.json_response({'schema':'smartcart.dual_uwb_view.v1','source_mode':'live','state':'running',
              'profile':'single_left','drive_authorized':False,'sensors':{'uwb_left':{'state':'live',
              'connected':True,'valid':not self.bad,'measurement_valid':True,'id_valid':True,
              'remaining_ms':200,'age_ms':0,'sample_sequence':self.seq}}})
        upstream=web.Application();upstream.router.add_get('/api/frame',frame);upstream.router.add_get('/api/dual-uwb',uwb)
        self.source=TestServer(upstream,host='127.0.0.1');await self.source.start_server()
        self.backend=MockBackend()
        self.app=make_app(self.backend,key='test-key',sensor_url=str(self.source.make_url('')).rstrip('/'))
        self.stop_reasons=[]
        original_stop=self.app['drive']._stop
        def record_stop(reason, now):
            self.stop_reasons.append(reason)
            return original_stop(reason, now)
        self.app['drive']._stop=record_stop
        self.client=TestClient(TestServer(self.app,host='127.0.0.1'));await self.client.start_server()

    async def asyncTearDown(self):
        await self.client.close();await self.source.close()

    async def test_socket_stops_on_sensor_fault(self):
        for _ in range(50):
            if self.app['sensor_guard'].status()[0]:break
            await asyncio.sleep(.02)
        self.assertTrue(self.app['sensor_guard'].status()[0])
        ws=await self.client.ws_connect('/ws/drive');await ws.send_json({'key':'test-key'})
        state=await ws.receive_json(timeout=2)
        await ws.send_json({'type':'arm','epoch':state['epoch'],'seq':1,'stamp':state['stamp'],
                            'forward':0,'turn':0,'pwm':50})
        for _ in range(20):
            state=await ws.receive_json(timeout=2)
            if state.get('armed'):break
        self.assertTrue(state['armed'])
        await ws.send_json({'type':'drive','epoch':state['epoch'],'seq':2,'stamp':state['stamp'],
                            'forward':1,'turn':0,'pwm':50})
        for _ in range(20):
            state=await ws.receive_json(timeout=2)
            if any(state.get('pwm',[0,0])):break
        self.assertGreater(state['pwm'][0],0,repr((state,self.stop_reasons)))
        self.bad=True
        for _ in range(25):
            state=await ws.receive_json(timeout=2)
            if not state['armed']:break
        self.assertFalse(state['armed']);self.assertEqual(state['pwm'],[0,0]);self.assertEqual(state['reason'],'uwb_invalid')
        self.bad=False;await asyncio.sleep(.2);self.assertFalse(self.app['drive'].armed)
        await ws.close()

    async def test_parallel_poll_respects_expiry_and_stalls(self):
        self.delay=.12
        ready=False
        for _ in range(30):
            await asyncio.sleep(.03)
            if self.app['sensor_guard'].status()[0] and self.seq >= 2:
                ready=True
                break
        self.assertTrue(ready, self.app['sensor_guard'].status())
        self.delay=.25
        await asyncio.sleep(.5)
        self.assertFalse(self.app['sensor_guard'].status()[0])

    def test_hardware_backend_is_rejected(self):
        class Hardware: hardware=True
        with self.assertRaises(ValueError):make_app(Hardware())

if __name__ == '__main__': unittest.main()
