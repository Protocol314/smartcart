"""In-process WebSocket protocol checks; no TCP sockets or GPIO hardware."""
import asyncio
import selectors
import time
import types
import unittest
from unittest.mock import patch

from aiohttp import WSMsgType

import drive
from smartcart_motor.manual_remote import MockBackend


class Socket:
    def __init__(self):
        self.closed=False
        self.incoming=asyncio.Queue()
        self.outgoing=asyncio.Queue()
    async def prepare(self,request): return None
    async def receive_json(self): return await self.incoming.get()
    async def send_json(self,data): await self.outgoing.put(data)
    async def close(self,**kwargs):
        if not self.closed:
            self.closed=True
            await self.incoming.put(None)
    def __aiter__(self): return self
    async def __anext__(self):
        item=await self.incoming.get()
        if item is None: raise StopAsyncIteration
        return types.SimpleNamespace(type=WSMsgType.TEXT,json=lambda:item)


class NoSocketSelector(selectors.SelectSelector):
    def select(self, timeout=None):
        if not self.get_map():
            time.sleep(max(0, timeout if timeout is not None else 0.01))
            return []
        return super().select(timeout)


class NoSocketLoop(asyncio.SelectorEventLoop):
    """In-process tests need no OS socketpair or cross-thread event-loop wakeups."""
    def __init__(self):
        super().__init__(selector=NoSocketSelector())

    def _make_self_pipe(self):
        self._ssock = self._csock = None

    def _close_self_pipe(self):
        pass

    def _write_to_self(self):
        pass


class ProtocolTests(unittest.IsolatedAsyncioTestCase):
    loop_factory = staticmethod(NoSocketLoop)

    async def asyncSetUp(self):
        self.backend=MockBackend()
        self.app=drive.make_app(self.backend,key="test-key")
        self.app["drive"].start()
        self.tasks=[]

    async def asyncTearDown(self):
        for ws,task in self.tasks:
            await ws.close()
            await asyncio.wait_for(task,2)
        self.app["drive"].close()

    async def connect(self,key="test-key"):
        ws=Socket()
        await ws.incoming.put({"key":key})
        async def launch():
            with patch.object(drive.web,"WebSocketResponse",return_value=ws):
                return await drive.drive_socket(types.SimpleNamespace(app=self.app))
        task=asyncio.create_task(launch())
        self.tasks.append((ws,task))
        result=await asyncio.wait_for(ws.outgoing.get(),1)
        return ws,result

    async def status(self,ws,predicate=lambda s:True):
        async def read():
            while True:
                s=await ws.outgoing.get()
                if predicate(s): return s
        return await asyncio.wait_for(read(),2)

    async def arm(self,ws,s):
        await ws.incoming.put(dict(type="arm",epoch=s["epoch"],seq=1,stamp=s["stamp"],
                                   forward=0,turn=0,pwm=180))
        return await self.status(ws,lambda s:s["armed"])

    async def test_socket_command_to_motor_and_stop_latch(self):
        ws,s=await self.connect()
        s=await self.arm(ws,s)
        for seq in range(2,7):
            await ws.incoming.put(dict(type="drive",epoch=s["epoch"],seq=seq,
                                      stamp=s["stamp"],forward=1,turn=0,pwm=180))
            s=await self.status(ws,lambda s:s["ack_seq"]>=seq)
        self.assertGreater(self.backend.values[0],0)
        await ws.incoming.put({"type":"stop"})
        s=await self.status(ws,lambda s:not s["armed"])
        self.assertEqual(self.backend.values,(0,0))
        await ws.incoming.put(dict(type="drive",epoch=s["epoch"],seq=7,stamp=s["stamp"],
                                   forward=1,turn=0,pwm=180))
        await asyncio.sleep(.08)
        self.assertEqual(self.backend.values,(0,0))
        self.assertFalse(self.app["drive"].armed)

    async def test_receive_silence_and_disconnect_stop(self):
        ws,s=await self.connect()
        await self.arm(ws,s)
        await asyncio.sleep(.43)
        self.assertEqual(self.backend.values,(0,0))
        self.assertFalse(self.app["drive"].armed)
        await ws.close()
        await asyncio.wait_for(self.tasks[-1][1],1)
        self.assertIsNone(self.app["drive"].owner)

    async def test_invalid_key_and_busy_never_stop_owner(self):
        ws,s=await self.connect()
        await self.arm(ws,s)
        for key in ("wrong","test-key"):
            other,result=await self.connect(key)
            self.assertIn("error",result)
            self.assertTrue(self.app["drive"].armed)
            await other.close()

    async def test_malformed_json_object_stops_and_closes(self):
        ws,s=await self.connect()
        await self.arm(ws,s)
        await ws.incoming.put(["not","an","object"])
        result=await self.status(ws,lambda s:"error" in s)
        self.assertIn("error",result)
        self.assertFalse(self.app["drive"].armed)
        self.assertEqual(self.backend.values,(0,0))

    def test_routes_and_origin_validation(self):
        base=types.SimpleNamespace(host="127.0.0.1:8080",scheme="http",headers={})
        self.assertTrue(drive.valid_origin(base))
        base.headers={"Origin":"http://evil.example"}
        self.assertFalse(drive.valid_origin(base))
        self.assertIn("/ws/drive",{r.resource.canonical for r in self.app.router.routes()})
        self.assertIn("manual_controls.js",(drive.ROOT/"index.html").read_text(encoding="utf-8"))


if __name__=="__main__": unittest.main()
