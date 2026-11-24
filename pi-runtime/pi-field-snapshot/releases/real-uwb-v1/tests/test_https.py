import asyncio
import hashlib
import importlib.util
from pathlib import Path
import shutil
import ssl
import sys
import tempfile
import unittest
import os

if sys.platform=='win32':
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE))
from certificates import private_ip,prepare,verified
from uwb_control import Config
import server as release_server
from aiohttp import CookieJar,Fingerprint,web
from aiohttp.test_utils import TestClient,TestServer

class CertificatesTests(unittest.TestCase):
    def test_private_addresses(self):
        for address in ('172.20.10.4','192.168.137.90','192.168.43.212','10.2.9.147'):
            self.assertEqual(private_ip(address),address)
        for address in ('0.0.0.0','8.8.8.8','127.0.0.1','224.0.0.1'):
            with self.assertRaises(ValueError):private_ip(address)

    @unittest.skipUnless(shutil.which('openssl'),'openssl required')
    def test_certificate_reuse_and_new_ip(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)
            first,new=prepare(root,'172.20.10.4')
            self.assertTrue(new);self.assertTrue(verified(first,'172.20.10.4'))
            same,new=prepare(root,'172.20.10.4')
            self.assertFalse(new);self.assertEqual(first,same)
            second,new=prepare(root,'192.168.137.90')
            self.assertFalse(new)
            self.assertEqual((first/'ca.crt').read_bytes(),(second/'ca.crt').read_bytes())
            self.assertTrue(verified(second,'192.168.137.90'))
            self.assertFalse(verified(first,'192.168.137.90'))

class HttpsIntegrationTests(unittest.IsolatedAsyncioTestCase):
    async def test_login_websocket_auto_and_stop(self):
        if not shutil.which('openssl'):self.skipTest('openssl required')
        drive_dir=Path(os.environ.get('SMARTCART_TEST_DRIVE_DIR',str(Path.home()/'smartcart/SmartCart_ManualDrive_180/smartcart')))
        reference_root=Path(os.environ.get('SMARTCART_TEST_REFERENCE_ROOT',str(Path.home()/'smartcart')))
        release_server.DRIVE_DIR=drive_dir
        release_server.ROOT=reference_root
        base=release_server.load_base()
        backend=base.MockBackend()
        with tempfile.TemporaryDirectory() as temporary:
            cert,_=prepare(Path(temporary),'172.20.10.4')
            tls=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            tls.load_cert_chain(cert/'server.crt',cert/'server.key')
            fingerprint=Fingerprint(hashlib.sha256(ssl.PEM_cert_to_DER_cert((cert/'server.crt').read_text())).digest())
            sequence=0
            async def sensor(request):
                nonlocal sequence
                sequence+=1
                return web.json_response({'source_mode':'live','profile':'single_left','state':'running','remaining_ms':500,
                    'sensors':{'uwb_left':{'connected':True,'valid':True,'measurement_valid':True,'id_valid':True,
                    'sample_sequence':sequence,'range_m':3.,'raw_angle_deg':0,'age_ms':0,'remaining_ms':500}}})
            sensor_app=web.Application();sensor_app.router.add_get('/api/dual-uwb',sensor)
            sensor_server=TestServer(sensor_app)
            await sensor_server.start_server()
            app=release_server.build_app(base,Config(),backend,'4321','172.20.10.4','1212')
            app['sensor_url']=str(sensor_server.make_url('')).rstrip('/')
            http_server=TestServer(app,scheme='https')
            await http_server.start_server(ssl=tls)
            client=TestClient(http_server,cookie_jar=CookieJar(unsafe=True))
            await client.start_server()
            headers={'Host':'172.20.10.4:8081','Origin':'https://172.20.10.4:8081'}
            try:
                r=await client.get('/api/drive/health',headers=headers,ssl=fingerprint)
                self.assertEqual(r.status,401)
                r=await client.post('/login',data={'key':'1212'},headers=headers,ssl=fingerprint,allow_redirects=False)
                self.assertEqual(r.status,303)
                r=await client.get('/',headers=headers,ssl=fingerprint)
                html=await r.text()
                self.assertEqual(r.status,200)
                self.assertIn('js/uwb_real.js',html)
                self.assertNotIn('src="js/virtual_autonomy.js"',html)
                await asyncio.sleep(.18)
                ws=await client.ws_connect('/ws/drive',headers=headers,ssl=fingerprint)
                await ws.send_json({'key':'4321'})
                state=await asyncio.wait_for(ws.receive_json(),1)
                self.assertFalse(state['armed'])
                self.assertEqual(state['controller_version'],'1.0.0')
                await ws.send_json(dict(type='arm',seq=1,epoch=state['epoch'],stamp=state['stamp'],forward=0,turn=0,pwm=50))
                for _ in range(8):
                    state=await asyncio.wait_for(ws.receive_json(),1)
                    if state['armed'] and state['ack_seq']>=1:break
                self.assertTrue(state['armed'])
                await ws.send_json(dict(type='auto',seq=2,epoch=state['epoch'],stamp=state['stamp'],pwm=50))
                for _ in range(8):
                    state=await asyncio.wait_for(ws.receive_json(),1)
                    if state['applied_seq']>=2 and any(state['pwm']):break
                self.assertTrue(state['autonomy']['enabled'])
                self.assertGreater(max(state['pwm']),0)
                self.assertLessEqual(max(state['pwm']),30)
                await ws.send_json({'type':'stop'})
                for _ in range(8):
                    state=await asyncio.wait_for(ws.receive_json(),1)
                    if not state['armed']:break
                self.assertFalse(state['armed']);self.assertEqual(state['pwm'],[0,0])
                await ws.close()
            finally:
                await client.close()
                await sensor_server.close()

if __name__=='__main__':unittest.main()
