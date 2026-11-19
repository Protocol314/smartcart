"""LAN 인증 경계의 독립 가상 시험. 실제 소켓과 GPIO는 열지 않는다."""
import asyncio
import importlib.util
from pathlib import Path
import sys
import types
import unittest

class Reply(Exception):
    def __init__(self, *args, **kwargs):
        super().__init__(*args)
        self.cookies = {}
    def set_cookie(self, name, value, **options):
        self.cookies[name] = (value, options)

web = types.SimpleNamespace(middleware=lambda fn: fn, Response=Reply, HTTPForbidden=type('Forbidden',(Reply,),{}),
    HTTPTooManyRequests=type('Limited',(Reply,),{}), HTTPUnauthorized=type('Unauthorized',(Reply,),{}),
    HTTPSeeOther=type('Redirect',(Reply,),{}), HTTPFound=type('Found',(Reply,),{}))
sys.modules['aiohttp'] = types.SimpleNamespace(web=web)
sys.modules['drive'] = types.SimpleNamespace(make_app=lambda *a, **k: None)
sys.modules['smartcart_motor'] = types.ModuleType('smartcart_motor')
sys.modules['smartcart_motor.manual_remote'] = types.SimpleNamespace(MockBackend=object)
path = Path(__file__).resolve().parents[1] / 'scripts/virtual_lan_server.py'
spec = importlib.util.spec_from_file_location('test_virtual_lan_module', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class Request:
    def __init__(self, path, method='GET', host='192.168.43.212:18083', origin=None, key='', cookie=''):
        self.path, self.method, self.host = path, method, host
        self.headers = {} if origin is None else {'Origin':origin}
        self.cookies = {'sc_virtual_session':cookie} if cookie else {}
        self.key = key
    async def post(self): return {'key':self.key}

def run(coro):
    try: coro.send(None)
    except StopIteration as done: return done.value
    raise AssertionError("예상하지 못한 대기")

class GuardTest(unittest.TestCase):
    def test_pin_format(self):
        self.assertTrue(module.valid_pin('0012'))
        for invalid in ('123', '12345', '１２３４', '12a4', '', None):
            self.assertFalse(module.valid_pin(invalid))

    def test_session_and_sensor_boundary(self):
        clock = [10.]
        guard = module.make_guard('192.168.43.212',18083,'test-key-at-runtime',clock=lambda:clock[0])
        async def handler(_): return '통과'
        with self.assertRaises(web.HTTPFound): run(guard(Request('/'),handler))
        with self.assertRaises(web.HTTPUnauthorized): run(guard(Request('/api/frame'),handler))
        with self.assertRaises(web.HTTPForbidden): run(guard(Request('/',host='other:18083'),handler))
        with self.assertRaises(web.HTTPForbidden): run(guard(Request('/login','POST',origin='https://wrong',key='test-key-at-runtime'),handler))
        with self.assertRaises(web.HTTPSeeOther) as result:
            run(guard(Request('/login','POST',origin='https://192.168.43.212:18083',key='test-key-at-runtime'),handler))
        token, options = result.exception.cookies['sc_virtual_session']
        self.assertTrue(options['secure'] and options['httponly'])
        self.assertEqual(run(guard(Request('/api/frame',cookie=token),handler)),'통과')
        clock[0] += 3601
        with self.assertRaises(web.HTTPUnauthorized): run(guard(Request('/api/frame',cookie=token),handler))

    def test_attempt_limit(self):
        guard=module.make_guard('192.168.43.212',18083,'test-key-at-runtime',clock=lambda:10.)
        async def handler(_): return None
        for _ in range(5):
            with self.assertRaises(web.HTTPUnauthorized):
                run(guard(Request('/login','POST',origin='https://192.168.43.212:18083',key='wrong'),handler))
        with self.assertRaises(web.HTTPTooManyRequests):
            run(guard(Request('/login','POST',origin='https://192.168.43.212:18083',key='test-key-at-runtime'),handler))

if __name__ == '__main__': unittest.main()
