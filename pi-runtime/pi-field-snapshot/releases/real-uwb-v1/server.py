#!/usr/bin/env python3
import argparse
import ast
import getpass
import importlib.util
import ipaddress
import json
from pathlib import Path
import socket
import ssl
import sys
from aiohttp import web
from uwb_control import VERSION,Config
from runtime import ReportingBackend,attach
from certificates import private_ip,choose_address,prepare

ROOT=Path.home()/'smartcart'
HERE=Path(__file__).resolve().parent
DRIVE_DIR=ROOT/'SmartCart_ManualDrive_180/smartcart'

def load_base():
    sys.path.insert(0,str(DRIVE_DIR))
    spec=importlib.util.spec_from_file_location('smartcart_real_v1_base',DRIVE_DIR/'drive.py')
    base=importlib.util.module_from_spec(spec)
    sys.modules[spec.name]=base
    spec.loader.exec_module(base)
    return base

def load_login():
    path=ROOT/'smartcart-virtual-4digit.DlQ3Pf7U/virtual_lan_server.py'
    # Loading only the reviewed standalone login definitions avoids importing
    # the mock-only drive module from the virtual application's directory.
    tree=ast.parse(path.read_text(encoding='utf-8'))
    keep=[n for n in tree.body if isinstance(n,(ast.Import,ast.ImportFrom,ast.FunctionDef,ast.Assign))
          and not (isinstance(n,ast.ImportFrom) and n.module in ('drive','smartcart_motor.manual_remote'))
          and not (isinstance(n,ast.FunctionDef) and n.name=='main')]
    namespace={}
    exec(compile(ast.Module(body=keep,type_ignores=[]),str(path),'exec'),namespace)
    return namespace

def build_app(base,config,backend,pin,address,password,start_worker=True):
    if getattr(backend,'hardware',None) is not False:
        raise ValueError('현장 안전 검수 전 실제 GPIO 출력 잠금')
    base.ROOT=HERE/'web'
    reporter=ReportingBackend(backend)
    app=base.make_app(reporter,key=pin,sensor_url='http://127.0.0.1:8765',start_worker=start_worker)
    attach(app,reporter,config)
    login=load_login()
    app.middlewares.insert(0,login['make_guard'](address,8081,password))
    return app

def main():
    parser=argparse.ArgumentParser(description='SmartCart Real UWB v1 — UWB follow + LiDAR proximity monitor')
    parser.add_argument('--address')
    parser.add_argument('--backend',choices=('gpio','mock'),default='gpio')
    args=parser.parse_args()
    if args.backend != 'mock':
        raise SystemExit('현장 안전 검수 전 실제 GPIO 출력 잠금: 이 검토본은 mock 시험만 허용합니다.')
    address=private_ip(args.address) if args.address else choose_address()
    with socket.socket() as probe:
        if sys.platform=='linux':probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        try:probe.bind((address,8081))
        except OSError as error:
            raise SystemExit('8081 unavailable: stop the previous server in its own terminal or check this Pi IP. '+str(error)) from None
    try:
        config=Config(**json.loads((HERE/'follow_config.json').read_text(encoding='utf-8')))
    except (OSError, ValueError, TypeError) as error:
        raise SystemExit('follow_config.json이 없거나 유효하지 않습니다. 실제 설정은 별도 보존하고 시험용 값으로 대체하지 마세요.') from error
    print('Automatic turn sign:',config.auto_turn_sign,'/ LiDAR yaw:',config.lidar_yaw_deg,flush=True)
    base=load_base()
    if args.backend=='gpio':
        model=Path('/proc/device-tree/model')
        if not model.is_file() or 'Raspberry Pi 4' not in model.read_text():
            raise SystemExit('GPIO backend requires Raspberry Pi 4')
    cert,new_ca=prepare(ROOT,address)
    public_ca=ROOT/'SmartCart_Real_UWB_v1_CA.crt'
    if public_ca.is_symlink() or (public_ca.exists() and not public_ca.is_file()):
        raise SystemExit('Refusing non-regular public CA export')
    public_ca.write_bytes((cert/'ca.crt').read_bytes())
    if new_ca:
        print('새 CA 인증서:',cert/'ca.crt',flush=True)
        print('브라우저 단말에 이 CA를 설치·신뢰해야 합니다. 인증서 경고를 우회하지 마세요.',flush=True)
    context=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.minimum_version=ssl.TLSVersion.TLSv1_2
    context.load_cert_chain(cert/'server.crt',cert/'server.key')
    password=getpass.getpass('화면 접속키 (숫자 4자리): ')
    if not (password.isascii() and password.isdigit() and len(password)==4) or password!=getpass.getpass('접속키 재입력: '):
        raise SystemExit('접속키 불일치')
    # Constructing RPiBackend does not open GPIO. The application's resource
    # startup opens it in the existing implementation at zero PWM.
    backend=base.RPiBackend((-1,-1)) if args.backend=='gpio' else base.MockBackend()
    app=build_app(base,config,backend,None,address,password)
    print(f'SmartCart Real UWB {VERSION} / {args.backend}',flush=True)
    print('화면: https://'+address+':8081/',flush=True)
    print('조이스틱 연결키:',app['key'],flush=True)
    print('거리 기준: BU04 측정거리 / LiDAR 감속·정지 감시 / 자동 우회 경로 없음 / PWM 상한:',config.pwm_cap,flush=True)
    print('CA 파일:',public_ca,flush=True)
    web.run_app(app,host=address,port=8081,ssl_context=context,access_log=None)

if __name__=='__main__':
    raise SystemExit(main())
