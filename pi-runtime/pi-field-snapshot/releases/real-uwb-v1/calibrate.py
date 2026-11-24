#!/usr/bin/env python3
"""Read-only direction/scan checks; optional backed-up config edits while the server is stopped."""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import shutil
import socket
import sys
import time
from urllib.request import urlopen
from uwb_control import Config,UwbFeed,Follower
from lidar_guard import LidarGuard

def main():
    parser=argparse.ArgumentParser(description='Read sensor/calibration values without opening GPIO')
    parser.add_argument('--auto-turn-sign',type=int,choices=(-1,1))
    parser.add_argument('--lidar-yaw',type=float)
    args=parser.parse_args()
    here=Path(__file__).resolve().parent
    path=here/'follow_config.json'
    values=json.loads(path.read_text(encoding='utf-8'))
    if args.auto_turn_sign is not None or args.lidar_yaw is not None:
        with socket.socket() as probe:
            if sys.platform=='linux':probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            try:probe.bind(('0.0.0.0',8081))
            except OSError:raise SystemExit('Stop the 8081 server in its own terminal with Ctrl+C first')
        if args.auto_turn_sign is not None:values['auto_turn_sign']=args.auto_turn_sign
        if args.lidar_yaw is not None:values['lidar_yaw_deg']=args.lidar_yaw
        Config(**values)
        backup=Path.home()/'smartcart/archive'/('before-calibration-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
        backup.mkdir(parents=True);shutil.copy2(path,backup/path.name)
        path.write_text(json.dumps(values,indent=2)+'\n',encoding='utf-8')
        print('Saved settings. Backup:',backup)
        print('Run: bash ~/smartcart/real.sh')
        return
    config=Config(**values);lidar=LidarGuard(config);uwb=UwbFeed(config);follow=Follower(config)
    print('GPIO is not opened. Auto turn sign:',config.auto_turn_sign,'LiDAR yaw:',config.lidar_yaw_deg,flush=True)
    print('Place the tag >1m in front-left: physical automatic turn should be positive (left).',flush=True)
    print('Check LiDAR FRONT with a real object, then LEFT/RIGHT. Distances are from the scanner.',flush=True)
    for _ in range(5):
        try:
            started=time.monotonic()
            with urlopen('http://127.0.0.1:8765/api/frame',timeout=2) as response:frame=json.load(response)
            now=time.monotonic();lidar.accept(frame,started,now)
            try:lidar.check(1,0,now)
            except ValueError:pass
            sectors={'FRONT':None,'LEFT':None,'RIGHT':None,'REAR':None}
            if lidar.current:
                for angle,r in lidar.current.rays:
                    name='FRONT' if abs(angle)<=30 else 'LEFT' if 60<=angle<=120 else 'RIGHT' if -120<=angle<=-60 else 'REAR' if abs(angle)>=150 else None
                    if name and r is not None:sectors[name]=r if sectors[name] is None else min(sectors[name],r)
            print('LiDAR:',lidar.frame_id,lidar.reason,'sector min m:',sectors,flush=True)
            started=time.monotonic()
            with urlopen('http://127.0.0.1:8765/api/dual-uwb',timeout=2) as response:data=json.load(response)
            uwb.accept(data,started,time.monotonic())
            try:
                obs=uwb.require(time.monotonic());f,t,reason=follow.command(obs)
                print('UWB range:',round(obs.range_m,3),'display heading:',round(math.degrees(obs.heading_rad),2),
                      'auto turn (+left/-right):',round(t*config.auto_turn_sign,3),reason,flush=True)
            except ValueError as error:print('UWB:',str(error),flush=True)
        except Exception as error:print('Read failed:',type(error).__name__,str(error),flush=True)
        time.sleep(.12)

if __name__=='__main__':main()
