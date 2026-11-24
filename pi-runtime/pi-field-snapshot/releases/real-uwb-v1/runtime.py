# REACTIVE_STREAM_FAST_V1
# REACTIVE_RECOVERY_V1
# REACTIVE_LIDAR_V1
"""Bridge to the installed GPIO implementation, preserving its ownership, ramp and watchdog."""
import asyncio
import contextlib
import math
import time
from aiohttp import ClientTimeout
from uwb_control import VERSION, UwbFeed, Follower
from zone_live import ZoneGuard as LidarGuard  # LIVE_ZONE_CONNECTION_V1

class ReportingBackend:
    def __init__(self, backend):
        self.backend=backend
        self.drive=None
        self.applied_seq=-1

    def __getattr__(self, name):
        return getattr(self.backend,name)

    def write(self,left,right):
        self.backend.write(left,right)
        # Called by the existing tick with its lock held, after successful write.
        if self.drive is not None:
            self.applied_seq=self.drive.sequence

class DriveBridge:
    def __init__(self, drive, reporter, config, clock=time.monotonic):
        if getattr(reporter.backend, 'hardware', None) is not False:
            raise ValueError('현장 안전 검수 전 실제 GPIO 출력 잠금')
        self.drive,self.reporter,self.config,self.clock=drive,reporter,config,clock
        self.feed=UwbFeed(config)
        self.follower=Follower(config)
        self.lidar=LidarGuard(config) if config.lidar_enabled else None
        if self.lidar:self.lidar.controller=self.follower  # POINT_SEGMENT_INTEGRATION_V1
        self.intent_axes=(0.,0.)
        self.enabled=False
        self.paused=False
        self.reason='off'
        self.axes=(0.,0.)

    def __getattr__(self,name):
        return getattr(self.drive,name)

    def reset_mode(self,reason='off'):
        if self.enabled:print('AUTO_MODE_OFF:',reason,'DRIVER:',self.drive.snapshot().get('reason'),'FAULT:',self.drive.snapshot().get('fault'),flush=True)
        self.enabled=False
        self.paused=False
        self.reason=reason
        self.axes=(0.,0.)
        self.follower.reset()
        self.intent_axes=(0.,0.)
        if self.lidar:self.lidar.reset_recovery()

    def snapshot(self):
        # Read the earlier applied sequence first, so it never acknowledges a newer
        # motor write than the PWM values in the following locked snapshot.
        applied=self.reporter.applied_seq
        state=self.drive.snapshot()
        state.setdefault('applied_seq',applied)
        obs=self.feed.current
        state['controller_version']=VERSION
        state['autonomy']={
            'enabled':self.enabled,'paused':self.paused,'reason':self.reason,'sensor_reason':self.feed.reason,
            'range_m':obs.range_m if obs else None,
            'heading_deg':math.degrees(obs.heading_rad) if obs else None,
            'remaining_ms':max(0.,(obs.deadline-self.clock())*1000) if obs else 0.,
            'forward':self.axes[0],'turn':self.axes[1],
            'reference':'BU04_measured_slant_range','obstacle_avoidance':self.lidar is not None,
            'lidar_enabled':self.lidar is not None,'auto_turn_sign':self.config.auto_turn_sign,
            'stop_distance_m':self.config.stop_distance_m,
            'start_distance_m':self.config.start_distance_m,'pwm_cap':self.config.pwm_cap,
            'confirmations':self.feed.confirmations,
            'outlier_count':self.feed.outlier_count,'last_outlier':self.feed.last_outlier,
        }
        state['lidar_guard']=self.lidar.snapshot(self.clock()) if self.lidar else {'enabled':False}
        state['sensor_interlock']=False
        return state

    def release(self,owner):
        if self.drive.owner==owner:
            self.reset_mode('disconnected')
        self.drive.release(owner)

    def pause_output(self,reason):
        """센서 이상 시 정지하고 재활성화를 요구한다."""
        now=self.clock()
        with self.drive.lock:
            if self.drive.armed:
                self.drive._stop(str(reason),now)
        self.reset_mode(str(reason))

    def monitor_sensor(self):
        if not self.enabled:
            if self.lidar:
                try:self.lidar.check(1.,0.,self.clock())
                except ValueError:pass
            return
        try:
            self.feed.require(self.clock())
            if self.lidar:self.lidar.check(*self.intent_axes,self.clock())
        except ValueError as error:
            if not self.paused or self.reason!=str(error):
                pass  # Pause status remains available on screen
            self.pause_output(str(error))

    def command(self,owner,data):
        if not isinstance(data,dict) or data.get('type')!='auto':
            self.drive.command(owner,data)
            self.reset_mode()
            return
        if owner!=self.drive.owner:
            raise ValueError('not_owner')
        # DROP_OLD_DRIVE_EPOCH_V1
        epoch = data.get("epoch")
        if type(epoch) is int and 0 <= epoch < self.drive.epoch:
            return
        try:
            if not self.drive.snapshot()['armed']:
                raise ValueError('rearm_required')
            pwm=data.get('pwm')
            if type(pwm) not in (int,float) or not math.isfinite(pwm) or not 0<=pwm<=180:
                raise ValueError('invalid_pwm')
            try:
                observation=self.feed.require(self.clock())
                forward,turn,reason=self.follower.command(observation)
                if self.lidar:
                    forward,turn,reason=self.lidar.steer(observation,forward,turn,reason,self.clock())
                turn*=self.config.auto_turn_sign
                self.intent_axes=(forward,turn)
                scale=self.lidar.check(forward,turn,self.clock()) if self.lidar else 1.
                if scale<1:reason='lidar_slowdown'
                paused=False
            except ValueError as error:
                self.pause_output(str(error))
                raise
            command=dict(data,type='drive',forward=forward,turn=turn,pwm=min(pwm,self.config.pwm_cap,self.lidar.output_pwm_cap() if self.lidar else self.config.pwm_cap)*scale)
            # Original owner/epoch/sequence/stamp validation remains in force.
            # DROP_LATE_AUTO_PACKET_V1
            now = self.drive.clock()
            timeout = self.drive.command.__func__.__globals__.get("COMMAND_TIMEOUT")
            stamp = command.get("stamp")
            seq = command.get("seq")
            epoch = command.get("epoch")
            if (
                type(timeout) in (int, float) and timeout > 0
                and type(stamp) in (int, float) and 0 <= stamp <= now
                and now - stamp >= timeout
                and type(seq) is int and self.drive.sequence < seq < 2**53
                and type(epoch) is int and epoch == self.drive.epoch
                and self.drive.armed and not self.drive.fault
            ):
                self.pause_output("auto_packet_late")
                print(f"AUTO_PACKET_LATE: age_ms={(now-stamp)*1000:.0f}", flush=True)
                return
            try:
                if self.drive.command(owner,command) is False:
                    return
            except (ValueError, TypeError):
                print(
                    f"AUTO_CMD_REJECT: stamp={stamp!r} now={now:.3f} "
                    f"seq={seq!r}/{self.drive.sequence} "
                    f"epoch={epoch!r}/{self.drive.epoch} "
                    f"forward={forward!r} turn={turn!r} pwm={command.get('pwm')!r}",
                    flush=True,
                )
                raise
            if not self.drive.snapshot()['armed']:
                raise ValueError('rearm_required')
            self.enabled,self.paused,self.reason,self.axes=True,paused,reason,(forward,turn)
        except Exception as error:
            self.reset_mode(str(error))
            print('UWB_AUTO_STOP:', str(error), flush=True)
            if self.drive.snapshot()['armed']:
                self.drive.command(owner,{'type':'stop'})
            raise ValueError(str(error)) from None

def attach(app, reporter, config):
    reporter.drive=app['drive']
    bridge=DriveBridge(app['drive'],reporter,config)
    app['drive']=bridge

    async def resources(application):
        async def poll():
            while True:
                started=time.monotonic()
                try:
                    async with application['client'].get(
                        application['sensor_url']+'/api/dual-uwb',
                        timeout=ClientTimeout(total=.18),allow_redirects=False,
                        headers={'Cache-Control':'no-cache'},
                    ) as response:
                        response.raise_for_status()
                        data=await response.json()
                    bridge.feed.accept(data,started,time.monotonic())
                except Exception as error:
                    bridge.feed.invalidate('uwb_http_'+type(error).__name__)
                bridge.monitor_sensor()
                await asyncio.sleep(.04)
        task=asyncio.create_task(poll())
        # SMARTCART_LATEST_PIPELINE_V1
        import copy
        pending_frame = None
        frame_generation = 0

        async def poll_lidar():
            nonlocal pending_frame, frame_generation
            while True:
                started=time.monotonic()
                try:
                    async with application['client'].get(
                        application['sensor_url']+'/api/frame',
                        timeout=ClientTimeout(total=.18),allow_redirects=False,
                        headers={'Cache-Control':'no-cache'},
                    ) as response:
                        response.raise_for_status()
                        data=await response.json()
                    frame_generation += 1
                    # Single slot: overwrite old unprocessed input, never queue it.
                    pending_frame=(frame_generation,data,started,time.monotonic(),None)
                except Exception as error:
                    frame_generation += 1
                    reason='lidar_http_'+type(error).__name__
                    bridge.lidar.invalidate(reason)
                    bridge.monitor_sensor()
                    pending_frame=(frame_generation,None,started,time.monotonic(),reason)
                await asyncio.sleep(.03)

        def decode_frame(working,packet):
            _,data,started,received,error=packet
            if error:working.invalidate(error)
            else:working.accept(data,started,received)
            return working

        async def decode_latest():
            seen=-1
            last_key=None
            last_new_at=time.monotonic()
            offline=False
            while True:
                packet=pending_frame
                if not offline and time.monotonic()-last_new_at>=4.0:
                    offline=True
                    print('LIDAR_STREAM_LOST: no new valid scan for 4s',flush=True)
                if packet is None or packet[0]==seen:
                    await asyncio.sleep(.005)
                    continue
                seen=packet[0]
                if packet[4] is not None:
                    bridge.lidar.invalidate(packet[4])
                    bridge.monitor_sensor()
                    continue
                working=copy.copy(bridge.lidar)
                try:
                    working=await asyncio.to_thread(decode_frame,working,packet)
                    scan=working.current
                    if scan is not None and time.monotonic()<scan.deadline:
                        key=(scan.frame_id,scan.sequence)
                        if key!=last_key:
                            last_key=key
                            last_new_at=time.monotonic()
                            if offline:
                                offline=False
                                print('LIDAR_STREAM_RECOVERED',flush=True)
                    # Publish only input fields. Do not overwrite steering/STOP state.
                    for name in ('current','last','frame_id','frame','obstacles','packet','decode_ms'):
                        if hasattr(working,name):setattr(bridge.lidar,name,getattr(working,name))
                    if working.current is None:
                        bridge.lidar.invalidate(working.reason)
                    elif bridge.lidar.reason=='waiting_for_lidar':
                        bridge.lidar.reason='lidar_received'
                except Exception as error:
                    bridge.lidar.invalidate('lidar_decode_'+type(error).__name__)
                bridge.monitor_sensor()

        async def plan_latest():
            previous=None
            while True:
                laser=bridge.lidar
                scan=laser.current
                observation=bridge.feed.current
                key=(getattr(scan,'sequence',None),getattr(observation,'sequence',None))
                now=time.monotonic()
                if (scan is None or observation is None or key==previous
                        or now>=min(scan.deadline,observation.deadline)):
                    await asyncio.sleep(.01)
                    continue
                previous=key
                # Immutable input snapshot; the receive/decode tasks keep running.
                working=copy.copy(laser)
                try:
                    await asyncio.to_thread(working.plan,observation)
                    current=laser.current
                    if (current is not None and current.frame_id==scan.frame_id and current.sequence==scan.sequence
                            and bridge.feed.current is not None and bridge.feed.current.sequence==observation.sequence
                            and time.monotonic()<min(scan.deadline,observation.deadline)):
                        for name in ('route_state','solution','plan_reason','plan_ms',
                                     'planned_sequence','preferred'):
                            if hasattr(working,name):setattr(laser,name,getattr(working,name))
                except Exception as error:
                    laser.plan_reason='lidar_planner_'+type(error).__name__
                    if hasattr(laser,'route_state'):laser.route_state=None
                    if hasattr(laser,'solution'):laser.solution=None
                bridge.monitor_sensor()
                await asyncio.sleep(.01)

        workers=[]
        if bridge.lidar:
            workers=[asyncio.create_task(fn()) for fn in
                     (poll_lidar,decode_latest)]
        try:
            yield
        finally:
            task.cancel()
            for worker in workers:worker.cancel()
            await asyncio.gather(task,*workers,return_exceptions=True)
    app.cleanup_ctx.append(resources)
    return bridge
