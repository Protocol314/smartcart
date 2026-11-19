"""Bridge to the installed GPIO implementation, preserving its ownership, ramp and watchdog."""
import asyncio
import contextlib
import math
import time
from aiohttp import ClientTimeout
from uwb_control import VERSION, UwbFeed, Follower
from lidar_guard import LidarGuard

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
        self.drive,self.reporter,self.config,self.clock=drive,reporter,config,clock
        self.feed=UwbFeed(config)
        self.follower=Follower(config)
        self.lidar=LidarGuard(config) if config.lidar_enabled else None
        self.intent_axes=(0.,0.)
        self.enabled=False
        self.paused=False
        self.reason='off'
        self.axes=(0.,0.)

    def __getattr__(self,name):
        return getattr(self.drive,name)

    def reset_mode(self,reason='off'):
        self.enabled=False
        self.paused=False
        self.reason=reason
        self.axes=(0.,0.)
        self.follower.reset()
        self.intent_axes=(0.,0.)

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
            'reference':'BU04_measured_slant_range','obstacle_avoidance':False,
            'lidar_enabled':self.lidar is not None,'auto_turn_sign':self.config.auto_turn_sign,
            'stop_distance_m':self.config.stop_distance_m,
            'start_distance_m':self.config.start_distance_m,'pwm_cap':self.config.pwm_cap,
            'confirmations':self.feed.confirmations,
            'outlier_count':self.feed.outlier_count,'last_outlier':self.feed.last_outlier,
        }
        state['lidar_guard']=self.lidar.snapshot(self.clock()) if self.lidar else {'enabled':False}
        state['sensor_interlock']=bool(self.enabled and self.lidar)
        return state

    def release(self,owner):
        if self.drive.owner==owner:
            self.reset_mode('disconnected')
        self.drive.release(owner)

    def pause_output(self,reason):
        """Brake without extending the command watchdog or revoking session arming."""
        was_paused=self.paused
        self.paused=True
        self.reason=str(reason)
        self.axes=(0.,0.)
        self.follower.reset()
        now=self.clock()
        with self.drive.lock:
            if not self.drive.armed:
                return
            if was_paused and not (self.drive.forward or self.drive.turn or any(self.drive.output)):
                return
            self.drive.forward=self.drive.turn=0.
            self.drive.ramp.stop(now)
            self.drive.output=(0,0)
            try:
                self.drive.backend.stop()
            except Exception as error:
                self.drive.fault=str(error)
                self.drive._stop('gpio_fault',now)
                self.reset_mode('gpio_fault')
                print('UWB_AUTO_STOP: gpio_fault',flush=True)

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
        try:
            if not self.drive.snapshot()['armed']:
                raise ValueError('rearm_required')
            pwm=data.get('pwm')
            if type(pwm) not in (int,float) or not math.isfinite(pwm) or not 0<=pwm<=180:
                raise ValueError('invalid_pwm')
            try:
                observation=self.feed.require(self.clock())
                forward,turn,reason=self.follower.command(observation)
                turn*=self.config.auto_turn_sign
                self.intent_axes=(forward,turn)
                scale=self.lidar.check(forward,turn,self.clock()) if self.lidar else 1.
                if scale<1:reason='lidar_slowdown'
                paused=False
            except ValueError as error:
                # Fresh authenticated heartbeats keep arming alive, but an invalid
                # target can only produce neutral output. No heartbeat is forged.
                self.pause_output(str(error))
                forward=turn=0.
                reason=str(error)
                paused=True
                scale=0.
            command=dict(data,type='drive',forward=forward,turn=turn,pwm=min(pwm,self.config.pwm_cap)*scale)
            # Original owner/epoch/sequence/stamp validation remains in force.
            self.drive.command(owner,command)
            if not self.drive.snapshot()['armed']:
                raise ValueError('rearm_required')
            self.enabled,self.paused,self.reason,self.axes=True,paused,reason,(forward,turn)
        except Exception as error:
            self.reset_mode(str(error))
            print('UWB_AUTO_STOP:', str(error), flush=True)
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
        async def poll_lidar():
            while True:
                started=time.monotonic()
                try:
                    async with application['client'].get(
                        application['sensor_url']+'/api/frame',
                        timeout=ClientTimeout(total=.18),allow_redirects=False,
                        headers={'Cache-Control':'no-cache'},
                    ) as response:
                        response.raise_for_status();data=await response.json()
                    bridge.lidar.accept(data,started,time.monotonic())
                except Exception as error:
                    bridge.lidar.invalidate('lidar_http_'+type(error).__name__)
                bridge.monitor_sensor()
                await asyncio.sleep(.05)
        laser_task=asyncio.create_task(poll_lidar()) if bridge.lidar else None
        try:
            yield
        finally:
            task.cancel()
            if laser_task:laser_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task
            if laser_task:
                with contextlib.suppress(asyncio.CancelledError):await laser_task
    app.cleanup_ctx.append(resources)
    return bridge
