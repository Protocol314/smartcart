"""Raw LaserScan proximity monitor; not a footprint planner or certified safety scanner."""
from dataclasses import dataclass
import math
from uwb_control import number

@dataclass(frozen=True)
class Scan:
    sequence: int
    stamp: tuple
    frame_id: str
    rays: tuple
    signature: tuple
    deadline: float

class LidarGuard:
    def __init__(self,config):
        self.config=config
        self.current=None
        self.last=None
        self.frame_id=None
        self.reason='waiting_for_lidar'
        self.profile=None
        self.blocked=True
        self.clear_hits=0
        self.clear_sequence=None
        self.nearest=None
        self.unknown=0
        self.checked=0
        self.scale=0.

    def invalidate(self,reason):
        self.current=None;self.reason=str(reason);self.blocked=True
        self.clear_hits=0;self.scale=0.
        self.nearest=None;self.unknown=0;self.checked=0

    def accept(self,frame,started,now):
        try:
            if frame.get('source_mode') not in ('ros','live') or frame.get('state')!='live' or frame.get('error') is not None:
                raise ValueError('lidar_live_required')
            scan=frame.get('scan') or {}
            source=scan.get('source') or {}
            if source.get('kind')!='ros' or source.get('original_kind') not in (None,'ros'):
                raise ValueError('lidar_ros_source_required')
            if scan.get('truncated') is not False:
                raise ValueError('lidar_truncated')
            ranges=scan.get('ranges')
            if not isinstance(ranges,list) or not 180<=len(ranges)<=8192:
                raise ValueError('lidar_ray_count')
            amin=number(scan.get('angle_min'),-4*math.pi,4*math.pi)
            amax=number(scan.get('angle_max'),-4*math.pi,4*math.pi)
            step=number(scan.get('angle_increment'),-math.pi,math.pi)
            if not 0<abs(step)<=math.radians(2.):raise ValueError('lidar_angular_resolution')
            if abs((amin+(len(ranges)-1)*step)-amax)>abs(step)*.1:
                raise ValueError('lidar_angular_geometry')
            if not 2*math.pi-abs(step)*2<=abs(amax-amin)<=2*math.pi+abs(step):
                raise ValueError('lidar_full_scan_required')
            rmin=number(scan.get('range_min'),0.,1.)
            rmax=number(scan.get('range_max'),self.config.lidar_slow_m,1000.)
            if rmin>=rmax:raise ValueError('lidar_range_limits')
            scan_time=number(scan.get('scan_time'),0.,.2)
            increment=number(scan.get('time_increment',0.),0.,.2)
            duration=max(scan_time,increment*(len(ranges)-1))
            if not 0<duration<=.2:raise ValueError('lidar_scan_timing')
            age=number(frame.get('age_ms'),0.,1000*self.config.lidar_age_limit_s)/1000
            stale=number(frame.get('stale_after_ms'),1.,10000)/1000
            deadline=started+min(self.config.lidar_age_limit_s-age-duration,stale-age)
            if now>=deadline:raise ValueError('lidar_stale')
            sequence=scan.get('sequence')
            header=scan.get('header') or {}
            fid=header.get('frame_id')
            stamp=header.get('stamp') or {}
            sec,nsec=stamp.get('sec'),stamp.get('nanosec')
            if type(sequence) is not int or sequence<0 or not isinstance(fid,str) or not fid:
                raise ValueError('lidar_identity')
            if type(sec) is not int or sec<0 or type(nsec) is not int or not 0<=nsec<1000000000 or sec==nsec==0:
                raise ValueError('lidar_acquisition_stamp')
            if self.frame_id is not None and fid!=self.frame_id:
                raise ValueError('lidar_frame_changed_restart_server')
            values=tuple(float(v) if type(v) in (int,float) and math.isfinite(v) and rmin<=v<=rmax else None for v in ranges)
            signature=(amin,amax,step,rmin,rmax,duration,values)
            previous=self.last
            stamp=(sec,nsec)
            if previous is not None:
                if sequence<previous.sequence or stamp<previous.stamp:
                    raise ValueError('lidar_sequence_regression_restart_server')
                if sequence==previous.sequence or stamp==previous.stamp:
                    if signature!=previous.signature:
                        raise ValueError('lidar_changed_without_new_stamp')
                    if self.current is None or now>=self.current.deadline:
                        raise ValueError('lidar_stale')
                    self.current=Scan(previous.sequence,stamp,fid,previous.rays,signature,min(self.current.deadline,deadline))
                    return
            yaw=math.radians(self.config.lidar_yaw_deg)
            rays=tuple((math.degrees(math.atan2(math.sin(amin+i*step+yaw),math.cos(amin+i*step+yaw))),r)
                       for i,r in enumerate(values))
            self.current=self.last=Scan(sequence,stamp,fid,rays,signature,deadline)
            self.frame_id=fid
        except (ValueError,TypeError,AttributeError) as error:
            self.invalidate(str(error))

    def check(self,forward,turn,now):
        if self.current is None:raise ValueError(self.reason)
        if now>=self.current.deadline:
            self.invalidate('lidar_stale');raise ValueError(self.reason)
        if abs(forward)<.08 and abs(turn)>=.08:
            profile='pivot'
        elif abs(turn)>=.08:
            profile='left' if turn>0 else 'right'
        else:profile='front'
        if profile!=self.profile:
            self.profile=profile
            if self.blocked:
                self.clear_hits=0;self.clear_sequence=None
        required=[r for angle,r in self.current.rays if profile=='pivot' or abs(angle)<=self.config.lidar_front_half_deg
                  or profile=='left' and self.config.lidar_front_half_deg<=angle<=135
                  or profile=='right' and -135<=angle<=-self.config.lidar_front_half_deg]
        self.checked=len(required);self.unknown=sum(r is None for r in required)
        self.nearest=min((r for r in required if r is not None),default=None)
        if not required or self.unknown:
            self.reason='lidar_unknown_motion_sector';self.blocked=True;self.clear_hits=0;self.scale=0.
            raise ValueError(self.reason)
        if self.nearest<=self.config.lidar_stop_m:
            self.reason='lidar_obstacle_stop';self.blocked=True;self.clear_hits=0;self.scale=0.
            raise ValueError(self.reason)
        if self.blocked:
            if self.nearest>=self.config.lidar_resume_m:
                if self.current.sequence!=self.clear_sequence:
                    self.clear_hits+=1;self.clear_sequence=self.current.sequence
            else:self.clear_hits=0
            if self.clear_hits<3:
                self.reason='lidar_confirming_clear';self.scale=0.;raise ValueError(self.reason)
            self.blocked=False
        self.scale=min(1.,max(0.,(self.nearest-self.config.lidar_stop_m)/
                                   (self.config.lidar_slow_m-self.config.lidar_stop_m)))
        self.reason='lidar_slowdown' if self.scale<1 else 'lidar_clear'
        return self.scale

    def snapshot(self,now):
        return {'enabled':True,'reason':self.reason,'blocked':self.blocked,
                'remaining_ms':max(0.,(self.current.deadline-now)*1000) if self.current else 0.,
                'frame_id':self.frame_id,'yaw_deg':self.config.lidar_yaw_deg,
                'nearest_m':self.nearest,'unknown_rays':self.unknown,'checked_rays':self.checked,
                'clear_confirmations':self.clear_hits,'profile':self.profile,'pwm_scale':self.scale,
                'distance_reference':'lidar_scan_origin','automatic_detour':False}
