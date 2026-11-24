"""Single-BU04 controller. No LiDAR association, avoidance, odometry or wheel-speed feedback."""
from dataclasses import dataclass
import math

VERSION = '1.3.0'
OUTLIER_DISTANCE_M = 1.0
OUTLIER_INTERVAL_S = .1

def number(value, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError('invalid_numeric_input')
    return float(value)

@dataclass(frozen=True)
class Config:
    stop_distance_m: float = 2.0
    start_distance_m: float = 2.25
    age_limit_s: float = .20
    pwm_cap: float = 30.
    angle_sign: int = -1
    angle_offset_deg: float = 0.
    forward_min: float = .10
    heading_deadband_deg: float = 5.
    pivot_heading_deg: float = 35.
    max_heading_deg: float = 70.
    confirmation_samples: int = 3
    auto_turn_sign: int = 1
    lidar_enabled: bool = True
    lidar_yaw_deg: float = 0.
    lidar_age_limit_s: float = .30
    lidar_stop_m: float = .60
    lidar_resume_m: float = .80
    lidar_slow_m: float = 1.20
    lidar_front_half_deg: float = 45.

    def __post_init__(self):
        number(self.stop_distance_m,.8,3.)
        number(self.start_distance_m,self.stop_distance_m+.1,4.)
        number(self.age_limit_s,.05,.35)  # UWB_CONTINUITY_035
        number(self.pwm_cap,0,180)
        if type(self.angle_sign) is not int or self.angle_sign not in (-1,1):
            raise ValueError('invalid_angle_sign')
        number(self.angle_offset_deg,-180,180)
        number(self.forward_min,.08,.5)
        number(self.heading_deadband_deg,1,10)
        number(self.pivot_heading_deg,20,45)
        number(self.max_heading_deg,self.pivot_heading_deg,80)
        if type(self.confirmation_samples) is not int or not 3 <= self.confirmation_samples <= 10:
            raise ValueError('invalid_confirmation_count')
        if type(self.auto_turn_sign) is not int or self.auto_turn_sign not in (-1,1):
            raise ValueError('invalid_auto_turn_sign')
        if type(self.lidar_enabled) is not bool:
            raise ValueError('invalid_lidar_enabled')
        number(self.lidar_yaw_deg,-180,180)
        number(self.lidar_age_limit_s,.10,.50)
        number(self.lidar_stop_m,.40,2.)
        number(self.lidar_resume_m,self.lidar_stop_m+.10,2.50)
        number(self.lidar_slow_m,self.lidar_resume_m+.10,3.)
        number(self.lidar_front_half_deg,30,60)

@dataclass(frozen=True)
class Observation:
    sequence: int
    range_m: float
    heading_rad: float
    sample_time: float
    deadline: float

class UwbFeed:
    def __init__(self, config):
        self.config=config
        self.current=None
        self.last=None
        self.seen=None
        self.confirmations=0
        self.reason='waiting_for_uwb'
        self.outlier_count=0
        self.last_outlier=None

    def invalidate(self, reason):
        import time
        reason = str(reason)
        trusted = getattr(self, '_trusted_target', None)
        soft = (reason in ('invalid_numeric_input', 'confirming_uwb')
                or reason.startswith('uwb_http_'))
        if soft and trusted is not None and time.monotonic() < trusted.deadline:
            self.current = trusted
            self.confirmations = self.config.confirmation_samples
            self.reason = 'uwb_holding_last_valid'
            return
        self.current = None
        self._trusted_target = None
        self.confirmations = 0
        self.reason = reason

    def discard_outlier(self,now):
        # Rejected coordinates cannot become a target or renew an older target.
        if self.current is None or now>=self.current.deadline:
            self.invalidate('uwb_stale_after_outlier')
        else:
            self.reason='uwb_outlier_ignored'

    def accept(self, payload, started, now):
        try:
            if payload.get('source_mode')!='live' or payload.get('profile')!='single_left' or payload.get('state')!='running':
                raise ValueError('single_left_live_required')
            left=(payload.get('sensors') or {}).get('uwb_left') or {}
            if not all(left.get(k) is True for k in ('connected','valid','measurement_valid','id_valid')):
                raise ValueError('uwb_measurement_or_id_invalid')
            sequence=left.get('sample_sequence')
            if type(sequence) is not int or sequence<0:
                raise ValueError('invalid_uwb_sequence')
            age=number(left.get('age_ms'),0,1000*self.config.age_limit_s)/1000
            remaining=min(number(left.get('remaining_ms'),0,2000),number(payload.get('remaining_ms'),0,2000))/1000
            distance=number(left.get('range_m'),.05,10)
            angle=number(left.get('raw_angle_deg'),-180,180)
            heading=math.radians(self.config.angle_sign*angle+self.config.angle_offset_deg)
            heading=math.atan2(math.sin(heading),math.cos(heading))
            deadline=started+min(self.config.age_limit_s-age,remaining)
            sample_time=started-age
            if now>=deadline:
                raise ValueError('uwb_stale')
            seen=self.seen
            if seen is not None:
                if sequence<seen.sequence:
                    raise ValueError('uwb_sequence_regression_restart_server')
                if sequence==seen.sequence:
                    if distance!=seen.range_m or heading!=seen.heading_rad:
                        raise ValueError('uwb_sample_changed_without_sequence')
                    if self.last is None or sequence!=self.last.sequence:
                        self.discard_outlier(now)
                        return
                    if self.current is None or now>=self.current.deadline:
                        raise ValueError('awaiting_new_uwb_sample')
                    # Reading the same sample must never extend its lifetime.
                    self.current=Observation(sequence,distance,heading,self.last.sample_time,min(self.current.deadline,deadline))
                    return
                raw_dt=sample_time-seen.sample_time
                if raw_dt<=0:
                    raise ValueError('uwb_sample_time_regression')
            else:
                raw_dt=OUTLIER_INTERVAL_S
            self.seen=Observation(sequence,distance,heading,sample_time,deadline)
            previous=self.last
            if previous is not None:
                dt=sample_time-previous.sample_time
                if dt<=0:
                    raise ValueError('uwb_sample_time_regression')
                old=(previous.range_m*math.cos(previous.heading_rad),previous.range_m*math.sin(previous.heading_rad))
                new=(distance*math.cos(heading),distance*math.sin(heading))
                jump=math.dist(old,new)
                limit=OUTLIER_DISTANCE_M*raw_dt/OUTLIER_INTERVAL_S
                reacquiring=self.current is None and now>=previous.deadline
                if not reacquiring and jump+1e-9>=limit:
                    self.outlier_count+=1
                    self.last_outlier={'sequence':sequence,'jump_m':jump,'limit_m':limit,
                                       'sample_interval_ms':raw_dt*1000}
                    self.discard_outlier(now)
                    return
                # A fresh replacement supersedes the previous sample's TTL.
                # Continuity concerns sample timestamps, not the old expiry.
                if reacquiring or dt>self.config.age_limit_s:
                    self.confirmations=0
            self.current=self.last=Observation(sequence,distance,heading,sample_time,deadline)
            self.confirmations=min(self.config.confirmation_samples,self.confirmations+1)
            self.reason='ready' if self.confirmations>=self.config.confirmation_samples else 'confirming_uwb'
        except (ValueError,TypeError,AttributeError) as error:
            self.invalidate(str(error))

    def require(self, now):
        trusted = getattr(self, '_trusted_target', None)
        if self.current is None:
            raise ValueError(self.reason)
        if now >= self.current.deadline:
            self.invalidate('uwb_stale')
            raise ValueError(self.reason)
        if self.confirmations < self.config.confirmation_samples:
            if trusted is not None and now < trusted.deadline:
                return trusted
            raise ValueError('confirming_uwb')
        self._trusted_target = self.current
        return self.current


class Follower:
    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.moving = False
        self.pivot = False
        self.steering = False
        self.heading = None
        self.sequence = None
        self.sample_time = None
        self.rate = 0.
        self.turn = 0.
        self.last_command_time = None

    def command(self, observation):
        import time
        c = self.config
        now = time.monotonic()
        dt = .05 if self.last_command_time is None else min(
            .10, max(.001, now - self.last_command_time))
        self.last_command_time = now
        raw = observation.heading_rad
        distance = observation.range_m

        if abs(raw) > math.radians(c.max_heading_deg):
            self.reset()
            raise ValueError('target_outside_forward_trial')

        if observation.sequence != self.sequence:
            if self.heading is None:
                self.heading = raw
            else:
                elapsed = max(.001, min(.2,
                    observation.sample_time - self.sample_time))
                delta = math.atan2(math.sin(raw-self.heading),
                                   math.cos(raw-self.heading))
                alpha = 1. - math.exp(-elapsed/.08)
                previous = self.heading
                self.heading += alpha * delta
                measured_rate = (self.heading-previous)/elapsed
                beta = 1. - math.exp(-elapsed/.10)
                self.rate += beta * (measured_rate-self.rate)
            self.sequence = observation.sequence
            self.sample_time = observation.sample_time

        if self.moving and distance <= c.stop_distance_m:
            self.moving = False
        elif not self.moving and distance >= c.start_distance_m:
            self.moving = True
        h = self.heading
        if not self.moving:
            self.pivot = False
            if abs(h) <= math.radians(10):
                self.turn = 0.
                return 0., 0., 'distance_hold'
            error = math.copysign(
                max(0., abs(h)-math.radians(3.)), h)
            target = max(-.80, min(.80,
                1.68*error + .144*self.rate))
            change = 1.8 * dt
            self.turn += max(-change, min(change, target-self.turn))
            return 0., self.turn, 'turning_in_place'
        self.pivot = abs(h) >= math.radians(20)

        deadband = math.radians(3.)
        error = math.copysign(max(0., abs(h)-deadband), h)
        # P turns toward target; D damps angle changes.
        # Use higher PD gains only while following forward.
        kp, kd = (1.68, .144) if self.pivot else (2.52, .216)
        target = max(-.80, min(.80, kp * error + kd * self.rate))
        if abs(h) < deadband and abs(self.rate) < .08:
            target = 0.
        # Limit steering-command changes; existing wheel ramp stays active.
        change = 1.8 * dt
        self.turn += max(-change, min(change, target-self.turn))

        if abs(h) >= math.radians(8):
            self.steering = True
        elif abs(h) <= math.radians(5):
            self.steering = False
        if not getattr(self, 'steering', False):
            self.turn = 0.
        if self.pivot:
            return 0., self.turn, 'turning_in_place'

        # Move forward while steering; reduce approach speed and corner speed.
        if distance <= 2.:
            forward = .70
        elif distance <= 3.:
            forward = .80
        elif distance <= 4.:
            forward = .90
        else:
            forward = 1.
        return forward, self.turn, 'following'
