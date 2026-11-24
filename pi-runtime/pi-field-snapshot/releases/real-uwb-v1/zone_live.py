"""UWB follow with direct LiDAR slowdown, hard stop, wait and side bias.
No route planner or motion-limit file is used. Thresholds are geometric
triggers, not a guarantee that the physical cart can brake within 10cm.
"""
import copy
import json
import math
import time
from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace
from lidar_guard import LidarGuard
from reactive_lidar import build_scene,assess,unknown_sector,rotation_clear,motion_visibility,rear_clear

DEFAULTS=dict(stop_m=.10,slow_m=.30,stop_resume_m=.15,front_resume_m=.35,
              front_half_deg=15.,push_heading_deg=12.,rotation_preview_deg=20.,
              resume_scans=3,join_gap_m=.15,pwm_cap=80.)
DEFAULTS.update({'lookahead_m': 0.3, 'reverse_wait_s': 5.0, 'reverse_pulse_s': 0.25, 'reverse_axis': 0.25, 'reverse_pwm_cap': 40.0, 'reverse_lookahead_m': 0.2, 'recovery_brake_s': 0.2, 'recovery_boost_s': 2.0, 'recovery_heading_deg': 18.0, 'recovery_reset_clear_s': 2.0})

class ZoneGuard(LidarGuard):
    def __init__(self,config):
        super().__init__(config)
        root=Path(__file__).parent
        self.geometry=json.loads((root/'live_zone_modules/cart_geometry.json').read_text(encoding='utf-8'))
        g=self.geometry
        keys=('front_extent_m','rear_extent_m','width_m','lidar_x_m','lidar_y_m')
        if (g.get('reference')!='front_track_midpoint' or not all(type(g.get(k)) in (int,float)
                and math.isfinite(g[k]) for k in keys)
                or min(g['front_extent_m'],g['rear_extent_m'],g['width_m'])<=0):
            raise ValueError('reactive_geometry_invalid')
        self.rules=dict(DEFAULTS)
        p=root/'reactive_lidar_config.json'
        if p.exists():self.rules.update(json.loads(p.read_text(encoding='utf-8')))
        self.rules['reverse_pulse_s'] = 2.0
        self.rules['reverse_wait_s'] = 5.0
        r=self.rules
        if not all(type(r[k]) in (int,float) and math.isfinite(r[k]) for k in DEFAULTS):
            raise ValueError('reactive_config_invalid')
        if not (0<r['stop_m']<r['stop_resume_m']<r['slow_m']<r['front_resume_m']<2
                and 0<r['front_half_deg']<=45 and 0<=r['push_heading_deg']<=20
                and 0<r['rotation_preview_deg']<=20 and 0<r['join_gap_m']<=.3
                and type(r['resume_scans']) is int and 1<=r['resume_scans']<=10
                and 0<r['pwm_cap']<=180):raise ValueError('reactive_config_invalid')
        if not (1<=r['reverse_wait_s']<=60 and 0<r['reverse_pulse_s']<=2. and .08<=r['reverse_axis']<=.5 and 0<r['reverse_pwm_cap']<=80 and .10<=r['reverse_lookahead_m']<=.3 and .1<=r['recovery_brake_s']<=2 and 0<r['recovery_boost_s']<=5 and 0<r['recovery_heading_deg']<=20 and 0<r['lookahead_m']<=.5 and 0<r['recovery_reset_clear_s']<=10):raise ValueError('reactive_recovery_config_invalid')
        self.controller=None
        self.frame=self.packet=None
        self.solution=self.route_state=None
        self.plan_reason='reactive_no_path_planner'
        self.plan_ms=0.;self.planned_sequence=None
        self.preferred=0
        self._permit=None;self._heading=0.
        self._halt=None;self._clear=0;self._clear_sequence=None
        self.metrics=None
        self.reset_recovery()
        self.scale=0.

    def accept(self,frame,started,now):
        previous=self.packet
        began=time.perf_counter()
        super().accept(frame,started,now)
        self.frame=frame if self.current else None
        if self.current is None:self.packet=None
        elif (previous is not None
              and previous[0].sequence==self.current.sequence
              and previous[0].frame_id==self.current.frame_id):
            # Parent already checked same-sequence signatures and deadline.
            # Reuse geometry only. NEVER renew the original sample lifetime.
            self.packet=(self.current,previous[1])
        else:
            self.packet=(self.current,build_scene(self.current,self.geometry,self.rules['join_gap_m']))
        self.decode_ms=(time.perf_counter()-began)*1000.

    def invalidate(self,reason):
        super().invalidate(reason)
        self.packet=None;self._permit=None

    def plan(self,observation):
        # Existing receive/decode/planner scheduling remains compatible.
        # All reactive decisions use the latest decoded scan synchronously.
        self.planned_sequence=self.current.sequence if self.current else None
        self.plan_reason='reactive_no_path_planner'
        self.plan_ms=0.

    def fresh(self,now):
        if self.current is None:self.fail(self.reason)
        if now>=self.current.deadline:self.fail('lidar_stale')
        if self.packet is None or self.packet[0].sequence!=self.current.sequence:
            self.fail('reactive_waiting_for_scan_decode')
        return self.packet[1]

    def fail(self,reason):
        self.reason=str(reason);self.blocked=True;self.scale=0.
        raise ValueError(self.reason)

    # UWB_FRONT_STOP_ONLY_V1
    def _gate(self, scene, forward, turn, heading, advance=True):
        import math

        if forward < 0:
            self.fail("reactive_auto_reverse_disabled")

        # scene.rays의 각도는 설치된 180도 보정이 적용된 차체 기준 각도다.
        sector = [r for a, r in scene.rays if abs(a) <= 30.0]
        valid = [r for r in sector if r is not None and math.isfinite(r) and r > 0]
        self.checked = len(sector)
        self.unknown = len(sector) - len(valid)

        if forward or turn:
            if not valid:
                self.fail("reactive_front_no_valid_scan")

            if min(valid) <= 1.0:
                self._front_block_seq = self.current.sequence
                if not self._matched_pivot_allowed(scene, forward, turn, heading):
                    self.fail("reactive_front_obstacle_1m")

            blocked_seq = getattr(self, "_front_block_seq", None)
            if blocked_seq is not None and min(valid) > 1.0:
                if self.current.sequence == blocked_seq or self.unknown:
                    self.fail("reactive_front_wait_clear")
                self._front_block_seq = None

            # 제자리 회전은 앞만 봐서는 뒤 모서리 충돌을 확인할 수 없다.
            if abs(turn) > 1e-9:
                angle = math.copysign(
                    math.radians(self.rules["rotation_preview_deg"]), turn
                )
                if not rotation_clear(scene, self.geometry, angle, self.rules["stop_m"]):
                    self.fail("reactive_rear_corner_sweep_blocked")

        self._halt = None
        self.metrics = {"minimum": scene.minimum,
                        "front": min(valid, default=math.inf), "scale": 1.0}
        return self.metrics


    def _follow_steer(self,observation,forward,turn,reason,now):
        scene=self.fresh(now)
        if now>=observation.deadline:self.fail('uwb_stale')
        h=observation.heading_rad
        m=self._gate(scene,forward,turn,h)
        if forward>0. and abs(m['push'])>1e-9:
            if self.controller is None:self.fail('reactive_controller_not_connected')
            h=h+math.radians(self.rules['push_heading_deg'])*m['push']
            h=math.atan2(math.sin(h),math.cos(h))
            try:virtual=replace(observation,heading_rad=h)
            except TypeError:
                virtual=SimpleNamespace(**vars(observation));virtual.heading_rad=h
            # Use installed steering/deadband/pivot logic; no new motor sign.
            follower=copy.copy(self.controller)
            forward,turn,reason=follower.command(virtual)
            m=self._gate(scene,forward,turn,h,advance=False)
            reason='reactive_push_left' if m['push']>0 else 'reactive_push_right'
        self._heading=h
        signed=turn*self.config.auto_turn_sign
        self._permit=(forward,signed,observation.deadline)
        self.metrics=m
        self.reason=reason if m['scale']>=1 else 'reactive_slowdown'
        return forward,turn,reason


    def snapshot(self,now):
        state=super().snapshot(now)
        m=self.metrics or {}
        def finite(v):return v if v is not None and math.isfinite(v) else None
        state.update(automatic_detour=False,reactive_avoidance=True,
            control_mode='reactive',body_clearance_m=finite(m.get('minimum')),
            front_body_clearance_m=finite(m.get('front')),
            left_body_clearance_m=finite(m.get('left')),
            right_body_clearance_m=finite(m.get('right')),
            decode_ms=getattr(self,'decode_ms',0.),visibility_ms=getattr(self,'visibility_ms',0.),
            stop_body_m=self.rules['stop_m'],slow_body_m=self.rules['slow_m'],
            route_state='STOP' if self.blocked else 'FOLLOW',
            route_reason=self.reason,waiting_for=self._halt,
            route_points=(),plan_ms=0.,pwm_limit=self.output_pwm_cap(),recovery_phase=self._phase,recovery_used=self._used,recovery_wait_since=self._wait_since,
            body_distance_reference='front_track_midpoint_rectangle_surface')
        return state

    def reset_recovery(self):
        self._phase='FOLLOW';self._phase_until=0.;self._boost_until=0.
        self._wait_since=None;self._used=False;self._clear_since=None
        self._recovery_side=1;self._last_auto=None
        self._visibility_scene=None;self._visibility_values={}

    def _visibility(self,scene,forward,turn,heading,length):
        if self._visibility_scene is not scene:
            self._visibility_scene=scene;self._visibility_values={}
        direction=0 if abs(forward)<.08 else 1 if forward>0 else -1
        rotation=0 if abs(turn)<.08 else 1 if turn>0 else -1
        key=(direction,rotation,heading,length,self.rules['rotation_preview_deg'])
        if key not in self._visibility_values:
            began=time.perf_counter()
            value=motion_visibility(scene,self.geometry,forward,turn,heading,length,self.rules['rotation_preview_deg'])
            self.visibility_ms=(time.perf_counter()-began)*1000.
            if len(self._visibility_values)>12:self._visibility_values={}
            self._visibility_values[key]=value
        return self._visibility_values[key]

    def _rear_ok(self,scene):
        visible=self._visibility(scene,-1.,0.,0.,self.rules['reverse_lookahead_m'])
        return rear_clear(scene,self.geometry,self.rules,visible)

    def output_pwm_cap(self):
        cap=self.rules['pwm_cap']
        return min(cap,self.rules['reverse_pwm_cap']) if self._phase=='REVERSE' else cap

    def _original__note_block(self,reason,now):
        if reason in ('reactive_front_wait','reactive_rear_corner_sweep_blocked','reactive_unknown_swept_space'):
            if self._wait_since is None:self._wait_since=now
            self._clear_since=None
            m=self.metrics or {}
            left,right=m.get('left',math.inf),m.get('right',math.inf)
            if right<left:self._recovery_side=1
            elif left<right:self._recovery_side=-1
        else:
            self._wait_since=None
            self._clear_since=None

    def _reverse(self,scene,observation,now):
        if now>=self._phase_until:
            self._phase='BRAKE';self._phase_until=now+self.rules['recovery_brake_s']
            self.fail('reactive_recovery_brake')
        self._gate(scene,-self.rules['reverse_axis'],0.,0.)
        self._heading=0.;self.reason='reactive_short_reverse'
        self._permit=(-self.rules['reverse_axis'],0.,observation.deadline)
        return -self.rules['reverse_axis'],0.,self.reason

    def _original_steer(self,observation,forward,turn,reason,now):
        scene=self.fresh(now)
        if now>=observation.deadline:self.fail('uwb_stale')
        self._last_auto=now
        # A deliberate UWB distance-hold never triggers an escape.
        if not (forward or turn):
            self.reset_recovery()
            return self._follow_steer(observation,forward,turn,reason,now)
        if self._phase=='REVERSE':
            return self._reverse(scene,observation,now)
        if self._phase=='BRAKE':
            if now<self._phase_until:self.fail('reactive_recovery_brake')
            self._phase='BOOST';self._boost_until=now+self.rules['recovery_boost_s']
        if self._phase=='BOOST' and now>=self._boost_until:self._phase='FOLLOW'
        target=observation
        if self._phase=='BOOST':
            h=observation.heading_rad+self._recovery_side*math.radians(self.rules['recovery_heading_deg'])
            h=math.atan2(math.sin(h),math.cos(h))
            try:target=replace(observation,heading_rad=h)
            except TypeError:
                target=SimpleNamespace(**vars(observation));target.heading_rad=h
            follower=copy.copy(self.controller)
            forward,turn,reason=follower.command(target)
        try:
            result=self._follow_steer(target,forward,turn,reason,now)
            self._wait_since=None
            if self._phase=='BOOST':
                self.reason='reactive_recovery_follow'
                result=(result[0],result[1],self.reason)
            if (self.metrics or {}).get('minimum',0.)>self.rules['slow_m']:
                if self._clear_since is None:self._clear_since=now
                elif now-self._clear_since>=self.rules['recovery_reset_clear_s']:self._used=False
            else:self._clear_since=None
            return result
        except ValueError as error:
            self._note_block(str(error),now)
            if (self._wait_since is not None and not self._used
                    and self._phase=='FOLLOW'
                    and now-self._wait_since>=self.rules['reverse_wait_s']):
                # Forward-only unknowns may trigger a single retreat only when
                # the rear is independently observed. Stale/invalid feeds,
                # <=10cm hard stop, manual STOP and missing rear coverage cannot.
                ok,why=self._rear_ok(scene)
                if not ok:self.fail(why)
                self._phase='REVERSE';self._used=True
                self._phase_until=now+self.rules['reverse_pulse_s']
                self._wait_since=None
                return self._reverse(scene,observation,now)
            raise

    def check(self,forward,turn,now):
        scene=self.fresh(now)
        if forward<0:
            if self._phase!='REVERSE':self.fail('reactive_reverse_not_active')
            if now>=self._phase_until:
                self._phase='BRAKE';self._phase_until=now+self.rules['recovery_brake_s']
                self.fail('reactive_recovery_brake')
        try:
            m=self._gate(scene,forward,turn/self.config.auto_turn_sign,self._heading)
        except ValueError as error:
            if self._last_auto is not None and now-self._last_auto<max(.5,self.config.age_limit_s*2):
                self._note_block(str(error),now)
            raise
        self.scale=m['scale'];self.blocked=False
        self.reason=('reactive_short_reverse' if forward<0 else
                     'reactive_recovery_follow' if self._phase=='BOOST' else
                     'reactive_slowdown' if self.scale<1 else 'reactive_follow')
        self.nearest=min((v for a,v in self.current.rays if v is not None),default=None)
        return self.scale


    # CONTOUR_RECOVERY_UWB_PRIORITY_V1
    def _long_front_contour(self):
        from reactive_lidar import box_hit

        scene = self.packet[1]
        front = self.geometry["front_extent_m"]
        half_width = self.geometry["width_m"] / 2
        corridor = (front, front + self.rules["slow_m"], -half_width, half_width)

        # LiDAR 이웃 선분을 연결하고, 같은 선분은 한 번만 센다.
        links = {}
        for a, b in scene.primitives:
            if a == b:
                continue
            links.setdefault(a, set()).add(b)
            links.setdefault(b, set()).add(a)

        visited = set()
        for start in links:
            if start in visited:
                continue
            visited.add(start)
            stack = [start]
            edges = set()
            blocks_front = False
            length = 0.0
            while stack:
                a = stack.pop()
                for b in links[a]:
                    edge = (a, b) if a < b else (b, a)
                    if edge not in edges:
                        edges.add(edge)
                        length += math.dist(a, b)
                        blocks_front |= box_hit(a, b, corridor)
                    if b not in visited:
                        visited.add(b)
                        stack.append(b)
            if blocks_front and length > 0.50:
                return True
        return False

    def _note_block(self, reason, now):
        # 짧은 윤곽, 판정 불가, 미측정만으로는 후진 타이머를 시작하지 않는다.
        if (reason == "reactive_front_wait"
                and self.packet is not None
                and self._long_front_contour()):
            return self._original_note_block(reason, now)
        self._wait_since = None
        self._clear_since = None

    def steer(self, observation, forward, turn, reason, now):
        scene = self.fresh(now)
        if now >= observation.deadline:
            self.fail("uwb_stale")
        if self._phase != "FOLLOW":
            self.reset_recovery()
        self._heading = observation.heading_rad
        self._pivot_observation = observation

        if self._front_matches_tag(scene, observation):
            forward = 0.0
            if abs(self._heading) > math.radians(20.0):
                turn = math.copysign(0.25, self._heading)
                reason = "uwb_matched_pivot"
            else:
                turn = 0.0
                reason = "uwb_heading_aligned"
        elif any(abs(a) <= 30.0 and r is not None and 0 < r <= 1.0
                 for a, r in scene.rays):
            self._front_block_seq = self.current.sequence
            self.fail("reactive_front_obstacle_1m")

        self._gate(scene, forward, turn, self._heading)
        return forward, turn, reason

    # UWB_MATCHED_PIVOT_V1
    def _front_matches_tag(self, scene, obs):
        if not (0 < obs.range_m <= 1.0
                and abs(obs.heading_rad) <= math.radians(30.0)):
            return False

        g = self.geometry
        # 두 센서 위치를 반영해 LiDAR 중심 기준으로 태그를 비교한다.
        tx = g["uwb_x_m"] - g["lidar_x_m"] + obs.range_m * math.cos(obs.heading_rad)
        ty = g["uwb_y_m"] - g["lidar_y_m"] + obs.range_m * math.sin(obs.heading_rad)
        if not (0 < math.hypot(tx, ty) <= 1.0
                and abs(math.atan2(ty, tx)) <= math.radians(30.0)):
            return False

        hits = [(a, r) for a, r in scene.rays
                if abs(a) <= 30.0 and r is not None and 0 < r <= 1.0]
        return bool(hits) and all(
            math.hypot(r * math.cos(math.radians(a)) - tx,
                       r * math.sin(math.radians(a)) - ty) <= 0.20
            for a, r in hits
        )

    def _matched_pivot_allowed(self, scene, forward, turn, heading):
        obs = getattr(self, "_pivot_observation", None)
        if (forward != 0 or not turn or obs is None
                or time.monotonic() >= obs.deadline
                or abs(heading) <= math.radians(20.0)
                or turn * heading <= 0
                or not self._front_matches_tag(scene, obs)):
            return False
        checked, unknown = self._visibility(
            scene, 0.0, turn, heading, self.rules["lookahead_m"]
        )
        return checked > 0 and unknown == 0


