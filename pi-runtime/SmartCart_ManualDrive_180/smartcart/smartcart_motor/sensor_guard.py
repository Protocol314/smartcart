"""가상 출력의 실측 센서 만료 감시. 실제 모터 주행 승인에는 사용하지 않는다."""
import math
import threading
import time


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class SensorGuard:
    def __init__(self, *, clock=time.monotonic, radius=1.0):
        if not finite(radius) or not .5 <= radius <= 3:
            raise ValueError('가상 정지 반경이 잘못되었습니다')
        self.clock, self.radius = clock, radius
        self.lock = threading.Lock()
        self.deadline = 0.0
        self.reason = 'sensor_waiting'
        self.scan_seq = self.uwb_seq = None
        self.scan_deadline = self.uwb_deadline = 0.0

    def invalidate(self, reason='sensor_unavailable'):
        with self.lock:
            self.deadline = 0.0
            self.reason = reason

    def update(self, frame, uwb, transport_ms=0):
        """서버에서 반환한 수명은 통신·처리 시간만큼 줄어든다."""
        now = self.clock()
        try:
            if not isinstance(frame, dict) or frame.get('schema') != 'smartcart.envelope.v1' or frame.get('source_mode') != 'ros' or frame.get('state') != 'live':
                raise ValueError('lidar_unavailable')
            age = frame.get('age_ms')
            if not finite(transport_ms) or not 0 <= transport_ms < 300 or not finite(age) or not 0 <= age < 300:
                raise ValueError('lidar_stale')
            age += transport_ms
            if age >= 300:
                raise ValueError('lidar_stale')
            scan = frame.get('scan')
            if not isinstance(scan, dict) or scan.get('truncated'):
                raise ValueError('lidar_invalid')
            values = scan.get('ranges')
            start, end, step, low, high = (scan.get(k) for k in ('angle_min','angle_max','angle_increment','range_min','range_max'))
            if (not isinstance(values, list) or not 3 <= len(values) <= 4096 or
                    not all(finite(x) for x in (start,end,step,low,high)) or not 0 < step <= .1 or
                    not 0 <= low <= self.radius <= high or
                    abs((end-start)-(len(values)-1)*step) > 1.5*step or
                    not 2*math.pi-1.5*step <= (len(values)-1)*step <= 2*math.pi+step):
                raise ValueError('lidar_invalid')
            if any(not finite(x) or x < low or x > high for x in values):
                raise ValueError('lidar_unobserved')
            if any(x <= self.radius for x in values):
                raise ValueError('obstacle')
            if (not isinstance(uwb, dict) or uwb.get('schema') != 'smartcart.dual_uwb_view.v1' or
                    uwb.get('source_mode') != 'live' or uwb.get('state') != 'running' or
                    uwb.get('profile') != 'single_left' or uwb.get('drive_authorized') is not False):
                raise ValueError('uwb_unavailable')
            sensors = uwb.get('sensors')
            if not isinstance(sensors, dict) or not isinstance(sensors.get('uwb_left'), dict):
                raise ValueError('uwb_invalid')
            sensor = sensors['uwb_left']
            if (sensor.get('state') != 'live' or any(sensor.get(k) is not True for k in ('connected','valid','measurement_valid','id_valid'))):
                raise ValueError('uwb_invalid')
            remaining, sample_age = sensor.get('remaining_ms'), sensor.get('age_ms')
            if not finite(remaining) or not finite(sample_age) or not 0 <= sample_age < 300 or not 0 < remaining <= 1000:
                raise ValueError('uwb_stale')
            remaining -= transport_ms
            sample_age += transport_ms
            if remaining <= 0 or sample_age >= 300:
                raise ValueError('uwb_stale')
            scan_seq, uwb_seq = scan.get('sequence'), sensor.get('sample_sequence')
            if type(scan_seq) is not int or scan_seq < 0 or type(uwb_seq) is not int or uwb_seq < 0:
                raise ValueError('sensor_invalid')
            with self.lock:
                if ((self.scan_seq is not None and scan_seq < self.scan_seq) or
                        (self.uwb_seq is not None and uwb_seq < self.uwb_seq)):
                    raise ValueError('sensor_invalid')
                scan_until = now + (300-age)/1000
                uwb_until = now + min(remaining/1000, (300-sample_age)/1000)
                if scan_seq == self.scan_seq:
                    scan_until = min(scan_until, self.scan_deadline)
                if uwb_seq == self.uwb_seq:
                    uwb_until = min(uwb_until, self.uwb_deadline)
                self.scan_seq, self.uwb_seq = scan_seq, uwb_seq
                self.scan_deadline, self.uwb_deadline = scan_until, uwb_until
                self.deadline, self.reason = min(scan_until, uwb_until), 'ready_virtual_only'
        except (TypeError, ValueError, OverflowError) as error:
            self.invalidate(str(error) if str(error) in {'lidar_unavailable','lidar_stale','lidar_invalid','lidar_unobserved','obstacle','uwb_unavailable','uwb_invalid','uwb_stale'} else 'sensor_invalid')

    def status(self):
        with self.lock:
            if self.clock() >= self.deadline:
                return False, self.reason if self.deadline == 0 else 'sensor_timeout'
            return True, self.reason
