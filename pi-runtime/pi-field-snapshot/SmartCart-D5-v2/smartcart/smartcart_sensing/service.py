"""독립 센서의 최신 상태를 합쳐 GUI에 전달한다. 위치 융합이나 제어는 하지 않는다."""
from __future__ import annotations

from smartcart_radar.core import ScanStore
from .ros import diagnostic_data, lidar_diagnostic_data
from .uwb import UwbStore


class SensingStore:
    """기존 LiDAR 봉투 규격을 유지하면서 UWB와 전체 상태를 추가한다."""

    def __init__(self, lidar: ScanStore, uwb: UwbStore, mode: str = "ros", tracking=None, dual_uwb=None):
        if mode not in {"ros", "demo", "replay"}:
            raise ValueError("지원하지 않는 센싱 모드입니다")
        if lidar.stop is not uwb.stop:
            raise ValueError("센서 종료 이벤트가 일치해야 합니다")
        self.lidar, self.uwb, self.mode = lidar, uwb, mode
        self.stop = lidar.stop
        if tracking is not None and tracking.stop is not self.stop:
            raise ValueError("추적 종료 이벤트가 일치해야 합니다")
        self.tracking = tracking
        if dual_uwb is not None and (mode != 'ros' or dual_uwb.stop is not self.stop):
            raise ValueError('좌우 UWB 표시의 출처·종료 이벤트가 일치해야 합니다')
        self.dual_uwb = dual_uwb

    def snapshot(self, *, now: float | None = None) -> dict:
        lidar = self.lidar.snapshot(now=now)
        uwb = self.uwb.snapshot(now=now)
        # ROS 진단과 GUI는 같은 신선도·출처·유효 반사점·ID·위치 조건을 사용한다.
        lidar_status, uwb_status = lidar_diagnostic_data(lidar), diagnostic_data(uwb)
        lidar_valid, uwb_valid = lidar_status["level"] == 0, uwb_status["level"] == 0
        dual = self.dual_uwb.snapshot(now=now) if self.dual_uwb is not None else None
        if dual is not None:
            uwb_valid = dual['valid']
        synthetic = (self.mode in {"demo", "replay"} or lidar_status["values"]["synthetic"] == "true"
                     or uwb.get("synthetic") is True)
        ready = lidar_valid and uwb_valid and not synthetic
        state = ("synthetic" if synthetic else "ready" if ready else
                 "degraded" if lidar_valid or uwb_valid or lidar.get("error") or uwb.get("error")
                 else "waiting")
        reasons = []
        if not lidar_valid:
            reasons.append("LiDAR 실측을 확인하십시오")
        if not uwb_valid:
            reasons.append('좌우 UWB의 수신·측정·ID 상태를 확인하십시오' if dual is not None
                           else "UWB 실측·ID·위치 유효성을 확인하십시오")
        if synthetic:
            reasons = ["명시적인 합성·재생 자료: 실측 또는 구동 허가가 아닙니다"]
        result = {**lidar, "uwb": uwb, "sensing": {
            "schema": "smartcart.sensing.v1", "state": state, "mode": self.mode,
            "ready": bool(ready), "frame_id": "base_link",
            "lidar_valid": lidar_valid, "uwb_valid": uwb_valid,
            "synthetic": synthetic, "reasons": reasons,
            "purpose": "read_only_sensor_monitor",
        }}
        if self.tracking is not None:
            result["tracking"] = self.tracking.snapshot(now=now)
        if dual is not None:
            result['dual_uwb'] = dual
            result['sensing'].update(uwb_backend='dual', uwb_received=dual['received'])
        return result
