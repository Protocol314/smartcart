"""D5의 좌표·기하·운영자 실측 증빙을 확인하며 원본 장착값은 변경하지 않는다."""
from dataclasses import asdict
import math

from smartcart_uwb.config import CollectorConfig
from smartcart_uwb.estimation_config import EstimationConfig
from .config import FieldConfig, ROLES


def inspect_mounting(mounting, settings, uwb_settings=None):
    if not isinstance(mounting, CollectorConfig) or not isinstance(settings, FieldConfig):
        raise ValueError("검증된 장착·현장 설정이 필요합니다")
    if uwb_settings is not None and not isinstance(uwb_settings, EstimationConfig):
        raise ValueError("검증된 UWB 추정 설정이 필요합니다")
    mounts = {**dict(mounting.sensors), "lidar_center": mounting.lidar}
    geometry, roles, issues, numeric_mounts = {}, {}, [], {}

    def issue(code, affected):
        issues.append({"code": code, "roles": list(affected)})
        for role in affected:
            geometry[role].append(code)

    for role in ROLES:
        geometry[role] = []
        mount = mounts[role]
        xyz = mount.translation_m
        numeric = (type(xyz) in (tuple, list) and len(xyz) == 3 and
                   all(type(value) in (int, float) and math.isfinite(value) for value in xyz) and
                   type(mount.yaw_deg) in (int, float) and math.isfinite(mount.yaw_deg))
        numeric_mounts[role] = numeric
        if not numeric:
            issue("mounting_nonfinite_or_invalid", [role])
            continue
        if not -180 <= mount.yaw_deg <= 180:
            issue("yaw_outside_signed_range", [role])
        if xyz[2] <= 0:
            issue("sensor_height_nonpositive", [role])
        if xyz[0] <= 0:
            issue("sensor_not_in_front_of_rear_axle", [role])
    if not geometry["uwb_left"] and not geometry["uwb_right"]:
        left, right = mounts["uwb_left"].translation_m, mounts["uwb_right"].translation_m
        if left[1] <= 0 or right[1] >= 0:
            issue("uwb_side_assignment_inconsistent", ["uwb_left", "uwb_right"])
        if math.dist(left[:2], right[:2]) < settings.min_uwb_baseline_m:
            issue("uwb_baseline_too_small", ["uwb_left", "uwb_right"])
    if not geometry["lidar_center"]:
        if abs(mounts["lidar_center"].translation_m[1]) > settings.max_lidar_lateral_offset_m:
            issue("lidar_not_central", ["lidar_center"])
    if mounting.lidar.frame_id != settings.expected_lidar_frame_id:
        issue("lidar_frame_mismatch", ["lidar_center"])

    for role in ROLES:
        mount, evidence = mounts[role], settings.mounting_evidence[role]
        reasons = list(geometry[role])
        if not mount.mounting_verified:
            reasons.append("mounting_flag_unverified")
        for name in ("translation", "yaw", "height", "level"):
            if not getattr(evidence, name + "_verified"):
                reasons.append(name + "_unverified")
        expected_reference = "scan_origin" if role == "lidar_center" else "rf_phase_center"
        if evidence.reference_point != expected_reference:
            reasons.append("sensor_reference_unverified")
        if not evidence.evidence_note.strip():
            reasons.append("measurement_evidence_missing")
        roles[role] = {"ready": not reasons, "geometry_valid": not geometry[role],
                       "state": "invalid" if geometry[role] else "unverified" if reasons else "requirements_met",
                       "frame_id": mount.frame_id,
                       "translation_m": list(mount.translation_m) if numeric_mounts[role] else None,
                       "yaw_deg": mount.yaw_deg if numeric_mounts[role] else None,
                       "mounting_verified": mount.mounting_verified,
                       "translation_state": "verified" if evidence.translation_verified else "unverified",
                       "yaw_state": "verified" if evidence.yaw_verified else "unverified",
                       "height_state": "verified" if evidence.height_verified else "unverified",
                       "level_state": "verified" if evidence.level_verified else "unverified",
                       "reference_point": evidence.reference_point,
                       "calibration_profile": mount.calibration_profile,
                       "calibration_verified": False, "reasons": reasons}

    coordinate_reasons = []
    if not settings.origin_verified:
        coordinate_reasons.append("rear_axle_origin_unverified")
    if not settings.axes_verified:
        coordinate_reasons.append("axes_unverified")
    range_reasons = []
    range_model = "unknown" if uwb_settings is None else uwb_settings.range_model
    height_state = "not_applicable" if range_model == "horizontal" else "unverified"
    if uwb_settings is None:
        range_reasons.append("estimation_settings_missing")
    elif range_model == "unknown":
        range_reasons.append("range_model_unknown")
    if not settings.range_model_verified:
        range_reasons.append("range_model_unverified")
    if uwb_settings is not None and not uwb_settings.assumptions_verified:
        range_reasons.append("estimation_assumptions_unverified")
    if range_model == "slant_known_height":
        if not settings.tag_height_verified:
            range_reasons.append("tag_height_unverified")
        else:
            height_state = "verified"
        if uwb_settings.tag_height_m <= 0:
            range_reasons.append("tag_height_nonpositive")
    geometry_valid = all(not value for value in geometry.values())
    ready = geometry_valid and not coordinate_reasons and not range_reasons and all(value["ready"] for value in roles.values())
    return {"schema": "smartcart.field_mounting.v1", "valid": geometry_valid, "ready": bool(ready),
            "scope": "configuration_prerequisites_only", "coordinate_contract": asdict(settings.coordinate_contract),
            "coordinate_ready": not coordinate_reasons, "coordinate_reasons": coordinate_reasons,
            "roles": roles, "issues": issues,
            "range_height": {"range_model": range_model, "tag_height_state": height_state,
                             "ready": not range_reasons, "reasons": range_reasons},
            "evidence_source": "operator_declaration", "physical_measurement_performed": False,
            "configuration_modified": False, "calibration_modified": False,
            "provisional": True, "drive_authorized": False}


def readiness_summary(identity_report, mounting_report):
    """서류상 사전조건을 합친다. 실시간 관측·정확도·모터 허가를 의미하지 않는다."""
    expected = ((identity_report, "smartcart.field_identity.v1"), (mounting_report, "smartcart.field_mounting.v1"))
    for report, schema in expected:
        if (type(report) is not dict or report.get("schema") != schema or type(report.get("ready")) is not bool
                or report.get("drive_authorized") is not False or type(report.get("roles")) is not dict
                or set(report["roles"]) != set(ROLES)
                or any(type(item) is not dict or type(item.get("ready")) is not bool for item in report["roles"].values())):
            raise ValueError("검증된 읽기 전용 식별·장착 요약이 필요합니다")
    sensors = {role: {"identity_ready": identity_report["roles"][role]["ready"],
                      "mounting_ready": mounting_report["roles"][role]["ready"],
                      "ready": identity_report["roles"][role]["ready"] and mounting_report["roles"][role]["ready"]}
               for role in ROLES}
    ready = identity_report["ready"] and mounting_report["ready"] and all(value["ready"] for value in sensors.values())
    return {"schema": "smartcart.field_readiness.v1", "ready": ready,
            "scope": "configuration_prerequisites_only", "sensors": sensors,
            "live_data_verified": False, "physical_measurement_performed": False,
            "provisional": True, "drive_authorized": False}


sensor_readiness_summary = readiness_summary
