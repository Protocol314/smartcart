"""실행 계획은 순수 설정만 읽으며 ROS·직렬 장치를 열지 않는다."""
from __future__ import annotations

import shutil
import sys

from .config import ROOT, asset_root


def _command(name, argv):
    return {"name": name, "argv": [str(value) for value in argv], "cwd": str(ROOT)}


def build_plan(config, mode="replay", *, single_left=False):
    if mode not in ("replay", "live"):
        raise ValueError("실행 모드는 replay 또는 live입니다")
    if single_left and mode != "live":
        raise ValueError("왼쪽 단독 구성은 실측 모드에서만 선택합니다")
    blockers, warnings, commands = [], [], []
    common = [sys.executable, "-m", "smartcart_runner.worker", "sensing", "serve", "--host", "127.0.0.1",
              "--port", str(config.http_port)]
    if mode == "replay":
        assets = asset_root()
        paths = {"input": assets / "samples/target_tracking.synthetic.jsonl",
                 "mounting": assets / "config/dual_uwb.d5.example.json",
                 "uwb": assets / "config/uwb_estimation.synthetic.json",
                 "tracking": assets / "config/target_tracking.synthetic.json"}
        for name, path in paths.items():
            if not path.is_file():
                blockers.append("replay_" + name + "_missing")
        commands.append(_command("replay_gui", common + ["--mode", "replay", "--uwb-mode", "off",
            "--tracking-input", paths["input"], "--tracking-mounting-config", paths["mounting"],
            "--tracking-uwb-config", paths["uwb"], "--tracking-config", paths["tracking"], "--tracking-loop"]))
    else:
        from smartcart_uwb.config import load_config
        from smartcart_uwb.estimation_config import load_estimation_config
        from smartcart_field.config import load_field_config
        from smartcart_field.mounting import inspect_mounting
        from smartcart_tracking.config import load_tracking_config
        mounting, field = load_config(config.mounting_config), load_field_config(config.field_config)
        uwb = load_estimation_config(config.uwb_config)
        load_tracking_config(config.tracking_config)
        devices = {**mounting.sensors, "lidar_center": field.lidar_device}
        if single_left:
            devices.pop("uwb_right")
        for role, device in devices.items():
            if device.port is None or not device.expected_device_identity or not device.device_identity_verified:
                blockers.append(role + "_identity_unassigned_or_unverified")
        if config.lidar_port is None:
            blockers.append("lidar_port_unassigned")
        elif config.lidar_port != field.lidar_device.port:
            blockers.append("lidar_port_binding_mismatch")
        if config.lidar_frame_id != mounting.lidar.frame_id or config.lidar_frame_id != field.expected_lidar_frame_id:
            blockers.append("lidar_frame_mismatch")
        geometry = inspect_mounting(mounting, field, uwb)
        if not geometry["valid"]:
            blockers.append("mounting_geometry_invalid")
        if not geometry["ready"]:
            warnings.append("mounting_or_range_model_unverified_positions_may_be_hidden")
        if config.lidar_mode == "managed":
            commands.append(_command("lidar", ["ros2", "launch", "sllidar_ros2", "sllidar_c1_launch.py",
                "serial_port:=" + (config.lidar_port or "<미배정>"), "serial_baudrate:=460800",
                "frame_id:=" + config.lidar_frame_id]))
        else:
            warnings.append("external_lidar_driver_must_already_publish_scan_same_host_and_ros_domain")
        commands.append(_command("uwb", [sys.executable, "-m", "smartcart_runner.worker", "uwb", "--config", config.mounting_config] + (["--left-only"] if single_left else [])))
        commands.append(_command("tracking_gui", common + ["--mode", "ros", "--uwb-mode", "off", "--tracking-mode", "ros",
            "--tracking-mounting-config", config.mounting_config, "--tracking-uwb-config", config.uwb_config,
            "--tracking-config", config.tracking_config] + (["--single-left"] if single_left else [])))
    return {"schema": "smartcart.run_plan.v1", "mode": mode, "commands": commands,
            "environment": {"ROS_LOCALHOST_ONLY": "1"}, "http_url": "http://127.0.0.1:" + str(config.http_port),
            "blockers": blockers, "warnings": warnings, "configuration_ready": not blockers,
            "port_opened": False, "process_started": False, "physical_acceptance": False,
            "drive_authorized": False, "sensor_freshness_verified": False,
            "profile": "single_left" if single_left else "dual",
            "notice": "실행 계획과 웹 서버 기동은 센서 수신·실물 검수·구동 승인이 아닙니다"}


def check_plan(config, plan):
    """설정·라이브러리·USB 열거만 확인하며 포트를 개방하지 않는다."""
    report = {**plan, "schema": "smartcart.run_check.v1", "blockers": list(plan["blockers"])}
    try:
        from smartcart_uwb.native import NativeParser
        with NativeParser():
            pass
    except (ValueError, OSError, RuntimeError) as error:
        report["blockers"].append("native_parser_unavailable")
    if plan["mode"] == "live":
        from smartcart_field.cli import environment_report
        from smartcart_field.config import load_field_config
        from smartcart_field.identity import check_identity, enumerate_inventory
        from smartcart_uwb.config import load_config
        environment = environment_report()
        report["runtime"] = environment
        if not environment["ready"]:
            report["blockers"].append("pi4_humble_runtime_unavailable")
        if config.lidar_mode == "managed" and shutil.which("ros2") is None:
            report["blockers"].append("ros2_executable_unavailable")
        try:
            identity = check_identity(load_config(config.mounting_config), load_field_config(config.field_config), enumerate_inventory(), single_left=plan.get("profile") == "single_left")
            report["identity"] = identity
            if not identity["ready"]:
                report["blockers"].append("usb_identity_check_incomplete")
        except (ImportError, OSError, ValueError):
            report["blockers"].append("usb_inventory_unavailable")
    report["ready"] = not report["blockers"]
    return report
