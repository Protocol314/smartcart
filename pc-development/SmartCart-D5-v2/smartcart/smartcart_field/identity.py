"""포트를 열지 않고 열거 정보와 명시적 세 역할 배정만 대조한다."""
from collections import Counter
from hashlib import sha256
import json

from smartcart_uwb.config import CollectorConfig
from smartcart_uwb.serial_reader import canonical_port, device_identity
from .config import DeviceBinding, FieldConfig, ROLES

MAX_INVENTORY_PORTS = 128
_INVENTORY_FIELDS = {"port", "canonical_port", "identity", "serial_number", "hwid", "vid", "pid",
                     "location", "aliases", "physical_key"}


def _text(value, *, optional=False):
    if optional and value is None:
        return None
    if type(value) is not str or len(value) > 512 or any(ord(char) < 32 for char in value):
        raise ValueError("장치 열거 문자열이 잘못되었습니다")
    try:
        value.encode("utf-8")
    except UnicodeError as error:
        raise ValueError("장치 열거 문자열은 올바른 UTF-8이어야 합니다") from error
    return value


def _fingerprint(value):
    return sha256(value.encode("utf-8")).hexdigest()[:16]


def enumerate_inventory(*, port_infos=None):
    """원문 식별값은 메모리에만 반환한다. Serial/open/read/write는 호출하지 않는다.

    by-id 포트는 canonical_port의 실제 경로 해석으로 같은 장치를 대조한다.
    기본 열거는 링크를 제외해 Pi 내장 UART의 불완전한 링크 메타데이터를 피한다.
    """
    if port_infos is None:
        from serial.tools import list_ports
        port_infos = list_ports.comports(include_links=False)
    grouped = {}
    for index, info in enumerate(port_infos):
        if index >= MAX_INVENTORY_PORTS:
            raise ValueError("장치 열거 항목은 128개 이하여야 합니다")
        port = _text(getattr(info, "device", None))
        resolved = canonical_port(port)
        serial_number = _text(getattr(info, "serial_number", None), optional=True)
        hwid = _text(getattr(info, "hwid", "") or "")
        location = _text(getattr(info, "location", None), optional=True)
        vid, pid = getattr(info, "vid", None), getattr(info, "pid", None)
        for number in (vid, pid):
            if number is not None and (type(number) is not int or not 0 <= number <= 65535):
                raise ValueError("장치 VID/PID가 잘못되었습니다")
        identity = _text(device_identity(info))
        if serial_number:
            physical = ["usb_serial", vid, pid, serial_number]
        elif location:
            # 다중 인터페이스의 :1.0/:1.1은 같은 USB 장치로 취급한다.
            physical = ["usb_location", vid, pid, location.split(":", 1)[0]]
        else:
            physical = ["identity", identity]
        record = {"port": port, "canonical_port": resolved, "identity": identity,
                  "serial_number": serial_number, "hwid": hwid, "vid": vid, "pid": pid,
                  "location": location, "aliases": [port],
                  "physical_key": _fingerprint(json.dumps(physical, ensure_ascii=False, separators=(",", ":")))}
        if resolved in grouped:
            previous = grouped[resolved]
            if any(previous[key] != record[key] for key in ("identity", "serial_number", "vid", "pid", "physical_key")):
                raise ValueError("같은 포트의 별칭에 서로 다른 장치 식별값이 있습니다")
            if port not in previous["aliases"]:
                previous["aliases"].append(port)
        else:
            grouped[resolved] = record
    return [grouped[key] for key in sorted(grouped)]


def _inventory(value):
    if type(value) is not list or len(value) > MAX_INVENTORY_PORTS:
        raise ValueError("상한이 있는 장치 목록이 필요합니다")
    for item in value:
        if type(item) is not dict or set(item) != _INVENTORY_FIELDS:
            raise ValueError("enumerate_inventory가 반환한 장치 목록이 필요합니다")
        for key in ("port", "canonical_port", "identity", "hwid", "physical_key"):
            _text(item[key])
        _text(item["serial_number"], optional=True)
        _text(item["location"], optional=True)
        for number in (item["vid"], item["pid"]):
            if number is not None and (type(number) is not int or not 0 <= number <= 65535):
                raise ValueError("장치 목록의 VID/PID가 잘못되었습니다")
        if canonical_port(item["port"]) != item["canonical_port"]:
            raise ValueError("장치 목록의 포트 별칭이 일치하지 않습니다")
        if (type(item["aliases"]) is not list or not 1 <= len(item["aliases"]) <= MAX_INVENTORY_PORTS
                or any(canonical_port(alias) != item["canonical_port"] for alias in item["aliases"])):
            raise ValueError("장치 별칭이 같은 물리 포트를 가리켜야 합니다")
    return value


def redact_inventory(inventory):
    """파일·기본 화면에 사용 가능한 요약이다. serial/hwid/by-id 원문을 넣지 않는다."""
    return [{"device_fingerprint": item["physical_key"],
             "identity_fingerprint": _fingerprint(item["identity"]),
             "alias_count": len(item["aliases"]), "has_serial_number": bool(item["serial_number"]),
             "has_hwid": bool(item["hwid"]), "vid": item["vid"], "pid": item["pid"]}
            for item in _inventory(inventory)]


def check_identity(mounting, settings, inventory, *, single_left=False):
    """세 역할을 정확히 배정한 경우에만 ready다. 포트·보정 설정을 변경하지 않는다."""
    if not isinstance(mounting, CollectorConfig) or not isinstance(settings, FieldConfig):
        raise ValueError("검증된 장착·현장 설정이 필요합니다")
    inventory = _inventory(inventory)
    bindings = {role: DeviceBinding(mounting.sensors[role].port, mounting.sensors[role].expected_device_identity,
                                    mounting.sensors[role].device_identity_verified)
                for role in ("uwb_left", "uwb_right")}
    bindings["lidar_center"] = settings.lidar_device
    identity_counts = Counter(item["identity"] for item in inventory)
    roles, assigned_ports, selected = {}, {}, {}
    active_roles = ("uwb_left", "lidar_center") if single_left else ROLES
    for role in active_roles:
        binding = bindings[role]
        reasons, candidates = [], []
        if binding.port is None:
            reasons.append("port_unassigned")
        else:
            resolved = canonical_port(binding.port)
            assigned_ports[role] = resolved
            candidates = [item for item in inventory if item["canonical_port"] == resolved]
            if not candidates:
                reasons.append("device_missing")
            elif len(candidates) != 1:
                reasons.append("ambiguous_port")
        if not binding.expected_device_identity:
            reasons.append("identity_unassigned")
        elif binding.expected_device_identity.lower() in ("hwid=", "hwid=n/a"):
            reasons.append("identity_not_specific")
        if not binding.device_identity_verified:
            reasons.append("identity_unverified")
        matched = False
        if len(candidates) == 1:
            selected[role] = candidates[0]
            matched = candidates[0]["identity"] == binding.expected_device_identity
            if not matched:
                reasons.append("identity_mismatch")
            elif identity_counts[binding.expected_device_identity] != 1:
                reasons.append("identity_not_unique")
        roles[role] = {"ready": not reasons, "state": "matched" if not reasons else "unverified",
                       "matched": matched, "identity_verified": binding.device_identity_verified,
                       "device_fingerprint": candidates[0]["physical_key"] if len(candidates) == 1 else None,
                       "reasons": reasons}
    issues = []
    for index, role in enumerate(active_roles):
        for other in active_roles[index + 1:]:
            code = None
            if role in assigned_ports and other in assigned_ports and assigned_ports[role] == assigned_ports[other]:
                code = "port_alias_conflict"
            elif role in selected and other in selected and selected[role]["physical_key"] == selected[other]["physical_key"]:
                code = "same_physical_device"
            if code:
                issues.append({"code": code, "roles": [role, other]})
                for affected in (role, other):
                    roles[affected]["reasons"].append(code)
                    roles[affected].update(ready=False, state="conflict")
    ready = all(value["ready"] for value in roles.values())
    return {"schema": "smartcart.field_identity.v1", "ready": ready, "valid": ready,
            "inventory_count": len(inventory), "roles": roles, "issues": issues,
            "port_opened": False, "lidar_owner": "existing_driver",
            "provisional": True, "drive_authorized": False}
