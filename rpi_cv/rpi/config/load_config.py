import re
import yaml
from typing import Dict, Any

UUID_RE = re.compile(
    r"^[0-9a-fA-F]{8}-"
    r"[0-9a-fA-F]{4}-"
    r"[0-9a-fA-F]{4}-"
    r"[0-9a-fA-F]{4}-"
    r"[0-9a-fA-F]{12}$"
)

def load_bt_config(path: str = "android_link.yaml") -> Dict[str, Any]:
    """Load Bluetooth config from YAML without applying defaults."""
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    if "bluetooth" not in data or not isinstance(data["bluetooth"], dict):
        raise ValueError("bluetooth section missing in YAML.")

    bt = data["bluetooth"]

    custom_uuid = bt.get("uuid")
    if not custom_uuid or not UUID_RE.match(custom_uuid):
        raise ValueError("bluetooth.uuid missing or invalid (must be a 128-bit UUID string).")

    bt["resolved_uuid"] = custom_uuid
    return data

def load_stm32_config(path: str = "stm32_link.yaml") -> Dict[str, Any]:
    """Load STM32 config from YAML without applying defaults."""
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    # No defaults applied; ensure structure if present
    if "serial_port" in data and not isinstance(data["serial_port"], dict):
        raise ValueError("serial_port must be a mapping if provided.")

    return data
