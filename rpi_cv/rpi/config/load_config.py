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

DEFAULTS = {
    "adapter": "hci0",
    "discoverable_cmd": "sudo hciconfig {adapter} piscan",
    "service_name": "MDP-Group2-RPi",
    "backlog": 1,
    "recv_bufsize": 1024,
    "newline_delimited": True,
}


def load_bt_config(path: str = "android_link.yaml") -> Dict[str, Any]:
    """Load and validate Bluetooth config from YAML, applying defaults."""
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    user_bt = data.get("bluetooth", {})

    # Merge defaults with user config (user values override defaults)
    bt = {**DEFAULTS, **user_bt}

    # UUID must be provided and valid
    custom_uuid = bt.get("uuid")
    if not custom_uuid or not UUID_RE.match(custom_uuid):
        raise ValueError(
            "bluetooth.uuid missing or invalid in YAML (must be a 128-bit UUID string)."
        )

    bt["resolved_uuid"] = custom_uuid
    data["bluetooth"] = bt
    return data
