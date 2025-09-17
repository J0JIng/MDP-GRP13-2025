import re
from pathlib import Path
from typing import Dict, Any, Optional

import yaml

UUID_RE = re.compile(
    r"^[0-9a-fA-F]{8}-"
    r"[0-9a-fA-F]{4}-"
    r"[0-9a-fA-F]{4}-"
    r"[0-9a-fA-F]{4}-"
    r"[0-9a-fA-F]{12}$"
)


def _resolve_path(path: Optional[str], default_file: str) -> Path:
    """Resolve a config file path.

    - If path is None: use package-local config/<default_file>
    - If path is absolute or exists as given: use it
    - Else: try relative to this module's directory
    """
    if path is None:
        return Path(__file__).resolve().parent / default_file

    p = Path(path)
    if p.is_absolute() or p.exists():
        return p

    # Fallback to package-local resolution
    return (Path(__file__).resolve().parent / p).resolve()


def load_bt_config(path: Optional[str] = None) -> Dict[str, Any]:
    """Load Bluetooth config from YAML and resolve UUID for service advertising.

    If `use_spp_uuid` is true, we will force the standard SPP UUID
    00001101-0000-1000-8000-00805F9B34FB regardless of `uuid`.
    Otherwise, we require a valid custom `uuid`.
    """
    cfg_path = _resolve_path(path, "android_link.yaml")
    with open(cfg_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    if "bluetooth" not in data or not isinstance(data["bluetooth"], dict):
        raise ValueError("bluetooth section missing in YAML.")

    bt = data["bluetooth"]

    # Prefer standard SPP UUID if requested (matches Android client's MY_UUID)
    if bt.get("use_spp_uuid", False):
        spp_uuid = bt.get("spp_uuid", "00001101-0000-1000-8000-00805F9B34FB")
        if not UUID_RE.match(spp_uuid):
            raise ValueError("bluetooth.spp_uuid invalid: must be a 128-bit UUID string.")
        bt["resolved_uuid"] = spp_uuid
        # Ensure 'uuid' key is present for callers/tests that expect it
        bt["uuid"] = spp_uuid
        return data

    # Else require a valid custom uuid
    custom_uuid = bt.get("uuid")
    if not custom_uuid or not UUID_RE.match(custom_uuid):
        raise ValueError("bluetooth.uuid missing or invalid (must be a 128-bit UUID string).")

    bt["resolved_uuid"] = custom_uuid
    # Normalize to ensure both keys exist
    bt["uuid"] = custom_uuid
    return data


def load_stm32_config(path: Optional[str] = None) -> Dict[str, Any]:
    """Load STM32 config from YAML without applying defaults."""
    cfg_path = _resolve_path(path, "stm32_link.yaml")
    with open(cfg_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    # No defaults applied; ensure structure if present
    if "serial_port" in data and not isinstance(data["serial_port"], dict):
        raise ValueError("serial_port must be a mapping if provided.")

    return data


def load_rpi_config(path: Optional[str] = None) -> Dict[str, Any]:
    """Load RPi config from YAML without applying defaults."""
    cfg_path = _resolve_path(path, "rpi.yaml")
    with open(cfg_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    # No defaults applied; ensure structure if present
    if "api" in data and not isinstance(data["api"], dict):
        raise ValueError("api must be a mapping if provided.")

    return data
