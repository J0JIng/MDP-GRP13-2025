from pathlib import Path
import pytest

from config.load_config import load_bt_config, load_stm32_config


BASE_DIR = Path(__file__).resolve().parents[1]
CONFIG_DIR = BASE_DIR / "config"


def test_load_bt_config_success():
    cfg_path = CONFIG_DIR / "android_link.yaml"
    data = load_bt_config(str(cfg_path))
    assert "bluetooth" in data
    bt = data["bluetooth"]
    # ensure uuid copied to resolved_uuid
    assert bt["resolved_uuid"] == bt["uuid"]


def test_load_bt_config_invalid_uuid(tmp_path: Path):
    bad = tmp_path / "bad.yaml"
    bad.write_text(
        """
bluetooth:
  uuid: not-a-valid-uuid
""",
        encoding="utf-8",
    )
    with pytest.raises(ValueError):
        load_bt_config(str(bad))


def test_load_stm32_config_success():
    cfg_path = CONFIG_DIR / "stm32_link.yaml"
    data = load_stm32_config(str(cfg_path))
    assert "serial_port" in data
    assert isinstance(data["serial_port"], dict)


def test_load_stm32_config_invalid(tmp_path: Path):
    bad = tmp_path / "bad_stm.yaml"
    bad.write_text(
        """
serial_port: []
""",
        encoding="utf-8",
    )
    with pytest.raises(ValueError):
        load_stm32_config(str(bad))
