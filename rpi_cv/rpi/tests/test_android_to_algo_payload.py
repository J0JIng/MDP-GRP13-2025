import importlib.util
import sys
import types
from types import ModuleType
from pathlib import Path


def load_android_module():
    """Load rpi_cv/rpi/android.py as a module without importing the unrelated top-level 'android' project.

    This also stubs the 'bluetooth' dependency so tests don't require PyBluez.
    """
    # Stub bluetooth module (PyBluez) to avoid import errors
    bt_stub = ModuleType("bluetooth")
    for k, v in {
        "RFCOMM": 1,
        "PORT_ANY": 1,
        "SERIAL_PORT_CLASS": 1,
        "SERIAL_PORT_PROFILE": 1,
        "BluetoothSocket": (lambda *args, **kwargs: None),
        "advertise_service": (lambda *args, **kwargs: None),
    }.items():
        setattr(bt_stub, k, v)
    sys.modules.setdefault("bluetooth", bt_stub)

    # Ensure rpi_cv/rpi is on sys.path so helper/* imports resolve
    rpi_dir = Path(__file__).resolve().parents[1]
    if str(rpi_dir) not in sys.path:
        sys.path.insert(0, str(rpi_dir))

    android_path = rpi_dir / "android.py"
    spec = importlib.util.spec_from_file_location("rpi_android", str(android_path))
    if spec is None or spec.loader is None:
        raise RuntimeError("Failed to load module spec for android.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)  # type: ignore[attr-defined]
    return module


def test_android_to_algo_payload_basic():
    android = load_android_module()

    class DummyRPiMain:
        pass

    ai = android.AndroidInterface(DummyRPiMain())

    msg = {
        "type": "START_TASK",
        "data": {
            "task": "IMAGE",
            "robot": {"id": "R", "x": 2, "y": 2, "dir": "N"},
            "obstacles": [
                {"id": 1, "x": 4, "y": 2, "dir": "E"},
                {"id": 2, "x": 4, "y": 5, "dir": "N"},
            ],
        },
    }

    result = ai.android_to_algo_payload(msg)

    assert result == {
        "robot_x": 2,
        "robot_y": 2,
        "robot_dir": 0,  # N -> 0
        "retrying": False,
        "obstacles": [
            {"id": 1, "x": 4, "y": 2, "d": 2},  # E -> 2
            {"id": 2, "x": 4, "y": 5, "d": 0},  # N -> 0
        ],
    }

    # print curl request with JSON body for manual testing
    print("\ncurl command to send payload:")
    import json
    print(
        f'curl -sS -i -H "Content-Type: application/json" '
        f"-d '{json.dumps(result)}' "
        f"http://192.168.13.13:5000/path"
    )
