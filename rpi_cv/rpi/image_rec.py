"""Simple standalone script to:
1. Connect to STM32.
2. Move robot forward 50cm.
3. Snap a picture (using libcamera-still) with a fabricated obstacle id & signal.
4. POST the image to the image recognition server (/image) and display the JSON response.

Assumptions:
- One forward unit (FW0x) corresponds to 10cm, so FW05 == 50cm.
- Image recognition Flask server is running (see cv/main.py) and reachable via
  configuration in rpi/config/stm32_link.yaml (api.ip & api.image_port) or we fall back
  to defaults 127.0.0.1:5001.
- A Raspberry Pi Camera is available and libcamera-still is installed.

You can edit OBSTACLE_ID and SIGNAL below if you want different naming.
"""

from __future__ import annotations

import os
import time
import json
import requests

from config.load_config import load_rpi_config
from stm.robot_controller import RobotController


OBSTACLE_ID = 10  # Default obstacle id if none provided in SNAP
SIGNAL = "L"      # Default signal/letter expected by server model (adjust as needed)

# Dummy algorithm-style command list to simulate what the algo server would send.
# Supported here:
#   FW###  move forward N cm (e.g. FW050)
#   BW###  move backward N cm (not executed currently, just logs)
#   SNAP<obstacleId>_<SignalChar>  capture & send image (e.g. SNAP10_L)
# You can edit this list or later fetch from /path API.
DUMMY_COMMANDS = [
    "FW050",      # move forward 50 cm
    "SNAP10_L",   # snap obstacle 10 expecting symbol L
]


def move_forward(robot: RobotController, dist_cm: int) -> bool:
    """Move robot forward dist_cm using RobotController API."""
    try:
        return bool(robot.move_forward(int(dist_cm)))
    except Exception as e:
        print(f"Error moving forward {dist_cm}cm: {e}")
        return False


def capture_image(obstacle_id: int, signal: str) -> str:
    """Capture an image using raspistill (legacy camera stack) with tuned parameters.

    Mimics the settings used in `a_2.py`. Returns filename or raises on failure.
    """
    ts = int(time.time())
    filename = f"{ts}_{obstacle_id}_{signal}.jpg"
    cmd_parts = [
        "raspistill",
        "-e", "jpg",
        "-n",
        "-t", "500",
        "-vf", "-hf",
        "-q", "100",
        "-sh", "40",
        "-ISO", "100",
        "-awb", "auto",
        "-ss", "20000",
        "-br", "50",
        "-co", "10",
        "-sa", "10",
        "-o", filename,
    ]
    cmd = " ".join(cmd_parts)
    print(f"Capturing image with raspistill: {filename}")
    code = os.system(cmd)
    if code != 0 or not os.path.exists(filename):
        raise RuntimeError(f"raspistill failed (exit {code}) or file missing")
    return filename


def post_image(filename: str) -> dict:
    """POST the image to the image recognition server and return parsed JSON."""
    cfg = load_rpi_config() or {}
    api = cfg.get("api", {}) if isinstance(cfg, dict) else {}
    host = api.get("pc_ip") or "192.168.13.13"
    port = api.get("image_port", 5001)
    url = f"http://{host}:{port}/image"

    print(f"Posting image to {url}")
    with open(filename, "rb") as f:
        resp = requests.post(url, files={"file": (filename, f)})
    try:
        data = resp.json()
    except Exception:
        data = {"raw": resp.text}
    if resp.status_code != 200:
        raise RuntimeError(f"Image API non-200 ({resp.status_code}): {data}")
    return data


def execute_dummy_commands(robot: RobotController, commands: list[str]):
    """Execute a list of dummy algorithm-like commands."""
    for cmd in commands:
        uc = cmd.upper().strip()
        if uc.startswith("FW") and uc[2:].isdigit():
            dist = int(uc[2:])
            print(f"[CMD] Forward {dist}cm")
            ok = move_forward(robot, dist)
            if not ok:
                print(f"  WARN: forward {dist} not acknowledged")
            # simple settle delay (could be proportional to dist)
            time.sleep(5)
        elif uc.startswith("SNAP"):
            # Expect pattern SNAP<id>_<Signal>
            payload = uc[4:]
            obstacle_id = OBSTACLE_ID
            sig = SIGNAL
            if '_' in payload:
                left, right = payload.split('_', 1)
                if left.isdigit():
                    obstacle_id = int(left)
                if len(right) >= 1:
                    sig = right[0]
            print(f"[CMD] SNAP obstacle={obstacle_id} signal={sig}")
            fn = capture_image(obstacle_id, sig)
            try:
                result = post_image(fn)
                print("  SNAP result:")
                print(json.dumps(result, indent=2))
            except Exception as e:
                print(f"  SNAP upload failed: {e}")
        elif uc.startswith("BW") and uc[2:].isdigit():
            print(f"[CMD] (Not implemented) Backward {uc[2:]}cm")
        else:
            print(f"[CMD] Unknown or unsupported: {cmd}")


def main():
    print("--- Image Recognition Dummy Algo Runner ---")
    port = "/dev/ttyACM0"  # adjust as necessary
    baud = 115200
    print(f"Initialising RobotController (port={port} baud={baud}) ...")
    try:
        robot = RobotController(port, baud)
    except Exception as e:
        print(f"Failed to initialise RobotController: {e}")
        return

    print("Executing dummy command list:")
    for c in DUMMY_COMMANDS:
        print("  -", c)
    execute_dummy_commands(robot, DUMMY_COMMANDS)


if __name__ == "__main__":
    main()
