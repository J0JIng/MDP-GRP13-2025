"""Walk-around capture script for a cuboid obstacle.

Behavior:
- Start 50cm away from one face of a cuboid obstacle.
- Immediately take a photo and send it to the image-rec server. Expect id=10 (Bullseye) to confirm obstacle.
- Then visit the other up-to-3 faces by performing a simple turn-move-turn sequence that orbits the cube.
- At each new face, take a photo and send to the server.
- Stop as soon as the returned payload has an id that is neither "NA" nor 10 (Bullseye), i.e. a valid symbol.

Notes/assumptions:
- Robot cannot strafe. To move to the next face we must go around the corner using TWO legs and THREE turns:
    Clockwise (to the right face): R90 → F[LEG_CM] → L90 → F[LEG_CM] → L90.
    Counter-clockwise (to the left face): L90 → F[LEG_CM] → R90 → F[LEG_CM] → R90.
- The leg distance between faces is configurable via LEG_CM (default 100cm). Adjust for your arena/object size (it should roughly match the corner offset path length between face centers).
- Captures use raspistill parameters similar to image_rec.py; if raspistill fails, fallback to libcamera-still.

References:
- rpi/image_rec.py for capture & POST logic
- rpi/manual.py and rpi/stm/robot_controller.py for movement API
"""

from __future__ import annotations

import os
import time
import json
from typing import Optional, Tuple

import requests

from config.load_config import load_rpi_config
from stm.robot_controller import RobotController


# Configuration constants
ROBOT_PORT = "/dev/ttyACM0"  # adjust as necessary for your Pi
ROBOT_BAUD = 115200

# Desired fixed standoff from each face (cm)
STANDOFF_CM = 50

# Obstacle side length (cm). If rectangular, you can split into OB_SIDE_X_CM/OB_SIDE_Y_CM.
# TODO: adjust this value to your obstacle. For a 50cm face, leave as 50.
OB_SIDE_CM = 50

# Distance to move between faces of the cuboid (cm). For a square footprint:
# L = D + S/2 (from face centerline to corner orbit, then to next face centerline)
LEG_CM = int(STANDOFF_CM + OB_SIDE_CM / 2)

# Initial obstacle assumptions
OBSTACLE_ID_FIRST = 10  # Expect first capture to recognize Bullseye (id 10)
SIGNAL_DEFAULT = "L"   # Symbol character for filename; server may not require this


# Paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGES_DIR = os.path.join(SCRIPT_DIR, "images")
os.makedirs(IMAGES_DIR, exist_ok=True)


def _api_url_from_config() -> str:
    cfg = load_rpi_config() or {}
    api = cfg.get("api", {}) if isinstance(cfg, dict) else {}
    host = api.get("ip") or "127.0.0.1"
    port = api.get("image_port", 5001)
    return f"http://{host}:{port}/image"


def capture_image(obstacle_id: int, signal: str = SIGNAL_DEFAULT) -> str:
    """Capture an image of the current view.

    Primary method: raspistill with tuned params (matching image_rec.py)
    Fallback: libcamera-still with sensible defaults

    Returns the image filename or raises RuntimeError.
    """
    ts = int(time.time())
    filename = f"{ts}_{obstacle_id}_{signal}.jpg"
    file_path = os.path.join(IMAGES_DIR, filename)

    def _run(cmd: str) -> bool:
        print(f"[CAM] {cmd}")
        code = os.system(cmd)
        return code == 0 and os.path.exists(file_path)

    # Try raspistill first (legacy)
    raspistill = "raspistill"
    # Quote the output path for safety on spaces
    out_arg = f'"{file_path}"'
    cmd_raspi = " ".join([
        raspistill,
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
        "-o", out_arg,
    ])
    if _run(cmd_raspi):
        return file_path

    # # Fallback to libcamera-still
    # libcamera = "libcamera-still"
    # cmd_lib = " ".join([
    #     libcamera,
    #     "-n",
    #     "-t", "500",
    #     "--sharpness", "1.0",
    #     "--contrast", "1.0",
    #     "--saturation", "1.0",
    #     "--brightness", "0.0",
    #     "--awb", "auto",
    #     "-o", filename,
    # ])
    # if _run(cmd_lib):
    #     return filename

    raise RuntimeError("Camera capture failed with both raspistill and libcamera-still")


def post_image(filename: str) -> dict:
    """POST the image to the image recognition server and return parsed JSON."""
    url = _api_url_from_config()
    print(f"[HTTP] POST {url} ({filename})")
    with open(filename, "rb") as f:
        resp = requests.post(url, files={"file": (os.path.basename(filename), f)})
    try:
        data = resp.json()
    except Exception:
        data = {"raw": resp.text}
    if resp.status_code != 200:
        raise RuntimeError(f"Image API non-200 ({resp.status_code}): {data}")
    return data


def extract_id(payload: dict) -> Optional[str]:
    """Extract the recognized id from payload, robust to nesting."""
    if not isinstance(payload, dict):
        return None
    if "id" in payload:
        # id could be numeric or string
        return str(payload.get("id"))
    # common alternates
    for key in ("result", "data"):  # nested containers
        if key in payload and isinstance(payload[key], dict) and "id" in payload[key]:
            return str(payload[key]["id"])
    return None


def move_to_next_face(robot: RobotController, leg_cm: int = LEG_CM, clockwise: bool = True) -> bool:
    """Navigate around the cuboid corner to the adjacent face.

    The robot starts centered in front of one face (distance ~ 50cm) facing that face.
    After this routine, it ends up centered in front of the adjacent face, facing that face.

    Path (clockwise=True):  R90 → F(leg_cm) → L90 → F(leg_cm) → L90
    Path (clockwise=False): L90 → F(leg_cm) → R90 → F(leg_cm) → R90

    Returns True if all sub-commands were acknowledged.
    """
    ok = True
    try:
        if clockwise:
            ok = robot.turn_right(90, True) and ok
            time.sleep(0.2)
            ok = robot.move_forward(int(leg_cm)) and ok
            time.sleep(max(3.0, leg_cm / 30.0))
            ok = robot.turn_left(90, True) and ok
            time.sleep(0.2)
            ok = robot.move_forward(int(leg_cm)) and ok
            time.sleep(max(3.0, leg_cm / 30.0))
            ok = robot.turn_left(90, True) and ok
            time.sleep(0.2)
        else:
            ok = robot.turn_left(90, True) and ok
            time.sleep(0.2)
            ok = robot.move_forward(int(leg_cm)) and ok
            time.sleep(max(3.0, leg_cm / 30.0))
            ok = robot.turn_right(90, True) and ok
            time.sleep(0.2)
            ok = robot.move_forward(int(leg_cm)) and ok
            time.sleep(max(3.0, leg_cm / 30.0))
            ok = robot.turn_right(90, True) and ok
            time.sleep(0.2)
    except Exception as e:
        print(f"[MOVE] Error moving to next face: {e}")
        return False
    return bool(ok)


def take_and_check(obstacle_id_hint: int, signal: str = SIGNAL_DEFAULT) -> Tuple[Optional[str], Optional[dict]]:
    """Capture and POST image; return (id_str, payload)."""
    try:
        fn = capture_image(obstacle_id_hint, signal)
    except Exception as e:
        print(f"[CAM] Capture failed: {e}")
        return None, None
    try:
        payload = post_image(fn)
    except Exception as e:
        print(f"[HTTP] Upload failed: {e}")
        return None, None
    try:
        print("[HTTP] Response:\n" + json.dumps(payload, indent=2))
    except Exception:
        print("[HTTP] Response (raw)", payload)
    return extract_id(payload), payload


def main():
    print("--- a_5: Cuboid walk-around capture ---")
    print(f"Using image API: {_api_url_from_config()}")

    # Initialise robot controller
    print(f"[INIT] RobotController (port={ROBOT_PORT} baud={ROBOT_BAUD})")
    try:
        robot = RobotController(ROBOT_PORT, ROBOT_BAUD)
    except Exception as e:
        print(f"[INIT] Failed to initialise RobotController: {e}")
        return

    # Start in front of side 1 (50cm away). Immediately capture.
    print("[STEP] Side 1 capture (expect id=10 Bullseye)")
    id1, _ = take_and_check(OBSTACLE_ID_FIRST, SIGNAL_DEFAULT)
    if id1 is None:
        print("[WARN] No id returned; continuing walk-around anyway")
    else:
        print(f"[INFO] Side 1 id = {id1}")

    # Visit the next up-to-3 faces; stop early on valid symbol
    faces_remaining = 3
    for i in range(1, faces_remaining + 1):
        print(f"[STEP] Move to face {i+1}")
        if not move_to_next_face(robot, LEG_CM, clockwise=True):
            print("[WARN] Movement possibly not acknowledged; proceeding to capture")
        print(f"[STEP] Capture at face {i+1}")
        sid, _payload = take_and_check(OBSTACLE_ID_FIRST, SIGNAL_DEFAULT)
        if sid is None:
            print("[INFO] No id; continue to next face")
            continue
        print(f"[INFO] Face {i+1} id = {sid}")
        # End condition: id not NA and not 10 (Bullseye)
        sid_up = sid.upper() if isinstance(sid, str) else str(sid)
        if sid_up != "NA" and sid_up != "10":
            print(f"[DONE] Found valid symbol at face {i+1}: id={sid_up}")
            break
    print("[EXIT] Program finished")


if __name__ == "__main__":
    main()
