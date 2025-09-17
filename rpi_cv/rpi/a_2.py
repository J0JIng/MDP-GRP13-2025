"""
Simple script for Raspberry Pi to take a photo and send it to the
image recognition server defined in config/rpi.yaml.

Notes:
- Requires raspistill to be available on the Pi (legacy camera stack)
- Requires the 'requests' Python package
"""

from __future__ import annotations

import os
import time
import subprocess
from pathlib import Path
from typing import Optional

import requests

from config.load_config import load_rpi_config


def get_image_server() -> tuple[str, int, Optional[int]]:
    """Read IP/port and timeout from rpi.yaml (supports ip or pc_ip).

    Returns (host, port, timeout_seconds_or_None)
    """
    cfg = load_rpi_config()
    api = cfg.get("api", {}) or {}
    host = api.get("ip") or api.get("pc_ip") or "127.0.0.1"
    port = int(api.get("image_port", 5001))
    timeouts = api.get("timeouts", {}) or {}
    img_timeout = timeouts.get("image")
    try:
        timeout_val = int(img_timeout) if img_timeout is not None else None
    except Exception:
        timeout_val = None
    return host, port, timeout_val


def take_photo(filename: str) -> str:
    """Capture a JPEG using raspistill and save to filename.

    Raises subprocess.CalledProcessError if capture fails.
    """
    cmd = [
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

    subprocess.run(cmd, check=True)
    return filename


def send_photo(url: str, filepath: str, timeout: Optional[int] = None) -> requests.Response:
    """POST the photo to the image recognition server's /image endpoint."""
    with open(filepath, "rb") as f:
        files = {"file": (os.path.basename(filepath), f, "image/jpeg")}
        resp = requests.post(url, files=files, timeout=timeout)
        return resp


def main() -> None:
    host, port, timeout = get_image_server()
    url = f"http://{host}:{port}/image"

    # Save into ./images directory
    out_dir = Path.cwd() / "images"
    out_dir.mkdir(parents=True, exist_ok=True)
    filename = out_dir / f"{int(time.time())}_123_L.jpg"

    print(f"Capturing image to: {filename}")
    try:
        take_photo(str(filename))
    except FileNotFoundError:
        print("ERROR: 'raspistill' not found. Run this on a Raspberry Pi with the legacy camera stack enabled.")
        return
    except subprocess.CalledProcessError as e:
        print(f"ERROR: Failed to capture image: {e}")
        return

    print(f"Sending image to: {url}")
    try:
        resp = send_photo(url, str(filename), timeout=timeout)
    except requests.exceptions.RequestException as e:
        print(f"ERROR: Failed to send image: {e}")
        return

    print(f"Response status: {resp.status_code}")
    try:
        print("Response JSON:", resp.json())
    except Exception:
        print("Response Text:")
        print((resp.text or "")[:500])


if __name__ == "__main__":
    main()
