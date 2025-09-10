import base64
import json
import os
import io
from typing import Dict, Any, Optional
from picamera import PiCamera
import cv2
import numpy as np
import time
from datetime import datetime


def capture_in_memory() -> Optional[np.ndarray]:
    stream = io.BytesIO()
    camera = PiCamera()
    try:
        camera.resolution = (640, 480)  # matches your resize target
        time.sleep(0.5)                 # sensor warm-up
        camera.capture(stream, format='jpeg', use_video_port=True)
    finally:
        camera.close()

    stream.seek(0)
    data = np.frombuffer(stream.read(), dtype=np.uint8)
    img = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if img is None:
        print("[Camera] Failed to decode captured image")
        return None
    return img


def get_image(final_image: bool = False) -> bytes:
    """
    Capture, resize, and return a JSON (bytes) payload with base64 JPEG.
    Returns:
      b'{"type":"IMAGE_TAKEN","final_image":bool,"ok":bool,"data":{"image":str},"error":str}'
    """
    encoded_string = ""
    err_msg = ""
    img = capture_in_memory()
    if img is not None:
        if img.shape[1] != 640 or img.shape[0] != 480:
            img = cv2.resize(img, (640, 480))
        ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if ok:
            encoded_string = base64.b64encode(buf.tobytes()).decode('utf-8')
        else:
            err_msg = "Failed to JPEG-encode image"
            print(f"[Camera] {err_msg}")
    else:
        err_msg = "In-memory capture returned None"
        print(f"[Camera] {err_msg}")

    message: Dict[str, Any] = {
        "type": "IMAGE_TAKEN",
        "final_image": final_image,
        "ok": err_msg == "",
        "data": {"image": encoded_string},
        "error": err_msg,
    }
    return json.dumps(message).encode('utf-8')
