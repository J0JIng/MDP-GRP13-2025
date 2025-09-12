import base64
import json
import os
import tempfile
from typing import Dict, Any, Optional
from picamera import PiCamera
import cv2
import time
from datetime import datetime


def capture_to_file() -> Optional[str]:
    """
    Capture a single JPEG frame to a temp file and return its path.
    """
    tmp_fd, tmp_path = tempfile.mkstemp(suffix=".jpg")
    os.close(tmp_fd)  # close the OS-level fd, cv2 will open the file itself

    camera = PiCamera()
    try:
        camera.resolution = (640, 480)
        time.sleep(0.5)  # sensor warm-up
        camera.capture(tmp_path, format='jpeg', use_video_port=True)
    finally:
        camera.close()

    return tmp_path


def get_image(final_image: bool = False) -> bytes:
    """
    Capture, resize, and return a JSON (bytes) payload with base64 JPEG.
    Returns:
      b'{"type":"IMAGE_TAKEN","final_image":bool,"ok":bool,"data":{"image":str},"error":str}'
    """
    encoded_string = ""
    err_msg = ""
    tmp_path = capture_to_file()

    if tmp_path and os.path.exists(tmp_path):
        img = cv2.imread(tmp_path)
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
            err_msg = "cv2.imread failed to decode image"
            print(f"[Camera] {err_msg}")
        os.remove(tmp_path)
    else:
        err_msg = "Failed to capture image to file"
        print(f"[Camera] {err_msg}")

    message: Dict[str, Any] = {
        "type": "IMAGE_TAKEN",
        "final_image": final_image,
        "ok": err_msg == "",
        "data": {"image": encoded_string},
        "error": err_msg,
    }
    return json.dumps(message).encode('utf-8')
