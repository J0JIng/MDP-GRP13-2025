import json
from typing import Any, Final, Literal, cast

Category = Literal[
    # Legacy/lowercase categories still used internally/logging
    "info", "error", "location", "image-rec", "mode", "status", "obstacle",
    # Android-app expected uppercase message types we send out
    "COORDINATES", "IMAGE_RESULTS", "PATH"
]

INFO: Final[Category] = "info"
ERROR: Final[Category] = "error"
LOCATION: Final[Category] = "location"
IMAGE_REC: Final[Category] = "image-rec"
MODE: Final[Category] = "mode"
STATUS: Final[Category] = "status"
OBSTACLE: Final[Category] = "obstacle"

_ALLOWED: set[Category] = {INFO, ERROR, LOCATION, IMAGE_REC, MODE, STATUS, OBSTACLE,
                           "COORDINATES", "IMAGE_RESULTS", "PATH"}


class AndroidMessage:
    """
    Class representing a message to/from the Android app.

    Wire format uses keys {"type", "data"} to match the Android client.
    Use the factory helpers below for messages the Android app parses
    directly from the RPi (COORDINATES, IMAGE_RESULTS, PATH). These helpers
    ensure the JSON schema matches Android expectations exactly.

    Summary of Android-expected shapes (see android/controllers/RpiController.java):
    - COORDINATES:
        {"type":"COORDINATES", "data": {"robot": {"x": int, "y": int, "dir": str}}}
    - IMAGE_RESULTS:
        {"type":"IMAGE_RESULTS", "data": {"obs_id": str|int, "img_id": str}}
    - PATH:
        {"type":"PATH", "data": {"path": list[list[int, int]]}}
    """

    def __init__(self, cat: Category, value: Any):
        # PyLance enforces Category at type-check time; this is a safety net at runtime.
        if cat not in _ALLOWED:  # will basically never trip if types are respected
            raise ValueError(f"Invalid category: {cat!r}")
        self._cat: Category = cast(Category, cat)
        self._value = value

    @property
    def cat(self) -> Category:
        return self._cat

    @property
    def value(self) -> Any:
        return self._value

    @property
    def jsonify(self) -> str:
        """Serialize to JSON string using the Android-preferred schema."""
        return json.dumps({"type": self._cat, "data": self._value})

    # -------- Factory helpers for Android-facing messages --------
    @classmethod
    def coordinates(cls, x: int, y: int, dir: str) -> "AndroidMessage":
        """Create a COORDINATES message with nested robot object and 'dir' key.

        Android expects: {"type":"COORDINATES", "data": {"robot": {"x", "y", "dir"}}}
        """
        payload = {"robot": {"x": int(x), "y": int(y), "dir": str(dir)}}
        return cls("COORDINATES", payload)

    @classmethod
    def image_results(cls, obs_id, img_id) -> "AndroidMessage":
        """Create an IMAGE_RESULTS message with keys 'obs_id' and 'img_id'."""
        if img_id == 'NA':  # img is recognized as NA
            img_id = '0'
        payload = {"obs_id": obs_id, "img_id": img_id}
        return cls("IMAGE_RESULTS", payload)

    @classmethod
    def path(cls, path_coords: list) -> "AndroidMessage":
        """Create a PATH message.

        path_coords should be a list of [x, y] pairs (ints).
        """
        return cls("PATH", {"path": path_coords})
