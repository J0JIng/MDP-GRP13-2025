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

    NOTE: Wire format updated to use keys {"type", "data"} to match Android.
    The logical category names remain the same: "info", "error", "location",
    "image-rec", "mode", "status", "obstacle".
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
