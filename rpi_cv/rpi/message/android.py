
import json
from typing import Any


class AndroidMessage:
    """
    Class representing a message to/from the Android app.
    """

    def __init__(self, cat: str, value: Any):
        self._cat = cat
        self._value = value

    @property
    def cat(self) -> str:
        return self._cat

    @property
    def value(self) -> Any:
        return self._value

    @property
    def jsonify(self) -> str:
        return json.dumps({'cat': self._cat, 'value': self._value})
