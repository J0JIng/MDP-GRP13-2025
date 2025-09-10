from message.android import AndroidMessage
import json


def test_android_message_jsonify_roundtrip():
    msg = AndroidMessage("info", {"a": 1, "b": "x"})
    js = msg.jsonify
    data = json.loads(js)
    assert data == {"cat": "info", "value": {"a": 1, "b": "x"}}
    assert msg.cat == "info"
    assert msg.value == {"a": 1, "b": "x"}
