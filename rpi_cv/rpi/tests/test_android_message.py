from message.android import AndroidMessage
import json


def test_android_message_jsonify_roundtrip():
    msg = AndroidMessage("info", {"a": 1, "b": "x"})
    js = msg.jsonify
    data = json.loads(js)
    assert data == {"type": "info", "data": {"a": 1, "b": "x"}}
    assert msg.cat == "info"
    assert msg.value == {"a": 1, "b": "x"}


def test_android_message_coordinates_factory():
    msg = AndroidMessage.coordinates(3, 5, "N")
    data = json.loads(msg.jsonify)
    assert data["type"] == "COORDINATES"
    assert "robot" in data["data"]
    assert data["data"]["robot"] == {"x": 3, "y": 5, "dir": "N"}


def test_android_message_image_results_factory():
    msg = AndroidMessage.image_results(obs_id="1", img_id="A")
    data = json.loads(msg.jsonify)
    assert data == {"type": "IMAGE_RESULTS", "data": {"obs_id": "1", "img_id": "A"}}


def test_android_message_path_factory():
    path = [[0, 0], [1, 2], [3, 4]]
    msg = AndroidMessage.path(path)
    data = json.loads(msg.jsonify)
    assert data == {"type": "PATH", "data": {"path": path}}
