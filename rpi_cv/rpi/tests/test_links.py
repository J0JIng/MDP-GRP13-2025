import sys
import types
from typing import Any, cast
from message.android import AndroidMessage
from link.stm32_link import STMLink


class FakeSocket:
    def __init__(self):
        self.sent = []
        self._recv_data = []

    def send(self, b: bytes):  # AndroidLink send path
        self.sent.append(b)

    def queue_recv(self, *messages: bytes):
        self._recv_data.extend(list(messages))

    def recv(self, bufsize: int):
        if not self._recv_data:
            return b""
        return self._recv_data.pop(0)


class FakeSerial:
    def __init__(self, lines):
        self.lines = list(lines)
        self.written = []

    def write(self, b: bytes):
        self.written.append(b)

    def readline(self):
        if not self.lines:
            return b""
        return self.lines.pop(0)

    def close(self):
        pass


def test_android_link_send_and_recv(monkeypatch):
    # Provide a minimal stub bluetooth module BEFORE import
    dummy_bt = types.ModuleType("bluetooth")
    dummy_bt.RFCOMM = 1  # type: ignore
    dummy_bt.SERIAL_PORT_CLASS = 1  # type: ignore
    dummy_bt.SERIAL_PORT_PROFILE = 1  # type: ignore

    def _no_use(*a, **k):
        raise AssertionError("Bluetooth functions should not be invoked in this unit test")
    dummy_bt.BluetoothSocket = _no_use  # type: ignore[attr-defined]
    dummy_bt.advertise_service = _no_use  # type: ignore[attr-defined]
    sys.modules.setdefault("bluetooth", dummy_bt)

    from link.android_link import AndroidLink  # import after stubbing

    link = AndroidLink()
    # Inject fake socket
    fake = FakeSocket()
    link.client_sock = cast(Any, fake)  # type: ignore[assignment]

    # Force newline behavior
    link.config["bluetooth"]["newline_delimited"] = True

    msg = AndroidMessage("info", "hi")
    link.send(msg)
    assert fake.sent, "Message should have been sent"
    assert fake.sent[0].decode("utf-8").endswith("\n")

    # Test recv path
    fake.queue_recv(b'{"type": "info", "data": "hello"}\n')
    received = link.recv()
    assert received == '{"type": "info", "data": "hello"}'


def test_stm_link_send_and_recv(monkeypatch):
    link = STMLink()
    fake_serial = FakeSerial([b"ACK\n"])
    link.serial_link = cast(Any, fake_serial)  # type: ignore[assignment]

    link.send("FW01")
    assert fake_serial.written == [b"FW01"]

    out = link.recv()
    assert out == "ACK"
