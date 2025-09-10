"""Socket utility helpers shared by PC and Android interfaces."""
from __future__ import annotations

from typing import Union


def sendall(sock, data: Union[bytes, bytearray, memoryview]):
    """Reliable sendall implementation.

    Some socket-like objects (e.g. certain Bluetooth stacks) may not expose a
    native sendall or have partial send behavior. This function ensures the
    full buffer is transmitted or raises an error.
    """
    if isinstance(data, (bytearray, memoryview)):
        view = memoryview(data).tobytes()
    else:
        view = data  # assume bytes

    total_sent = 0
    data_len = len(view)
    while total_sent < data_len:
        sent = sock.send(view[total_sent:])
        if sent is None:
            # treat None same as 0 (no progress)
            sent = 0
        if sent == 0:
            raise RuntimeError("Socket connection broken during sendall")
        total_sent += sent


def to_bytes(msg) -> bytes:
    """Coerce message into bytes.

    Accepts str, bytes, bytearray, memoryview, or arbitrary objects (stringified).
    """
    if isinstance(msg, bytes):
        return msg
    if isinstance(msg, (bytearray, memoryview)):
        return bytes(msg)
    if isinstance(msg, str):
        return msg.encode("utf-8")
    return str(msg).encode("utf-8")
