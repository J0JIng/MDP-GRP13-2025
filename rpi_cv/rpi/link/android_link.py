import os
from typing import Optional
import bluetooth
from config.load_config import load_bt_config
from link.base import Link
from message.android import AndroidMessage
import time


class AndroidLink(Link):
    """Class for communicating with Android tablet over Bluetooth connection.

    ## General Format
    Messages between the Android app and Raspi will be in the following format:
    ```json
    {"type": "XXX", "data": ...}
    ```

    The `type` field (logical category) has the following possible values:
    - `info`: general messages
    - `error`: error messages, usually in response of an invalid action
    - `location`: the current location of the robot (in Path mode)
    - `image-rec`: image recognition results
    - `mode`: the current mode of the robot (`manual` or `path`)
    - `status`: status updates of the robot (`running` or `finished`)
    - `obstacle`: list of obstacles

    ## Android to RPi

    #### Set Obstacles
    The contents of `obstacles` together with the configured turning radius (`settings.py`) will be passed to the Algorithm API.
    ```json
        {
            "type": "obstacles",
            "data": {
                "obstacles": [{"x": 5, "y": 10, "id": 1, "d": 2}],
                "mode": "0"
            }
        }
    ```
    RPi will store the received commands and path and make a call to the Algorithm API

    ### Start
    Signals to the robot to start dispatching the commands (when obstacles were set).
    ```json
    {"type": "control", "data": "start"}
    ```

    If there are no commands in the queue, the RPi will respond with an error:
    ```json
    {"type": "error", "data": "Command queue is empty, did you set obstacles?"}
    ```

    ### Image Recognition

    #### RPi to Android
    ```json
    {"type": "IMAGE_RESULTS", "data": {"obs_id": "1", "img_id":  "A"}}
    ```

    ### Location Updates (RPi to Android)
    In Path mode, the robot will periodically notify Android with the updated location of the robot.
    ```json
    {"type": "COORDINATES", "data": {"robot": {"x": 1, "y": 1, "dir": "N"}}}
    ```
    where `x`, `y` is the location of the robot, and `dir` is its direction (N/E/S/W).

    ### Path (RPi to Android)
    Full path information sent as a list of [x, y] pairs.
    ```json
    {"type": "PATH", "data": {"path": [[1,1], [1,2], [2,2]]}}
    ```
    """

    def __init__(self):
        super().__init__()
        self.logger.debug("Initializing AndroidLink")
        self.client_sock = None
        self.server_sock = None
        self.config = load_bt_config()
        # Incremental receive buffer and queue for framed messages
        self._recv_buf: str = ""
        self._recv_queue: list[str] = []

    def connect(self):
        """Start Bluetooth server and wait for Android to connect."""
        bt = self.config["bluetooth"]
        adapter = bt["adapter"]
        discoverable_cmd = bt["discoverable_cmd"].format(adapter=adapter)
        service_name = bt["service_name"]
        uuid = bt["resolved_uuid"]
        backlog = int(bt["backlog"])

        self.logger.info("Bluetooth connection starting (adapter=%s, service=%s)", adapter, service_name)
        os.system(discoverable_cmd)

        try:
            self.server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            self.server_sock.bind(("", bluetooth.PORT_ANY))
            self.server_sock.listen(backlog)

            port = self.server_sock.getsockname()[1]
            self.logger.info("Advertising service '%s' on RFCOMM channel %s", service_name, port)

            bluetooth.advertise_service(
                self.server_sock,
                service_name,
                service_id=uuid,
                service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS],
                profiles=[bluetooth.SERIAL_PORT_PROFILE],
            )

            self.logger.info("Awaiting Bluetooth connection on RFCOMM CHANNEL %s", port)
            self.client_sock, client_info = self.server_sock.accept()
            self.logger.info("Accepted connection from: %s", client_info)

        except Exception as e:
            self.logger.error("Error in Bluetooth link connection: %s", e)
            if self.client_sock:
                self.client_sock.close()
            if self.server_sock:
                self.server_sock.close()
            raise e

    def disconnect(self):
        """Disconnect and close sockets."""
        try:
            self.logger.info("Disconnecting Bluetooth link")
            if self.server_sock:
                # self.server_sock.shutdown(bluetooth.SHUT_RDWR) # Not sure why this appears in the reference code, shutdown is not a method as of pybluez 0.30
                self.server_sock.close()
            if self.client_sock:
                # self.client_sock.shutdown(bluetooth.SHUT_RDWR) # Not sure why this appears in the reference code, shutdown is not a method as of pybluez 0.30
                self.client_sock.close()
            self.client_sock = None
            self.server_sock = None
            self.logger.info("Disconnected Bluetooth link")
        except Exception as e:
            self.logger.error("Failed to disconnect Bluetooth link: %s", e)

    def send(self, message: AndroidMessage, tries: int = 1) -> None:
        """Send a Message to Android over Bluetooth.

        Notes:
        - We append a newline for framing (Android side can split on '\n').
        - Use sendall() semantics to reduce partial writes.
        - Optional short delay between messages can reduce coalescing on the
          Android read() side; configurable via bluetooth.send_throttle_ms.
        - The `tries` parameter controls how many times the payload is sent;
          the default (2) results in the message being transmitted twice.
        """
        newline = self.config["bluetooth"].get("newline_delimited", True)
        throttle_ms = int(self.config["bluetooth"].get("send_throttle_ms", 0))

        if not self.client_sock:
            self.logger.warning("Bluetooth client socket not available")
            return

        max_attempts = max(int(tries), 1)
        payload_str = message.jsonify
        if newline:
            payload_str += "\n"
        data = payload_str.encode("utf-8")

        for attempt in range(1, max_attempts + 1):
            try:
                total_sent = 0
                while total_sent < len(data):
                    sent = self.client_sock.send(data[total_sent:])
                    # Some mock/stub sockets return None; treat as full slice written
                    if sent is None:
                        sent = len(data) - total_sent
                    if sent == 0:
                        raise OSError("Bluetooth socket connection broken during send")
                    total_sent += sent

                self.logger.debug(
                    "Sent to Android (attempt %d/%d): %s", attempt, max_attempts, message.jsonify
                )

                if throttle_ms > 0:
                    time.sleep(throttle_ms / 1000.0)

            except OSError as e:
                if attempt == max_attempts:
                    self.logger.error("Error sending message to Android: %s", e)
                    raise e
                self.logger.warning(
                    "Bluetooth send failed on attempt %d/%d: %s", attempt, max_attempts, e
                )
            except Exception as e:
                if attempt == max_attempts:
                    self.logger.error("Unexpected error sending message to Android: %s", e)
                    raise e
                self.logger.warning(
                    "Unexpected error during Bluetooth send on attempt %d/%d: %s",
                    attempt,
                    max_attempts,
                    e,
                )
            time.sleep(1)

    def recv(self) -> Optional[str]:
        """Receive one complete JSON message from Android.

        Handles both newline-delimited and raw concatenated/fragmented JSON by buffering
        and extracting complete objects using either '\n' or balanced-brace parsing.
        Returns one message per call, or None if no complete message is available yet.
        """
        if not self.client_sock:
            self.logger.warning("Bluetooth client socket not available")
            return None

        # bufsize = int(self.config["bluetooth"].get("recv_bufsize", 1024))
        bufsize = self.config["bluetooth"].get("recv_bufsize", 1024)
        try:
            # If we already have framed messages queued, serve one immediately
            if self._recv_queue:
                return self._recv_queue.pop(0)

            tmp = self.client_sock.recv(bufsize)
            if tmp is None:
                return None
            self.logger.debug("Raw recv bytes: %r", tmp)
            # Decode and append to buffer (JSON payloads are UTF-8)
            try:
                chunk = tmp.decode("utf-8", errors="ignore")
            except Exception:
                chunk = tmp.decode("latin-1", errors="ignore")
            self._recv_buf += chunk

            # Fast-path: extract newline-delimited messages first
            while "\n" in self._recv_buf:
                line, sep, rest = self._recv_buf.partition("\n")
                self._recv_buf = rest
                line = line.strip()
                if line:
                    self._recv_queue.append(line)

            # Fallback: extract balanced-brace JSON objects from remaining buffer
            extracted_any = True
            while extracted_any:
                extracted_any = False
                msg = self._try_extract_json_object()
                if msg is not None:
                    self._recv_queue.append(msg)
                    extracted_any = True

            if self._recv_queue:
                framed = self._recv_queue.pop(0)
                self.logger.debug("Framed from Android: %s", framed)
                return framed
            return None
        except OSError as e:
            self.logger.error("Error receiving message from Android: %s", e)
            raise e
        except Exception as e:
            self.logger.error("Unexpected error receiving message from Android: %s", e)
            raise e

    def _try_extract_json_object(self) -> Optional[str]:
        """Attempt to extract one balanced-brace JSON object from the buffer.

        This parser is naive but sufficient for our payloads: it counts braces while
        skipping those that appear inside quoted strings and handling escapes.
        Returns the extracted JSON string, or None if incomplete.
        """
        buf = self._recv_buf
        if not buf:
            return None

        # Skip leading whitespace/noise until first '{'
        start = buf.find('{')
        if start == -1:
            # No JSON object start yet; drop excessive leading noise if any
            if len(buf) > 4096:  # prevent unbounded growth
                self._recv_buf = ""
            return None

        depth = 0
        in_string = False
        escape = False

        for i in range(start, len(buf)):
            ch = buf[i]
            if in_string:
                if escape:
                    escape = False
                    continue
                if ch == '\\':
                    escape = True
                elif ch == '"':
                    in_string = False
            else:
                if ch == '"':
                    in_string = True
                elif ch == '{':
                    depth += 1
                elif ch == '}':
                    depth -= 1
                    if depth == 0:
                        # Extract complete object
                        obj = buf[start:i+1].strip()
                        # Keep the remainder in buffer
                        self._recv_buf = buf[i+1:]
                        return obj
        # Incomplete object; keep buffer as-is
        return None
