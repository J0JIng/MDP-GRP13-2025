import os
from typing import Optional
import bluetooth
from config.load_config import load_bt_config
from link.base import Link
from message.android import AndroidMessage


class AndroidLink(Link):
    """Class for communicating with Android tablet over Bluetooth connection. 

    ## General Format
    Messages between the Android app and Raspi will be in the following format:
    ```json
    {"cat": "xxx", "value": "xxx"}
    ```

    The `cat` (for category) field with the following possible values:
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
    "cat": "obstacles",
    "value": {
        "obstacles": [{"x": 5, "y": 10, "id": 1, "d": 2}],
        "mode": "0"
    }
    }
    ```
    RPi will store the received commands and path and make a call to the Algorithm API

    ### Start
    Signals to the robot to start dispatching the commands (when obstacles were set).
    ```json
    {"cat": "control", "value": "start"}
    ```

    If there are no commands in the queue, the RPi will respond with an error:
    ```json
    {"cat": "error", "value": "Command queue is empty, did you set obstacles?"}
    ```

    ### Image Recognition 

    #### RPi to Android
    ```json
    {"cat": "image-rec", "value": {"image_id": "A", "obstacle_id":  "1"}}
    ```

    ### Location Updates (RPi to Android)
    In Path mode, the robot will periodically notify Android with the updated location of the robot.
    ```json
    {"cat": "location", "value": {"x": 1, "y": 1, "d": 0}}
    ```
    where `x`, `y` is the location of the robot, and `d` is its direction.
    """

    def __init__(self):
        super().__init__()
        self.logger.debug("Initializing AndroidLink")
        self.client_sock = None
        self.server_sock = None
        self.config = load_bt_config()

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
            raise

    def disconnect(self):
        """Disconnect and close sockets."""
        try:
            self.logger.info("Disconnecting Bluetooth link")
            if self.server_sock:
                self.server_sock.close()
            if self.client_sock:
                self.client_sock.close()
            self.client_sock = None
            self.server_sock = None
            self.logger.info("Disconnected Bluetooth link")
        except Exception as e:
            self.logger.error("Failed to disconnect Bluetooth link: %s", e)

    def send(self, message: AndroidMessage) -> None:
        """Send a Message to Android over Bluetooth."""
        newline = self.config["bluetooth"].get("newline_delimited", True)
        if self.client_sock:
            try:
                payload_str = message.jsonify
                if newline:
                    payload_str += "\n"
                self.client_sock.send(payload_str.encode("utf-8"))
                self.logger.debug("Sent to Android: %s", message.jsonify)
            except OSError as e:
                self.logger.error("Error sending message to Android: %s", e)
                raise
        else:
            self.logger.warning("Bluetooth client socket not available")

    def recv(self) -> Optional[str]:
        """Receive message from Android."""
        if not self.client_sock:
            self.logger.warning("Bluetooth client socket not available")
            return None

        bufsize = int(self.config["bluetooth"].get("recv_bufsize", 1024))
        try:
            tmp = self.client_sock.recv(bufsize)
            self.logger.debug("Raw recv bytes: %r", tmp)
            message = tmp.strip().decode("utf-8")
            self.logger.debug("Received from Android: %s", message)
            return message
        except OSError as e:
            self.logger.error("Error receiving message from Android: %s", e)
            raise
