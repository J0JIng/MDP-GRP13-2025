import logging
from queue import Queue
from typing import Any, Dict, List
import bluetooth as bt
import socket
import sys
import subprocess
import json
from helper.socket_utils import sendall, to_bytes
from config.load_config import load_bt_config, load_rpi_config
from helper.logger import prepare_logger
from rpi_main import RPiMain


class AndroidInterface:
    """
    Represents the interface between the Raspberry Pi and an Android device over Bluetooth.

    Args:
    - RPiMain: Instance of the RPiMain class.

    Attributes:
    - RPiMain (RPiMain): Instance of the RPiMain class.
    - host (str): IP address of the Raspberry Pi.
    - uuid (str): Bluetooth UUID for the service.
    - msg_queue (Queue): Queue for storing messages.
    - socket (BluetoothSocket): Bluetooth socket for communication.
    - port (int): Port number for the socket connection.
    - client_socket (BluetoothSocket): Socket for communication with the connected Android device.
    - client_info (tuple): Information about the connected Android client.
    """

    def __init__(self, rpi_main: RPiMain):
        # Initialize AndroidInterface with RPiMain instance
        self.rpi_main = rpi_main
        # self.host = RPI_IP
        # self.uuid = BT_UUID
        self.rpi_config = load_rpi_config()
        self.bt_config = load_bt_config()
        # self.host = self.rpi_config.get("api", {}).get("rpi_ip", "192.168.13.13")
        # self.uuid = self.bt_config.get("uuid", "8357ac59-b23b-4e2d-850e-560b6dd6fcb5")
        self.msg_queue = Queue()
        self.logger = prepare_logger(__name__, level=logging.DEBUG)

    def connect(self):
        host = self.rpi_config.get("api", {}).get("ip", "192.168.13.13")
        uuid = self.bt_config.get("uuid", "00001101-0000-1000-8000-00805F9B34FB")
        service_name = self.bt_config.get("bluetooth", {}).get("service_name", "MDP-Group13-RPi")

        # Grant permission for Bluetooth access
        # subprocess.run("sudo chmod o+rw /var/run/sdp", shell=True)

        # Establish and bind socket
        self.socket = bt.BluetoothSocket(bt.RFCOMM)
        self.logger.info("[Android] BT socket established successfully.")

        try:
            self.port = self.socket.getsockname()[1]
            self.logger.info(f"[Android] Waiting for connection on RFCOMM channel {self.port}")
            self.socket.bind((host, bt.PORT_ANY))
            self.logger.info("[Android] BT socket binded successfully.")

            # Turning advertisable
            subprocess.run("sudo hciconfig hci0 piscan", shell=True)
            self.socket.listen(128)

            # Advertise Bluetooth service
            bt.advertise_service(self.socket, service_name, service_id=uuid,
                                 service_classes=[uuid, bt.SERIAL_PORT_CLASS],
                                 profiles=[bt.SERIAL_PORT_PROFILE])

        except socket.error as e:
            self.logger.error(f"[Android] ERROR: Android socket binding failed - {e}")
            sys.exit()

        self.logger.info("[Android] Waiting for Android connection...")

        try:
            self.client_socket, self.client_info = self.socket.accept()
            self.logger.info(f"[Android] Accepted connection from {self.client_info}")

        except socket.error as e:
            self.logger.error(f"[Android] ERROR: connection failed - {e}")

    def disconnect(self):
        # Close the Bluetooth socket
        try:
            self.socket.close()
            self.logger.info("[Android] Disconnected from Android successfully.")
        except Exception as e:
            self.logger.error(f"[Android] ERROR: Failed to disconnect from Android - {e}")

    def reconnect(self):
        # Disconnect and then connect again
        self.disconnect()
        self.connect()

    def listen(self):
        # Continuously listen for messages from Android
        bt_buffer_size = self.bt_config.get("bluetooth", {}).get("buffer_size", 1024)
        msg_log_max_size = self.bt_config.get("bluetooth", {}).get("msg_log_max_size", 150)

        while True:
            try:
                message = self.client_socket.recv(bt_buffer_size)

                if not message:
                    self.logger.warning("[Android] Android disconnected remotely. Reconnecting...")
                    self.reconnect()

                decodedMsg = message.decode("utf-8")
                if len(decodedMsg) <= 1:
                    continue

                self.logger.debug(f"[Android] Read from Android: {decodedMsg[:msg_log_max_size]}")
                parsed_message = json.loads(decodedMsg)
                msg_type = parsed_message["type"]

                # Route messages to the appropriate destination
                if msg_type == 'NAVIGATION':
                    self.rpi_main.STM.msg_queue.put(message)

                if msg_type == 'START_TASK':
                    parsed_message = self.android_to_algo_payload(parsed_message)
                    parsed_message_bytes = json.dumps(parsed_message).encode("utf-8")
                    self.rpi_main.PC.msg_queue.put(parsed_message_bytes)

                if msg_type == 'FASTEST_PATH':
                    # TODO: implement payload conversion if needed
                    self.rpi_main.PC.msg_queue.put(message)

            except (socket.error, IOError, Exception, ConnectionResetError) as e:
                self.logger.error(f"[Android] ERROR: {e}")

    def send(self):
        # Continuously send messages to Android
        msg_log_max_size = self.bt_config.get("bluetooth", {}).get("msg_log_max_size", 150)

        while True:
            message = self.msg_queue.get()
            # Test code start
#             message_ori =   {
#                 "type": "IMAGE_RESULTS",
            # "data": {
            # "obs_id": "11",
            # "img_id": "30",
            # }
            # }
            # message = {
            #     "type": "NAVIGATION",
            #     "data": {
            #     "commands":  ["LF045", "RF045", "SF040", "RF045", "LF045"],
            #     # "commands": ["UF150"],

            #     "path": [[0,1], [1,1], [2,1], [3,1], [3,3]]
            #     }
            # }
            # message_ori = "hello from rpi"
#             message = json.dumps(message).encode("utf-8")
            # test code end

            exception = True
            while exception:
                try:
                    data = to_bytes(message)
                    sendall(self.client_socket, data)
                    self.logger.debug("[Android] Write to Android: " + data.decode("utf-8",
                                      errors="ignore")[:msg_log_max_size])
                except Exception as e:
                    self.logger.error(f"[Android] ERROR: Failed to write to Android - {e}")
                    self.reconnect()  # reconnect and resend
                else:
                    exception = False  # done sending, get next message

    def android_to_algo_payload(self, msg: Dict[str, Any], default_retrying: bool = False) -> Dict[str, Any]:
        """Convert Android START_TASK message into Algorithm /path input.

        Android shape (msg):
            {"type": "START_TASK", "data": {"task": str, "robot": {...}, "obstacles": [...]}}

        Algorithm /path expected shape:
            {"robot_x": int, "robot_y": int, "robot_dir": int, "retrying": bool,
            "obstacles": [{"id": int, "x": int, "y": int, "d": int}, ...]}
        """
        if msg.get("type") != "START_TASK":
            # raise BridgeError("Expected type START_TASK from Android.")
            self.logger.error("Expected type START_TASK from Android.")
            return {}

        data = msg.get("data") or {}
        robot = data.get("robot") or {}
        obstacles = data.get("obstacles") or []

        try:
            rx = int(robot["x"])  # already zero-based from Android
            ry = int(robot["y"])  # already zero-based from Android
            rdir = self.android_dir_to_algo(str(robot["dir"]))
        except KeyError as e:
            # raise BridgeError(f"Missing robot field: {e}")
            self.logger.error(f"Missing robot field: {e}")
            return {}
        except ValueError:
            self.logger.error("Robot x/y must be integers.")
            return {}

        if not self.is_valid_interior(rx, ry):
            self.logger.error(
                f"Robot at ({rx},{ry}) is invalid for the algorithm. Keep within 1..18 (outer ring is blocked)."
            )
            return {}

        algo_obstacles: List[Dict[str, Any]] = []
        for idx, o in enumerate(obstacles):
            try:
                oid = int(o["id"]) if isinstance(o.get("id"), (int, str)) else idx
                ox = int(o["x"])  # zero-based
                oy = int(o["y"])  # zero-based
                od = self.android_dir_to_algo(str(o["dir"]))
            except KeyError as e:
                # raise BridgeError(f"Missing obstacle field: {e} (at obstacle index {idx})")
                self.logger.error(f"Missing obstacle field: {e} (at obstacle index {idx})")
                return {}
            except ValueError:
                self.logger.error(f"Obstacle fields must be integers (at obstacle index {idx}).")
                return {}

            if not self.is_valid_interior(ox, oy):
                self.logger.error(
                    f"Obstacle id {oid} at ({ox},{oy}) is invalid for the algorithm. Keep within 1..18."
                )

            algo_obstacles.append({"id": oid, "x": ox, "y": oy, "d": od})

        retrying = bool(data.get("retrying", default_retrying))

        return {
            "robot_x": rx,
            "robot_y": ry,
            "robot_dir": rdir,
            "retrying": retrying,
            "obstacles": algo_obstacles,
        }

    def android_dir_to_algo(self, d: str) -> int:
        DIR_MAP_STR_TO_INT = {"N": 0, "E": 2, "S": 4, "W": 6}
        # DIR_MAP_INT_TO_STR = {v: k for k, v in DIR_MAP_STR_TO_INT.items()}

        if d not in DIR_MAP_STR_TO_INT:
            # raise BridgeError(f"Unknown direction string '{d}'. Expected one of N/E/S/W.")
            self.logger.error(f"Unknown direction string '{d}'. Expected one of N/E/S/W.")
            return -1
        return DIR_MAP_STR_TO_INT[d]

    def is_valid_interior(self, x: int, y: int) -> bool:
        return (INTERIOR_MIN <= x <= INTERIOR_MAX_X) and (INTERIOR_MIN <= y <= INTERIOR_MAX_Y)


WIDTH = 20
HEIGHT = 20
INTERIOR_MIN = 1
INTERIOR_MAX_X = WIDTH - 2  # 18
INTERIOR_MAX_Y = HEIGHT - 2  # 18
