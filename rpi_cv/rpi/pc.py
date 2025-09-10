import logging
from queue import Queue
import socket
import json

from config.load_config import load_bt_config, load_rpi_config
from helper.logger import prepare_logger
from helper.socket_utils import sendall, to_bytes


class PCInterface:
    def __init__(self, RPiMain, task2):
        # Initialize PCInterface with RPiMain instance and connection details
        self.RPiMain = RPiMain
        self.rpi_config = load_rpi_config()
        self.bt_config = load_bt_config()
        # self.host = RPI_IP
        # self.port = PC_PORT
        self.client_socket = None
        self.msg_queue = Queue()
        self.send_message = False
        self.obs_id = 1
        self.task2 = task2
        self.logger = prepare_logger(__name__, level=logging.DEBUG)

    def connect(self):
        host = self.rpi_config.get("api", {}).get("ip", "192.168.13.13")
        port = self.rpi_config.get("api", {}).get("port", 5000)

        # Establish a connection with the PC
        if self.client_socket is not None:
            self.disconnect()

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                # allow the socket to be reused immediately after it is closed
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.logger.info("[PC] Socket established successfully.")
                # sock.bind((self.host, self.port))
                sock.bind((host, port))
                sock.listen(128)

                self.logger.info("[PC] Waiting for PC connection...")
                self.client_socket, self.address = sock.accept()  # blocks until a client connects to the server
                self.send_message = True
        except socket.error as e:
            self.logger.error(f"[PC] Failed to connect - {e}")
        else:
            self.logger.info(f"[PC] PC connected successfully: {self.address}")

    def disconnect(self):
        # Disconnect from the PC
        try:
            if self.client_socket is not None:
                self.client_socket.close()
                self.client_socket = None
                self.send_message = False
                self.logger.info("[PC] Disconnected from PC successfully.")
        except Exception as e:
            self.logger.error(f"[PC] Failed to disconnect from PC: {e}")

    def reconnect(self):
        # Disconnect and then connect again
        self.disconnect()
        self.connect()

    def listen(self):
        # Continuously listen for messages from the PC
        msg_log_max_size = self.bt_config.get("bluetooth", {}).get("msg_log_max_size", 150)

        while True:
            try:
                if self.client_socket is not None:
                    # # Receive the length of the message
                    length_bytes = self.client_socket.recv(4)
                    if not length_bytes:
                        self.logger.warning("[PC] Client disconnected.")
                        break
                    message_length = int.from_bytes(length_bytes, byteorder="big")

                    # Receive the message
                    message = self.client_socket.recv(message_length)
                    if not message:
                        self.send_message = False
                        self.logger.warning("[PC] PC disconnected remotely. Reconnecting...")
                        self.reconnect()

                    decoded_msg = message.decode("utf-8")
                    if len(decoded_msg) <= 1:
                        continue

                    print("[PC] Read from PC:", decoded_msg[:msg_log_max_size])
                    # Switch to debug logging for message content
                    self.logger.debug(f"[PC] Read from PC: {decoded_msg[:msg_log_max_size]}")
                    parsed_msg = json.loads(decoded_msg)
                    msg_type = parsed_msg["type"]

                    # Route messages to the appropriate destination
                    # PC -> Rpi -> STM
                    if msg_type == 'NAVIGATION':
                        self.RPiMain.STM.msg_queue.put(message)

                    # PC -> Rpi -> Android
                    elif msg_type == 'IMAGE_RESULTS' or msg_type in ['COORDINATES', 'PATH']:
                        # Real code
                        # TODO: comment if no android connection during testing
                        self.RPiMain.Android.msg_queue.put(message)
                        if self.task2:
                            if self.obs_id == 1:
                                if parsed_msg["data"]["img_id"] == "39":  # left
                                    direction = "FIRSTLEFT"
                                else:
                                    direction = "FIRSTRIGHT"
                                path_message = {"type": "NAVIGATION", "data": {
                                    "commands": [direction, "SB025", "YF150"], "path": []}}
                                self.obs_id += 1
                            else:
                                if parsed_msg["data"]["img_id"] == "39":  # left
                                    direction = "SECONDLEFT"
                                else:
                                    direction = "SECONDRIGHT"
                                path_message = {"type": "NAVIGATION", "data": {"commands": [direction], "path": []}}

                            json_path_message = json.dumps(path_message)
                            encode_path_message = json_path_message.encode("utf-8")
                            self.RPiMain.STM.msg_queue.put(encode_path_message)
                        # Temp code: pass
                        # pass

                    elif msg_type == "FASTEST_PATH":
                        path_message = {"type": "NAVIGATION", "data": {"commands": ["YF150"], "path": []}}
                        json_path_message = json.dumps(path_message)
                        encode_path_message = json_path_message.encode("utf-8")
                        self.RPiMain.STM.msg_queue.put(encode_path_message)

                    else:
                        self.logger.error(f"[PC] Received message with unknown type: {message}")
                else:
                    self.logger.error("[PC] Client socket is not initialized.")
                    return None

            except (socket.error, IOError, Exception, ConnectionResetError) as e:
                self.logger.error(f"[PC] ERROR: {e}")

    def send(self):
        # Continuously send messages to the PC Client
        while True:
            if not self.send_message:
                continue

            raw_msg = self.msg_queue.get()  # expected bytes
            try:
                msg_str = to_bytes(raw_msg).decode("utf-8")  # normalize then decode
            except Exception:
                # fallback: stringify representation
                msg_str = str(raw_msg)

            exception = True
            while exception:
                try:
                    packet = self.prepend_msg_size(msg_str)
                    sendall(self.client_socket, packet)
                    self.logger.debug(f"[PC] Write to PC: first 100= {packet[:100]}")
                except Exception as e:
                    self.logger.error(f"[PC] Failed to write to PC - {e}")
                    self.reconnect()
                else:
                    exception = False

    def prepend_msg_size(self, message):
        message_bytes = message.encode("utf-8")
        message_len = len(message_bytes)

        length_bytes = message_len.to_bytes(4, byteorder="big")
        return length_bytes + message_bytes
