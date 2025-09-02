from typing import Optional
import serial
from config.load_config import load_stm32_config
from link.base import Link


class STMLink(Link):
    """Class for communicating with STM32 microcontroller over UART serial connection.

    ### RPi to STM32
    RPi sends the following commands to the STM32.

    #### Path mode commands
    High speed forward/backward, with turning radius of `3x1`
    - `FW0x`: Move forward `x` units
    - `BW0x`: Move backward `x` units
    - `FL00`: Move to the forward-left location
    - `FR00`: Move to the forward-right location
    - `BL00`: Move to the backward-left location
    - `BR00`: Move to the backward-right location

    #### Manual mode commands
    - `FW--`: Move forward indefinitely
    - `BW--`: Move backward indefinitely
    - `TL--`: Steer left indefinitely
    - `TR--`: Steer right indefinitely
    - `STOP`: Stop all servos

    ### STM32 to RPi
    After every command received on the STM32, an acknowledgement (string: `ACK`) must be sent back to the RPi.
    This signals to the RPi that the STM32 has completed the command, and is ready for the next command.

    """

    def __init__(self):
        """
        Constructor for STMLink.
        """
        super().__init__()
        self.logger.debug("Initializing STMLink")
        self.serial_link = None
        self.config = load_stm32_config()

    def connect(self):
        """Connect to STM32 using serial UART connection, given the serial port and the baud rate"""
        self.logger.info("Connecting to STM32")
        serial_port_id= self.config["serial_port"]["id"]
        baud_rate = self.config["serial_port"]["baud_rate"]
        self.serial_link = serial.Serial(port=serial_port_id, baudrate=baud_rate)
        self.logger.info("Connected to STM32")

    def disconnect(self):
        """Disconnect from STM32 by closing the serial link that was opened during connect()"""
        self.logger.info("Disconnecting from STM32")
        if self.serial_link:
            self.serial_link.close()
            self.serial_link = None
            self.logger.info("Disconnected from STM32")
        else:
            self.logger.warning("Serial link to STM32 was not open")

    def send(self, message: str) -> None:
        """Send a message to STM32, utf-8 encoded

        Args:
            message (str): message to send
        """
        self.logger.info(f"Sending to STM32: {message}")
        if self.serial_link:
            self.serial_link.write(f"{message}".encode("utf-8"))
            self.logger.info(f"Sent to STM32: {message}")
        else:
            self.logger.warning("Serial link to STM32 was not open")

    def recv(self) -> Optional[str]:
        """Receive a message from STM32, utf-8 decoded

        Returns:
            Optional[str]: message received
        """
        self.logger.info("Receiving from STM32")
        if self.serial_link:
            message = self.serial_link.readline().strip().decode("utf-8")
            self.logger.info(f"Received from STM32: {message}")
            return message
        else:
            self.logger.warning("Serial link to STM32 was not open")
            return None
