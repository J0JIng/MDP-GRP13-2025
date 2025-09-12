from android import AndroidInterface
from rpi_main import RPiMain
from helper.socket_utils import sendall, to_bytes
import time


def main():
    android = AndroidInterface(RPiMain(task2=False))

    # Connect and wait for Android to pair/connect
    android.connect()

    try:
        # Send a simple test message to Android
        message = "hello from rpi"
        sendall(android.client_socket, to_bytes(message))
        print(f"[AndroidTest] Sent: {message}")

        # Tiny pause to ensure the message flushes before closing
        time.sleep(0.5)
    finally:
        # Always disconnect cleanly
        android.disconnect()
        print("[AndroidTest] Disconnected.")


if __name__ == "__main__":
    main()
