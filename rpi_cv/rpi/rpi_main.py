import logging
from threading import Thread
from android import AndroidInterface
from helper.logger import prepare_logger
from pc import PCInterface
from stm import STMInterface
import argparse


class RPiMain:
    def __init__(self, task2):
        # Initialize interfaces
        self.logger = prepare_logger(__name__, level=logging.DEBUG)
        self.Android = AndroidInterface(self)
        self.PC = PCInterface(self, task2=task2)
        self.STM = STMInterface(self, task2=task2)

    def connect_components(self):
        # Connect all components
        self.Android.connect()
        self.PC.connect()
        self.STM.connect()

    def cleanup(self):
        # Disconnect from all components
        self.Android.disconnect()
        self.PC.disconnect()
        # self.STM.disconnect()

    def run(self):
        self.logger.info("[RPiMain] Starting RPiMain...")

        # Connect components
        self.connect_components()
        self.logger.info("[RPiMain] Components connected successfully")

        # Create threads for sending messages
        Android_send = Thread(target=self.Android.send, name="Android_send_thread")
        PC_send = Thread(target=self.PC.send, name="PC_send_thread")
        STM_send = Thread(target=self.STM.send, name="STM_send_thread")

        # Create threads for receiving messages
        Android_listen = Thread(target=self.Android.listen, name="Android_listen_thread")
        PC_listen = Thread(target=self.PC.listen, name="PC_listen_thread")

        # Start sending threads
        Android_send.start()
        PC_send.start()
        STM_send.start()
        self.logger.info("[RPiMain] Sending threads started successfully")

        # Start listening threads
        Android_listen.start()
        PC_listen.start()
        self.logger.info("[RPiMain] Listening threads started successfully")

        # Wait for threads to end
        Android_send.join()
        PC_send.join()
        STM_send.join()
        Android_listen.join()
        PC_listen.join()

        self.logger.info("[RPiMain] All threads concluded, cleaning up...")

        # Cleanup after threads finish
        self.cleanup()

        self.logger.info("[RPiMain] Exiting RPiMain...")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run RPi main controller for task 1 or task 2.")
    parser.add_argument("--task", type=int, choices=[1, 2], default=1,
                        help="Task number to run: 1 (default) or 2.")
    args = parser.parse_args()

    task2_flag = (args.task == 2)
    print(f"[RPiMain] Launching with task={args.task} (task2_flag={task2_flag})")
    rpi = RPiMain(task2_flag)
    rpi.run()
