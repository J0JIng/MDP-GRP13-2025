from link.android_link import AndroidLink
from link.stm32_link import STMLink
from time import sleep

from message.android import AndroidMessage


def main():
    android_link = AndroidLink()
    android_link.connect()

    msg = AndroidMessage(cat="info", value="Hello from RPi!")
    android_link.send(msg)
    sleep(30)

    # stm_link = STMLink()


if __name__ == "__main__":
    main()
