from link.android_link import AndroidLink
from link.stm32_link import STMLink
from time import sleep

def main():
    android_link = AndroidLink()
    android_link.connect()

    sleep(30)

    # stm_link = STMLink()


if __name__ == "__main__":
    main()
