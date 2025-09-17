import asyncio
from time import sleep

from robot_controller import RobotController


def cb_fn(*args):
    print("Obstacle detected!")
    print(args)


async def main():
    robot = RobotController('COM4', 115200)
    asyncio.create_task(robot.sig_obst_callback(cb_fn))

    while 1:
        print(x := robot.get_quaternion())
        # assert x is not None
        await asyncio.sleep(0.2)
        print(x := robot.get_yaw())
        #assert x is not None
        await asyncio.sleep(0.2)
        print(x := robot.get_gyro_Z())
        #assert x is not None
        await asyncio.sleep(0.2)
        print(x := robot.get_ir_L())
        #assert x is not None
        await asyncio.sleep(0.2)
        print(x := robot.get_ir_R())
        #assert x is not None
        await asyncio.sleep(0.2)
        print(x := robot.move_forward(10))
        #assert x
        await asyncio.sleep(0.2)
        print(x := robot.move_backward(10))
       # assert x
        await asyncio.sleep(0.2)


if __name__ == '__main__':
    asyncio.run(main())