from stm.robot_controller import RobotController

ROBOT_PORT = "/dev/ttyACM0"  # adjust as necessary for your Pi
ROBOT_BAUD = 115200


def movement_test():
    robot = RobotController(ROBOT_PORT, ROBOT_BAUD)

    for i in range(4):
        ok = True
        print(f"Movement test iteration {i+1}/4")

        print("Moving forward 20cm...")
        ok = robot.move_forward(20) and ok
        if not ok:
            print("  WARN: forward 20 not acknowledged")
        else:
            print("  OK")

        print("Turning right 90 degrees...")
        ok = robot.turn_right(90, True) and ok
        if not ok:
            print("  WARN: turn right 90 not acknowledged")
        else:
            print("  OK")


if __name__ == "__main__":
    movement_test()
