from stm.robot_controller import RobotController


def main() -> None:
    """Simple script to move the robot forward by 50 cm."""
    # Adjust the port/baudrate if your setup differs.
    robot = RobotController('COM4', 115200)
    ok = robot.move_forward(50)
    print(f"Move forward 50cm command acknowledged: {ok}")


if __name__ == "__main__":
    main()
