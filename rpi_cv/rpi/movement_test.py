from stm.robot_controller import RobotController
import time

ROBOT_PORT = "/dev/ttyACM0"  # adjust as necessary for your Pi
ROBOT_BAUD = 115200


def movement_test():
    robot = RobotController(ROBOT_PORT, ROBOT_BAUD)

#     for i in range(4):
#         print(f"Movement test iteration {i+1}/4")
#
#         print("Moving forward 20cm...")
#         ok = robot.move_forward(20)
#         if not ok:
#             print("  WARN: forward 20 not acknowledged")
#             return False
#         print("  OK")

#         print("Turning right 90 degrees...")
#         ok = robot.turn_right(90, True)
#         if not ok:
#             print("  WARN: turn right 90 not acknowledged")
#             return False
#         print("  OK")
#
#     return True

#     robot.crawl_forward_until_obstacle()
#     robot.turn_left(90, True)
#     time.sleep(10)
#     robot.turn_left(90, False)
#     robot.crawl_forward(60)
#     time.sleep(5)
#     robot.crawl_backward(60)

#     robot.turn_right(90, True)
#     time.sleep(3)
#     robot.turn_left(90, True)
#     time.sleep(3)
#     robot.crawl_forward(10)
#     time.sleep(3)
#     robot.turn_left(90, True)
#     time.sleep(3)
#     robot.crawl_forward(20)
#     time.sleep(3)
#     robot.turn_left(90, False)
#     time.sleep(3)
#     robot.crawl_backward(60)
#     time.sleep(3)
#     robot.turn_right(90, False)
#     time.sleep(3)
#     robot.turn_left(90, False)
#     robot.crawl_forward(20)
#     robot.set_reset_sensor_values()
#     robot.turn_left(90, True)
#     robot.turn_left(90, False)
#     robot.crawl_forward(60)

#     robot.set_reset_sensor_values()
#     robot.set_reset_sensor_values()
#     robot.set_reset_sensor_values()
#     robot.set_reset_sensor_values()
#     robot.set_reset_sensor_values()
#     robot.set_reset_sensor_values()
#     robot.set_reset_sensor_values()
#     robot.set_reset_sensor_values()
#     robot.set_reset_sensor_values()
#     robot.set_reset_sensor_values()
#     print(f"Reset sensor ok: {ok}")
    robot.set_reset_sensor_values()
#     robot.crawl_forward(100)
# =================================================
#     Get past second obs (right arrow)
#    robot.crawl_forward_until_obstacle(30)
#     robot.turn_right(90, True)
#     robot.move_til_left_obs_turn(90, True)
#     robot.crawl_forward(5)
#     robot.turn_left(90, True)
#     robot.move_til_left_obs_turn(90, True)
#     robot.crawl_forward(70)
#     robot.turn_left(90, True)
#     robot.turn_right(90, True)
# =================================================
#     Get past second obs (left arrow):
#     robot.turn_left(90, True)
#     robot.move_til_right_obs_turn(90, True)
#     robot.crawl_forward(5)
#     robot.turn_right(90, True)
#     robot.move_til_right_obs_turn(90, True)
# =================================================

#     robot.crawl_forward_until_obstacle(35)
    # robot.crawl_backward_from_obstacle(30)
#     robot.turn_right(45, True)
#     robot.crawl_forward(10)
#     robot.turn_left(90, True)
#     robot.crawl_forward(10)
#     robot.turn_right(45, True)
#     robot.crawl_forward_until_obstacle(35)
#     robot.turn_right(90, True)
#     robot.move_til_left_obs_turn(90, True)
#     robot.crawl_forward(5)
#     robot.turn_left(90, True)
#     robot.move_til_left_obs_turn(90, True)
#     robot.crawl_backward_from_obstacle(40)
#     robot.turn_right(45, True)
#     robot.crawl_forward(5)
#     robot.turn_left(45, True)
#     robot.crawl_forward(15)
#     robot.turn_left(45, True)
#     robot.turn_right(45, True)
#     robot.crawl_backward(30) # use ultrasonic sensor for more accurate measurement
#     robot.turn_right(45, True)
#     robot.crawl_forward(10)
#     robot.turn_left(45, True)
#     robot.turn_right(90, True)

#     robot.position_from_obstacle(35)
#     robot.move_forward_until_obstacle(45)
#     robot.crawl_forward_until_obstacle(30)
#     robot.crawl_backward_from_obstacle(30)

    robot.position_from_obstacle(30)

    robot.turn_right(45, True)
    robot.crawl_forward(10)
    robot.turn_left(90, True)
    robot.crawl_forward(10)
    robot.turn_right(45, True)
    robot.crawl_backward(25)

    robot.position_from_obstacle(30)

    robot.turn_left(90, True)
    robot.crawl_backward(30)
    robot.move_til_right_obs_turn(90, True)
    robot.crawl_forward(5)
    robot.turn_right(90, True)
    robot.move_til_right_obs_turn(90, True)
    length_2_obs = robot.get_last_successful_arg()
    robot.return_to_carpark(length_2_obs/2, True)


#     robot.position_from_obstacle(30)


# if __name__ == "__main__":
#    if not movement_test():
#        print("Movement test aborted due to earlier failure.")
#        raise SystemExit(1)
if __name__ == "__main__":
    robot = RobotController(ROBOT_PORT, ROBOT_BAUD)  # extra added
    movement_test()
    robot.close()  # extra added (defined a new close function in robot controller)
