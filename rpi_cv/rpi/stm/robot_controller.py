from enum import Enum
from typing import Optional, Callable
import asyncio
import time
from gpiozero import DistanceSensor
from time import sleep

from stm.serial_cmd_base_ll import SerialCmdBaseLL
import RPi.GPIO as GPIO

'''
Python API that abstracts away the low-level serial communication with the robot.
Users are expected to handle general exceptions, such as IOError, ValueError, etc.
Additionally, None or False is returned if the command was not attended to by the robot.

DMA buffer flushing is handled on the robot side, DO NOT spam commands at intervals less
than 20ms. It will auto recover from unresponsive state within 20ms.

Commands that are ack-ed are guaranteed to be executed (as this means that the robot has internally
delegated the task to the motor controller thread).
unless an overriding command is sent OR the robot overrides it. 
For instance, an obstacle is detected while forward cmd is given.

It is recommended to space out commands. If not, the robot will queue up to 10 commands, and execute 
them sequentially. The resulting motion is less predicable.

Requests are generally cheaper than commands, and can be assumed to return within ACK_TIMEOUT_MS.

'''


class PinState(Enum):
    HIGH = 1
    LOW = 0
    Z = -1


class RobotController:
    PIN_US_TRIG: int = 15  # TODO DEFINE
    PIN_US_ECHO: int = 14  # TODO DEFINE
    MOVE_COMPLETION_TIMEOUT_S: float = 60.0
    MOVE_START_TIMEOUT_S: float = 5.0
    MOVE_STOP_STABLE_WINDOW_S: float = 0.2
    MOVE_POLL_INTERVAL_S: float = 0.05
    MOVE_INITIAL_DELAY_S: float = 0.05

    def __init__(self, port: str, baudrate: int, _inst_obstr_cb: Optional[Callable[..., None]] = None):
        self.drv = SerialCmdBaseLL(port, baudrate)

        self.distance_sensor = DistanceSensor(echo=24, trigger=23, max_distance=4.0)
        # GPIO.setmode(GPIO.BCM)
        # self.cmd_pin_state = PinState.Z
        # self.obstr_pin_state = PinState.Z

        # GPIO.setup(self.PIN_COMMAND, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        # GPIO.setup(self.PIN_OBSTACLE, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # if GPIO.input(self.PIN_COMMAND) == GPIO.HIGH:
        #     print("[CONTROLLER] WARN: COMMAND PIN N/C OR UNEXPECTED STATE")
        # else:
        #     self.cmd_pin_state = PinState.LOW

        # if GPIO.input(self.PIN_OBSTACLE) == GPIO.HIGH:
        #     print("[CONTROLLER] WARN: OBSTACLE PIN N/C OR UNEXPECTED STATE")
        # else:
        #     self.obstr_pin_state = PinState.LOW

        # self._inst_obstr_cb = _inst_obstr_cb
        # if self._inst_obstr_cb is not None:
        #     GPIO.add_event_detect(self.PIN_OBSTACLE,
        #                           GPIO.RISING,
        #                           callback=self.sig_obst_callback,
        #                           bouncetime=50)

    def validate_dist(self, dist: int) -> None:
        '''
        Validate the distance must be within 0-999
        '''
        if dist < 0 or dist > 999:
            raise ValueError("Invalid distance, must be 0-999")

    def validate_angle(self, angle: int) -> None:
        '''
        Validate the distance must be within 0-359
        '''
        if angle < 0 or angle > 359:
            raise ValueError("Invalid angle, must be 0-359")

    def move_forward_until_obstacle(self, no_brakes: bool = False, retry: bool = True) -> bool:
        '''
        Command robot to move FORWARD until obstacle detected.
        returns True if command was acknowledged, False otherwise.
        '''
        attempts = 3 if retry else 1

        for _ in range(attempts):
            self.drv.construct_cmd()
            self.drv.add_cmd_byte(True)
            self.drv.add_module_byte(self.drv.Modules.MOTOR)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.FWD_CHAR)
            self.drv.add_args_bytes(999)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.PAD_CHAR)
            if no_brakes:
                self.drv.add_motor_cmd_byte(self.drv.MotorCmd.LINEAR_CHAR)
            self.drv.pad_to_end()

            ack = self.drv.ll_is_valid(self.drv.send_cmd())
            if not ack:
                continue

            time.sleep(self.MOVE_INITIAL_DELAY_S)

            try:
                detection = self.poll_obstruction()
            except Exception:
                detection = None

            halted = self.halt(retry=False)
            if not halted:
                return False

            if detection:
                return True

        return False

    def move_forward(self, dist: int, no_brakes: bool = False, retry: bool = True) -> bool:
        '''
        Command robot to move FORWARD backward by [dist] cm.
        0 <= dist <= 999
        999 is interpreted as "move FORWARD until obstacle detected".
        returns True if command was acknowledged, False otherwise.
        '''

        self.validate_dist(dist)

        attempts = 3 if retry else 1
        for i in range(attempts):
            self.drv.construct_cmd()
            self.drv.add_cmd_byte(True)
            self.drv.add_module_byte(self.drv.Modules.MOTOR)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.FWD_CHAR)
            self.drv.add_args_bytes(dist)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.PAD_CHAR)  # empty
            if no_brakes:
                self.drv.add_motor_cmd_byte(self.drv.MotorCmd.LINEAR_CHAR)
            self.drv.pad_to_end()

            ack = self.drv.ll_is_valid(self.drv.send_cmd())
            if ack:
                return self._wait_for_motion_complete()

        return False

    def move_backward(self, dist: int, no_brakes: bool = False, retry: bool = True) -> bool:
        '''
        Command robot to move BACKWARD by [dist] cm.
        0 <= dist <= 999
        999 is interpreted as "move BACKWARD until obstacle detected".
        returns True if command was acknowledged, False otherwise.
        '''

        self.validate_dist(dist)

        attempts = 3 if retry else 1
        for i in range(attempts):
            self.drv.construct_cmd()
            self.drv.add_cmd_byte(True)
            self.drv.add_module_byte(self.drv.Modules.MOTOR)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.BWD_CHAR)
            self.drv.add_args_bytes(dist)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.PAD_CHAR)  # empty
            if no_brakes:
                self.drv.add_motor_cmd_byte(self.drv.MotorCmd.LINEAR_CHAR)
            self.drv.pad_to_end()

            ack = self.drv.ll_is_valid(self.drv.send_cmd())
            if ack:
                return self._wait_for_motion_complete()

        return False

    def crawl_forward(self, dist: int) -> bool:
        '''
        Command robot to move FORWARD by [dist] cm. IN A SLOW MANNER. 
        0 <= dist <= 999
        999 is interpreted as "move FORWARD until obstacle detected".
        returns True if command was acknowledged, False otherwise.
        '''

        self.validate_dist(dist)
        self.drv.construct_cmd()
        self.drv.add_cmd_byte(True)
        self.drv.add_module_byte(self.drv.Modules.MOTOR)
        self.drv.add_motor_cmd_byte(self.drv.MotorCmd.FWD_CHAR)
        self.drv.add_args_bytes(dist)
        self.drv.add_motor_cmd_byte(self.drv.MotorCmd.CRAWL_CHAR)
        self.drv.pad_to_end()
        ack = self.drv.ll_is_valid(self.drv.send_cmd())
        if not ack:
            return False
        return self._wait_for_motion_complete()

    def crawl_forward_until_obstacle(self, retry: bool = True) -> bool:
        '''
        Command robot to crawl FORWARD until an obstacle is detected.
        returns True if command was acknowledged and obstacle detected, False otherwise.
        '''

        attempts = 3 if retry else 1

        for _ in range(attempts):
            self.drv.construct_cmd()
            self.drv.add_cmd_byte(True)
            self.drv.add_module_byte(self.drv.Modules.MOTOR)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.FWD_CHAR)
            self.drv.add_args_bytes(999)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.CRAWL_CHAR)
            self.drv.pad_to_end()

            ack = self.drv.ll_is_valid(self.drv.send_cmd())
            if not ack:
                continue

            time.sleep(self.MOVE_INITIAL_DELAY_S)

            try:
                detection = self.poll_obstruction()
            except Exception:
                detection = None

            halted = self.halt(retry=False)
            if not halted:
                return False

            if detection:
                return True

        return False

    def crawl_backward(self, dist: int) -> bool:
        '''
        Command robot to move BACKWARD by [dist] cm. IN A SLOW MANNER. 
        0 <= dist <= 999
        999 is interpreted as "move BACKWARD until obstacle detected".
        returns True if command was acknowledged, False otherwise.
        '''

        self.validate_dist(dist)
        self.drv.construct_cmd()
        self.drv.add_cmd_byte(True)
        self.drv.add_module_byte(self.drv.Modules.MOTOR)
        self.drv.add_motor_cmd_byte(self.drv.MotorCmd.BWD_CHAR)
        self.drv.add_args_bytes(dist)
        self.drv.add_motor_cmd_byte(self.drv.MotorCmd.CRAWL_CHAR)
        self.drv.pad_to_end()
        ack = self.drv.ll_is_valid(self.drv.send_cmd())
        if not ack:
            return False
        return self._wait_for_motion_complete()

    def turn_left(self, angle: int, dir: bool, no_brakes: bool = False, retry: bool = True) -> bool:
        '''
        Command robot to turn LEFT/right by [angle] degrees and in the direction specified by [dir].
        0 <= angle <= 359
        dir = True means turn forward, dir = False means turn backward.
        returns True if command was acknowledged, False otherwise.
        '''

        self.validate_angle(angle)

        attempts = 3 if retry else 1
        for i in range(attempts):
            self.drv.construct_cmd()
            self.drv.add_cmd_byte(True)
            self.drv.add_module_byte(self.drv.Modules.MOTOR)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.LEFT_CHAR)
            self.drv.add_args_bytes(angle)
            if dir:
                self.drv.add_motor_cmd_byte(self.drv.MotorCmd.FWD_CHAR)
            else:
                self.drv.add_motor_cmd_byte(self.drv.MotorCmd.BWD_CHAR)

            if no_brakes:
                self.drv.add_motor_cmd_byte(self.drv.MotorCmd.LINEAR_CHAR)

            self.drv.pad_to_end()
            ack = self.drv.ll_is_valid(self.drv.send_cmd())
            if ack:
                return self._wait_for_motion_complete()

        return False

    def turn_right(self, angle: int, dir: bool, no_brakes: bool = False, retry: bool = True) -> bool:
        '''
        Command robot to turn right by [angle] degrees and in the direction specified by [dir].
        0 <= angle <= 359
        dir = True means turn forward, dir = False means turn backward.
        returns True if command was acknowledged, False otherwise.
        '''

        self.validate_angle(angle)

        attempts = 3 if retry else 1
        for i in range(attempts):
            self.drv.construct_cmd()
            self.drv.add_cmd_byte(True)
            self.drv.add_module_byte(self.drv.Modules.MOTOR)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.RIGHT_CHAR)
            self.drv.add_args_bytes(angle)
            if dir:
                self.drv.add_motor_cmd_byte(self.drv.MotorCmd.FWD_CHAR)
            else:
                self.drv.add_motor_cmd_byte(self.drv.MotorCmd.BWD_CHAR)

            if no_brakes:
                self.drv.add_motor_cmd_byte(self.drv.MotorCmd.LINEAR_CHAR)

            self.drv.pad_to_end()
            ack = self.drv.ll_is_valid(self.drv.send_cmd())
            if ack:
                return self._wait_for_motion_complete()

        return False

    def halt(self, retry: bool = True):
        '''
        Command robot to halt.
        returns True if command was acknowledged, False otherwise.
        '''

        attempts = 3 if retry else 1
        for i in range(attempts):
            self.drv.construct_cmd()
            self.drv.add_cmd_byte(True)
            self.drv.add_module_byte(self.drv.Modules.MOTOR)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.HALT_CHAR)
            self.drv.pad_to_end()
            ack = self.drv.ll_is_valid(self.drv.send_cmd())
            if ack:
                return self._wait_for_motion_complete()

        return False

    def _wait_for_motion_complete(self) -> bool:
        """
        Block until the robot reports that motors have stopped moving.

        Returns True when motion completes before timeout, False otherwise.
        """

        # Give the firmware a short window to start executing before polling.
        time.sleep(self.MOVE_INITIAL_DELAY_S)

        start_time = time.monotonic()
        last_motion_time = start_time
        observed_motion = False

        while (time.monotonic() - start_time) <= self.MOVE_COMPLETION_TIMEOUT_S:
            status = self.poll_is_moving()
            if status is None:
                return False

            now = time.monotonic()
            if status != 0:
                observed_motion = True
                last_motion_time = now
            else:
                if observed_motion and (now - last_motion_time) >= self.MOVE_STOP_STABLE_WINDOW_S:
                    return True
                if (not observed_motion) and (now - start_time) >= self.MOVE_START_TIMEOUT_S:
                    return True

            time.sleep(self.MOVE_POLL_INTERVAL_S)

        return False

    def get_quaternion(self) -> Optional[list]:
        '''
        Requests the quaternion vector a + bi + cj + dk,
        which are computed from accel/gyro data.
        '''

        self.drv.construct_cmd()
        self.drv.add_cmd_byte(False)
        self.drv.add_module_byte(self.drv.Modules.SENSOR)
        self.drv.add_sensor_byte(self.drv.SensorCmd.QTRN_ALL)
        self.drv.pad_to_end()
        ret = self.drv.send_cmd()

        ret = ret.split(';')
        ret = [x.strip() for x in ret]
        if len(ret) != 4:
            return None
        try:
            ret = [float(x) for x in ret]
        except ValueError:
            return None
        return ret

    def get_gyro_Z(self) -> Optional[float]:
        '''
        Requests the gyroscope Z axis angular velocity in degrees per second.
        '''

        self.drv.construct_cmd()
        self.drv.add_cmd_byte(False)
        self.drv.add_module_byte(self.drv.Modules.SENSOR)
        self.drv.add_sensor_byte(self.drv.SensorCmd.GZ)
        self.drv.pad_to_end()
        ret = self.drv.send_cmd()
        try:
            ret = float(ret)
        except ValueError:
            return None
        return ret

    def get_yaw(self) -> Optional[float]:
        '''
        Requests the yaw angle in degrees, which is given by
        arctan(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2)).
        '''

        self.drv.construct_cmd()
        self.drv.add_cmd_byte(False)
        self.drv.add_module_byte(self.drv.Modules.SENSOR)
        self.drv.add_sensor_byte(self.drv.SensorCmd.QTRN_YAW)
        self.drv.pad_to_end()
        ret = self.drv.send_cmd()
        try:
            ret = float(ret)
        except ValueError:
            return None
        return ret

    def get_ir_L(self) -> Optional[float]:
        '''
        Requests the LEFT cm-approximation derived from the reflected IR illuminance,
        given by the formula where x is the analog signal in mV.

        Note that this measurement is not very accurate and is dependent on the surface reflectance.
        '''

        self.drv.construct_cmd()
        self.drv.add_cmd_byte(False)
        self.drv.add_module_byte(self.drv.Modules.SENSOR)
        self.drv.add_sensor_byte(self.drv.SensorCmd.IR_LEFT)
        self.drv.pad_to_end()
        ret = self.drv.send_cmd()
        try:
            ret = float(ret)
        except ValueError:
            return None
        return ret

    def get_ir_R(self) -> Optional[float]:
        '''
        Requests the RIGHT cm-approximation derived from the reflected IR illuminance,
        given by the formula where x is the analog signal in mV.

        Note that this measurement is not very accurate and is dependent on the surface reflectance.
        '''

        self.drv.construct_cmd()
        self.drv.add_cmd_byte(False)
        self.drv.add_module_byte(self.drv.Modules.SENSOR)
        self.drv.add_sensor_byte(self.drv.SensorCmd.IR_RIGHT)
        self.drv.pad_to_end()
        ret = self.drv.send_cmd()
        try:
            ret = float(ret)
        except ValueError:
            return None
        return ret

    def set_threshold_stop_distance_left(self, dist: int) -> bool:

        self.validate_dist(dist)
        self.drv.construct_cmd()
        self.drv.add_cmd_byte(True)
        self.drv.add_module_byte(self.drv.Modules.SENSOR)
        self.drv.add_sensor_byte(self.drv.SensorCmd.IR_LEFT)
        self.drv.add_args_bytes(dist)
        self.drv.pad_to_end()
        return self.drv.ll_is_valid(self.drv.send_cmd())

    def set_threshold_stop_distance_right(self, dist: int) -> bool:

        self.validate_dist(dist)
        self.drv.construct_cmd()
        self.drv.add_cmd_byte(True)
        self.drv.add_module_byte(self.drv.Modules.SENSOR)
        self.drv.add_sensor_byte(self.drv.SensorCmd.IR_RIGHT)
        self.drv.add_args_bytes(dist)
        self.drv.pad_to_end()
        return self.drv.ll_is_valid(self.drv.send_cmd())

    def set_threshold_disable_obstacle_detection_left(self) -> bool:

        self.drv.construct_cmd()
        self.drv.add_cmd_byte(True)
        self.drv.add_module_byte(self.drv.Modules.SENSOR)
        self.drv.add_sensor_byte(self.drv.SensorCmd.IR_LEFT)
        self.drv.add_args_bytes(999)
        self.drv.pad_to_end()
        return self.drv.ll_is_valid(self.drv.send_cmd())

    def set_threshold_disable_obstacle_detection_right(self) -> bool:

        self.drv.construct_cmd()
        self.drv.add_cmd_byte(True)
        self.drv.add_module_byte(self.drv.Modules.SENSOR)
        self.drv.add_sensor_byte(self.drv.SensorCmd.IR_RIGHT)
        self.drv.add_args_bytes(999)
        self.drv.pad_to_end()
        return self.drv.ll_is_valid(self.drv.send_cmd())

    async def sig_obst_callback(self, channel) -> None:
        self._inst_obstr_cb = channel
        if self._inst_obstr_cb is not None:
            _d_ret = self.get_last_successful_arg()
            self._inst_obstr_cb(_d_ret)
        await asyncio.sleep(0.01)

    def get_last_successful_arg(self):
        self.drv.construct_cmd()
        self.drv.add_cmd_byte(False)
        self.drv.add_module_byte(self.drv.Modules.AUX)
        self.drv.add_sensor_byte(self.drv.SensorCmd.LAST_HALT)
        self.drv.pad_to_end()
        ret = self.drv.send_cmd()
        try:
            ret = float(ret)
        except ValueError:
            return None
        return ret

    def poll_obstruction(self):
        sensor = getattr(self, "distance_sensor", None)
        if sensor is None:
            return None

        try:
            while True:
                # sensor.distance is in meters (float 0.0–1.0+)
                distance_cm = float(sensor.distance * 100)
                print(f"{distance_cm:.1f} cm")
                if distance_cm <= 50.0:
                    return True
                sleep(0.1)

        except KeyboardInterrupt:
            return None

    def poll_is_moving(self):
        self.drv.construct_cmd()
        self.drv.add_cmd_byte(False)
        self.drv.add_module_byte(self.drv.Modules.SENSOR)
        self.drv.add_sensor_byte(self.drv.SensorCmd.MOTOR_MOV)
        self.drv.pad_to_end()
        ret = self.drv.send_cmd()
        try:
            ret = int(ret)
        except ValueError:
            return None
        return ret
