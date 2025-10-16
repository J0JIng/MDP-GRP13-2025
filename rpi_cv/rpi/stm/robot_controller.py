from enum import Enum
from typing import Deque, Optional, Callable
import asyncio
import logging
import math
import statistics
import time
from time import sleep

import warnings
from collections import deque

import pigpio
from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero.exc import DistanceSensorNoEcho

from stm.serial_cmd_base_ll import SerialCmdBaseLL
import RPi.GPIO as GPIO
from helper.logger import prepare_logger


logger = prepare_logger(__name__, level=logging.DEBUG)

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


def angle_acb_deg(height: float, base: float) -> float:
    """Angle at C given legs AB (opposite) and BC (adjacent)."""
    if height <= 0 or base < 0:
        raise ValueError("Lengths must be non-negative and AB>0")
    return math.degrees(math.atan2(height, base))


class PinState(Enum):
    HIGH = 1
    LOW = 0
    Z = -1


class UltrasonicSensor:
    """Ultrasonic distance helper backed by gpiozero + pigpio with smoothing."""

    WINDOW_SIZE = 7
    STEP_CM = 15.0
    STEP_RATE = 0.25
    PENDING_TOLERANCE_CM = 5.0
    PERIOD_S = 0.10
    GLITCH_FILTER_US = 100

    def __init__(
        self,
        trigger_pin: int,
        echo_pin: int,
        max_distance_m: float = 4.0,
        pigpio_host: Optional[str] = None,
        pigpio_port: int = 8888,
    ) -> None:
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.max_distance_m = max_distance_m
        self.max_distance_cm = max_distance_m * 100.0

        self.pi = (
            pigpio.pi()
            if pigpio_host is None
            else pigpio.pi(pigpio_host, pigpio_port)
        )
        if not self.pi.connected:
            raise RuntimeError(
                "pigpio daemon not running. Start it with: sudo systemctl enable --now pigpiod"
            )

        factory_kwargs = {}
        if pigpio_host is not None:
            factory_kwargs["host"] = pigpio_host
        if pigpio_port != 8888:
            factory_kwargs["port"] = pigpio_port

        factory = PiGPIOFactory(**factory_kwargs)

        try:
            self.sensor = DistanceSensor(
                echo=self.echo_pin,
                trigger=self.trigger_pin,
                max_distance=self.max_distance_m,
                pin_factory=factory,
                queue_len=1
            )
        except Exception:
            self.pi.stop()
            raise

        self.pi.set_pull_up_down(self.echo_pin, pigpio.PUD_DOWN)
        self.pi.set_glitch_filter(self.echo_pin, self.GLITCH_FILTER_US)

        self._window: Deque[float] = deque(maxlen=self.WINDOW_SIZE)
        self._last_ok: Optional[float] = None
        self._pending: Optional[float] = None
        self._last_read_ts: float = 0.0

    def _read_raw_distance_cm(self) -> Optional[float]:
        try:
            logger.debug("UltrasonicSensor._read_raw_distance_cm: reading sensor now")

            distance_ratio = self.sensor.distance

            logger.debug("UltrasonicSensor._read_raw_distance_cm: sensor.distance returned %s", distance_ratio)
            if distance_ratio is None:
                logger.debug("UltrasonicSensor._read_raw_distance_cm: sensor.distance returned None")
                return None

            try:
                raw_cm = float(distance_ratio) * 100.0
            except (TypeError, ValueError) as exc:
                logger.debug(
                    "UltrasonicSensor._read_raw_distance_cm: invalid distance value %r (%s)",
                    distance_ratio,
                    exc,
                )
                return None

            logger.debug("UltrasonicSensor._read_raw_distance_cm: read raw_cm=%.2f", raw_cm)
        except DistanceSensorNoEcho:
            logger.debug("UltrasonicSensor._read_raw_distance_cm: no echo")
            return None
        except Exception as e:
            logger.debug("UltrasonicSensor._read_raw_distance_cm: error reading sensor: %s", e)
            return None

        logger.debug("UltrasonicSensor._read_raw_distance_cm: raw_cm=%.2f", raw_cm)

        if not math.isfinite(raw_cm):
            return None

        return max(0.0, min(raw_cm, self.max_distance_cm))

    def _read_smoothed_distance_cm(self, force_read: bool = False) -> Optional[float]:
        now = time.monotonic()
        if force_read:
            logger.debug("UltrasonicSensor._read_smoothed_distance_cm: force_read=True, calling _read_raw_distance_cm")
            measurement_cm = self._read_raw_distance_cm()
            logger.debug("UltrasonicSensor._read_raw_distance_cm: %s", measurement_cm)
            if measurement_cm is None:
                return None
            self._last_read_ts = now
            return measurement_cm

        if (
            self._last_ok is not None
            and (now - self._last_read_ts) < self.PERIOD_S
        ):
            return min(self._last_ok, self.max_distance_cm)

        raw_cm = self._read_raw_distance_cm()
        if raw_cm is None:
            return None

        self._window.append(raw_cm)
        if not self._window:
            return None

        med = statistics.median(self._window)
        if self._last_ok is None:
            self._last_ok = med
            self._pending = None
        else:
            thresh = max(self.STEP_CM, self.STEP_RATE * self._last_ok)
            if abs(med - self._last_ok) > thresh:
                if self._pending is None or abs(med - self._pending) > self.PENDING_TOLERANCE_CM:
                    self._pending = med
                else:
                    self._last_ok = self._pending
                    self._pending = None
            else:
                # Weighted smoothing keeps gradual changes responsive without oscillation.
                self._last_ok = 0.7 * self._last_ok + 0.3 * med
                self._pending = None

        if self._last_ok is None:
            return None

        self._last_ok = max(0.0, min(self._last_ok, self.max_distance_cm))
        self._last_read_ts = now
        return self._last_ok

    def read_distance(self) -> Optional[float]:
        """Return the current distance measurement in cm, forcing a fresh sensor read."""
        measurement_cm = self._read_raw_distance_cm()
        if measurement_cm is None:
            return None
        return measurement_cm

    def read_distance_smoothed(self, force_read: bool = True) -> Optional[float]:
        """Return the smoothed distance measurement in cm; optionally forces a new sample."""
        measurement_cm = self._read_smoothed_distance_cm(force_read=force_read)
        logger.debug("UltrasonicSensor.read_distance_smoothed: %s", measurement_cm)
        if measurement_cm is None:
            return None
        return measurement_cm

    @property
    def distance(self) -> Optional[float]:
        """
        Returns the distance in cm or None if no valid reading is available.
        """
        samples = []
        for idx in range(5):
            measurement = self.read_distance()
            logger.debug("UltrasonicSensor.distance sample #%d: %s", idx + 1, measurement)
            if measurement is not None and math.isfinite(measurement):
                samples.append(round(measurement))
            if idx < 4:
                time.sleep(0.1)
        if len(samples) < 3:
            logger.debug("UltrasonicSensor.distance insufficient samples: %s", samples)
            return None
        samples.sort()
        median = float(samples[len(samples) // 2])
        logger.debug("UltrasonicSensor.distance median=%.2f from samples=%s", median, samples)
        return median

    def close(self) -> None:
        try:
            self.sensor.close()
        except Exception:
            pass

        if self.pi is not None and self.pi.connected:
            try:
                self.pi.set_glitch_filter(self.echo_pin, 0)
            except Exception:
                pass
            self.pi.stop()


class RobotController:
    PIN_US_TRIG: int = 23
    PIN_US_ECHO: int = 24
    MOVE_COMPLETION_TIMEOUT_S: float = 60.0
    MOVE_START_TIMEOUT_S: float = 5.0
    MOVE_STOP_STABLE_WINDOW_S: float = 0.2
    MOVE_POLL_INTERVAL_S: float = 0.05
    MOVE_INITIAL_DELAY_S: float = 0.05
    CMD_RETRY_BACKOFF_BASE_S: float = 0.1
    CRAWL_CHUNK_SIZE_CM: int = 20
    CRAWL_CHUNK_DELAY_S: float = 0.2

    def __init__(self, port: str, baudrate: int, _inst_obstr_cb: Optional[Callable[..., None]] = None):
        self.drv = SerialCmdBaseLL(port, baudrate)

        GPIO.setmode(GPIO.BCM)
        self._gpio_initialized = True
        trig_pin = getattr(self, "PIN_US_TRIG", 23)
        echo_pin = getattr(self, "PIN_US_ECHO", 24)
        self.distance_sensor = UltrasonicSensor(trigger_pin=trig_pin, echo_pin=echo_pin, max_distance_m=4.0)
        self.base = []
        self.base.append(10)  # first obstacle
        self.base.append(10)  # buffer between back of robot and first obstacle
        self.base.append(7)
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

        for attempt in range(attempts):
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
                self._sleep_cmd_retry(attempt, attempts)
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

            self._sleep_cmd_retry(attempt, attempts)

        return False

    def move_forward(self, dist: int, no_brakes: bool = False, retry: bool = True) -> bool:
        '''
        Command robot to move FORWARD backward by [dist] cm.
        0 <= dist <= 999
        999 is interpreted as "move FORWARD until obstacle detected".
        returns True if command was acknowledged, False otherwise.
        '''
        self.set_reset_sensor_values()
        self.validate_dist(dist)

        attempts = 3 if retry else 1
        for attempt in range(attempts):
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
                logger.debug(
                    "move_forward: ack received dist=%s no_brakes=%s attempt=%d",
                    dist,
                    no_brakes,
                    attempt + 1,
                )
                return self._wait_for_motion_complete()

            self._sleep_cmd_retry(attempt, attempts)
            logger.debug(
                "move_forward retry %d/%d dist=%s no_brakes=%s",
                attempt + 1,
                attempts,
                dist,
                no_brakes,
            )

        return False

    def move_backward(self, dist: int, no_brakes: bool = False, retry: bool = True) -> bool:
        '''
        Command robot to move BACKWARD by [dist] cm.
        0 <= dist <= 999
        999 is interpreted as "move BACKWARD until obstacle detected".
        returns True if command was acknowledged, False otherwise.
        '''
        self.set_reset_sensor_values()
        self.validate_dist(dist)

        attempts = 3 if retry else 1
        for attempt in range(attempts):
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

            self._sleep_cmd_retry(attempt, attempts)

        return False

    def crawl_forward(self, dist: int, retry: bool = True, chunk_large_moves: bool = False) -> bool:
        '''
        Command robot to move FORWARD by [dist] cm. IN A SLOW MANNER. 
        0 <= dist <= 999
        999 is interpreted as "move FORWARD until obstacle detected".
        returns True if command was acknowledged, False otherwise.

        chunk_large_moves: when True and dist > 20cm, the movement is broken
        into segments of up to 20cm with a small pause in between.
        '''
        self.set_reset_sensor_values()
        self.validate_dist(dist)
        if (
            chunk_large_moves
            and dist != 999
            and dist > self.CRAWL_CHUNK_SIZE_CM
        ):
            logger.info(
                "crawl_forward: chunking dist=%s with chunk_size=%s",
                dist,
                self.CRAWL_CHUNK_SIZE_CM,
            )
            return self._execute_chunked_crawl(dist, self.drv.MotorCmd.FWD_CHAR, retry)

        logger.debug("crawl_forward: sending single crawl dist=%s", dist)
        return self._send_crawl_distance(dist, self.drv.MotorCmd.FWD_CHAR, retry)

    def crawl_forward_until_obstacle(self, dist=30, retry: bool = True) -> bool:
        '''
        Command robot to crawl FORWARD until an obstacle is detected.
        returns True if command was acknowledged and obstacle detected, False otherwise.
        '''

        attempts = 3 if retry else 1

        for attempt in range(attempts):
            self.drv.construct_cmd()
            self.drv.add_cmd_byte(True)
            self.drv.add_module_byte(self.drv.Modules.MOTOR)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.FWD_CHAR)
            self.drv.add_args_bytes(999)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.CRAWL_CHAR)
            self.drv.pad_to_end()

            ack = self.drv.ll_is_valid(self.drv.send_cmd())
            if not ack:
                self._sleep_cmd_retry(attempt, attempts)
                continue

            time.sleep(self.MOVE_INITIAL_DELAY_S)

            try:
                detection = self.poll_obstruction(dist)
            except Exception:
                detection = None

            halted = self.halt(retry=False)
            if not halted:
                return False

            if detection:
                return True

            self._sleep_cmd_retry(attempt, attempts)

        return False

    def crawl_backward(self, dist: int, retry: bool = True, chunk_large_moves: bool = False) -> bool:
        '''
        Command robot to move BACKWARD by [dist] cm. IN A SLOW MANNER. 
        0 <= dist <= 999
        999 is interpreted as "move BACKWARD until obstacle detected".
        returns True if command was acknowledged, False otherwise.

        chunk_large_moves: when True and dist > 20cm, the movement is broken
        into segments of up to 20cm with a small pause in between.
        '''
        self.set_reset_sensor_values()
        self.validate_dist(dist)
        if (
            chunk_large_moves
            and dist != 999
            and dist > self.CRAWL_CHUNK_SIZE_CM
        ):
            logger.info(
                "crawl_backward: chunking dist=%s with chunk_size=%s",
                dist,
                self.CRAWL_CHUNK_SIZE_CM,
            )
            return self._execute_chunked_crawl(dist, self.drv.MotorCmd.BWD_CHAR, retry)

        logger.debug("crawl_backward: sending single crawl dist=%s", dist)
        return self._send_crawl_distance(dist, self.drv.MotorCmd.BWD_CHAR, retry)

    def crawl_backward_from_obstacle(self, dist: int = 30, retry: bool = True) -> bool:
        '''
        Crawl backward until the robot is [dist] cm away from the obstacle in front.
        Does not move if the current distance already exceeds [dist].
        '''

        self.validate_dist(dist)

        initial_distance = self.poll_obstruction(read_once=True)
        if initial_distance is None:
            return False

        if initial_distance >= dist:
            return True

        attempts = 3 if retry else 1

        for attempt in range(attempts):
            self.drv.construct_cmd()
            self.drv.add_cmd_byte(True)
            self.drv.add_module_byte(self.drv.Modules.MOTOR)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.BWD_CHAR)
            self.drv.add_args_bytes(999)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.CRAWL_CHAR)
            self.drv.pad_to_end()

            ack = self.drv.ll_is_valid(self.drv.send_cmd())
            if not ack:
                self._sleep_cmd_retry(attempt, attempts)
                continue

            time.sleep(self.MOVE_INITIAL_DELAY_S)

            try:
                detection = self._poll_until_distance_at_least(dist)
            except Exception:
                detection = None

            halted = self.halt(retry=False)
            if not halted:
                return False

            if detection:
                return True

            if detection is None:
                return False

            self._sleep_cmd_retry(attempt, attempts)

        return False

    def position_from_obstacle(self, dist: int, retry: bool = True, first: bool = False) -> bool:
        '''
        Adjust the robot so that its front is approximately [dist] cm from the obstacle.
        Moves forward if it is too far, or backward if it is too close.
        '''
        self.set_reset_sensor_values()
        logger.info("position_from_obstacle: target distance=%s cm", dist)
        self.validate_dist(dist)

        current_distance = 0

        if first:
            while current_distance is None or current_distance <= 50:
                current_distance = self.poll_obstruction(read_once=True)
                sleep(0.5)
        else:
            current_distance = self.poll_obstruction(read_once=True)

        self.base.append(current_distance)
        logger.debug("position_from_obstacle: initial distance reading=%s", current_distance)
        if current_distance is None:
            logger.warning("position_from_obstacle: ultrasonic reading unavailable")
            return False

        delta_cm = float(current_distance) - float(dist)
        logger.debug(
            "position_from_obstacle: delta_cm=%.2f (target=%s)",
            delta_cm,
            dist,
        )
        if abs(delta_cm) < 0.5:
            logger.debug("position_from_obstacle: already within tolerance")
            return True

        move_cm = int(round(abs(delta_cm)))
        if move_cm <= 0:
            logger.debug("position_from_obstacle: rounded movement zero, nothing to do")
            return True

        if delta_cm > 0:
            logger.info(
                "position_from_obstacle: moving forward by %s cm to reach target",
                move_cm,
            )
            # return self.crawl_forward(move_cm, retry=retry)
            return self.move_forward(move_cm, retry=retry)

        logger.info(
            "position_from_obstacle: moving backward by %s cm to reach target",
            move_cm,
        )
        # return self.crawl_backward(move_cm, retry=retry)
        return self.move_backward(move_cm, retry=retry)

    def _send_crawl_distance(self, dist: int, motor_cmd: SerialCmdBaseLL.MotorCmd, retry: bool) -> bool:
        attempts = 3 if retry else 1

        for attempt in range(attempts):
            logger.debug(
                "_send_crawl_distance attempt %d/%d motor_cmd=%s dist=%s",
                attempt + 1,
                attempts,
                getattr(motor_cmd, "name", motor_cmd),
                dist,
            )
            self.drv.construct_cmd()
            self.drv.add_cmd_byte(True)
            self.drv.add_module_byte(self.drv.Modules.MOTOR)
            self.drv.add_motor_cmd_byte(motor_cmd)
            self.drv.add_args_bytes(dist)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.CRAWL_CHAR)
            self.drv.pad_to_end()
            ack = self.drv.ll_is_valid(self.drv.send_cmd())
            if ack:
                logger.debug(
                    "_send_crawl_distance ack received motor_cmd=%s dist=%s",
                    getattr(motor_cmd, "name", motor_cmd),
                    dist,
                )
                return self._wait_for_motion_complete()

            self._sleep_cmd_retry(attempt, attempts)
            logger.debug(
                "_send_crawl_distance retry scheduled motor_cmd=%s dist=%s",
                getattr(motor_cmd, "name", motor_cmd),
                dist,
            )

        logger.warning(
            "_send_crawl_distance exhausted retries motor_cmd=%s dist=%s",
            getattr(motor_cmd, "name", motor_cmd),
            dist,
        )
        return False

    def _execute_chunked_crawl(self, dist: int, motor_cmd: SerialCmdBaseLL.MotorCmd, retry: bool) -> bool:
        remaining = dist

        while remaining > 0:
            segment = min(self.CRAWL_CHUNK_SIZE_CM, remaining)
            logger.debug(
                "_execute_chunked_crawl: segment=%s remaining_before=%s",
                segment,
                remaining,
            )
            if not self._send_crawl_distance(segment, motor_cmd, retry):
                logger.warning(
                    "_execute_chunked_crawl: segment failed motor_cmd=%s", getattr(motor_cmd, "name", motor_cmd)
                )
                return False

            remaining -= segment
            if remaining > 0:
                logger.debug(
                    "_execute_chunked_crawl: sleeping between segments remaining_after=%s",
                    remaining,
                )
                time.sleep(self.CRAWL_CHUNK_DELAY_S)

        logger.debug("_execute_chunked_crawl: completed all segments")
        return True

    def turn_left(self, angle: int, dir: bool, no_brakes: bool = False, retry: bool = True) -> bool:
        '''
        Command robot to turn LEFT/right by [angle] degrees and in the direction specified by [dir].
        0 <= angle <= 359
        dir = True means turn forward, dir = False means turn backward.
        returns True if command was acknowledged, False otherwise.
        '''
        self.set_reset_sensor_values()
        self.validate_angle(angle)

        attempts = 3 if retry else 1
        for attempt in range(attempts):
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

            self._sleep_cmd_retry(attempt, attempts)

        return False

    def turn_right(self, angle: int, dir: bool, no_brakes: bool = False, retry: bool = True) -> bool:
        '''
        Command robot to turn right by [angle] degrees and in the direction specified by [dir].
        0 <= angle <= 359
        dir = True means turn forward, dir = False means turn backward.
        returns True if command was acknowledged, False otherwise.
        '''
        self.set_reset_sensor_values()
        self.validate_angle(angle)

        attempts = 3 if retry else 1
        for attempt in range(attempts):
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

            self._sleep_cmd_retry(attempt, attempts)

        return False

    def move_til_left_obs_turn(self, angle: int, dir: bool, no_brakes: bool = False, retry: bool = True) -> bool:
        '''
        Command robot to turn right by [angle] degrees and in the direction specified by [dir].
        0 <= angle <= 359
        dir = True means turn forward, dir = False means turn backward.
        returns True if command was acknowledged, False otherwise.
        '''
        self.set_reset_sensor_values()
        self.validate_angle(angle)

        attempts = 3 if retry else 1
        for attempt in range(attempts):
            self.drv.construct_cmd()
            self.drv.add_cmd_byte(True)
            self.drv.add_module_byte(self.drv.Modules.MOTOR)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.MOV_TIL_OBS)
            self.drv.add_args_bytes(angle)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.LEFT_CHAR)

            if no_brakes:
                self.drv.add_motor_cmd_byte(self.drv.MotorCmd.LINEAR_CHAR)

            self.drv.pad_to_end()
            ack = self.drv.ll_is_valid(self.drv.send_cmd())
            if ack:
                return self._wait_for_motion_complete()

            self._sleep_cmd_retry(attempt, attempts)

        return False

    def move_til_right_obs_turn(self, angle: int, dir: bool, no_brakes: bool = False, retry: bool = True) -> bool:
        '''
        Command robot to turn right by [angle] degrees and in the direction specified by [dir].
        0 <= angle <= 359
        dir = True means turn forward, dir = False means turn backward.
        returns True if command was acknowledged, False otherwise.
        '''
        self.set_reset_sensor_values()
        self.validate_angle(angle)

        attempts = 3 if retry else 1
        for attempt in range(attempts):
            self.drv.construct_cmd()
            self.drv.add_cmd_byte(True)
            self.drv.add_module_byte(self.drv.Modules.MOTOR)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.MOV_TIL_OBS)
            self.drv.add_args_bytes(angle)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.RIGHT_CHAR)

            if no_brakes:
                self.drv.add_motor_cmd_byte(self.drv.MotorCmd.LINEAR_CHAR)

            self.drv.pad_to_end()
            ack = self.drv.ll_is_valid(self.drv.send_cmd())
            if ack:
                return self._wait_for_motion_complete()

            self._sleep_cmd_retry(attempt, attempts)

        return False

    def halt(self, retry: bool = True):
        '''
        Command robot to halt.
        returns True if command was acknowledged, False otherwise.
        '''

        attempts = 3 if retry else 1
        for attempt in range(attempts):
            self.drv.construct_cmd()
            self.drv.add_cmd_byte(True)
            self.drv.add_module_byte(self.drv.Modules.MOTOR)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.HALT_CHAR)
            self.drv.pad_to_end()
            ack = self.drv.ll_is_valid(self.drv.send_cmd())
            if ack:
                return self._wait_for_motion_complete()

            self._sleep_cmd_retry(attempt, attempts)

        return False

    def _sleep_cmd_retry(self, attempt: int, attempts: int) -> None:
        if attempt >= attempts - 1:
            return
        delay = self.CMD_RETRY_BACKOFF_BASE_S * (2 ** attempt)
        time.sleep(delay)

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
                logger.warning("_wait_for_motion_complete: poll_is_moving returned None")
                return False

            now = time.monotonic()
            if status != 0:
                observed_motion = True
                last_motion_time = now
            else:
                if observed_motion and (now - last_motion_time) >= self.MOVE_STOP_STABLE_WINDOW_S:
                    logger.debug("_wait_for_motion_complete: motion stopped after %.2fs", now - start_time)
                    return True
                if (not observed_motion) and (now - start_time) >= self.MOVE_START_TIMEOUT_S:
                    logger.debug("_wait_for_motion_complete: no motion detected within start timeout")
                    return True

            time.sleep(self.MOVE_POLL_INTERVAL_S)

        self.set_reset_sensor_values()
        logger.warning(
            "_wait_for_motion_complete: timeout after %.2fs", time.monotonic() - start_time
        )
        return False

    def _poll_until_distance_at_least(self, dist: float) -> Optional[bool]:
        start_time = time.monotonic()

        while (time.monotonic() - start_time) <= self.MOVE_COMPLETION_TIMEOUT_S:
            measurement = self.poll_obstruction(read_once=True)
            if measurement is None:
                time.sleep(self.MOVE_POLL_INTERVAL_S)
                continue

            if measurement >= dist:
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
            raise Exception("Invalid return value for last successful arg")
        return ret

    def poll_obstruction(self, dist_from_obstacle: float = 30.0, read_once: bool = False):
        sensor = self.distance_sensor
        logger.debug("sensor object: %s", sensor)
        logger.debug("poll_obstruction: dist_from_obstacle=%.1f read_once=%s", dist_from_obstacle, read_once)
        counter = 0
        if sensor is None:
            logger.debug("poll_obstruction: sensor is None")
            return None

        if read_once:
            try:
                measurement = sensor.read_distance_smoothed(force_read=True)
                logger.debug("poll_obstruction: single reading=%.1f cm", measurement)
            except Exception as e:
                logger.debug(f"poll_obstruction: error reading sensor: {e}")
                return None
            if measurement is None:
                logger.debug("poll_obstruction: sensor reading is None")
                return None

            logger.debug("poll_obstruction: single reading=%.1f cm", measurement)
            return float(measurement)

        dist_from_obstacle += 5.0
        last4 = deque(maxlen=4)

        try:
            while True:
                # sensor.distance is returned in metres; convert to centimetres for comparison.
                try:
                    measurement_m = sensor.distance
                except DistanceSensorNoEcho:
                    logger.debug("poll_obstruction: no echo detected, retrying")
                    sleep(0.05)
                    continue
                except Exception as exc:
                    logger.debug("poll_obstruction: error reading sensor: %s", exc)
                    sleep(0.05)
                    continue

                if measurement_m is None or not math.isfinite(measurement_m):
                    sleep(0.05)
                    continue

                distance_cm = float(measurement_m) * 100.0
                logger.debug("poll_obstruction: raw reading=%.1f cm", distance_cm)
                if distance_cm <= dist_from_obstacle:
                    logger.info(
                        "poll_obstruction: threshold reached (<= %.1f cm)", dist_from_obstacle
                    )
                    return True
                # last4.append(distance_cm)

                # # Only decide once we have 3 samples; require all three below threshold
                # if len(last4) == 4 and all(v <= dist_from_obstacle for v in last4):
                #     return True
                sleep(0.1)

        except KeyboardInterrupt:
            logger.info("poll_obstruction: interrupted by keyboard")
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

    def set_reset_sensor_values(self) -> bool:
        attempts = 3

        for attempt in range(attempts):
            self.drv.construct_cmd()
            self.drv.add_cmd_byte(True)
            self.drv.add_module_byte(self.drv.Modules.SENSOR)
            self.drv.add_sensor_byte(self.drv.SensorCmd.RST_SEN_VAL)
            self.drv.add_args_bytes(0)
            self.drv.pad_to_end()

            if self.drv.ll_is_valid(self.drv.send_cmd()):
                return True

            self._sleep_cmd_retry(attempt, attempts)

        return False

    def close(self):
        # 1) Close gpiozero devices
        sensor = getattr(self, "distance_sensor", None)
        if sensor is not None:
            try:
                sensor.close()
            except Exception:
                pass

        # 2) Close your serial driver if it supports it
        drv = getattr(self, "drv", None)
        if drv and hasattr(drv, "close"):
            try:
                drv.close()
            except Exception:
                pass

        # 3) Only if you used RPi.GPIO directly anywhere (setups, event_detect, etc.)
        if getattr(self, "_gpio_initialized", False):
            try:
                GPIO.cleanup()
            except Exception:
                pass
            finally:
                self._gpio_initialized = False

    def T2_O1(self, dir: bool):
        attempts = 3

        for attempt in range(attempts):
            self.drv.construct_cmd()
            self.drv.add_cmd_byte(True)
            self.drv.add_module_byte(self.drv.Modules.AUX)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.T2_O1_CHAR)
            self.drv.add_motor_cmd_byte(
                self.drv.MotorCmd.LEFT_CHAR) if dir else self.drv.add_motor_cmd_byte(
                self.drv.MotorCmd.RIGHT_CHAR)
            self.drv.pad_to_end()
            if self.drv.ll_is_valid(self.drv.send_cmd()):
                return True

            self._sleep_cmd_retry(attempt, attempts)

        return False

    def T2_O2(self, dir: bool):
        attempts = 3

        for attempt in range(attempts):
            self.drv.construct_cmd()
            self.drv.add_cmd_byte(True)
            self.drv.add_module_byte(self.drv.Modules.AUX)
            self.drv.add_motor_cmd_byte(self.drv.MotorCmd.T2_02_CHAR)
            self.drv.add_motor_cmd_byte(
                self.drv.MotorCmd.LEFT_CHAR) if dir else self.drv.add_motor_cmd_byte(
                self.drv.MotorCmd.RIGHT_CHAR)
            self.drv.pad_to_end()
            if self.drv.ll_is_valid(self.drv.send_cmd()):
                return True

            self._sleep_cmd_retry(attempt, attempts)

        return False

    def return_to_carpark(self, height: float, right_turn: bool):
        """
        height should be half of the length of the second obstacle
        """
        base = sum(self.base)
        height += 10  # add 10cm for robot offset from 2nd obstacle
        angle = int(angle_acb_deg(height, base))
        logger.debug(f"ANGLE: {angle}, HEIGHT: {height}, BASE: {base}, RIGHT TURN: {right_turn}")

        if right_turn:  # +- 5 degrees to compensate for drift
            self.turn_right(angle, True)
        else:
            self.turn_left(angle, True)

        hyp = math.hypot(height, base)
        # self.crawl_forward(int(hyp))
        self.move_forward(int(hyp))
        if right_turn:
            self.turn_left(angle, True)
        else:
            self.turn_right(angle, True)
        self.position_from_obstacle(18)

    def return_to_carpark_v2(self, height: float, right_turn: bool):
        """
        height should be half of the length of the second obstacle
        """
        self.move_forward(self.base[-1] + 40)

        base = self.base[-2] + 23  # add 30 becauase of initial move_forward of 30 before first obstacle
        height += 10  # add 10cm for robot offset from 2nd obstacle
        angle = int(angle_acb_deg(height, base))
        logger.debug(f"ANGLE: {angle}, HEIGHT: {height}, BASE: {base}, RIGHT TURN: {right_turn}")

        if right_turn:  # +- 5 degrees to compensate for drift
            self.turn_right(angle, True)
        else:
            self.turn_left(angle, True)

        hyp = math.hypot(height, base)
        # self.crawl_forward(int(hyp))
        self.move_forward(int(hyp))
        if right_turn:
            self.turn_left(angle, True)
        else:
            self.turn_right(angle, True)
        self.position_from_obstacle(15)
