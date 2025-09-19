"""Manual mode orchestrator.

This script connects to Android over Bluetooth and uses RobotController to
execute manual movement commands received from Android.

Expected Android message format (JSON):
	{"type": "NAVIGATION", "data": {"commands": ["SF050", "LF090", "STOP"]}}

Supported command tokens (case-insensitive):
  Dynamic distance / angle tokens:
	SF<ddd>  standard forward <dist> cm   (e.g. SF050)
	SB<ddd>  standard backward <dist> cm  (e.g. SB050)
	F<dist>  forward <dist> cm            (alias, e.g. F50 or F050)
	B<dist>  backward <dist> cm
	CF<dist> crawl forward <dist> cm
	CB<dist> crawl backward <dist> cm
	LF<aaa>  turn left <angle> deg, forward direction  (e.g. LF090)
	RF<aaa>  turn right <angle> deg, forward direction (e.g. RF090)
	L<angle> turn left <angle> deg (assumes forward)
	R<angle> turn right <angle> deg (assumes forward)
	LB<angle>/RB<angle> turn using backward direction (optional)
	HALT / STOP immediate halt

On connect this script sends mode 'manual' and an info banner.
"""

import json
import os
import time
from typing import Optional, Callable, Tuple

from helper.logger import prepare_logger
from link.android_link import AndroidLink
from message.android import AndroidMessage

from stm.robot_controller import RobotController

# try:
# 	from stm.robot_controller import RobotController  # type: ignore
# except Exception as _imp_err:  # pragma: no cover
# 	RobotController = None  # type: ignore
# 	_ROBOT_IMPORT_ERROR = _imp_err
# else:  # pragma: no cover
# 	_ROBOT_IMPORT_ERROR = None

ROBOT_PORT = "/dev/ttyACM0"  # TODO: Check this before running
ROBOT_BAUD = 115200


class ManualController:
    def __init__(self):
        self.logger = prepare_logger()
        self.android = AndroidLink()
        self._running = True
        self._started = False  # gate to ignore FW/BW/etc until start if desired
        self.robot: Optional[RobotController] = None  # type: ignore
        self._cmd_handlers: dict[str, Callable[[], bool]] = {}

    def _init_robot(self):
        # if RobotController is None:
        #     raise RuntimeError(f"RobotController unavailable: {_ROBOT_IMPORT_ERROR}")
        port = ROBOT_PORT
        baud = ROBOT_BAUD
        self.logger.info("Initialising RobotController (port=%s baud=%s)", port, baud)
        self.robot = RobotController(port, baud)  # type: ignore
        self._cmd_handlers = {}

    def _dispatch_dynamic(self, token: str) -> Tuple[bool, bool]:
        """Parse and execute a dynamic command token.

        Returns (recognized, success):
          recognized: pattern matches a supported command syntax
          success: robot method returned truthy (command executed)

        Supported dynamic formats (case-insensitive):
          SF<ddd> / SB<ddd>   standard speed forward/back cm
          F<dist> / B<dist>   forward/back cm (alias)
          CF<dist> / CB<dist> crawl forward/back cm
          LF<aaa> / RF<aaa>   turn left/right angle deg using forward direction
          LB<aaa> / RB<aaa>   turn left/right angle deg using backward direction
          L<angle> / R<angle> simple turn (assumes forward direction)
          HALT / STOP         immediate halt
        """
        if self.robot is None:
            return False, False
        r = self.robot
        up = token.upper()

        # direct halts
        if up in {"HALT", "STOP"}:
            return True, bool(r.halt())

        # Standard speed prefixed movement: SFxxx / SBxxx
        if up.startswith('S') and len(up) >= 3 and up[1] in {'F', 'B'} and up[2:].isdigit():
            dist = int(up[2:])
            ok = r.move_forward(dist) if up[1] == 'F' else r.move_backward(dist)
            return True, bool(ok)

        # Turn with explicit direction char: LF090 / RF090 / LB090 / RB090
        if up[0] in {'L', 'R'} and len(up) >= 3 and up[1] in {'F', 'B'} and up[2:].isdigit():
            ang = int(up[2:])
            fwd = (up[1] == 'F')
            if up[0] == 'L':
                return True, bool(r.turn_left(ang, fwd))
            return True, bool(r.turn_right(ang, fwd))

        # Remaining variants
        try:
            if up.startswith("CF") and up[2:].isdigit():
                return True, bool(r.crawl_forward(int(up[2:])))
            if up.startswith("CB") and up[2:].isdigit():
                return True, bool(r.crawl_backward(int(up[2:])))
            if up.startswith("F") and up[1:].isdigit():
                return True, bool(r.move_forward(int(up[1:])))
            if up.startswith("B") and up[1:].isdigit():
                return True, bool(r.move_backward(int(up[1:])))
            if up.startswith("L") and up[1:].isdigit():
                return True, bool(r.turn_left(int(up[1:]), True))
            if up.startswith("R") and up[1:].isdigit():
                return True, bool(r.turn_right(int(up[1:]), True))
        except ValueError:
            return False, False
        return False, False

    def start(self):
        self.logger.info("Connecting Android link for manual mode")
        self.android.connect()
        try:
            self._init_robot()
        except Exception as e:
            self.logger.error("Cannot initialise robot controller: %s", e)
            self.android.send(AndroidMessage('error', 'Robot controller init failed; manual mode unusable.'))
            return
        self.android.send(AndroidMessage('info', 'Manual mode connected.'))
        self.android.send(AndroidMessage('mode', 'manual'))
        self.android.send(AndroidMessage('status', 'idle'))
        self._recv_loop()

    def _handle_navigation(self, commands):
        if not isinstance(commands, list):
            self.logger.warning("NAVIGATION commands not list: %r", commands)
            return
        if self.robot is None:
            self.logger.error("Robot not initialised; ignoring commands.")
            return
        executed = 0
        failed = 0
        unknown = 0
        for c in commands:
            raw = str(c).strip()
            up = raw.upper()
            recognized, success = self._dispatch_dynamic(raw)
            if recognized and success:
                if not self._started and up not in {"STOP", "HALT"}:
                    self._started = True
                    self.android.send(AndroidMessage('status', 'running'))
                self.logger.debug("Executed %s", raw)
                executed += 1
            elif recognized and not success:
                self.logger.warning("Command recognized but robot reported failure: %s", raw)
                failed += 1
            else:
                self.logger.warning("Unrecognized command token: %s", raw)
                unknown += 1
        if executed:
            self.android.send(AndroidMessage(
                'info', f'Manual: {executed} executed, {failed} failed, {unknown} unknown.'))
        else:
            self.android.send(AndroidMessage('info', f'Manual: none executed (failed={failed}, unknown={unknown}).'))

    def _recv_loop(self):
        self.logger.info("Entering manual receive loop")
        while self._running:
            try:
                raw = self.android.recv()
            except OSError:
                self.logger.error("Android link dropped; shutting down manual controller")
                break
            if raw is None:
                continue
            try:
                msg = json.loads(raw)
            except Exception:
                self.logger.warning("Invalid JSON from Android: %r", raw)
                continue
            mtype = msg.get('type')
            data = msg.get('data')
            if mtype == 'NAVIGATION':
                commands = (data or {}).get('commands') if isinstance(data, dict) else None
                self._handle_navigation(commands)
            else:
                self.logger.debug("Ignoring unsupported message type: %s", mtype)

    def stop(self):
        self._running = False
        if self.robot is not None:
            try:
                self.robot.halt()
            except Exception:
                pass
        self.android.send(AndroidMessage('status', 'finished'))
        self.android.disconnect()
        self.logger.info("Manual controller stopped")


def main():
    mc = ManualController()
    try:
        mc.start()
    except KeyboardInterrupt:
        mc.stop()
    except Exception:
        mc.stop()


if __name__ == '__main__':
    main()
