"""Manual mode orchestrator.

This script connects to Android over Bluetooth and uses RobotController to
execute manual movement commands received from Android.

Expected Android message format (JSON):
	{"type": "NAVIGATION", "data": {"commands": ["FW--", "TL--", "STOP"]}}

Supported commands in the list:
	FW--  forward (continuous / until obstacle)
	BW--  backward (continuous / until obstacle)
	TL--  turn left 90°
	TR--  turn right 90°
	STOP  halt immediately

On connect this script sends mode 'manual' and an info banner.
"""

import json
import os
import time
from typing import Optional, Callable

from helper.logger import prepare_logger
from link.android_link import AndroidLink
from message.android import AndroidMessage

try:
	from stm.robot_controller import RobotController  # type: ignore
except Exception as _imp_err:  # pragma: no cover
	RobotController = None  # type: ignore
	_ROBOT_IMPORT_ERROR = _imp_err
else:  # pragma: no cover
	_ROBOT_IMPORT_ERROR = None

VALID_MANUAL = {"FW--", "BW--", "TL--", "TR--", "STOP"}
ROBOT_PORT = "COM4"
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
		if RobotController is None:
			raise RuntimeError(f"RobotController unavailable: {_ROBOT_IMPORT_ERROR}")
		port = ROBOT_PORT
		baud = ROBOT_BAUD
		self.logger.info("Initialising RobotController (port=%s baud=%s)", port, baud)
		self.robot = RobotController(port, baud)  # type: ignore
		rob = self.robot
		self._cmd_handlers = {
			"FW--": (lambda r=rob: r.move_forward(999)),  # type: ignore[attr-defined]
			"BW--": (lambda r=rob: r.move_backward(999)),  # type: ignore[attr-defined]
			"TL--": (lambda r=rob: r.turn_left(90, True)),  # type: ignore[attr-defined]
			"TR--": (lambda r=rob: r.turn_right(90, True)),  # type: ignore[attr-defined]
			"STOP": (lambda r=rob: r.halt()),  # type: ignore[attr-defined]
		}

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
		sent_any = False
		for c in commands:
			c = str(c).strip().upper()
			if c not in VALID_MANUAL:
				self.logger.warning("Ignored invalid manual command: %s", c)
				continue
			if not self._started and c != 'STOP':
				self._started = True
				self.android.send(AndroidMessage('status', 'running'))
			fn = self._cmd_handlers.get(c)
			if fn is None:
				self.logger.warning("No handler for command: %s", c)
				continue
			try:
				ack = fn()
				self.logger.debug("Executed %s -> %s", c, ack)
			except Exception as e:
				self.logger.error("Error executing %s: %s", c, e)
				continue
			sent_any = True
		if sent_any:
			self.android.send(AndroidMessage('info', 'Manual commands executed.'))

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
