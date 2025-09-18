"""Manual mode orchestrator.

This script connects to Android over Bluetooth and uses RobotController to
execute manual movement commands received from Android.

Expected Android message format (JSON):
	{"type": "NAVIGATION", "data": {"commands": ["SF050", "LF090", "STOP"]}}

Supported command tokens (case-insensitive):
  Legacy fixed tokens:
	FW--  forward (continuous / until obstacle)
	BW--  backward (continuous / until obstacle)
	TL--  turn left 90° (forward direction)
	TR--  turn right 90° (forward direction)
	STOP  halt immediately

  Dynamic distance / angle tokens (Android current buttons use these):
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
	LB<angle>/RB<angle> (optional) turn using backward direction if provided
	HALT / STOP immediate halt

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

LEGACY_TOKENS = {"FW--", "BW--", "TL--", "TR--", "STOP"}
ROBOT_PORT = "/dev/ttyACM0" # TODO: Check this before running
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
		# legacy static handlers for backwards compatibility
		self._cmd_handlers = {
			"FW--": (lambda r=rob: r.move_forward(999)),  # type: ignore[attr-defined]
			"BW--": (lambda r=rob: r.move_backward(999)),  # type: ignore[attr-defined]
			"TL--": (lambda r=rob: r.turn_left(90, True)),  # type: ignore[attr-defined]
			"TR--": (lambda r=rob: r.turn_right(90, True)),  # type: ignore[attr-defined]
			"STOP": (lambda r=rob: r.halt()),  # type: ignore[attr-defined]
		}

	def _dispatch_dynamic(self, token: str) -> bool:
		"""Parse and execute a dynamic command token.

		Supported dynamic formats (case-insensitive):
		  F<dist>    forward dist cm (0-999, F999 = until obstacle)
		  B<dist>    backward dist cm
		  CF<dist>   crawl forward dist cm
		  CB<dist>   crawl backward dist cm
		  L<angle>   turn left <angle> deg (dir=True)
		  R<angle>   turn right <angle> deg (dir=True)
		  HALT / STOP immediate halt

		Angle 90 is the typical left/right; other values allowed (0-359).
		Returns True if dispatched, False if not recognised.
		"""
		if self.robot is None:
			return False
		r = self.robot
		up = token.upper()
		# direct halts
		if up in {"HALT", "STOP"}:
			return bool(r.halt())
		# Standard speed prefixed movement: SFxxx / SBxxx
		if up.startswith('S') and len(up) >= 3 and up[1] in {'F','B'} and up[2:].isdigit():
			dist = int(up[2:])
			if up[1] == 'F':
				return bool(r.move_forward(dist))
			else:
				return bool(r.move_backward(dist))
		# Turn with explicit direction char: LF090 / RF090 / LB090 / RB090
		if up[0] in {'L','R'} and len(up) >= 3 and up[1] in {'F','B'} and up[2:].isdigit():
			ang = int(up[2:])
			fwd = (up[1] == 'F')
			if up[0] == 'L':
				return bool(r.turn_left(ang, fwd))
			else:
				return bool(r.turn_right(ang, fwd))
		# crawl variants
		try:
			if up.startswith("CF"):
				dist = int(up[2:])
				return bool(r.crawl_forward(dist))
			if up.startswith("CB"):
				dist = int(up[2:])
				return bool(r.crawl_backward(dist))
			# linear forward/back
			if up.startswith("F") and up[1:].isdigit():
				dist = int(up[1:])
				return bool(r.move_forward(dist))
			if up.startswith("B") and up[1:].isdigit():
				dist = int(up[1:])
				return bool(r.move_backward(dist))
			# turns
			if up.startswith("L") and up[1:].isdigit():
				ang = int(up[1:])
				return bool(r.turn_left(ang, True))
			if up.startswith("R") and up[1:].isdigit():
				ang = int(up[1:])
				return bool(r.turn_right(ang, True))
		except ValueError:
			return False
		return False

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
			raw = str(c).strip()
			up = raw.upper()
			# Backwards compatibility first
			if up in LEGACY_TOKENS:
				if not self._started and up != 'STOP':
					self._started = True
					self.android.send(AndroidMessage('status', 'running'))
				fn = self._cmd_handlers.get(up)
				if fn is None:
					self.logger.warning("No legacy handler for command: %s", up)
					continue
				try:
					ack = fn()
					self.logger.debug("Executed legacy %s -> %s", up, ack)
				except Exception as e:
					self.logger.error("Error executing %s: %s", up, e)
					continue
				sent_any = True
				continue
			# Dynamic parsing
			dispatched = self._dispatch_dynamic(raw)
			if dispatched:
				if not self._started and up not in {"STOP", "HALT"}:
					self._started = True
					self.android.send(AndroidMessage('status', 'running'))
				self.logger.debug("Executed dynamic %s", raw)
				sent_any = True
			else:
				self.logger.warning("Ignored unknown command token: %s", raw)
		if sent_any:
			self.android.send(AndroidMessage('info', 'Manual commands executed.'))
		else:
			self.android.send(AndroidMessage('info', 'No manual commands executed.'))

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
