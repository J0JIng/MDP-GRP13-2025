import json
from multiprocessing import Manager, Process
import os
import queue
import time
from pathlib import Path
from config.consts import SYMBOL_MAP
from config.load_config import load_rpi_config
from helper.logger import prepare_logger
from link.android_link import AndroidLink
from link.stm32_link import STMLink
from typing import Optional
import requests

from message.android import AndroidMessage
from model.pi_action import PiAction

# Direction mapping between RPi/algo (int) and Android (char)
# Algo expects Direction values: 0(N),2(E),4(S),6(W); 8 is SKIP used for obstacles post-success
DIR_INT_TO_CHAR = {0: "N", 2: "E", 4: "S", 6: "W"}
DIR_CHAR_TO_INT = {"N": 0, "E": 2, "S": 4, "W": 6}


class RaspberryPi:
    """
    Class that represents the Raspberry Pi.
    """

    def __init__(self):
        """
        Initializes the Raspberry Pi.
        """
        self.logger = prepare_logger()
        self.config = load_rpi_config()
        self.android_link = AndroidLink()
        self.stm_link = STMLink()

        self.manager = Manager()

        self.android_dropped = self.manager.Event()
        self.unpause = self.manager.Event()

        self.movement_lock = self.manager.Lock()
        self.retrylock = self.manager.Lock()

        self.android_queue = self.manager.Queue()  # Messages to send to Android
        # Messages that need to be processed by RPi
        self.rpi_action_queue = self.manager.Queue()
        # Messages that need to be processed by STM32, as well as snap commands
        self.command_queue = self.manager.Queue()
        # X,Y,D coordinates of the robot after execution of a command
        self.path_queue = self.manager.Queue()
        # Ack results from STM32 motor execution
        self.stm_ack_queue = self.manager.Queue()
        self.stm_link.set_ack_queue(self.stm_ack_queue)

        self.proc_recv_android = None
        self.proc_recv_stm32 = None
        self.proc_android_sender = None
        self.proc_command_follower = None
        self.proc_rpi_action = None
        self.expecting_rs_ack = self.manager.Value('b', False)
        self.success_obstacles = self.manager.list()
        self.failed_obstacles = self.manager.list()
        self.obstacles = self.manager.dict()
        self.current_location = self.manager.dict()
        self.failed_attempt = False
        self.adjustment_pending = self.manager.Value('i', 0)
        self.adjustment_event = self.manager.Event()
        self.adjustment_last_result = self.manager.Value('b', True)

    def start(self):
        """Starts the RPi orchestrator"""
        try:
            ### Start up initialization ###

            self.android_link.connect()
            self.android_queue.put(AndroidMessage(
                'info', 'You are connected to the RPi!'))
            self.stm_link.connect()
            self.check_api()

            # Define child processes
            self.proc_recv_android = Process(target=self.recv_android)
            self.proc_recv_stm32 = Process(target=self.recv_stm)
            self.proc_android_sender = Process(target=self.android_sender)
            self.proc_command_follower = Process(target=self.command_follower)
            self.proc_rpi_action = Process(target=self.rpi_action)

            # Start child processes
            self.proc_recv_android.start()
            self.proc_recv_stm32.start()
            self.proc_android_sender.start()
            self.proc_command_follower.start()
            self.proc_rpi_action.start()

            self.logger.info("Child Processes started")

            ### Start up complete ###

            # Send success message to Android
            self.android_queue.put(AndroidMessage('info', 'Robot is ready!'))
            # Inform Android of mode; Android expects a type/data schema, value semantics unchanged
            self.android_queue.put(AndroidMessage('mode', 'path'))
            self.reconnect_android()

        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        """Stops all processes on the RPi and disconnects gracefully with Android and STM32"""
        self.android_link.disconnect()
        self.stm_link.disconnect()
        self.logger.info("Program exited!")

    def reconnect_android(self):
        """Handles the reconnection to Android in the event of a lost connection."""
        self.logger.info("Reconnection handler is watching...")

        while True:
            # Wait for android connection to drop
            self.android_dropped.wait()
            self.logger.error("Android link is down!")

            # Kill child processes
            self.logger.debug("Killing android child processes")
            if self.proc_android_sender:
                self.proc_android_sender.kill()
            if self.proc_recv_android:
                self.proc_recv_android.kill()

            # Wait for the child processes to finish
            if self.proc_android_sender:
                self.proc_android_sender.join()
                assert self.proc_android_sender.is_alive() is False
            if self.proc_recv_android:
                self.proc_recv_android.join()
                assert self.proc_recv_android.is_alive() is False

            self.logger.debug("Android child processes killed")

            # Clean up old sockets
            self.android_link.disconnect()

            # Reconnect
            self.android_link.connect()

            # Recreate Android processes
            self.proc_recv_android = Process(target=self.recv_android)
            self.proc_android_sender = Process(target=self.android_sender)

            # Start previously killed processes
            self.proc_recv_android.start()
            self.proc_android_sender.start()

            self.logger.info("Android child processes restarted")
            self.android_queue.put(AndroidMessage("info", "You are reconnected!"))
            self.android_queue.put(AndroidMessage('mode', 'path'))

            self.android_dropped.clear()

    def recv_android(self) -> None:
        """
        [Child Process] Processes messages received from Android.
        Android -> RPi supported message types:
          - START_TASK: { type, data: { task, robot, obstacles[] } }
          - FASTEST_PATH: { type, data: { task, robot, obstacles[] } }
          - NAVIGATION: { type, data: { commands: [..] } }
        Legacy formats are no longer accepted.
        """
        while True:
            msg_str: Optional[str] = None
            try:
                msg_str = self.android_link.recv()
            except OSError:
                self.android_dropped.set()
                self.logger.debug("Event set: Android connection dropped")

            if msg_str is None:
                continue

            try:
                message: dict = json.loads(msg_str)
            except Exception:
                self.logger.warning("Invalid JSON from Android: %r", msg_str)
                continue

            mtype = message.get("type")
            mdata = message.get("data")

            if not mtype:
                self.logger.warning("Android message missing 'type': %r", message)
                continue

            # START_TASK / FASTEST_PATH: normalize to algo payload and queue
            if mtype in ("START_TASK", "FASTEST_PATH"):
                try:
                    data = mdata or {}
                    raw_obstacles = data.get("obstacles", [])

                    # Convert Android obstacle shape {id: str/int, x:int, y:int, dir: 'N'|'E'|'S'|'W'}
                    converted = []
                    for o in raw_obstacles:
                        d_char = str(o.get("dir", "N")).upper()
                        d_int = DIR_CHAR_TO_INT.get(d_char, 0)
                        converted.append({
                            "id": int(o.get("id")),
                            "x": int(o.get("x")),
                            "y": int(o.get("y")),
                            "d": d_int,
                        })

                    # Map task/type to mode expected by algo: 0=exploration, 1=fastest path
                    task = str(data.get("task", "")).upper()
                    mode = "1" if (mtype == "FASTEST_PATH" or task == "FASTEST_PATH") else "0"

                    value = {"obstacles": converted, "mode": mode}

                    # Queue action for algo request
                    self.rpi_action_queue.put(PiAction(cat="obstacles", value=value))
                    self.logger.debug("Enqueued obstacles/mode from %s: %s", mtype, value)

                    # Optionally set current robot location if provided
                    robot = data.get("robot")
                    if robot:
                        try:
                            self.current_location['x'] = int(robot.get("x"))
                            self.current_location['y'] = int(robot.get("y"))
                            self.current_location['d'] = DIR_CHAR_TO_INT.get(str(robot.get("dir", "N")).upper(), 0)
                        except Exception:
                            pass
                except Exception as e:
                    self.logger.error("Failed to handle %s: %s", mtype, e)

            # Manual navigation commands
            elif mtype == "NAVIGATION":
                try:
                    commands = (mdata or {}).get("commands", [])
                    if isinstance(commands, list):
                        self.clear_queues()
                        for c in commands:
                            # Normalize Android command prefixes to STM32 format
                            c = str(c)
                            if len(c) >= 2:
                                pfx, rest = c[:2], c[2:]
                                map_pfx = {"SF": "FS", "SB": "BS", "LF": "TL", "RF": "TR"}
                                c = map_pfx.get(pfx, pfx) + rest
                            self.command_queue.put(c)
                        self.android_queue.put(AndroidMessage('info', 'Manual commands enqueued.'))
                    else:
                        self.logger.warning("NAVIGATION commands not a list: %r", commands)
                except Exception as e:
                    self.logger.error("Failed to handle NAVIGATION: %s", e)

            else:
                self.logger.warning("Unknown Android message type: %s", mtype)

    def recv_stm(self) -> None:
        """
        [Child Process] Receive acknowledgement messages from STM32, and release the movement lock
        """
        while True:

            try:
                success = self.stm_ack_queue.get(timeout=0.5)
                message: Optional[str] = "ACK" if success else "NACK"
            except queue.Empty:
                message = self.stm_link.recv()

            if message is None:
                continue

            if message.startswith("ACK"):
                if self.expecting_rs_ack.value:
                    self.expecting_rs_ack.value = False
                    self.logger.debug("ACK for RS00 from STM32 received.")
                    continue
                if self.adjustment_pending.value > 0:
                    self.adjustment_pending.value -= 1
                    self.adjustment_last_result.value = True
                    self.adjustment_event.set()
                    self.logger.debug("ACK for adjustment command received; retaining movement lock.")
                    continue
                try:
                    self.movement_lock.release()
                    try:
                        self.retrylock.release()
                    except:
                        pass
                    self.logger.debug(
                        "ACK from STM32 received, movement lock released.")

                    cur_location = self.path_queue.get_nowait()

                    self.current_location['x'] = cur_location['x']
                    self.current_location['y'] = cur_location['y']
                    self.current_location['d'] = cur_location['d']
                    self.logger.info(
                        f"self.current_location = {self.current_location}")
                    # Android expects type "COORDINATES" with nested robot object
                    # and direction as a cardinal string (N/E/S/W)
                    dir_char = DIR_INT_TO_CHAR.get(int(cur_location['d']), 'N')
                    self.android_queue.put(
                        AndroidMessage.coordinates(
                            x=int(cur_location['x']),
                            y=int(cur_location['y']),
                            dir=dir_char,
                        )
                    )

                except queue.Empty:
                    self.logger.debug("Path queue empty when processing ACK; skipping coordinate update.")
                except Exception:
                    self.logger.warning("Tried to release a released lock!")
            elif message.startswith("NACK"):
                if self.adjustment_pending.value > 0:
                    self.adjustment_pending.value -= 1
                    self.adjustment_last_result.value = False
                    self.adjustment_event.set()
                    self.logger.warning("Adjustment command returned NACK from STM32.")
                    continue
                self.logger.warning("Received NACK from STM32; releasing movement lock to avoid deadlock.")
                try:
                    self.movement_lock.release()
                except Exception:
                    self.logger.debug("Movement lock already released after NACK.")
                if hasattr(self, "retrylock"):
                    try:
                        self.retrylock.release()
                    except Exception:
                        pass
                self.android_queue.put(AndroidMessage('error', 'Robot reported failure executing a movement command.'))
            else:
                self.logger.warning(
                    f"Ignored unknown message from STM: {message}")

    def android_sender(self) -> None:
        """
        [Child process] Responsible for retrieving messages from android_queue and sending them over the Android link.
        """
        while True:
            # Retrieve from queue
            try:
                message: AndroidMessage = self.android_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            try:
                self.android_link.send(message)
                self.logger.debug(f"Sent to Android: {message.jsonify}")
            except OSError:
                self.android_dropped.set()
                self.logger.debug("Event set: Android dropped")

    def command_follower(self) -> None:
        """
        [Child Process]
        """
        while True:
            # Retrieve next movement command
            command: str = self.command_queue.get()
            self.logger.debug("wait for unpause")
            # Wait for unpause event to be true [Main Trigger]
            try:
                self.logger.debug("wait for retrylock")
                self.retrylock.acquire()
                self.retrylock.release()
            except:
                self.logger.debug("wait for unpause")
                self.unpause.wait()
            self.logger.debug("wait for movelock")
            # Acquire lock first (needed for both moving, and snapping pictures)
            self.movement_lock.acquire()

            # STM32 Commands - Send straight to STM32
            stm32_prefixes = ("FS", "BS", "FW", "BW", "FL", "FR", "BL",
                              "BR", "TL", "TR", "A", "C", "DT", "STOP", "ZZ", "RS", "OB")
            if command.startswith(stm32_prefixes):
                if command.startswith("RS"):
                    self.expecting_rs_ack.value = True
                else:
                    self.expecting_rs_ack.value = False
                self.stm_link.send(command)
                self.logger.debug(f"Sending to STM32: {command}")

            # Snap command
            elif command.startswith("SNAP"):
                obstacle_id_with_signal = command.replace("SNAP", "")

                self.rpi_action_queue.put(
                    PiAction(cat="snap", value=obstacle_id_with_signal))

            # End of path
            elif command == "FIN":
                self.logger.info(
                    f"At FIN, self.failed_obstacles: {self.failed_obstacles}")
                self.logger.info(
                    f"At FIN, self.current_location: {self.current_location}")
                if len(self.failed_obstacles) != 0 and self.failed_attempt == False:

                    new_obstacle_list = list(self.failed_obstacles)
                    for i in list(self.success_obstacles):
                        # {'x': 5, 'y': 11, 'id': 1, 'd': 4}
                        i['d'] = 8
                        new_obstacle_list.append(i)

                    self.logger.info("Attempting to go to failed obstacles")
                    self.failed_attempt = True
                    self.request_algo(
                        {'obstacles': new_obstacle_list, 'mode': '0'},
                        self.current_location['x'],
                        self.current_location['y'],
                        self.current_location['d'],
                        retrying=True)
                    self.retrylock = self.manager.Lock()
                    self.movement_lock.release()
                    continue

                self.unpause.clear()
                self.movement_lock.release()
                self.logger.info("Commands queue finished.")
                self.android_queue.put(AndroidMessage("info", "Commands queue finished."))
                self.android_queue.put(AndroidMessage("status", "finished"))
                self.rpi_action_queue.put(PiAction(cat="stitch", value=""))
            else:
                raise Exception(f"Unknown command: {command}")

    def rpi_action(self):
        """
        [Child Process]
        """
        while True:
            action: PiAction = self.rpi_action_queue.get()
            self.logger.debug(
                f"PiAction retrieved from queue: {action.cat} {action.value}")

            if action.cat == "obstacles":
                for obs in action.value['obstacles']:
                    self.obstacles[obs['id']] = obs
                # Use current robot pose if available
                try:
                    rx = int(self.current_location.get('x', 1))
                    ry = int(self.current_location.get('y', 1))
                    rd = int(self.current_location.get('d', 0))
                except Exception:
                    rx, ry, rd = 1, 1, 0
                self.request_algo(action.value, robot_x=rx, robot_y=ry, robot_dir=rd)
            elif action.cat == "snap":
                self.snap_and_rec(obstacle_id_with_signal=action.value)
            elif action.cat == "stitch":
                self.request_stitch()

    def snap_and_rec(self, obstacle_id_with_signal: str) -> None:
        """
        RPi snaps an image and calls the API for image-rec.
        The response is then forwarded back to the android
        :param obstacle_id_with_signal: the current obstacle ID followed by underscore followed by signal
        """
        API_IP = self.config['api']['pc_ip']
        IMAGE_API_PORT = self.config['api']['image_port']

        obstacle_id, signal = obstacle_id_with_signal.split("_")
        self.logger.info(f"Capturing image for obstacle id: {obstacle_id}")
        self.android_queue.put(AndroidMessage(
            "info", f"Capturing image for obstacle id: {obstacle_id}"))
        url = f"http://{API_IP}:{IMAGE_API_PORT}/image"
        filename = f"{int(time.time())}_{obstacle_id}_{signal}.jpg"
        images_dir = Path(__file__).resolve().parent / "images"
        try:
            images_dir.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            self.logger.warning("Unable to create images directory %s: %s", images_dir, e)
        filepath = images_dir / filename

        retry_count = 0
        current_shutter_us = 20000
        max_shutter_us = 120000
        min_shutter_us = 8000
        increase_factor = 1.6
        decrease_factor = 0.6
        adjustment_done = False

        def _perform_adjustment(command: str, description: str) -> bool:
            self.logger.info(description)
            try:
                self.adjustment_event.clear()
            except Exception:
                pass
            self.adjustment_pending.value += 1
            self.adjustment_last_result.value = False
            try:
                self.stm_link.send(command)
            except Exception as exc:
                self.logger.error("Failed to send %s to STM32: %s", command, exc)
                if self.adjustment_pending.value > 0:
                    self.adjustment_pending.value -= 1
                self.adjustment_event.set()
                return False

            signalled = self.adjustment_event.wait(timeout=5)
            if not signalled:
                self.logger.warning("Timed out waiting for STM32 to execute %s", command)
                return False

            return bool(self.adjustment_last_result.value)

        while True:

            retry_count += 1

            rpi_str_parts = [
                "raspistill",
                "-e", "jpg",
                "-n",
                "-t", "500",
                "-vf", "-hf",
                "-q", "100",
                "-sh", "40",
                "-ISO", "100",
                "-awb", "auto",
                "-ss", str(current_shutter_us),
                "-br", "50",
                "-co", "10",
                "-sa", "10",
                "-o", str(filepath),
            ]
            rpistr = " ".join(rpi_str_parts)

            self.logger.debug(
                "Attempt %d capturing with shutter %dus", retry_count, current_shutter_us
            )
            os.system(rpistr)

            self.logger.debug("Requesting from image API")

            with open(filepath, 'rb') as image_file:
                response = requests.post(
                    url, files={"file": (filename, image_file)})

            if response.status_code != 200:
                self.logger.error(
                    "Something went wrong when requesting path from image-rec API. Please try again.")
                return

            results = json.loads(response.content)

            # Higher brightness retry

            if results['image_id'] not in ('NA', '10') or retry_count > 6:   # success or max retries reached
                break
            if results['image_id'] in ('NA', '10') and not adjustment_done:  # failure on first attempt
                if _perform_adjustment("BS10", "Image rec failed on first attempt; moving backward 10cm."):
                    self.logger.debug("Backward adjustment completed successfully.")
                else:
                    self.logger.warning("Backward adjustment may have failed; continuing attempts regardless.")
                adjustment_done = True
            elif retry_count > 3:
                self.logger.info(f"Image recognition results: {results}")
                self.logger.info("Recapturing with lower shutter speed...")
                current_shutter_us = max(
                    int(current_shutter_us * decrease_factor),
                    min_shutter_us,
                )
                self.logger.debug(
                    "Lowering shutter for next attempt to %dus", current_shutter_us
                )
            elif retry_count <= 3:
                self.logger.info(f"Image recognition results: {results}")
                self.logger.info("Recapturing with higher shutter speed...")
                current_shutter_us = min(
                    int(current_shutter_us * increase_factor),
                    max_shutter_us,
                )
                self.logger.debug(
                    "Increasing shutter for next attempt to %dus", current_shutter_us
                )

        if adjustment_done:
            if _perform_adjustment("FS10", "Returning to original position; moving forward 10cm."):
                self.logger.debug("Forward adjustment completed successfully.")
            else:
                self.logger.warning("Forward adjustment may have failed; robot position may be offset.")

        # release lock so that bot can continue moving
        self.movement_lock.release()
        try:
            self.retrylock.release()
        except:
            pass

        self.logger.info(f"results: {results}")
        self.logger.info(f"self.obstacles: {self.obstacles}")
        self.logger.info(
            f"Image recognition results: {results} ({SYMBOL_MAP.get(results['image_id'])})")

        if results['image_id'] in ('NA', '10'):
            self.failed_obstacles.append(
                self.obstacles[int(results['obstacle_id'])])
            self.logger.info(
                f"Added Obstacle {results['obstacle_id']} to failed obstacles.")
            self.logger.info(f"self.failed_obstacles: {self.failed_obstacles}")
        else:
            self.success_obstacles.append(
                self.obstacles[int(results['obstacle_id'])])
            self.logger.info(
                f"self.success_obstacles: {self.success_obstacles}")
        # Send image results using Android message factory (consistent schema)
        self.android_queue.put(
            AndroidMessage.image_results(
                obs_id=str(results.get("obstacle_id")),
                img_id=str(results.get("image_id")),
            )
        )

    def request_algo(self, data, robot_x=1, robot_y=1, robot_dir=0, retrying=False):
        """
        Requests for a series of commands and the path from the Algo API.
        The received commands and path are then queued in the respective queues
        """
        API_IP = self.config['api']['pc_ip']
        ALGO_API_PORT = self.config['api']['algo_port']
        timeouts = (self.config.get('api', {}) or {}).get('timeouts', {})
        algo_timeout = int(timeouts.get('algo', 3))

        self.logger.info("Requesting path from algo...")
        self.android_queue.put(AndroidMessage(
            "info", "Requesting path from algo..."))
        self.logger.info(f"data: {data}")
        # Keep mode/big_turn for forward compatibility, though current server ignores them
        body = {
            **data,
            "big_turn": 0,
            "robot_x": int(robot_x),
            "robot_y": int(robot_y),
            "robot_dir": int(robot_dir),
            "retrying": bool(retrying),
        }
        url = f"http://{API_IP}:{ALGO_API_PORT}/path"
        # Two-attempt retry for transient errors
        response = None
        for attempt in range(1, 3):
            try:
                response = requests.post(url, json=body, timeout=algo_timeout)
                break
            except requests.exceptions.Timeout:
                self.logger.warning("Algo API timeout (attempt %d/2) to %s", attempt, url)
                if attempt == 2:
                    self.android_queue.put(AndroidMessage(
                        "error", "Algo API timeout while requesting path."))
                    return
            except requests.exceptions.ConnectionError:
                self.logger.warning("Algo API connection error (attempt %d/2) to %s", attempt, url)
                if attempt == 2:
                    self.android_queue.put(AndroidMessage(
                        "error", "Algo API connection error while requesting path."))
                    return
            except Exception as e:
                self.logger.warning("Algo API unexpected error (attempt %d/2): %s", attempt, e)
                if attempt == 2:
                    self.android_queue.put(AndroidMessage(
                        "error", "Unexpected error contacting Algo API."))
                    self.logger.exception("Unexpected error POSTing to Algo API: %s", e)
                    return

        # Error encountered at the server, return early
        if response is None:
            self.android_queue.put(AndroidMessage(
                "error", "Algo API request did not complete."))
            self.logger.error("Algo API request did not complete (no response object).")
            return
        if response.status_code != 200:
            # Try to include brief details for troubleshooting
            err_snip = None
            try:
                err_snip = response.text[:300]
            except Exception:
                pass
            self.android_queue.put(AndroidMessage(
                "error", "Algo API returned a non-200 status for path request."))
            self.logger.error(
                "Algo API path request failed: status=%s body_snip=%r", response.status_code, err_snip)
            return

        # Parse response
        try:
            payload = response.json()
        except ValueError:
            self.android_queue.put(AndroidMessage(
                "error", "Algo API response was not valid JSON."))
            self.logger.error("Algo API response not JSON: %r",
                              response.text[:300] if hasattr(response, 'text') else None)
            return

        data_node = payload.get('data') if isinstance(payload, dict) else None
        if not isinstance(data_node, dict):
            self.android_queue.put(AndroidMessage(
                "error", "Algo API response missing 'data' object."))
            self.logger.error("Algo API response missing 'data': %r", payload)
            return

        commands = data_node.get('commands', [])
        path = data_node.get('path', [])

        if not isinstance(commands, list) or not isinstance(path, list):
            self.android_queue.put(AndroidMessage(
                "error", "Algo API response had invalid 'commands' or 'path' types."))
            self.logger.error("Algo API invalid shapes: commands=%r path=%r", type(commands), type(path))
            return

        # Log commands received
        self.logger.debug(f"Commands received from API: {commands}")

        # Put commands and paths into respective queues
        self.clear_queues()
        for c in commands:
            try:
                self.command_queue.put(str(c))
            except Exception:
                self.logger.warning("Skipped invalid command value: %r", c)
        for p in path[1:]:  # ignore first element as it is the starting position of the robot
            try:
                if isinstance(p, dict) and 'x' in p and 'y' in p and 'd' in p:
                    self.path_queue.put({
                        'x': int(p['x']),
                        'y': int(p['y']),
                        'd': int(p['d']),
                    })
                elif isinstance(p, (list, tuple)):
                    # Accept [x,y] or [x,y,d]
                    x = int(p[0])
                    y = int(p[1])
                    d = int(p[2]) if len(p) > 2 and p[2] is not None else 0
                    self.path_queue.put({'x': x, 'y': y, 'd': d})
                else:
                    self.logger.warning("Skipped invalid path entry: %r", p)
            except Exception:
                self.logger.warning("Failed to normalize path entry: %r", p)

        # Also notify Android with the full path in the format it expects
        # Android expects a list of [x, y] pairs, not dicts or direction info
        try:
            def _to_pair(p):
                # Support either dicts {x,y,(d)} or already-formed [x,y]
                if isinstance(p, dict):
                    if 'x' in p and 'y' in p and p['x'] is not None and p['y'] is not None:
                        return [int(p['x']), int(p['y'])]
                    return None
                if isinstance(p, (list, tuple)) and len(p) >= 2 and p[0] is not None and p[1] is not None:
                    return [int(p[0]), int(p[1])]
                # Fallback: skip unknown shapes
                return None

            path_pairs = [pair for pair in (_to_pair(p) for p in path) if pair is not None]
            self.android_queue.put(AndroidMessage.path(path_pairs))
        except Exception:
            pass

        self.android_queue.put(AndroidMessage(
            "info", "Commands and path received Algo API. Robot is ready to move."))
        self.logger.info(
            "Commands and path received Algo API. Robot is ready to move.")

        # Auto-start movement if we have commands
        try:
            if not self.command_queue.empty():
                self.logger.info("Gyro reset! (auto-start)")
                self.expecting_rs_ack.value = True
                self.stm_link.send("RS00")
                self.unpause.set()
                self.android_queue.put(AndroidMessage('status', 'running'))
        except Exception:
            pass

    def request_stitch(self):
        """Sends a stitch request to the image recognition API to stitch the different images together"""
        API_IP = self.config['api']['pc_ip']
        IMAGE_API_PORT = self.config['api']['image_port']
        timeouts = (self.config.get('api', {}) or {}).get('timeouts', {})
        image_timeout = int(timeouts.get('image', 3))

        url = f"http://{API_IP}:{IMAGE_API_PORT}/stitch"
        # Two-attempt retry for transient errors
        response = None
        for attempt in range(1, 3):
            try:
                response = requests.get(url, timeout=image_timeout)
                break
            except requests.exceptions.Timeout:
                self.logger.warning("Image API timeout (attempt %d/2) to %s", attempt, url)
                if attempt == 2:
                    self.android_queue.put(AndroidMessage(
                        "error", "Image API timeout while requesting stitch."))
                    return
            except requests.exceptions.ConnectionError:
                self.logger.warning("Image API connection error (attempt %d/2) to %s", attempt, url)
                if attempt == 2:
                    self.android_queue.put(AndroidMessage(
                        "error", "Image API connection error while requesting stitch."))
                    return
            except Exception as e:
                self.logger.warning("Image API unexpected error (attempt %d/2): %s", attempt, e)
                if attempt == 2:
                    self.android_queue.put(AndroidMessage(
                        "error", "Unexpected error contacting Image API for stitch."))
                    self.logger.exception("Unexpected error GETting stitch: %s", e)
                    return

        # If error, then log, and send error to Android
        if response is None:
            self.android_queue.put(AndroidMessage(
                "error", "Image API request did not complete for stitch."))
            self.logger.error("Image API stitch request did not complete (no response object).")
            return
        if response.status_code != 200:
            # Notify android
            self.android_queue.put(AndroidMessage(
                "error", "Something went wrong when requesting stitch from the API."))
            self.logger.error(
                "Something went wrong when requesting stitch from the API.")
            return

        self.logger.info("Images stitched!")
        self.android_queue.put(AndroidMessage("info", "Images stitched!"))

    def clear_queues(self):
        """Clear both command and path queues"""
        while not self.command_queue.empty():
            self.command_queue.get()
        while not self.path_queue.empty():
            self.path_queue.get()

    def check_api(self) -> bool:
        """Check whether both image recognition and algorithm API servers are up.
        Returns:
            bool: True if both running, False otherwise.
        """
        API_IP = self.config['api']['pc_ip']
        IMAGE_API_PORT = self.config['api']['image_port']
        ALGO_API_PORT = self.config['api']['algo_port']

        def probe(url: str, name: str) -> bool:
            try:
                resp = requests.get(url, timeout=1)
                if resp.status_code == 200:
                    self.logger.debug(f"{name} API is up!")
                    return True
                self.logger.warning(f"{name} API unhealthy, status={resp.status_code}")
                return False
            except requests.exceptions.ConnectionError:
                self.logger.warning(f"{name} API Connection Error")
                return False
            except requests.exceptions.Timeout:
                self.logger.warning(f"{name} API Timeout")
                return False
            except Exception as e:
                self.logger.warning(f"{name} API Exception: {e}")
                return False

        image_ok = probe(f"http://{API_IP}:{IMAGE_API_PORT}/status", "Image")
        algo_ok = probe(f"http://{API_IP}:{ALGO_API_PORT}/status", "Algo")

        return image_ok and algo_ok


if __name__ == "__main__":
    rpi = RaspberryPi()
    rpi.start()
