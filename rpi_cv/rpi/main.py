from link.android_link import AndroidLink
from link.stm32_link import STMLink
from time import sleep
import json
import requests

from config.load_config import load_rpi_config
from message.android import AndroidMessage
from week_8 import DIR_CHAR_TO_INT


def main():
    android_link = AndroidLink()
    android_link.connect()

    # Small hello so Android sees we're alive
    android_link.send(AndroidMessage("info", "RPi connected. Send START_TASK or FASTEST_PATH."))
    recv = android_link.recv()  # Just to clear the buffer

    if not recv:
        android_link.disconnect()
        return

    message = json.loads(recv)
    mtype = message.get("type")
    mdata = message.get("data")

    if mtype in ("START_TASK", "FASTEST_PATH"):
        print(f"Starting task with data: {mdata}")

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
                "d": int(d_int),
            })

        # Defaults for robot start. Adjust if Android supplies robot pose later.
        robot_x = int((data.get("robot") or {}).get("x", 1))
        robot_y = int((data.get("robot") or {}).get("y", 1))
        # Accept either dir as int (0/2/4/6) or char (N/E/S/W)
        rdir = (data.get("robot") or {}).get("dir", 0)
        if isinstance(rdir, str):
            rdir = DIR_CHAR_TO_INT.get(rdir.upper(), 0)
        robot_dir = int(rdir)

        # Build request body for algo server
        payload = {
            "obstacles": converted,
            "retrying": False,
            "robot_x": robot_x,
            "robot_y": robot_y,
            "robot_dir": robot_dir,
        }

        # Resolve API host/port from config with sensible fallbacks
        cfg = load_rpi_config()
        api = cfg.get("api", {}) if isinstance(cfg, dict) else {}
        host = api.get("ip") or api.get("pc_ip") or "127.0.0.1"
        port = int(api.get("algo_port", 5000))
        url = f"http://{host}:{port}/path"

        # Optional timeout from config
        timeouts = (api.get("timeouts") or {})
        timeout = int(timeouts.get("algo", 5))

        try:
            android_link.send(AndroidMessage("info", f"Requesting path from algo at {host}:{port}..."))
            resp = requests.post(url, json=payload, timeout=timeout)
        except requests.exceptions.Timeout:
            android_link.send(AndroidMessage("error", "Algo API timeout."))
            print("Algo API timeout")
            android_link.disconnect()
            return
        except requests.exceptions.ConnectionError:
            android_link.send(AndroidMessage("error", "Algo API connection error."))
            print("Algo API connection error")
            android_link.disconnect()
            return
        except Exception as e:
            android_link.send(AndroidMessage("error", f"Algo API error: {e}"))
            print(f"Algo API error: {e}")
            android_link.disconnect()
            return

        if resp.status_code != 200:
            snippet = None
            try:
                snippet = resp.text[:300]
            except Exception:
                snippet = str(resp.status_code)
            android_link.send(AndroidMessage("error", "Algo API returned non-200 status."))
            print(f"Algo API non-200: {resp.status_code} body_snip={snippet!r}")
            android_link.disconnect()
            return

        try:
            payload = resp.json()
        except ValueError:
            android_link.send(AndroidMessage("error", "Algo API response not JSON."))
            print("Algo API response not JSON")
            android_link.disconnect()
            return

        data_node = payload.get("data") if isinstance(payload, dict) else None
        if not isinstance(data_node, dict):
            android_link.send(AndroidMessage("error", "Algo API response missing 'data'."))
            print("Algo API response missing 'data'")
            android_link.disconnect()
            return

        commands = data_node.get("commands", [])
        path = data_node.get("path", [])

        # Convert path dicts to [x, y] pairs for Android PATH message
        def to_pair(p):
            if isinstance(p, dict):
                x = p.get("x")
                y = p.get("y")
                if x is None or y is None:
                    return None
                return [int(x), int(y)]
            if isinstance(p, (list, tuple)) and len(p) >= 2:
                return [int(p[0]), int(p[1])]
            return None

        path_pairs = [pp for pp in (to_pair(p) for p in path) if pp is not None]
        android_link.send(AndroidMessage.path(path_pairs))
        android_link.send(AndroidMessage(
            "info", f"Received {len(commands)} commands and {len(path_pairs)} path points."))

    print(f"Received: {recv}")
    android_link.disconnect()
    sleep(30)

    # stm_link = STMLink()


if __name__ == "__main__":
    main()
