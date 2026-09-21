#!/usr/bin/env python3
"""
"""

import json
import time

import paho.mqtt.client as mqtt

MAZE_ROWS = 13
MAZE_COLS = 13
MQTT_BROKER = "localhost" 
MQTT_PORT = 1883
POSE_TOPIC = "robot/pose"

# wall bit per side, OR'd together
WALL_N, WALL_E, WALL_S, WALL_W = 0x1, 0x2, 0x4, 0x8

WALLS = [
    [12, 6, 12, 6, 13, 4, 0, 4, 5, 6, 12, 5, 6],
    [10, 11, 10, 10, 12, 3, 8, 2, 12, 1, 1, 6, 10],
    [8, 5, 3, 8, 1, 6, 9, 2, 9, 6, 13, 2, 10],
    [10, 12, 4, 3, 12, 1, 4, 0, 6, 9, 4, 2, 10],
    [10, 10, 8, 5, 3, 13, 2, 10, 9, 6, 10, 11, 10],
    [8, 3, 9, 4, 5, 6, 8, 1, 6, 10, 9, 5, 2],
    [10, 12, 4, 1, 6, 10, 9, 6, 10, 8, 5, 5, 2],
    [8, 1, 2, 12, 3, 10, 12, 3, 9, 2, 12, 6, 10],
    [9, 6, 10, 10, 12, 1, 2, 12, 5, 1, 0, 1, 3],
    [14, 8, 1, 1, 3, 12, 1, 3, 12, 4, 2, 12, 6],
    [8, 0, 4, 7, 12, 3, 12, 6, 10, 9, 1, 2, 10],
    [10, 10, 9, 6, 10, 12, 2, 8, 1, 7, 12, 0, 2],
    [9, 1, 5, 1, 1, 3, 8, 1, 5, 5, 3, 9, 3],
]
# Maze map -- the same data as WALLS above, drawn. row 0 = SOUTH (bottom),
# col 0 = WEST (left). The gaps in the top and bottom edges are EXIT_CELLS.
#
#            0  1  2  3  4  5  6  7  8  9 10 11 12   <- col
#          +--+--+--+--+--+--+  +--+--+--+--+--+--+
#   row 12 |                 |              |     |
#          +  +  +--+  +  +  +  +  +--+--+  +  +  +
#   row 11 |  |  |     |  |     |        |        |
#          +  +  +  +--+  +--+  +  +  +--+--+  +  +
#   row 10 |           |     |     |  |        |  |
#          +  +  +--+--+--+  +--+--+  +  +  +  +  +
#   row  9 |  |           |        |        |     |
#          +--+  +  +  +  +--+  +  +--+--+  +--+--+
#   row  8 |     |  |  |        |                 |
#          +  +--+  +  +--+  +  +--+--+  +  +  +  +
#   row  7 |        |     |  |     |     |     |  |
#          +  +  +  +--+  +  +--+  +  +  +--+--+  +
#   row  6 |  |           |  |     |  |           |
#          +  +--+--+  +--+  +  +--+  +  +--+--+  +
#   row  5 |     |           |        |  |        |
#          +  +  +  +--+--+--+  +  +--+  +  +--+  +
#   row  4 |  |  |        |     |  |     |  |  |  |
#          +  +  +  +--+  +--+  +  +  +--+  +  +  +
#   row  3 |  |        |              |        |  |
#          +  +--+--+  +--+  +--+  +--+  +--+  +  +
#   row  2 |        |        |     |     |     |  |
#          +  +--+  +  +  +--+  +  +  +--+--+  +  +
#   row  1 |  |  |  |  |     |     |           |  |
#          +  +  +  +  +--+  +  +  +--+  +  +--+  +
#   row  0 |     |     |                 |        |
#          +--+--+--+--+--+--+  +--+--+--+--+--+--+
#            0  1  2  3  4  5  6  7  8  9 10 11 12   <- col

# the 2 known exits: (row, col, facing)
EXIT_CELLS = [
    (0, 6, 'south'),
    (MAZE_ROWS - 1, 6, 'north'),
]

BOT_CMD_TOPIC = "bot/cmd"         
PELLETS_TOPIC = "pellets/pose"    
CMD_VEL_TOPIC = "robot/cmd_vel"  

# yaw -> dr, dc, wall bit. 0=EAST, 90=NORTH, 180=WEST, 270=SOUTH
HEADING_DELTA = {
    0.0:   (0, 1, WALL_E),
    90.0:  (1, 0, WALL_N),
    180.0: (0, -1, WALL_W),
    270.0: (-1, 0, WALL_S),
}


# ============================================================================
# YOUR ALGORITHM GOES HERE. Everything above and below is plumbing.
# ============================================================================
import heapq

# yaw the bot must face to leave through an exit
_EXIT_YAW = {'east': 0.0, 'north': 90.0, 'west': 180.0, 'south': 270.0}

# (desired_yaw - current_yaw) mod 360  ->  relative command
# yaw grows counter-clockwise (EAST=0 -> NORTH=90), so +90 is a LEFT turn.
_TURN_TO_CMD = {0: "FRONT", 90: "LEFT", 180: "BACK", 270: "RIGHT"}
_TURN_COST = {"FRONT": 0, "LEFT": 1, "RIGHT": 1, "BACK": 2}


def _in_bounds(r, c):
    return 0 <= r < MAZE_ROWS and 0 <= c < MAZE_COLS


def _opposite(bit):
    """N<->S, E<->W wall bit."""
    return ((bit << 2) | (bit >> 2)) & 0xF


def _neighbors(cell):
    """Reachable neighbour cells (wall must be open on BOTH sides, so a
    one-sided wall in the data is still respected)."""
    r, c = cell
    for _yaw, (dr, dc, bit) in HEADING_DELTA.items():
        nr, nc = r + dr, c + dc
        if not _in_bounds(nr, nc):
            continue
        if WALLS[r][c] & bit:
            continue
        if WALLS[nr][nc] & _opposite(bit):
            continue
        yield (nr, nc)


def _heuristic(a, b):
    """Manhattan distance -- admissible on a 4-connected grid."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _astar(start, goal):
    """A* on the maze grid. Returns [start, ..., goal] or None."""
    if start == goal:
        return [start]
    open_heap = [(_heuristic(start, goal), 0, start)]   # (f, g, cell)
    came_from = {}
    best_g = {start: 0}
    while open_heap:
        _f, g, cur = heapq.heappop(open_heap)
        if cur == goal:
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            path.reverse()
            return path
        if g > best_g.get(cur, float("inf")):
            continue                                    # stale heap entry
        for nb in _neighbors(cur):
            ng = g + 1
            if ng < best_g.get(nb, float("inf")):
                best_g[nb] = ng
                came_from[nb] = cur
                heapq.heappush(open_heap, (ng + _heuristic(nb, goal), ng, nb))
    return None


def _yaw_to_neighbor(cell, nxt):
    """Absolute yaw needed to step from cell to the adjacent cell nxt."""
    dr, dc = nxt[0] - cell[0], nxt[1] - cell[1]
    for yaw, (ddr, ddc, _bit) in HEADING_DELTA.items():
        if (dr, dc) == (ddr, ddc):
            return yaw
    return None


def _relative_cmd(current_yaw, desired_yaw):
    turn = int(round((desired_yaw - current_yaw) % 360.0)) % 360
    return _TURN_TO_CMD[turn]


def choose_command(pacbot_cell, pacbot_yaw, pellets_remaining):
    """FRONT/LEFT/RIGHT/BACK to send now, or None. pacbot_cell=(row,col),
    pacbot_yaw one of HEADING_DELTA's keys, pellets_remaining=set of
    (row,col). Strategy: repeatedly A* to the nearest remaining pellet,
    and once none are left, A* to the nearest exit and leave through it.
    Stateless: it is re-run after every pose ack, so it works whether a
    LEFT/RIGHT/BACK command turns-and-moves or only turns."""
    cell = (int(pacbot_cell[0]), int(pacbot_cell[1]))
    if not _in_bounds(*cell):
        return None                                     # already outside

    # snap yaw to the nearest multiple of 90 (guards against float noise)
    yaw = (round(float(pacbot_yaw) / 90.0) * 90.0) % 360.0

    # the pellet under the bot is collected on arrival
    targets = [p for p in pellets_remaining if tuple(p) != cell]

    desired_yaw = None

    if targets:
        # ---- phase 1: nearest pellet by real path length -----------------
        best_path, best_key = None, None
        for p in targets:
            path = _astar(cell, tuple(p))
            if path is None or len(path) < 2:
                continue
            first_cmd = _relative_cmd(yaw, _yaw_to_neighbor(cell, path[1]))
            key = (len(path), _TURN_COST[first_cmd])    # tie-break: fewer turns
            if best_key is None or key < best_key:
                best_key, best_path = key, path
        if best_path is not None:
            desired_yaw = _yaw_to_neighbor(cell, best_path[1])

    if desired_yaw is None and not targets:
        # ---- phase 2: head for the closest exit and walk out -------------
        best_exit, best_path, best_key = None, None, None
        for (er, ec, facing) in EXIT_CELLS:
            path = _astar(cell, (er, ec))
            if path is None:
                continue
            key = len(path)
            if best_key is None or key < best_key:
                best_key, best_exit, best_path = key, (er, ec, facing), path
        if best_exit is not None:
            if len(best_path) == 1:                     # standing on the exit cell
                desired_yaw = _EXIT_YAW[best_exit[2]]   # step out through the gap
            else:
                desired_yaw = _yaw_to_neighbor(cell, best_path[1])

    if desired_yaw is None:
        return None
    return _relative_cmd(yaw, desired_yaw)
# ============================================================================


def parse_pellets(payload):
    return {tuple(cell) for cell in json.loads(payload)}


def main():
    state = {
        "running": False,
        "pellets": set(),
        "cell": (MAZE_ROWS // 2, MAZE_COLS // 2),
        "yaw": 0.0,
        "in_flight": False,
        "got_pose": False,
        "got_pellets": False,
    }

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="Controller")

    def decide_and_send():
        if (not state["running"] or state["in_flight"]
                or not state["got_pose"] or not state["got_pellets"]):
            return
        print(f"[debug] pose={state['cell']} yaw={state['yaw']} pellets={state['pellets']}")
        cmd = choose_command(state["cell"], state["yaw"], set(state["pellets"]))
        if cmd is not None:
            state["in_flight"] = True
            client.publish(CMD_VEL_TOPIC, cmd)
            print(f"[controller] {state['cell']} yaw={state['yaw']} -> {cmd}, "
                  f"pellets_left={len(state['pellets'])}")

    def on_message(client, userdata, msg):
        try:
            if msg.topic == BOT_CMD_TOPIC:
                running = msg.payload.decode().startswith("1")
                was_running = state["running"]
                state["running"] = running
                if running and not was_running:
                    decide_and_send()   # kick off the reactive loop on Start
            elif msg.topic == PELLETS_TOPIC:
                state["pellets"] = parse_pellets(msg.payload.decode())
                state["got_pellets"] = True
                decide_and_send()
            elif msg.topic == POSE_TOPIC:
                data = json.loads(msg.payload.decode())
                state["cell"] = (int(data["col"]), int(data["row"]))   # wire is swapped
                state["got_pose"] = True
                state["yaw"] = float(data.get("yaw", 0.0))
                state["in_flight"] = False   # this pose is the ack for our last command
                decide_and_send()   # every pose/command-ack triggers the next step
        except Exception as e:
            print("[controller] mqtt parse error:", e)

    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.subscribe([(BOT_CMD_TOPIC, 0), (PELLETS_TOPIC, 0), (POSE_TOPIC, 0)])
    client.loop_start()

    print(f"[controller] ready; sending one '{CMD_VEL_TOPIC}' command at a time, "
          f"reacting to '{POSE_TOPIC}'/'{PELLETS_TOPIC}' feedback")

    try:
        while True:
            time.sleep(0.2)  
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()


