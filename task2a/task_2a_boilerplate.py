"""PacBot Task 2A - solver boilerplate.

Fill in the parts marked TODO. Everything else - the MQTT connection, the
message formats, the maze layout and the robot's dimensions - is done for
you and matches the simulator.

The task: the robot spawns in a random cell facing a random open direction.
Drive through the pellet's cell, then leave the maze through the exit on
the -y (south) side of cell (14, 14). Score: 20 for solving + 10 for the pellet
- 5 per wall collision.

What you get over MQTT:
  pacbot/poses    once at startup (retained): the robot's spawn pose
                  -> Solver.on_poses()
  pacbot/pellets  once at startup (retained): the pellet's pose
                  -> Solver.on_pellets()
  pacbot/sensors  every physics step (10 ms): ToF distances, gyro,
                  accelerometer, wheel encoder ticks -> Solver.step()
  pacbot/result   once, when the run ends: {"solved", "collisions"}
What you send:
  pacbot/wheel_vel  wheel speeds in rad/s - the return value of Solver.step()

As given, this file connects, reads the poses and the pellet, and keeps the
robot still.

Run:
    ./task_2a_launch                 # terminal 1
    python3 task_2a_boilerplate.py   # terminal 2
"""
import json
import math

import paho.mqtt.client as mqtt

# ------------------------------------------------------------------ MQTT
MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"      # sim -> us, every physics step
TOPIC_POSES = "pacbot/poses"          # sim -> us, once, retained: robot spawn pose
TOPIC_PELLETS = "pacbot/pellets"      # sim -> us, once, retained: pellet pose
TOPIC_RESULT = "pacbot/result"        # sim -> us, once, when the run ends
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"  # us -> sim

# ------------------------------------------------------------------ maze
N = 15            # cells per side
PITCH = 0.22      # m, cell centre to cell centre
WALL_HALF = 0.005  # m, half the wall thickness
# Cells are (row, col), as the maze drawing and pacbot/poses use: row 0 at
# the top, column 0 at the left. As in Task 2B, cell (row, col) is centred
# at world (col * PITCH, (N - 1 - row) * PITCH) - see cell_centre().
EXIT = (N - 1, N - 1, 3)  # the only opening in the border: -y side of (14, 14)

# Directions: 0 = +x, 1 = +y, 2 = -x, 3 = -y (the numbering pacbot/poses uses).
# STEP[d] is the (d_row, d_col) of one cell in direction d: +x is the next
# column, +y the row above.
STEP = ((0, 1), (-1, 0), (0, -1), (1, 0))

# Copy of kLayout in maze_2a.cpp - the maze is the same on every launch.
# Drawn in the Task 2B format (see task2b/maze_2b.py): line 2 * row + 1 is
# a row of cells and the four chars from 4 * col are one cell. '---' is a
# wall on the +y / -y side of a cell (the lines above / below it), '|' a
# wall on its -x / +x side (the chars left / right of it). ' E ' in the
# bottom line marks the exit, on the -y side of (14, 14).
LAYOUT = (
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+",
    "|                           |       |               |       |",
    "+---+---+   +---+   +   +   +   +   +---+   +---+   +---+   +",
    "|           |           |   |   |       |       |           |",
    "+   +   +   +   +---+---+---+   +---+   +   +   +---+---+   +",
    "|   |           |               |   |   |   |   |   |       |",
    "+   +---+---+---+   +---+   +---+   +   +---+   +   +   +---+",
    "|   |               |               |       |   |           |",
    "+   +   +---+---+---+   +   +---+   +---+   +   +---+   +   +",
    "|   |       |           |       |   |       |       |       |",
    "+   +---+   +   +   +---+---+   +   +   +---+---+   +---+   +",
    "|           |   |       |       |   |       |       |   |   |",
    "+   +   +   +   +---+   +   +---+   +---+   +   +---+   +   +",
    "|   |       |       |   |   |   |       |       |       |   |",
    "+   +   +   +---+---+   +   +   +   +   +   +---+   +   +   +",
    "|   |   |           |       |       |           |   |       |",
    "+   +   +---+---+   +   +---+---+   +---+---+   +   +---+---+",
    "|   |   |           |           |       |       |           |",
    "+   +   +   +---+---+---+---+   +   +   +   +---+---+---+   +",
    "|   |   |               |       |   |   |           |       |",
    "+   +---+---+   +---+   +   +---+   +   +---+   +   +   +---+",
    "|   |       |   |           |           |       |   |   |   |",
    "+   +   +   +   +   +---+   +---+---+   +   +---+   +   +   +",
    "|   |   |           |   |               |   |               |",
    "+   +   +---+---+---+   +---+---+---+   +   +---+   +---+   +",
    "|   |       |                   |       |       |       |   |",
    "+   +   +   +   +   +   +---+   +   +---+---+   +---+   +   +",
    "|   |   |       |   |           |   |       |   |       |   |",
    "+   +   +---+---+   +   +   +   +   +---+   +   +---+---+   +",
    "|               |           |               |               |",
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+ E +",
)


def _wall_table():
    """{(row, col, d): True if side d of cell (row, col) is walled}."""
    walls = {}
    for row in range(N):
        for col in range(N):
            r, c = 2 * row + 1, 4 * col
            walls[row, col, 0] = LAYOUT[r][c + 4] == "|"                # +x: right
            walls[row, col, 1] = LAYOUT[r - 1][c + 1:c + 4] == "---"    # +y: above
            walls[row, col, 2] = LAYOUT[r][c] == "|"                    # -x: left
            walls[row, col, 3] = LAYOUT[r + 1][c + 1:c + 4] == "---"    # -y: below
    return walls


# WALLS[row, col, d] is True if there is a wall on side d of cell (row, col).
WALLS = _wall_table()


def cell_centre(cell):
    """World (x, y) of the centre of cell (row, col)."""
    return cell[1] * PITCH, (N - 1 - cell[0]) * PITCH


def inside(row, col):
    return 0 <= row < N and 0 <= col < N


def neighbours(cell):
    """Open neighbouring cells of `cell`, as (direction, cell) pairs.
    The exit leads outside the maze and is not included."""
    row, col = cell
    out = []
    for d in range(4):
        n = (row + STEP[d][0], col + STEP[d][1])
        if not WALLS[row, col, d] and inside(*n):
            out.append((d, n))
    return out


# ----------------------------------------------------------------- robot
WHEEL_R = 0.017           # m, wheel radius
TRACK = 0.078             # m, wheel centre to wheel centre
TICKS_PER_REV = 360       # encoder ticks per wheel revolution
M_PER_TICK = 2 * math.pi * WHEEL_R / TICKS_PER_REV
WHEEL_MAX = 30.0          # rad/s, the simulator's clamp
AXLE_BACK = 0.033         # m, the wheel axle is this far behind the chassis centre
NO_HIT = 2.0              # m, what a ToF reads when nothing is within its 2 m range

# ToF sensors: name -> (x, y) from the chassis centre, and the direction
# the ray points relative to the robot's heading. Note the names:
# "sl" / "sr" face FORWARD; "fl" / "fr" face the SIDES.
SENSORS = (
    ("sl", (-0.005, 0.034), 0.0),
    ("sr", (-0.005, -0.034), 0.0),
    ("fl", (0.036, 0.005), math.pi / 2),
    ("fr", (0.036, -0.005), -math.pi / 2),
)


def wrap(a):
    """Fold an angle into (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def wheels(v, omega):
    """(left, right) wheel speeds in rad/s for a forward speed v (m/s) and
    a turn rate omega (rad/s, positive = counter-clockwise / left)."""
    left = (v - omega * TRACK / 2) / WHEEL_R
    right = (v + omega * TRACK / 2) / WHEEL_R
    return clamp(left, -WHEEL_MAX, WHEEL_MAX), clamp(right, -WHEEL_MAX, WHEEL_MAX)


# ============================================================== planning

def plan_route(start, heading, pellet):
    """Plan the route: from `start` facing `heading`, through `pellet`, out
    of the exit.

    start, pellet: (row, col) cells. heading: 0-3.
    Returns a list of actions for the solver to drive, for example
    [("run", 3), ("turn", 1), ("run", 2), ...] - "run n" drives n cells
    straight on, "turn d" pivots to face direction d. The last run should
    carry the robot out through the exit.

    TODO: search the maze (WALLS / neighbours()). A breadth-first search
    finds the fewest cells; turns are slow, so it can pay to count them too.
    """
    return []


# ================================================================ solver

class Solver:
    """One sensor frame in, wheel speeds out."""

    def __init__(self):
        self.t = 0.0            # s of sim time since the first sensor frame
        self.spawn = None       # (dir, cell) of the robot at spawn
        self.pellet = None      # (row, col) cell of the pellet
        self.actions = None     # the plan, once made
        self.prev_ticks = None  # (left, right) encoder ticks last frame
        self.done = False
        # TODO: your pose estimate and controller state.

    def log(self, text):
        print(f"[{self.t:7.2f}s] {text}", flush=True)

    def on_poses(self, msg):
        """pacbot/poses:
        {"bot": [row, col], "dir": 0-3}
        The chassis centre spawns on cell_centre(cell) (the axle is
        AXLE_BACK behind it)."""
        bot, d = msg.get("bot"), msg.get("dir")
        if isinstance(bot, list) and d is not None and self.actions is None:
            self.spawn = (d, tuple(bot))
            self.log(f"spawn {self.spawn[1]} facing {self.spawn[0]}")

    def on_pellets(self, msg):
        """pacbot/pellets:
        {"pellet": [row, col]} - the pellet's cell; it sits on the cell centre."""
        pellet = msg.get("pellet")
        if pellet and self.actions is None:
            self.pellet = tuple(pellet)
            self.log(f"pellet {self.pellet}")

    def step(self, s):
        """pacbot/sensors, one physics step:
        {"fl": m, "fr": m, "sl": m, "sr": m,      ToF distances (see SENSORS)
         "gyro": [x, y, z],                        rad/s; z is the turn rate
         "accel": [x, y, z],                       m/s^2
         "ticks_l": int, "ticks_r": int,           running encoder counts
         "dt": 0.01}                               s
        Returns (left, right) wheel speeds in rad/s."""
        self.t += s["dt"]
        ticks = (s["ticks_l"], s["ticks_r"])
        if self.prev_ticks is None:
            self.prev_ticks = ticks
        # Distance each wheel rolled since the last frame.
        d_left = (ticks[0] - self.prev_ticks[0]) * M_PER_TICK
        d_right = (ticks[1] - self.prev_ticks[1]) * M_PER_TICK
        self.prev_ticks = ticks
        turn_rate = s["gyro"][2]

        if self.done or self.spawn is None or self.pellet is None:
            return 0.0, 0.0
        if self.actions is None:
            self.actions = plan_route(self.spawn[1], self.spawn[0], self.pellet)
            self.log(f"plan: {self.actions}")

        # TODO: track the robot's pose from d_left, d_right and turn_rate
        #       (and the ToF readings, to correct it).
        # TODO: drive self.actions one by one, e.g. with wheels(v, omega).
        return 0.0, 0.0


# ================================================================== main

def _client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:  # paho-mqtt < 2
        return mqtt.Client()


def main():
    solver = Solver()
    client = _client()

    def send(cli, left, right):
        # The simulator parses exactly this shape - keep json's defaults.
        cli.publish(TOPIC_WHEEL_VEL, json.dumps({"left": float(left), "right": float(right)}))

    def on_message(cli, _userdata, msg):
        if msg.topic == TOPIC_SENSORS:
            send(cli, *solver.step(json.loads(msg.payload)))
        elif msg.topic == TOPIC_POSES:
            if msg.payload:  # an empty retained message just clears the topic
                solver.on_poses(json.loads(msg.payload))
        elif msg.topic == TOPIC_PELLETS:
            if msg.payload:
                solver.on_pellets(json.loads(msg.payload))
        elif msg.topic == TOPIC_RESULT:
            solver.log(f"result: {msg.payload.decode()}")
            solver.done = True
            send(cli, 0.0, 0.0)
            cli.disconnect()

    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    for topic in (TOPIC_POSES, TOPIC_PELLETS, TOPIC_RESULT, TOPIC_SENSORS):
        client.subscribe(topic)
    print("connected, waiting for the simulator", flush=True)
    client.loop_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
