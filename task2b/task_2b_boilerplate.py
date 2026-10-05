"""Boilerplate for PB Task 2B.

All the plumbing, none of the thinking. It connects to the broker, loads the
official maze, receives the pellet positions for this run, and publishes a
wheel velocity command on every sensor frame. Your job is the one TODO block
in `decide()`.

Run it in two terminals, from the folder you unpacked:

    ./task_2b_launch                 # terminal 1: the simulator
    python3 task_2b_boilerplate.py   # terminal 2: this file

Neither takes a maze argument. The official maze is built into both, so there
is no path to get wrong and no way for your planner and the simulator to end
up looking at different walls. If the broker is not already running:

    sudo systemctl enable --now mosquitto

Everything you need is in this file: the maze, where you start and finish, and
every message the simulator will send you. Read on.

------------------------------------------------------------------------------
WHAT YOU ARE GIVEN

The maze. Fixed for the whole Sub-Task and published, so you may read it,
precompute against it, and plan routes offline. `Maze.official()` hands it to
you already parsed, from a copy embedded in maze_2b.py, which is the same text
built into the simulator. This is deliberately
not a mapping task - the ToF sensors are for driving accurately, not for
discovering walls.

The pellets. THREE cells, delivered over `pacbot/pellets` when the run starts,
and different on every single run. You cannot hardcode a route. Every possible
pellet set has been checked to need the same optimal driving time, so no draw
is luckier than another; the only thing that separates two teams is how well
they plan and drive.

------------------------------------------------------------------------------
THE MAZE

This is the whole thing. It never changes, so plan against it offline.
`Maze.official()` returns it already parsed; the drawing below is the same
maze for reading with your eyes.

        col    0   1   2   3   4   5   6   7   8   9  10  11  12  13  14
             +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
     row 0   |                           |       |       |               |
             +   +---+---+   +---+   +   +   +   +   +   +   +---+   +---+
     row 1   |   |               |   |       |       |   |       |       |
             +   +   +   +---+   +   +   +   +---+   +   +   +---+---+   +
     row 2   |   |   |       |   |       |       |   |               |   |
             +   +   +   +---+   +   +   +---+   +   +---+   +   +   +   +
     row 3   |       |               |           |           |   |       |
             +---+---+   +---+---+   +   +---+   +---+---+---+   +   +---+
     row 4   |   |               |   |               |                   |
             +   +   +---+   +   +   +---+---+---+   +---+   +   +---+   +
     row 5   |       |       |   |               |           |           |
             +   +---+   +---+   +   +---+---+   +   +---+---+   +---+   +
     row 6   |       |   |       |           |       |           |       |
             +---+   +   +   +---+---+---+   +---+---+   +   +---+   +---+
     row 7   |                           |               |               E
             +   +---+---+   +---+---+   +   +   +---+---+   +   +---+   +
     row 8   |           |                   |               |           |
             +   +   +   +   +---+---+---+   +---+   +   +---+   +---+---+
     row 9   |   |   |       |                   |   |                   |
             +   +   +---+   +   +---+   +---+   +   +---+---+   +---+   +
     row 10  |           |           |           |           |   |       |
             +   +---+   +---+---+   +   +---+---+   +   +   +   +   +   +
     row 11  |       |           |                   |   |   |   |   |   |
             +---+   +   +---+   +---+---+   +---+   +   +   +   +   +   +
     row 12  |       |               |           |       |           |   |
             +   +   +---+   +---+   +   +---+   +---+   +   +---+   +   +
     row 13  |   |           |       |       |               |           |
             +   +   +   +---+   +   +---+   +   +   +---+   +   +---+   +
     row 14  S       |           |                       |               |
             +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+

You START outside the west border at the `S` gap on row 14, facing EAST, and
drive into cell (14, 0). You FINISH by leaving cell (7, 14) through the `E`
gap in the east border. Crossing that plane is the only thing that ends a run
successfully.

Coordinates. Cells are (row, col), both zero-based, with row 0 at the TOP of
the drawing and col 0 at the LEFT. Headings are

    EAST = 0    NORTH = 1    WEST = 2    SOUTH = 3

The order runs counter-clockwise, so (heading + 1) % 4 is a 90 degree turn to
the LEFT, and (heading + 3) % 4 is a turn to the RIGHT. You spawn facing EAST,
which is heading 0.

Cell centres in metres. This is the frame the ghost's x/y arrive in, and the
one to dead-reckon in:

    x = col * 0.22
    y = (14 - row) * 0.22

So cell (14, 0) sits at (0.00, 0.00) and cell (0, 14) at (3.08, 3.08). Driving
EAST increases x; driving NORTH increases y. Going back the other way:

    col = round(x / 0.22)
    row = 14 - round(y / 0.22)

You spawn at (-0.15, 0.00), in the short chute outside the west border, so
your first move is eastward through the gap. The run ends when your chassis
crosses x = 3.19, which is the east border plane half a pitch beyond the
centres of column 14.

Asking the maze about walls, rather than working it out from the drawing:

    maze.can_move((7, 0), Maze.EAST)     # -> True / False
    maze.grid                            # 15
    maze.pitch                           # 0.22
    maze.spawn_row, maze.exit_row        # 14, 7

------------------------------------------------------------------------------
THE WIRE PROTOCOL

  pacbot/sensors    simulator -> you, 100 times a second
    {"fl": .., "fr": .., "sl": .., "sr": ..,
     "gyro":  [wx, wy, wz], "accel": [ax, ay, az],
     "ticks_l": .., "ticks_r": .., "dt": .., "t": ..}

    The physics runs at 500 Hz; you see every fifth step, so dt is always
    0.010. The simulator does not wait for you between frames - it keeps
    stepping and the wheels hold your last command.

    fl  metres to the wall on the LEFT,  90 deg left of forward  (+y)
    fr  metres to the wall on the RIGHT, 90 deg right of forward (-y)
    sl  metres to the wall AHEAD, 0 deg, from left of centre     (+x)
    sr  metres to the wall AHEAD, 0 deg, from right of centre    (+x)

      All four rays are axis-aligned, so a reading IS the perpendicular
      distance. No trigonometry needed.

      Do not be misled by the names: `fl`/`fr` look sideways and `sl`/`sr`
      look forward. They are inherited from Task 1B so your 1B code still
      works.

      RANGE. The sensors see 2.0 m and no further, like the real hardware.
      A reading of 2.0 means "at least 2 m away, or nothing there at all" -
      you cannot tell those apart, and neither can a real ToF module. Every
      reading is therefore in [0.0, 2.0].

      THE PELLETS ARE INVISIBLE TO THESE SENSORS. They are simulated as
      sites, which the rangefinders cannot see, so you cannot sweep the maze
      looking for them - a ToF ray passes straight through a pellet and
      reports the wall behind it. The only source of pellet positions is the
      pacbot/pellets topic below.

    gyro[2]        yaw rate about z, rad/s. Integrate it to measure turns.
    accel          body-frame acceleration, m/s^2.
    ticks_l,       RUNNING encoder counts, 360 per wheel revolution, positive
    ticks_r        forward. These are cumulative totals, not rates: subtract
                   successive readings to get how far each wheel turned, so a
                   dropped message loses nothing.

                       dl = ticks_l - prev_ticks_l
                       dr = ticks_r - prev_ticks_r
                       distance = WHEEL_RADIUS * ((dl + dr) / 2) * 2*pi/360
    dt             seconds covered by this frame.
    t              simulator clock, seconds since the run started.
  pacbot/pellets    simulator -> you, at startup and after each collection
    {"pellets": [[row, col], [row, col], [row, col]]}

    The pellets STILL on the board. The list shrinks as you collect them, and
    an empty list means all three are yours. Retained, so a late subscriber
    still gets it.

    Wrapped in an object with a named key, like Task 2A's pellet message,
    so the topic can gain more fields later without breaking your parser.

  pacbot/ghost      simulator -> you, every sensor frame
    {"row": .., "col": .., "x": .., "y": ..}

    Where the patrolling ghost is RIGHT NOW: its cell, and its exact position
    in metres. RETAINED, so subscribing late still tells you where it is.

    It never chases you, but touching it ends your run with whatever pellets
    you have already collected. Its route is never published: watch it and
    work the circuit out yourself. It is slower than you are and it is on a
    circuit, so waiting always works.

  pacbot/result     simulator -> you, ONCE, when the run ends
    {"solved": true, "collisions": 0}

    Sent when the maze is solved, or when the window is closed on a run that
    was not solved. It tells you the run is over and roughly how it went.
    "solved" means all three pellets AND out through the exit; exiting with
    two pellets is not solved.

    It deliberately carries no score, no time and no pellet detail. Those are
    in the sealed result.json only. Not retained, so it can never be mistaken
    for the previous run's outcome.

  pacbot/wheel_vel  you -> simulator
    {"left": <rad/s>, "right": <rad/s>}

    Clamped to +-30 rad/s. Publish one per sensor frame.

    The simulator does NOT wait for you. It keeps stepping at the wall clock
    and the wheels hold whatever command arrived last, exactly as a real
    robot coasts on its last instruction between control updates. Miss a
    frame and you simply steer on a slightly older reading; nothing breaks.

    The keys must appear in THIS ORDER: the simulator matches the message
    against a fixed pattern, so {"right": .., "left": ..} is discarded
    without an error. Building the dict as {"left": .., "right": ..} and
    passing it to json.dumps gives the right shape, as the code below does.

------------------------------------------------------------------------------
GEOMETRY YOU WILL NEED

    cell pitch      0.22 m          maze.pitch
    wheel radius    0.017 m
    half track      0.039 m         wheel centre to robot centreline
    grid            15 x 15         maze.grid

A pellet is collected when the chassis centre passes within 0.06 m of the cell
centre, so clipping a corner does not count. The run ends when the chassis
crosses the exit plane on the east border.
"""
import json
import math

import paho.mqtt.client as mqtt

import maze_2b
from maze_2b import Maze

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"      # simulator publishes, this file subscribes
TOPIC_PELLETS = "pacbot/pellets"      # simulator publishes, this file subscribes
TOPIC_GHOST = "pacbot/ghost"          # simulator publishes, this file subscribes
TOPIC_RESULT = "pacbot/result"        # simulator publishes, this file subscribes
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"  # this file publishes, simulator subscribes

WHEEL_RADIUS = 0.017  # m
HALF_TRACK = 0.039    # m, wheel centre to centreline
WHEEL_LIMIT = 30.0    # rad/s, the simulator clamps to this
TICKS_PER_REV = 360   # encoder counts per wheel revolution
TICK_RAD = 2.0 * math.pi / TICKS_PER_REV   # radians of wheel turn per tick


def _mqtt_client():
    # paho-mqtt >= 2.0 requires picking a callback API version explicitly.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def wheels(forward, turn):
    """Differential drive mixer: +turn steers LEFT (counter-clockwise)."""
    return forward - turn, forward + turn


class Controller:
    def __init__(self, maze):
        self.maze = maze
        self.pellets = []     # cells still on the board; filled from MQTT
        self.have_pellets = False
        self.elapsed = 0.0
        self.ghost = None     # latest pacbot/ghost payload, or None
        self.prev_ticks = None  # (left, right) counts from the previous frame

    def on_pellets(self, msg):
        """Called whenever the remaining-pellet list arrives or changes.

        msg is {"pellets": [[row, col], ...]}.
        """
        self.pellets = [tuple(c) for c in msg.get("pellets", [])]
        self.have_pellets = True
        print(f"pellets remaining: {self.pellets}")

        # TODO (optional): plan here rather than in decide(). You are told the
        # pellets once, at the start, and the maze never changes - so the whole
        # route can be computed now, before the robot has moved.

    def decide(self, s):
        """One sensor frame in, one wheel command out.

        `s` is the decoded pacbot/sensors payload. Return (left, right) in
        rad/s.
        """
        self.elapsed += s["dt"]

        left_dist = s["fl"]          # m to the wall on the left
        right_dist = s["fr"]         # m to the wall on the right
        front_dist = min(s["sl"], s["sr"])   # m to the wall ahead
        yaw_rate = s["gyro"][2]      # rad/s about z
        # Ticks are cumulative counts, so speed is a DIFFERENCE, not a
        # reading. 360 counts per wheel revolution.
        tl, tr = s["ticks_l"], s["ticks_r"]
        if self.prev_ticks is None or s["dt"] <= 0:
            speed = 0.0
        else:
            dl, dr = tl - self.prev_ticks[0], tr - self.prev_ticks[1]
            speed = (WHEEL_RADIUS * ((dl + dr) / 2.0) * TICK_RAD) / s["dt"]
        self.prev_ticks = (tl, tr)

        # Nothing to do until we know where the pellets are.
        if not self.have_pellets:
            return 0.0, 0.0

        # ------------------------------------------------------------------
        # TODO: your control logic goes here.
        #
        # A workable shape, if you want one:
        #   1. Plan a route: spawn -> the 3 pellets in some order -> exit.
        #      Remember that turns cost real time, so the shortest route in
        #      cells is often not the fastest one.
        #   2. Turn that route into a list of moves: forward one cell, turn
        #      left, turn right.
        #   3. Execute one move at a time. Integrate `speed * dt` to know when
        #      a cell has been covered; integrate `yaw_rate * dt` to know when
        #      a 90 degree turn is done.
        #   4. While driving straight, keep centred with a PID on
        #      (left_dist - right_dist) so you do not scrape a wall.
        #
        # Unused so far - delete these lines once you use them.
        _ = (left_dist, right_dist, front_dist, yaw_rate, speed)

        forward = 0.0
        turn = 0.0
        # ------------------------------------------------------------------

        return wheels(forward, turn)


def main():
    # The official maze, embedded - no file to find, no path to pass.
    # The id printed here must match the one the simulator prints in its
    # banner ("maze official <id> (built in)"). If they ever differ, your
    # planner and the simulator are looking at different walls.
    maze = Maze.official()
    print(f"official maze {maze_2b.MAZE_ID}: {maze.grid}x{maze.grid}, "
          f"spawn row {maze.spawn_row}, exit row {maze.exit_row}")

    controller = Controller(maze)
    client = _mqtt_client()

    def on_message(cli, userdata, msg):
        payload = json.loads(msg.payload.decode())
        if msg.topic == TOPIC_PELLETS:
            controller.on_pellets(payload)
            return
        if msg.topic == TOPIC_GHOST:
            controller.ghost = payload      # {"row":..,"col":..,"x":..,"y":..}
            return
        if msg.topic == TOPIC_RESULT:
            print(f"run over: solved={payload['solved']}, "
                  f"collisions={payload['collisions']}")
            return
        left, right = controller.decide(payload)
        left = max(-WHEEL_LIMIT, min(WHEEL_LIMIT, float(left)))
        right = max(-WHEEL_LIMIT, min(WHEEL_LIMIT, float(right)))
        # Key order matters: the simulator matches a fixed pattern. See the
        # pacbot/wheel_vel notes in the header comment.
        cli.publish(TOPIC_WHEEL_VEL,
                    json.dumps({"left": left, "right": right}))

    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    client.subscribe(TOPIC_PELLETS)
    client.subscribe(TOPIC_GHOST)
    client.subscribe(TOPIC_RESULT)
    client.subscribe(TOPIC_SENSORS)
    print("connected; waiting for the simulator")
    client.loop_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
