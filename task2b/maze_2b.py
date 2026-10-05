"""Reader for the PacBot Task 2B maze file format.

Ships to teams alongside the official maze. The maze is fixed for the whole
competition and published, so this is not something you have to discover with
the ToF sensors - it is a map you are given. What you are not given is where
the pellets are: those arrive over MQTT when the run starts, and they change
every run.

The format itself is documented in MAZE_FORMAT.md. In short, a header of
`key value` lines followed by an ASCII drawing:

    +---+---+---+---+
    |               |
    +   +---+---+   +
    |   |       |   |
    +   +   +   +   +
    S       |       E
    +---+---+   +---+
    |               |
    +---+---+---+---+

`S` marks the entrance gap in the west border, `E` the exit gap in the east
border. Cell interiors carry no information, so you can annotate them.

Coordinates. Cells are `(row, col)`, both zero-based, with row 0 at the top of
the drawing and column 0 at the left. Headings are EAST/NORTH/WEST/SOUTH =
0/1/2/3, which runs counter-clockwise, so `(heading + 1) % 4` is a 90 degree
turn to the LEFT and `(heading + 3) % 4` is a turn to the right. The
robot enters cell `(spawn_row, 0)` heading EAST and must leave cell
`(exit_row, grid - 1)` through the east border.

    from maze_2b import Maze
    maze = Maze.official()
    maze.can_move((7, 0), Maze.EAST)   # -> True / False
"""

# --------------------------------------------------------------------------
# BEGIN GENERATED MAZE - written by scripts/embed_maze.py, do not edit by hand.
# The official maze, embedded so there is no file to find and no path to get
# wrong. This is the same text compiled into task_2b_launch, so your planner
# and the simulator cannot disagree about the walls.
MAZE_ID = "9fda2b8ce520f79b"
MAZE_SHA256 = "9fda2b8ce520f79b2be484e7132529998763380e9ca66906f7c2098b1e27683a"
MAZE_TEXT = """\
# PacBot Task 2B maze. Edit the drawing below by hand; run
#   ./maze_tool validate <file>
# to check it before use. Cell interiors are ignored, so you can
# annotate them freely. Pellets are NOT stored here - they are drawn
# per run from the pool this maze defines.
version 1
grid 15
pitch 0.2200
spawn 14
exit 7

+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
|                           |       |       |               |
+   +---+---+   +---+   +   +   +   +   +   +   +---+   +---+
|   |               |   |       |       |   |       |       |
+   +   +   +---+   +   +   +   +---+   +   +   +---+---+   +
|   |   |       |   |       |       |   |               |   |
+   +   +   +---+   +   +   +---+   +   +---+   +   +   +   +
|       |               |           |           |   |       |
+---+---+   +---+---+   +   +---+   +---+---+---+   +   +---+
|   |               |   |               |                   |
+   +   +---+   +   +   +---+---+---+   +---+   +   +---+   +
|       |       |   |               |           |           |
+   +---+   +---+   +   +---+---+   +   +---+---+   +---+   +
|       |   |       |           |       |           |       |
+---+   +   +   +---+---+---+   +---+---+   +   +---+   +---+
|                           |               |               E
+   +---+---+   +---+---+   +   +   +---+---+   +   +---+   +
|           |                   |               |           |
+   +   +   +   +---+---+---+   +---+   +   +---+   +---+---+
|   |   |       |                   |   |                   |
+   +   +---+   +   +---+   +---+   +   +---+---+   +---+   +
|           |           |           |           |   |       |
+   +---+   +---+---+   +   +---+---+   +   +   +   +   +   +
|       |           |                   |   |   |   |   |   |
+---+   +   +---+   +---+---+   +---+   +   +   +   +   +   +
|       |               |           |       |           |   |
+   +   +---+   +---+   +   +---+   +---+   +   +---+   +   +
|   |           |       |       |               |           |
+   +   +   +---+   +   +---+   +   +   +---+   +   +---+   +
S       |           |                       |               |
+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
"""
# END GENERATED MAZE
# --------------------------------------------------------------------------

WALL = "#"
OPEN = "."


class MazeError(ValueError):
    """Raised when a maze file is malformed. Carries the line number."""


class Maze:
    EAST, NORTH, WEST, SOUTH = 0, 1, 2, 3

    # (d_row, d_col) per heading, in EAST, NORTH, WEST, SOUTH order. North is
    # -row, so row 0 is the north edge.
    DELTA = ((0, 1), (-1, 0), (0, -1), (1, 0))

    def __init__(self, grid, pitch, spawn_row, exit_row, h_walls, v_walls):
        self.grid = grid
        self.pitch = pitch
        self.spawn_row = spawn_row
        self.exit_row = exit_row
        # h_walls[r] separates cell-row r-1 from r; v_walls[c] separates
        # cell-column c-1 from c. Both are grid+1 lists of grid characters.
        self.h_walls = h_walls
        self.v_walls = v_walls

    # ------------------------------------------------------------- loading

    @classmethod
    def official(cls):
        """The official Task 2B maze. This is the one you want.

        No file on disk and no path to get wrong: the maze is embedded, and it
        is the same text compiled into task_2b_launch, so your planner and the
        simulator cannot end up looking at different walls.

        The embedded text is checked against its own hash first. Editing the
        MAZE_TEXT block cannot change the simulation - the simulator has its
        own copy built in and never reads this file - so the only thing a
        change here can do is make your planner route through walls that are
        still there. That failure is miserable to debug, so it is caught here
        instead, loudly, the moment you ask for the maze.
        """
        import hashlib

        digest = hashlib.sha256(MAZE_TEXT.encode()).hexdigest()
        if digest != MAZE_SHA256:
            raise MazeError(
                "the embedded maze in maze_2b.py has been modified.\n"
                f"  expected sha256 {MAZE_SHA256}\n"
                f"  found            {digest}\n"
                "The simulator still uses the real maze, so your planner would\n"
                "be routing through walls that are actually there. Restore the\n"
                "MAZE_TEXT block from the copy you were given."
            )
        return cls.parse(MAZE_TEXT)

    @classmethod
    def load(cls, path):
        """Parse a maze file. Only needed if you are drawing your own to
        practise on; the competition always runs the official maze."""
        with open(path, "r", encoding="utf-8") as handle:
            return cls.parse(handle.read())

    @classmethod
    def parse(cls, text):
        lines = [line.rstrip("\r") for line in text.split("\n")]

        grid, pitch, spawn_row, exit_row = None, 0.22, None, None
        start = 0
        for i, raw in enumerate(lines):
            if raw.startswith("+"):
                start = i
                break
            if not raw.strip() or raw.lstrip().startswith("#"):
                continue
            parts = raw.split()
            key, value = parts[0], (parts[1] if len(parts) > 1 else "")
            try:
                if key == "version":
                    if int(value) != 1:
                        raise MazeError(f"line {i + 1}: unsupported version {value}")
                elif key == "grid":
                    grid = int(value)
                elif key == "pitch":
                    pitch = float(value)
                elif key == "spawn":
                    spawn_row = int(value)
                elif key == "exit":
                    exit_row = int(value)
                else:
                    raise MazeError(f"line {i + 1}: unknown header key {key!r}")
            except ValueError as exc:
                raise MazeError(f"line {i + 1}: bad value for {key!r}: {value!r}") from exc
        else:
            raise MazeError("no maze drawing found (no line starting with '+')")

        if grid is None:
            raise MazeError("header is missing 'grid'")
        if spawn_row is None or not 0 <= spawn_row < grid:
            raise MazeError("header 'spawn' missing or out of range")
        if exit_row is None or not 0 <= exit_row < grid:
            raise MazeError("header 'exit' missing or out of range")

        want_lines, want_cols = 2 * grid + 1, 4 * grid + 1
        drawing = lines[start:start + want_lines]
        if len(drawing) < want_lines:
            raise MazeError(
                f"line {start + 1}: expected {want_lines} drawing lines for grid "
                f"{grid}, found {len(drawing)}"
            )

        h_walls = [[WALL] * grid for _ in range(grid + 1)]
        v_walls = [[WALL] * grid for _ in range(grid + 1)]

        for r in range(grid + 1):
            # Editors strip trailing whitespace, so pad rather than reject.
            corner = drawing[2 * r].ljust(want_cols)
            for c in range(grid):
                if corner[4 * c] != "+":
                    raise MazeError(
                        f"line {start + 2 * r + 1}: expected '+' at column {4 * c + 1}"
                    )
                segment = corner[4 * c + 1:4 * c + 4]
                if segment == "---":
                    h_walls[r][c] = WALL
                elif not segment.strip():
                    h_walls[r][c] = OPEN
                else:
                    raise MazeError(
                        f"line {start + 2 * r + 1}: expected '---' or blank at column "
                        f"{4 * c + 2}, found {segment!r}"
                    )
            if r == grid:
                break

            cells = drawing[2 * r + 1].ljust(want_cols)
            for c in range(grid + 1):
                char = cells[4 * c]
                # 'S' and 'E' mark the border gaps and read as open passage.
                if char == "|":
                    v_walls[c][r] = WALL
                elif char in (" ", "S", "E"):
                    v_walls[c][r] = OPEN
                else:
                    raise MazeError(
                        f"line {start + 2 * r + 2}: expected '|', space, 'S' or 'E' at "
                        f"column {4 * c + 1}, found {char!r}"
                    )

        return cls(grid, pitch, spawn_row, exit_row,
                   ["".join(row) for row in h_walls],
                   ["".join(row) for row in v_walls])

    # -------------------------------------------------------------- queries

    @property
    def spawn_cell(self):
        return (self.spawn_row, 0)

    @property
    def exit_cell(self):
        return (self.exit_row, self.grid - 1)

    def in_bounds(self, cell):
        r, c = cell
        return 0 <= r < self.grid and 0 <= c < self.grid

    def wall_north_of(self, cell):
        return self.h_walls[cell[0]][cell[1]] == WALL

    def wall_south_of(self, cell):
        return self.h_walls[cell[0] + 1][cell[1]] == WALL

    def wall_west_of(self, cell):
        return self.v_walls[cell[1]][cell[0]] == WALL

    def wall_east_of(self, cell):
        return self.v_walls[cell[1] + 1][cell[0]] == WALL

    def can_move(self, cell, heading):
        """Can the robot drive one cell from `cell` in `heading`?

        False at the outer border even where a gap exists: leaving the grid is
        the entrance and exit, not ordinary movement.
        """
        dr, dc = self.DELTA[heading]
        nxt = (cell[0] + dr, cell[1] + dc)
        if not self.in_bounds(nxt):
            return False
        if heading == self.NORTH:
            return not self.wall_north_of(cell)
        if heading == self.EAST:
            return not self.wall_east_of(cell)
        if heading == self.SOUTH:
            return not self.wall_south_of(cell)
        return not self.wall_west_of(cell)

    def neighbours(self, cell):
        """Every cell reachable from `cell` in one move, as (heading, cell)."""
        out = []
        for heading in range(4):
            if self.can_move(cell, heading):
                dr, dc = self.DELTA[heading]
                out.append((heading, (cell[0] + dr, cell[1] + dc)))
        return out

    # ---------------------------------------------------------- world frame

    def cell_center(self, cell):
        """World (x, y) of a cell centre, matching the simulator's layout.

        Columns run along +x and rows along -y, so the simulator's top-down
        view looks like the maze file's drawing.
        """
        return (cell[1] * self.pitch, (self.grid - 1 - cell[0]) * self.pitch)

    def cell_for_xy(self, x, y):
        col = min(max(int(round(x / self.pitch)), 0), self.grid - 1)
        row = min(max(self.grid - 1 - int(round(y / self.pitch)), 0), self.grid - 1)
        return (row, col)
