import math

from nav.config import GRID_SIZE, COST_INFLUENCE_RADIUS, COST_MAX_EXTRA, ELEVATION_COST_FACTOR

DIAGONAL_COST = math.sqrt(2)
CARDINAL_DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)]
DIAGONAL_DIRS = [(-1, -1), (-1, 1), (1, -1), (1, 1)]


class Grid:
    FREE = 0
    OBSTACLE = 1
    START = 2
    GOAL = 3


    def __init__(self, size=None):
        # Defaults to the pygame visualizer's GRID_SIZE; the scale
        # benchmark (nav/scale_benchmark.py) is the only caller that
        # passes something else -- everywhere else, Grid() behaves
        # exactly as before.
        self.size = size or GRID_SIZE
        self.cells = []
        for _ in range(self.size):
            row = []
            for _ in range(self.size):
                row.append(Grid.FREE)
            self.cells.append(row)
        self.start = None
        self.goal = None
        # 4-directional (False) or 8-directional (True) movement
        self.diagonal = False
        # Weighted-terrain mode (obstacle inflation, see compute_cost_map)
        self.cost_map_enabled = False
        self.cost = [[1.0 for _ in range(self.size)] for _ in range(self.size)]
        # Terrain elevation (world units, 0.0 = flat baseline everywhere).
        # A separate array from `cost`, not folded into it: cost-map
        # inflation is symmetric (a cell's cost to enter is the same
        # regardless of which neighbor you came from), but elevation cost
        # is inherently directional -- climbing into a cell costs more,
        # descending into the exact same cell from the exact same
        # neighbor doesn't -- so it can't be expressed as a single
        # per-cell multiplier the way `cost` is. See get_neighbors.
        # Populated directly (`grid.elevation[r][c] = ...`) by whatever
        # builds the grid, the same way obstacles are set directly via
        # `grid.cells[r][c] = Grid.OBSTACLE` rather than through a setter.
        self.elevation = [[0.0 for _ in range(self.size)] for _ in range(self.size)]
        # Whether get_neighbors actually charges the elevation cost
        # below -- off by default, same opt-in pattern as
        # cost_map_enabled, so a grid with elevation data set but this
        # left off plans exactly as if the terrain were flat.
        self.elevation_aware = False


    def clear(self):
        for row in range(self.size):
            for col in range(self.size):
                self.cells[row][col] = Grid.FREE
        self.start = None
        self.goal = None
        self.elevation = [[0.0 for _ in range(self.size)] for _ in range(self.size)]
        self.refresh_cost_map()


    def is_valid(self, row, col):
        return 0 <= row < self.size and 0 <= col < self.size


    def is_obstacle(self, row, col):
        return self.is_valid(row, col) and self.cells[row][col] == Grid.OBSTACLE
    
    
    def is_free(self, row, col):
        return self.is_valid(row, col) and self.cells[row][col] == Grid.FREE
    

    def toggle_obstacle(self, row, col):
        if not self.is_valid(row, col):
            return
        
        if self.cells[row][col] == Grid.START or self.cells[row][col] == Grid.GOAL:
            return
        
        if self.cells[row][col] == Grid.FREE:
            self.cells[row][col] = Grid.OBSTACLE

        else:
            self.cells[row][col] = Grid.FREE

        self.refresh_cost_map()


    def place_start(self, row, col):
        if not self.is_valid(row, col) or self.cells[row][col] == Grid.OBSTACLE:
            return
        
        if self.start is not None:
            start_row, start_col = self.start
            self.cells[start_row][start_col] = Grid.FREE
        
        if (row, col) == self.goal:
            self.goal = None
        
        self.cells[row][col] = Grid.START
        self.start = (row, col)

    
    def place_goal(self, row, col):
        if not self.is_valid(row, col) or self.cells[row][col] == Grid.OBSTACLE:
            return
        
        if self.goal is not None:
            goal_row, goal_col = self.goal
            self.cells[goal_row][goal_col] = Grid.FREE
        
        if (row, col) == self.start:
            self.start = None
        
        self.cells[row][col] = Grid.GOAL
        self.goal = (row, col)

    
    def _elevation_cost(self, row, col, r, c):
        """Extra cost for moving from (row, col) into (r, c), on top of
        the baseline step/terrain cost -- zero unless elevation_aware is
        on, and even then, zero unless (r, c) is actually *uphill* of
        (row, col). Flat and downhill moves cost exactly the same as
        they would on level ground; only climbing costs extra. Since
        this is always >= 0, it only ever raises a step's true cost
        above the baseline Manhattan/octile already assume, never below
        it -- see WRITEUPS.md for why that's exactly what keeps both
        heuristics admissible under this cost model."""
        if not self.elevation_aware:
            return 0.0
        gain = self.elevation[r][c] - self.elevation[row][col]
        return max(0.0, gain) * ELEVATION_COST_FACTOR

    def get_neighbors(self, row, col):
        """
        Passable neighbors of (row, col) as (cell, step_cost) pairs.
        Cardinal steps cost 1, diagonal steps (only when self.diagonal is
        on) cost sqrt(2) -- each scaled by self.cost[r][c], the terrain
        weight of the cell being entered (1.0 everywhere unless
        compute_cost_map has inflated it near an obstacle), plus an
        elevation surcharge for climbing (see _elevation_cost, only
        charged when self.elevation_aware is on). Diagonal moves are
        skipped if either of the two orthogonal cells they'd cut past is
        an obstacle -- otherwise a "diagonal" move could clip straight
        through a solid wall corner.
        """
        neighbors = []
        for dr, dc in CARDINAL_DIRS:
            r, c = row + dr, col + dc
            if self.is_valid(r, c) and not self.is_obstacle(r, c):
                cost = 1 * self.cost[r][c] + self._elevation_cost(row, col, r, c)
                neighbors.append(((r, c), cost))

        if self.diagonal:
            for dr, dc in DIAGONAL_DIRS:
                r, c = row + dr, col + dc
                if not self.is_valid(r, c) or self.is_obstacle(r, c):
                    continue
                if self.is_obstacle(row + dr, col) or self.is_obstacle(row, col + dc):
                    continue
                cost = DIAGONAL_COST * self.cost[r][c] + self._elevation_cost(row, col, r, c)
                neighbors.append(((r, c), cost))

        return neighbors

    def refresh_cost_map(self):
        """Recompute the cost field if cost-map mode is on, else keep it flat."""
        size = len(self.cells)
        if self.cost_map_enabled:
            self.compute_cost_map()
        else:
            self.cost = [[1.0 for _ in range(size)] for _ in range(size)]

    def compute_cost_map(self, influence_radius=COST_INFLUENCE_RADIUS, max_extra=COST_MAX_EXTRA):
        """
        Weighted-terrain cost field: every free cell within influence_radius
        of an obstacle costs more to enter, decreasing linearly back to the
        1.0 baseline at the edge of the radius. Mirrors Nav2's costmap
        inflation layer -- it pushes A*/Dijkstra to route with clearance
        around obstacles instead of hugging every wall. Spreads outward
        from each obstacle rather than scanning every free cell against
        every obstacle, so cost stays proportional to obstacle count *
        radius^2, not grid size * obstacle count.
        """
        size = len(self.cells)
        self.cost = [[1.0 for _ in range(size)] for _ in range(size)]
        for row in range(size):
            for col in range(size):
                if self.cells[row][col] != Grid.OBSTACLE:
                    continue
                for dr in range(-influence_radius, influence_radius + 1):
                    for dc in range(-influence_radius, influence_radius + 1):
                        r, c = row + dr, col + dc
                        if not self.is_valid(r, c) or self.cells[r][c] == Grid.OBSTACLE:
                            continue
                        dist = math.hypot(dr, dc)
                        if dist > influence_radius:
                            continue
                        extra = max_extra * (1 - dist / influence_radius)
                        self.cost[r][c] = max(self.cost[r][c], 1.0 + extra)