import math

from nav.config import (
    GRID_SIZE, COST_INFLUENCE_RADIUS, COST_MAX_EXTRA,
    TERRAIN_GRASS, TERRAIN_COST,
)

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
        # Terrain layer (Step 3) -- purely cosmetic/cost, every cell
        # starts as grass (TERRAIN_COST[TERRAIN_GRASS] == 1.0, so this is
        # a no-op on cost until something gets painted). Populated via
        # paint_terrain, the same setter-method pattern place_start/
        # place_goal use, rather than direct assignment like `cells` --
        # terrain needs to reject obstacle/start/goal cells, so it needs
        # a real method, not just an array.
        self.terrain = [[TERRAIN_GRASS for _ in range(self.size)] for _ in range(self.size)]


    def clear(self):
        for row in range(self.size):
            for col in range(self.size):
                self.cells[row][col] = Grid.FREE
        self.start = None
        self.goal = None
        self.terrain = [[TERRAIN_GRASS for _ in range(self.size)] for _ in range(self.size)]
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


    def paint_terrain(self, row, col, terrain_type):
        """Set (row, col)'s terrain type -- no-op if the cell is out of
        bounds or is an obstacle, start, or goal (terrain is undergrowth
        the ground itself has; those three occupy the cell instead of
        sitting on it). Recomputes the cost field immediately so the new
        terrain's multiplier (see TERRAIN_COST) is reflected right away,
        same as toggle_obstacle already does for obstacle-inflation cost.
        """
        if not self.is_valid(row, col):
            return
        if self.cells[row][col] in (Grid.OBSTACLE, Grid.START, Grid.GOAL):
            return
        self.terrain[row][col] = terrain_type
        self.refresh_cost_map()


    def get_neighbors(self, row, col):
        """
        Passable neighbors of (row, col) as (cell, step_cost) pairs.
        Cardinal steps cost 1, diagonal steps (only when self.diagonal is
        on) cost sqrt(2) -- each scaled by self.cost[r][c], the terrain
        weight of the cell being entered (1.0 everywhere unless
        compute_cost_map has inflated it near an obstacle). Diagonal
        moves are skipped if either of the two orthogonal cells they'd
        cut past is an obstacle -- otherwise a "diagonal" move could clip
        straight through a solid wall corner.
        """
        neighbors = []
        for dr, dc in CARDINAL_DIRS:
            r, c = row + dr, col + dc
            if self.is_valid(r, c) and not self.is_obstacle(r, c):
                cost = 1 * self.cost[r][c]
                neighbors.append(((r, c), cost))

        if self.diagonal:
            for dr, dc in DIAGONAL_DIRS:
                r, c = row + dr, col + dc
                if not self.is_valid(r, c) or self.is_obstacle(r, c):
                    continue
                if self.is_obstacle(row + dr, col) or self.is_obstacle(row, col + dc):
                    continue
                cost = DIAGONAL_COST * self.cost[r][c]
                neighbors.append(((r, c), cost))

        return neighbors

    def refresh_cost_map(self):
        """Recompute the cost field: obstacle-inflation cost if cost-map
        mode is on (else flat 1.0 baseline), then always multiply in each
        cell's terrain cost on top (see TERRAIN_COST) -- terrain is a
        real property of the ground, so unlike the K-toggle inflation
        halo it applies whether or not cost-map mode is on. Applying it
        last, uniformly, is also what lets draw_grid recover "was this
        cell inflated by K" on its own: dividing grid.cost back by the
        cell's own terrain multiplier gives exactly the pre-terrain
        inflation value, without needing to store that separately.
        """
        size = len(self.cells)
        if self.cost_map_enabled:
            self.compute_cost_map()
        else:
            self.cost = [[1.0 for _ in range(size)] for _ in range(size)]
        for row in range(size):
            for col in range(size):
                self.cost[row][col] *= TERRAIN_COST[self.terrain[row][col]]

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