import math

from nav.config import GRID_SIZE, COST_INFLUENCE_RADIUS, COST_MAX_EXTRA

DIAGONAL_COST = math.sqrt(2)
CARDINAL_DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)]
DIAGONAL_DIRS = [(-1, -1), (-1, 1), (1, -1), (1, 1)]


class Grid:
    FREE = 0
    OBSTACLE = 1
    START = 2
    GOAL = 3


    def __init__(self):
        self.cells = []
        for _ in range(GRID_SIZE):
            row = []
            for _ in range(GRID_SIZE):
                row.append(Grid.FREE)
            self.cells.append(row)
        self.start = None
        self.goal = None
        # 4-directional (False) or 8-directional (True) movement
        self.diagonal = False
        # Weighted-terrain mode (obstacle inflation, see compute_cost_map)
        self.cost_map_enabled = False
        self.cost = [[1.0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]


    def clear(self):
        for row in range(GRID_SIZE):
            for col in range(GRID_SIZE):
                self.cells[row][col] = Grid.FREE
        self.start = None
        self.goal = None
        self.refresh_cost_map()


    def is_valid(self, row, col):
        return 0 <= row < GRID_SIZE and 0 <= col < GRID_SIZE
    

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

    
    def get_neighbors(self, row, col):
        """
        Passable neighbors of (row, col) as (cell, step_cost) pairs.
        Cardinal steps cost 1, diagonal steps (only when self.diagonal is
        on) cost sqrt(2) -- each scaled by self.cost[r][c], the terrain
        weight of the cell being entered (1.0 everywhere unless
        compute_cost_map has inflated it near an obstacle). Diagonal moves
        are skipped if either of the two orthogonal cells they'd cut past
        is an obstacle -- otherwise a "diagonal" move could clip straight
        through a solid wall corner.
        """
        neighbors = []
        for dr, dc in CARDINAL_DIRS:
            r, c = row + dr, col + dc
            if self.is_valid(r, c) and not self.is_obstacle(r, c):
                neighbors.append(((r, c), 1 * self.cost[r][c]))

        if self.diagonal:
            for dr, dc in DIAGONAL_DIRS:
                r, c = row + dr, col + dc
                if not self.is_valid(r, c) or self.is_obstacle(r, c):
                    continue
                if self.is_obstacle(row + dr, col) or self.is_obstacle(row, col + dc):
                    continue
                neighbors.append(((r, c), DIAGONAL_COST * self.cost[r][c]))

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