import math

from nav.config import GRID_SIZE

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


    def clear(self):
        for row in range(GRID_SIZE):
            for col in range(GRID_SIZE):
                self.cells[row][col] = Grid.FREE
        self.start = None
        self.goal = None
    

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
        Cardinal steps always cost 1. Diagonal steps (only when
        self.diagonal is on) cost sqrt(2), and are skipped if either
        of the two orthogonal cells they'd cut past is an obstacle --
        otherwise a "diagonal" move could clip straight through a
        solid wall corner.
        """
        neighbors = []
        for dr, dc in CARDINAL_DIRS:
            r, c = row + dr, col + dc
            if self.is_valid(r, c) and not self.is_obstacle(r, c):
                neighbors.append(((r, c), 1))

        if self.diagonal:
            for dr, dc in DIAGONAL_DIRS:
                r, c = row + dr, col + dc
                if not self.is_valid(r, c) or self.is_obstacle(r, c):
                    continue
                if self.is_obstacle(row + dr, col) or self.is_obstacle(row, col + dc):
                    continue
                neighbors.append(((r, c), DIAGONAL_COST))

        return neighbors