from nav.config import GRID_SIZE


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
    

    def clear(self):
        for row in range(GRID_SIZE):
            for col in range(GRID_SIZE):
                self.cells[row][col] = Grid.FREE
    

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
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        neighbors = []
        for dr, dc in directions:
            r = row + dr
            c = col + dc
            if self.is_valid(r, c) and not self.is_obstacle(r, c):
                neighbors.append((r, c))
        return neighbors