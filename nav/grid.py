import pygame
import sys
from nav.config import GRID_SIZE


class Grid:
    FREE = 0
    OBSTACLE = 1


    def __init__(self):
        self.cells = []
        for _ in range(GRID_SIZE):
            row = []
            for _ in range(GRID_SIZE):
                row.append(Grid.FREE)
            self.cells.append(row)

    
    def toggle(self, row, col):
        if self.is_valid(row, col):
            if self.cells[row][col] == Grid.FREE:
                self.cells[row][col] = Grid.OBSTACLE
            else:
                self.cells[row][col] = Grid.FREE
    

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