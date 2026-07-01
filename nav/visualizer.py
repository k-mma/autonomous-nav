import sys
import pygame

from nav.config import GRID_SIZE, CELL_SIZE, WINDOW_SIZE, WHITE, BLACK, GRAY
from nav.grid import Grid


def draw_grid(screen, grid):
    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            if grid.is_obstacle(row, col):
                color = BLACK
            else:
                color = WHITE
            rect = pygame.Rect(col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, GRAY, rect, 1)


def handle_click(grid, pos):
    x, y = pos
    col = x // CELL_SIZE
    row = y // CELL_SIZE
    grid.toggle(row, col)


def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
    pygame.display.set_caption("Autonomous Nav")
    clock = pygame.time.Clock()

    grid = Grid()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                handle_click(grid, event.pos)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c:
                    grid.clear()

        screen.fill(WHITE)
        draw_grid(screen, grid)
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()