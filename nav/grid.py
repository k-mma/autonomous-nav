import pygame
import sys


# CONFIGURATION

GRID_SIZE = 20
CELL_SIZE = 30          # Pixels per cell
WINDOW_SIZE = GRID_SIZE * CELL_SIZE

# Free cell
WHITE = (255, 255, 255)
# Obstacle
BLACK = (30, 30, 30)
# Grid lines
GRAY = (200, 200, 200)


# STATE

grid = []
for _ in range(GRID_SIZE):
    row = []
    for _ in range(GRID_SIZE):
        row.append(0)
    grid.append(row)


# FUNCTIONS

# Drawing the grid
def draw_grid(screen):
    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            if grid[row][col] == 1:
                color = BLACK
            else:
                color = WHITE
            rect = pygame.Rect(col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, GRAY, rect, 1)  # cell border


# Clear grid
def clear_grid(screen):
    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            grid[row][col] = 0

# Handling input (mouse clicks)
def handle_click(pos):
    x, y = pos
    col = x // CELL_SIZE
    row = y // CELL_SIZE
    if 0 <= row < GRID_SIZE and 0 <= col < GRID_SIZE:
        grid[row][col] = 1 - grid[row][col]  # toggle 0 <-> 1


# GAME LOOP

def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
    clock = pygame.time.Clock()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                handle_click(event.pos)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c:
                    clear_grid(screen)

        # Draw
        screen.fill(WHITE)
        draw_grid(screen)

        # # Clear screen
        # keys = pygame.key.get_pressed()
        # if keys[pygame.K_c]:
        #     clear_grid(screen)
        
        # flip() the display to put work on screen
        pygame.display.flip()
        # Caps framerate
        clock.tick(60)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()