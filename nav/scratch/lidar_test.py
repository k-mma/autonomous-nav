from nav.algorithms import find_path
from nav.grid import Grid
from nav.sensor import LidarSensor, KnownGrid

START, GOAL = (0, 0), (0, 9)
# Hidden from the robot until it gets close enough to sense them.
HIDDEN_OBSTACLES = [(0, 4), (1, 4), (2, 4)]
RADIUS = 3
MAX_STEPS = 30


def build_grid():
    grid = Grid()
    for row, col in HIDDEN_OBSTACLES:
        grid.cells[row][col] = Grid.OBSTACLE
    grid.place_start(*START)
    grid.place_goal(*GOAL)
    return grid


if __name__ == "__main__":
    grid = build_grid()
    sensor = LidarSensor(RADIUS)

    pos = START
    seen = sensor.sense(grid, pos)
    print(f"start at {pos}, sensed: {seen or 'nothing'}")

    known = KnownGrid(sensor.known_obstacles)
    path, _, reason, _ = find_path(known, "astar", pos, GOAL)
    print(f"initial plan on known-only map: {path} (reason={reason})\n")

    step_index = 1
    for step in range(MAX_STEPS):
        if pos == GOAL:
            print("reached goal")
            break
        if path is None or step_index >= len(path):
            print("stuck -- no known path to goal")
            break

        pos = path[step_index]
        step_index += 1
        newly_seen = sensor.sense(grid, pos)

        if not newly_seen:
            print(f"step {step}: moved to {pos}, nothing new")
            continue

        print(f"step {step}: moved to {pos}, newly sensed {newly_seen}")
        remaining = set(path[step_index:])
        if newly_seen & remaining:
            print("  new obstacle on planned route -- replanning")
            known = KnownGrid(sensor.known_obstacles)
            path, _, reason, _ = find_path(known, "astar", pos, GOAL)
            step_index = 1
            print(f"  new plan: {path} (reason={reason})")
        else:
            print("  new obstacle is off the current route, no replan needed")

    print(f"\nfinal position: {pos}")
    print(f"known obstacles at the end: {sensor.known_obstacles}")
    print(f"ground truth obstacles:     {set(HIDDEN_OBSTACLES)}")
