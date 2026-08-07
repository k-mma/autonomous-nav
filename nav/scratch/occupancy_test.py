"""
Does nav/occupancy.py's OccupancyGrid actually behave like an occupancy
grid -- probability climbing toward 1 with repeated occupied
observations, dropping toward 0 with repeated free observations, never
moving backward under a run of consistent observations, and staying at
the prior for a cell nobody ever looked at?
"""
from nav.config import OCCUPANCY_PRIOR
from nav.occupancy import OccupancyGrid

SIZE = 10
TOLERANCE = 1e-6


def check_never_observed():
    grid = OccupancyGrid(SIZE)
    p = grid.probability(3, 3)
    ok = abs(p - OCCUPANCY_PRIOR) < TOLERANCE
    print(f"never-observed cell: probability={p:.4f} (prior={OCCUPANCY_PRIOR}) -- {'OK' if ok else 'FAIL'}")
    return ok


def check_repeated_occupied():
    grid = OccupancyGrid(SIZE)
    for _ in range(20):
        grid.observe_occupied(5, 5)
    p = grid.probability(5, 5)
    ok = p > 0.95
    print(f"20x observe_occupied: probability={p:.4f} -- {'OK' if ok else 'FAIL'}")
    return ok


def check_repeated_free():
    grid = OccupancyGrid(SIZE)
    for _ in range(20):
        grid.observe_free(5, 5)
    p = grid.probability(5, 5)
    ok = p < 0.05
    print(f"20x observe_free: probability={p:.4f} -- {'OK' if ok else 'FAIL'}")
    return ok


def check_monotonic_climb():
    """Each additional occupied observation should never make the
    estimate go down (and by symmetry, the same for observe_free) --
    log-odds are strictly additive per observation, so this is really a
    check that the log-odds <-> probability conversion is monotonic."""
    grid = OccupancyGrid(SIZE)
    prev = grid.probability(1, 1)
    ok = True
    for i in range(15):
        grid.observe_occupied(1, 1)
        p = grid.probability(1, 1)
        if p < prev - TOLERANCE:
            ok = False
            print(f"  step {i}: probability DROPPED ({prev:.4f} -> {p:.4f}) after observe_occupied")
        prev = p
    print(f"monotonic climb under repeated observe_occupied: {'OK' if ok else 'FAIL'}")
    return ok


def check_conflicting_observation_recovers():
    """A cell driven to near-certain-occupied should be pullable back
    down by enough contradicting observations -- the point of
    OCCUPANCY_LOGODDS_CLAMP (nav/config.py) is that a saturated belief
    isn't numerically stuck at 0.0/1.0. 30 observe_free calls is enough
    to walk the log-odds all the way from +CLAMP back down to -CLAMP
    given OCCUPANCY_LOGODDS_FREE's magnitude (see nav/config.py)."""
    grid = OccupancyGrid(SIZE)
    for _ in range(20):
        grid.observe_occupied(2, 2)
    high = grid.probability(2, 2)
    for _ in range(30):
        grid.observe_free(2, 2)
    low = grid.probability(2, 2)
    ok = high > 0.9 and low < 0.1
    print(f"saturate occupied then contradict with free: {high:.4f} -> {low:.4f} -- {'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------
# Thin wrappers so `pytest` collects and runs the checks above as real
# tests; the checks themselves (and the standalone `python3 <this file>`
# run below) are unchanged.


def test_never_observed():
    assert check_never_observed()


def test_repeated_occupied():
    assert check_repeated_occupied()


def test_repeated_free():
    assert check_repeated_free()


def test_monotonic_climb():
    assert check_monotonic_climb()


def test_conflicting_observation_recovers():
    assert check_conflicting_observation_recovers()


if __name__ == "__main__":
    checks = [
        check_never_observed(),
        check_repeated_occupied(),
        check_repeated_free(),
        check_monotonic_climb(),
        check_conflicting_observation_recovers(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
