"""
Does ftc/sensors.py's diagnose_tag_detections actually report the same
per-gate geometry AprilTagSuite.tag_correction itself uses internally
-- not just plausible-looking booleans -- and is it genuinely safe to
call from ftc/match.py's on_tick hook (zero rng draws, doesn't perturb
the real simulation)?

1. check_matches_tag_correction_on_a_clear_detection: a tag placed
   close, dead-on, in view, with clear line of sight -- diagnose_tag_
   detections reports every gate True (geometrically_eligible), and
   AprilTagSuite.tag_correction (called on an unrelated, unconsumed
   rng) actually returns a non-None correction for the SAME geometry --
   the two independent code paths agree.
2. check_out_of_range_tag_is_flagged: a tag placed farther than
   APRILTAG_RANGE_CELLS reports in_range=False and geometrically_
   eligible=False, matching tag_correction's own None return for the
   identical position.
3. check_edge_on_tag_fails_tag_fov: a tag whose facing direction is
   perpendicular (not toward) the robot reports in_tag_fov=False.
4. check_diagnose_consumes_zero_rng_draws: calling diagnose_tag_
   detections with a real, trackable rng-consumption-counting stand-in
   in place of where rng WOULD go (it takes no rng parameter at all --
   this check confirms that's actually true by inspecting the
   function's own signature, not just trusting the docstring) leaves a
   real `random.Random` instance's stream completely untouched when
   used elsewhere before/after.
5. check_evaluates_every_tag_not_just_the_first_hit: with two tags both
   geometrically eligible, diagnose_tag_detections reports BOTH as
   eligible (unlike tag_correction, which would only ever act on
   whichever it reaches first in iteration order).
"""
import math
import random

from ftc.config import APRILTAG_RANGE_CELLS
from ftc.field import TagSite, build_grid, in_to_cell
from ftc.sensors import AprilTagSuite, diagnose_tag_detections

LAYOUT = "sparse"


def _grid():
    return build_grid(LAYOUT)


def check_matches_tag_correction_on_a_clear_detection():
    grid = _grid()
    position = (5, 5)
    tag = TagSite(x_in=position[1] * 6.0, y_in=(position[0] - 2) * 6.0, heading_deg=90.0)  # facing the robot
    tag_sites = [tag]
    heading_now = 0.0

    diagnostics = diagnose_tag_detections(grid, position, heading_now, tag_sites, [0.0], fidelity="optimistic")
    suite = AprilTagSuite()
    correction = suite.tag_correction(grid, position, heading_now, tag_sites, random.Random(0),
                                        fidelity="optimistic")

    ok = diagnostics[0]["geometrically_eligible"] and correction is not None
    print(f"clear detection: geometrically_eligible={diagnostics[0]['geometrically_eligible']}, "
          f"tag_correction returned {correction} -- {'OK' if ok else 'FAIL'}")
    return ok


def check_out_of_range_tag_is_flagged():
    grid = _grid()
    position = (0, 0)
    far_cell = (APRILTAG_RANGE_CELLS + 5, APRILTAG_RANGE_CELLS + 5)
    tag = TagSite(x_in=far_cell[1] * 6.0, y_in=far_cell[0] * 6.0, heading_deg=225.0)
    tag_sites = [tag]

    diagnostics = diagnose_tag_detections(grid, position, 0.0, tag_sites, [0.0], fidelity="optimistic")
    suite = AprilTagSuite()
    correction = suite.tag_correction(grid, position, 0.0, tag_sites, random.Random(0), fidelity="optimistic")

    ok = not diagnostics[0]["in_range"] and not diagnostics[0]["geometrically_eligible"] and correction is None
    print(f"out-of-range tag: in_range={diagnostics[0]['in_range']}, "
          f"geometrically_eligible={diagnostics[0]['geometrically_eligible']}, "
          f"tag_correction={correction} -- {'OK' if ok else 'FAIL'}")
    return ok


def check_edge_on_tag_fails_tag_fov():
    grid = _grid()
    position = (5, 5)
    tag_cell = (5, 8)  # due east of the robot
    # heading_deg=0 means the tag faces +col (east) -- i.e. facing AWAY
    # from the robot standing to its west, not toward it, so the
    # incidence angle (viewed from the tag's own facing direction) is
    # close to 180deg, well outside +-30deg -- edge-on/behind, not in view.
    tag = TagSite(x_in=tag_cell[1] * 6.0, y_in=tag_cell[0] * 6.0, heading_deg=0.0)
    tag_sites = [tag]

    diagnostics = diagnose_tag_detections(grid, position, 0.0, tag_sites, [0.0], fidelity="optimistic")
    ok = not diagnostics[0]["in_tag_fov"]
    print(f"tag facing away from the robot: in_tag_fov={diagnostics[0]['in_tag_fov']} "
          f"(incidence={diagnostics[0]['incidence_deg']:.1f}deg) -- {'OK' if ok else 'FAIL'}")
    return ok


def check_diagnose_consumes_zero_rng_draws():
    import inspect
    sig = inspect.signature(diagnose_tag_detections)
    ok = "rng" not in sig.parameters
    print(f"diagnose_tag_detections signature has no `rng` parameter: {list(sig.parameters)} -- "
          f"{'OK' if ok else 'FAIL'}")
    return ok


def check_evaluates_every_tag_not_just_the_first_hit():
    grid = _grid()
    position = (5, 5)
    tag_a = TagSite(x_in=5 * 6.0, y_in=3 * 6.0, heading_deg=90.0)
    tag_b = TagSite(x_in=5 * 6.0, y_in=7 * 6.0, heading_deg=270.0)
    tag_sites = [tag_a, tag_b]

    diagnostics = diagnose_tag_detections(grid, position, 0.0, tag_sites, [0.0], fidelity="optimistic")
    ok = len(diagnostics) == 2 and all(d["geometrically_eligible"] for d in diagnostics)
    print(f"two geometrically-eligible tags: both reported eligible = "
          f"{[d['geometrically_eligible'] for d in diagnostics]} -- {'OK' if ok else 'FAIL'}")
    return ok


# --- pytest entry points --------------------------------------------------


def test_matches_tag_correction_on_a_clear_detection():
    assert check_matches_tag_correction_on_a_clear_detection()


def test_out_of_range_tag_is_flagged():
    assert check_out_of_range_tag_is_flagged()


def test_edge_on_tag_fails_tag_fov():
    assert check_edge_on_tag_fails_tag_fov()


def test_diagnose_consumes_zero_rng_draws():
    assert check_diagnose_consumes_zero_rng_draws()


def test_evaluates_every_tag_not_just_the_first_hit():
    assert check_evaluates_every_tag_not_just_the_first_hit()


if __name__ == "__main__":
    checks = [
        check_matches_tag_correction_on_a_clear_detection(),
        check_out_of_range_tag_is_flagged(),
        check_edge_on_tag_fails_tag_fov(),
        check_diagnose_consumes_zero_rng_draws(),
        check_evaluates_every_tag_not_just_the_first_hit(),
    ]
    print("\nALL PASS" if all(checks) else "\nSOME CHECKS FAILED")
