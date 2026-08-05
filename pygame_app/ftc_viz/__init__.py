"""
Pygame-only rendering for ftc/ match traces (ftc/trace.py) -- the
RoadRunner-style "robot moving across the field" demo. Lives under
pygame_app/ alongside every other pygame-specific module (see
README.md's "Repo layout": "Pygame-only files live under pygame_app/"),
not under ftc/ itself, since drawing is a presentation concern
completely separate from the simulation ftc/match.py and ftc/trace.py
own -- nothing in ftc/ imports pygame, and nothing here re-implements
any match logic, it only reads already-computed MatchTrace snapshots.
"""
