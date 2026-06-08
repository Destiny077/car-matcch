# Straight Line Test

Purpose: isolate chassis hardware/software matching before running the full competition route.

Burn file:

- `D:\amend car\2026-06-08_straight_line_test\2026v1.bin`

Expected behavior:

- Robot initializes only the chassis.
- No vision task runs.
- No arm or paw action runs.
- Left motors M1/M3 run forward at speed `11`, right motors M2/M4 run forward at speed `12`, for about `5s`.
- This version compensates the observed right drift from the first straight test.
- Then all motors stop.
- Screen shows encoder values:
  - line 1: M1
  - line 3: M2
  - line 5: M3
  - line 7: M4

What to record after burning:

- Does the robot move physically forward, backward, sideways, rotate, or drift?
- Which direction does the robot drift: left or right?
- Are all four wheel encoder values changing?
- Are any encoder values negative while the wheel is physically moving forward?
- Are any encoder values much larger or smaller than the others?
- Does any wheel spin in the opposite direction?

Do not use this version for the full competition route. It is only for chassis calibration.
