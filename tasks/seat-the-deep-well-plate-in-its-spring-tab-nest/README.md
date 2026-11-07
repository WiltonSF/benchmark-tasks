# Seat a deep-well plate in a spring-tab deck nest

A liquid-handler deck nest holds its plate with a spring tab at the far end that pushes the plate against a datum
wall at the near end. At rest the tab leaves the pocket 4 mm shorter than the plate, so a plate lowered straight down
lands on the tab. The robot uses the plate's own far end to push the tab back, lowers the plate into the pocket, and
lets go; the tab then pushes it home against the datum wall.

Built for yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), station scene. `parts.py` (the
parts kit: multi-colour parts with hinges and sliders) and `lots.py` (layout lots) must sit next to `task.py`;
`task.py` imports both.

## Lots

Three starting places for the plate; the nest never moves. Lot A (the screenshot): long side front to back, centred
20.0 cm in front of the arm bases and 21.0 cm right of the midline, to the right of the nest. Lot B: long side front to
back, centred 18.0 cm in front and 2.0 cm right, on the other side of the nest. Lot C: turned a quarter turn (long side
left to right), centred 16.5 cm in front and 22.5 cm right. Trial k uses lot k mod 3, lot A first, so a run needs
`--epochs 3` (or a multiple); `TASK_LOT=A`, `B` or `C` pins every trial to one lot. A replay of lot A's waypoints
grips nothing in lots B and C and averages at most 0.33. The lots stop a single fixed habit or replayed layout, not a
memorised replay of all three.

## Run and score a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Set the plate in the nest on the deck."

# Dream-1, all three lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 900 --epochs 3 --store-frames --log-dir logs/dream

# Scripted demo, all three lots
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 900 --epochs 3 --store-frames --log-dir logs/demo
```

- **Runtime per trial:** at most 45 s of simulated time (900 steps at 20 Hz); about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started, so about three times that for the three lots.
- **Seeds:** none needed. Submitted runs use `-E object_jitter=0`; the demo's aim correction (below) was measured in
  those layouts. With `--epochs 3` the videos and screenshot show lot A and the score is the mean of the three trials.

## Scoring as implemented

`score()` and `success()` in `task.py`, pass/fail, the same in every lot:

- **Flat on the floor:** all four corners of the plate's underside (both ends), computed from its centre, orientation
  and true size, within 1 mm of the nest floor. One end raised more than 1 mm (about 0.45 degrees) fails.
- **In the pocket:** its centre within 2.5 mm of the pocket's centre line (1.5 mm play plus 1 mm).
- **At the datum:** its near end (from its centre and true length) from 0.5 mm into to 1 mm off the datum wall's face.
- **Tab on the plate:** the tab's joint at least 2 mm from its stop.
- **Released:** an `on_step` callback measures, every control step, the distance from both arms' finger and gripper
  link meshes to the plate's true box; it must be more than 5 mm. Offline (no robot) this test always passes.
- **Still:** under 1 cm/s and 0.5 rad/s.

The trial ends early once `success()` has held for 2 s (`SUCCESS_HOLD_S`); the score is `score()` at the end, which
is 1.0 whenever the conditions above hold. The plate's A1 corner is not scored (the model plate has none).

The tab is a parts-kit slider with a 60 N/m spring whose rest point an `on_step` callback sets 25 mm past the stop
(a preload), so it presses toward the datum with about 1.5 N at its stop and 2.4 N at full travel, more than the
plate's friction on the floor.

## Measured in traces (scripted, object_jitter=0)

| Run | Result | Score |
|---|---|---|
| First demo: plate aimed at the pocket's centre line | The plate sat 7 mm to the robot's right of the aim and came to rest on the side wall | 0 |
| Plate released 6 mm above the floor | The tab pushed it against the datum but friction on the tab's face held it 3.6 mm above the floor | 0 |
| Released 2 mm above the floor (the earlier final demo) | Near end on the floor, far end's underside 1.3 mm up (0.6 degrees): passed the old 2-degree tilt test, fails the new 1 mm test | 0 now |
| Lowered onto the floor before opening, lots A, B, C | Lot A seated flat (corners 0 to 0.4 mm up); in lots B and C the open fingers, still 3 to 5 mm off their aim while coming down fast, landed on the plate's edge and tipped it | 1, 0, 0 |
| With a settle pause 30 mm above the plate | The plate shifted less in the grip and came down on a side wall's top, 2.4 to 4.9 mm right of the aim | 0, 0, 0 |
| Final aim, 3.8 mm left of the centre line (trace-20261107T170330) | Seated in every lot: corners 0 to 0.2 mm above the floor, near end at the datum, tab held back 4.0 mm; success began only once the fingers had opened and started to lift | 1, 1, 1 |

## Scripted demo

`demo()` drives the right arm from the trial's lot (the plate's start, which the cameras show): over the plate, stop
30 mm above grip height and settle, grip it across its 86 mm width 12 mm below its top (in lot C the grip is turned a
quarter turn to match, and turned back on the way to the nest), lift, carry it over the nest with its far end 5 mm
short of the tab, lower until its underside is 3 mm above the walls, move 26 mm toward the tab (it meets the tab after
5 mm and pushes it to its full 15 mm travel; the arm gives way for the rest), lower onto the floor (commanded 2 mm
below it), open and lift clear. Fixed waypoints per lot.

## Checks without a simulator

`near_misses.py` lists cases that `factory/check.py` scores with `task.py`: seated (1.0); 0.8 mm off the datum (1.0);
far end's underside 0.9 mm and exactly 1.0 mm up (1.0), 1.1 mm and 1.3 mm up (0); proud on the tab, near end on the
floor and far end 4.4 mm up (1.97 degrees, 0); 2 mm short of the datum, held 3 mm up, still moving, tilted 3 degrees,
wedged 3.6 mm up between tab and datum, lying on the walls' tops (0); and the fixed habits, each 0: lowered straight
down (far end on the tab), set down beside the nest, and left short of the datum with the tab still back.
