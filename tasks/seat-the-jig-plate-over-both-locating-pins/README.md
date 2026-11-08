# Seat a jig plate on its round and diamond locating pins, the one way round it fits

A steel fixture bar carries two blue locating pins 90 mm apart: a round pin 20 mm across and a diamond pin 10 mm
across the pins' line and 8 mm along it. A loose 14 x 7 cm aluminium jig plate, 8 mm thick, lies on a riser to the
robot's right. It has a 26 mm hole and a 16 mm hole, 90 mm apart and 10 mm off the plate's middle. The robot must
lower the plate so the round pin goes through the large hole and the diamond pin through the small one, and the plate
lies flat on the bar. The round pin cannot enter the small hole, so the plate fits only one way round; turned end for
end it rests on the round pin's top.

The fixture's way round and the plate's way round on its riser change independently between four layout lots (trial
k uses lot k mod 4), so the plate's start does not tell which way to turn it. The plate lies 15 degrees off the bar's
direction, so seating it takes either a short clockwise turn (15 degrees) or a long anticlockwise one (165 degrees,
seen from above):

| Lot | Fixture | Plate on the riser | Turn that seats it |
|---|---|---|---|
| A (screenshot) | centred 32 cm out on the midline, round pin on the left | long sides 15 degrees anticlockwise from side to side, small hole at the left end | 165 degrees anticlockwise |
| B | turned end for end (round pin on the right), 8 mm further out | as A | 15 degrees clockwise |
| C | as A, 7 mm nearer the robot and 12 mm left | turned end for end (small hole at the right end) | 15 degrees clockwise |
| D | turned end for end, 5 mm further out and 10 mm right | as C | 165 degrees anticlockwise |

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena, run through Inspect Robots), on the `station` scene, with `parts.py` (the plate and fixture built from
several shapes) and `lots.py` (layout lots) next to it. `near_misses.py` lists near-successes and wrong outcomes with
the score each must get; it runs without a simulator.

## Run a trial

From this folder, on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Put the jig plate on the fixture over both locating pins."

# Dream-1, all four lots (the score is the mean)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 900 --epochs 4 --store-frames

# Scripted demo, all four lots
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 900 --epochs 4 --store-frames
```

Put `TASK_LOT=A`, `B`, `C` or `D` in front to run every trial in one lot. Without `--epochs 4` only lot A runs, and
a replay of lot A's motion scores 1.0 there, so always run all four lots.

- **Runtime per trial:** at most 45 s of simulated time (900 steps at 20 Hz); the demo ends at 1.0 after about 13 s.
  About 1-2 minutes of wall time per trial on an L40S once Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0`, so every object starts exactly where its lot puts it.

## Scoring details

`success()` reads the plate's and the fixture's poses:

- **Flat on the bar:** the four lowest of the plate's eight corners (from its true size and orientation) are each
  within 1.5 mm of the bar's top.
- **Pins through:** where the round pin's vertical axis meets the plate's mid-plane lies at most 3 mm (the pin's
  10 mm radius inside the 13 mm half-width) from the large hole's centre along each of the plate's axes; and the
  vertical lines through the diamond pin's four edges all meet the mid-plane inside the small hole. The plate counts
  either face up (holes go through), but not turned end for end.
- **Fixture never moved:** checked every step against where the trial's lot put it (5 mm, 3 degrees of turn or
  tilt). The fixture is a 5 kg loose part only so lots can move it; on a real bench it is clamped. The record is
  cleared at every reset.
- **Released:** after every control step an `on_step` callback places both arms' `gripper`, `tip_left` and
  `tip_right` collision hulls and measures their distance to points on the plate's faces; under 5 mm counts as
  touching. Offline (`near_misses.py`) it counts as released.
- **Still:** `resting()` (under 1 cm/s and 0.5 rad/s). Success must hold for 3 s; `score()` is 1.0 then, else 0.

Fixed habits, checked offline with `lots.use()` (`python near_misses.py` runs them after the near misses; each must
average 0.5 or less). Each habit puts the plate flat on the lot's pins with its holes' midpoint centred, so only the
way round decides:

| Habit | A | B | C | D | Mean |
|---|---|---|---|---|---|
| always the lot-A way round (small hole to the right) | 1 | 0 | 1 | 0 | 0.50 |
| turn anticlockwise until along the bar | 1 | 0 | 0 | 1 | 0.50 |
| turn clockwise until along the bar | 0 | 1 | 1 | 0 | 0.50 |
| replay lot A's final pose | 1 | 0 | 0 | 0 | 0.25 |
| replay lot A's motion (same turn, same place) | 1 | 0 | 0 | 0 | 0.25 |
| the right way round (reading which pin is large) | 1 | 1 | 1 | 1 | 1.00 |

An earlier version had three lots in which a quarter turn anticlockwise always seated the plate, so that habit
scored 1.0; the lots above replace them.

## Scripted demo

`demo()` drives the right arm with the gripper straight down, turned only about the vertical. It reads the
fixture's place and turn and the plate's turn for the trial's lot (what the cameras show), grips the plate across
its width at its middle, closing 3 mm past its edges, lifts it, turns it so the large hole lies over the round pin,
lowers it to 7 mm above the pin tops, settles, lowers it until the pins are in the holes, lets go 6 mm above the bar
and moves away. Of the two grips across the width it takes the one that ends at the same wrist turn over the pins
(the right wrist's last joint turns only 120 degrees each way, which is why the plate starts 15 degrees off the bar
rather than square across it). Its target leads 2.5 mm away from the robot and 2.5 mm to its left, the held plate's
lag measured in a GPU trace.

## Measured on the GPU

- **Demo:** 1.0 in each of lots A, B, C and D. With an earlier 4 mm outward lead, lot B's plate came down 3.2 mm too
  far out and rested on a pin top; with the present lead the plate seated within 0.4 mm of its centred pose in every
  lot (trace), seated about 10 s in and ending at 1.0 about 13 s in.
- **Dream-1 (900 steps per lot):** 0 in each of lots A, B, C and D. In lot A the right arm picked the plate up and held it
  by the bar's right-hand end, beside the pins, never over them.
