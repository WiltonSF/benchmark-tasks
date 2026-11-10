# Push the cooker back so its rear foot goes under the anti-tip bracket

A small freestanding cooker stands out from a wall, too wide to grip. At the foot of the wall stands a black
anti-tip bracket: a low hood over a tunnel open toward the robot, with a plate up the wall behind it. Freestanding
ranges must be fitted with one (UL 858): when the range is pushed home, one rear foot slides into the tunnel, so the
range cannot tip forward. The hood is hidden under the cooker by the time the foot reaches it, so the cooker has to
be lined up with it first, while it is still visible, and then pushed straight back. The instruction names the goal
(the rear foot nearest the bracket under its hood); the robot has to work out where the cooker must be to get it
there.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts) and `lots.py` (layout lots), both in
this folder; they import only NumPy and `yam_arena`. `near_misses.py` lists near-successes and wrong outcomes with
the score each must get; they are plain-Python checks of `score()` and `success()` and need no simulator.

## Lots

There are four layouts; the bracket and the cooker's start move between them (positions in cm, y to the robot's
left):

| Lot | Cooker start y, turn | Bracket centre line y | Foot that counts | Cooker centre y when lined up |
|---|---|---|---|---|
| A (screenshot) | -1, square | 12.5 | left rear | 4 |
| B | -2.5, square | -6.5 | right rear | 2 |
| C | -0.5, 5 deg anticlockwise | 5.5 | left rear | -3 |
| D | 0.5, 3 deg anticlockwise | -12.5 | right rear | -4 |

No lined-up position is the wall's centre (y 0) or any lot's start. Trial k uses lot k mod 4, lot A first, so a run
needs `--epochs 4` (or a multiple); `TASK_LOT=A` (B, C, D) pins every trial to one lot. Offline, with each habit's end
state put home against the wall, these fixed habits score (lots A, B, C, D): pushing straight back from the start 0,
0, 0, 0; centring the cooker on the wall 0, 0, 0, 0; always lining up the left rear foot with the bracket 1, 0, 1, 0
(0.5); always the right rear foot 0, 1, 0, 1 (0.5). Pushed straight back, the foot in lot C hits a side wall of the
bracket and the cooker stops short; in lots A, B and D it passes beside the bracket and the cooker goes home
unsecured. The lots stop a fixed habit, not a memorised replay of all four layouts. The bracket is a loose 3 kg part
only so the lots can move it.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Push the cooker back against the wall so the rear foot nearest the black bracket goes under its hood."

# Dream-1, all four lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 4 --store-frames --log-dir ~/yam-eval/cooker-dream

# Scripted demo, all four lots (each reaches 1.0 in about 28 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 700 --epochs 4 --store-frames --log-dir ~/yam-eval/cooker-demo
```

The run prints `task_score`, the mean over the trials. Scoring is `score()` and `success()` in `task.py`, described
in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started; a scored run is four trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## Scoring details the simulator applies

- "Still" means moving under 1 cm/s and turning under 0.5 rad/s (`ObjectState.resting()`); success must hold 4 s (so a trial ends with the arms clear of the cooker).
- **Standing and square:** the cooker's lean and its turn about the vertical both at most 3 degrees, and each foot's
  pad centre (from the cooker's pose) within 2 mm of the table.
- **Home:** the centre of the cooker's back face within 10 mm of the face of the bracket's wall plate (3 mm thick,
  1 mm off the wall), where the cooker's back stops.
- **Foot under the hood:** the centre of the rear foot on the bracket's side (left in lots A and C, right in B and D),
  taken in the bracket's own frame, within 15 mm of its centre front to back (its 3 cm depth) and within 6 mm of the
  tunnel's centre line. The pad fits the tunnel with 5 mm a side, so 6 mm allows for contact penetration.
- **Lined up (0.25):** standing and square, the cooker's centre at least 5 cm further from the robot than its lot's
  start, and that foot's centre within 5 mm of the tunnel's centre line.
- **Let go:** a `parts.on_step` callback reads both arms' finger and gripper link poses every step and measures the
  shortest distance from their collision hulls (yam_arena.analysis) to every shape of the cooker, feet included; it
  must exceed 5 mm. Offline (`near_misses.py`) there is no robot, so this check counts as clear.
- **Fault record (whole trial):** set, and the score is 0 for the rest of the trial, if at any step the cooker leans
  more than 15 degrees, or the bracket's centre is more than 5 mm (in the plane of the table) from its lot's place,
  or the bracket is turned or tilted more than 3 degrees. It is cleared only by a scene reset: the cooker back within
  0.5 mm of its start and at rest, after a jump of over 20 mm in one control step. A robot cannot move it that way.

## Scripted demo

`demo()` uses fixed waypoints computed from the lot's layout, which the top camera shows (the bracket is in view
past the cooker at the start); it reads nothing from the simulator. Both arms first drop straight down well outside the
cooker, then come round to its front with closed fingers, 10 cm above the table, never over it (the cooker is taller
than the arms' wrists clear). One arm (the left when the cooker must move right, else the right)
pushes the middle of the cooker's side face, fingertips 5.5 cm up, at 2 cm/s until the bracketed foot is in line with
the tunnel, then backs off; the other arm waits in front. Then both arms push the front face at 9.5 cm either side of
the line, at 3 cm/s, until the back meets the bracket's wall plate (commanded 1 cm past contact), withdraw 5 cm, rise and move aside to
where they came down. The
sideways push turns the cooker by up to about 10 degrees; the two-arm push squares it again before the foot reaches
the bracket. The two fingers' faces sit about 12.5 mm ahead of the TCP when pushing forward and about 39 mm to the
side when pushing sideways; both offsets were measured on the GPU and are constants in `task.py`.

## Measured in simulation

Trace on the stock stack, `--epochs 4` (trace-20261109T200305), then lot D again after its start turn was changed
(trace-20261109T201114): each lot scored 1.0. At the end the counted foot's centre was 4.6, 3.7, 3.8 and 4.2 mm off
the tunnel's centre line (lots A to D) and within 1 mm of the hood's centre front to back, the cooker square within
about 1 degree; the bracket moved under 0.5 mm. With lot D started turned 4 degrees clockwise, the sideways push
turned the cooker 17 degrees, the foot reached the bracket 7 mm off line and caught on its side wall 2 cm short of
the wall plate, and that trial scored 0. A 24 cm tall cooker was tried and dropped: the arms' wrists overhang the
fingertips and knocked it over 50 degrees round before any push.

## Other ways to succeed, and what scores 0

- **Also full score:** either arm or both, any push points, lifting the cooker over the bracket and setting it down
  with its foot in the tunnel, or walking it in corner by corner, as long as it ends installed, released and still.
- **Score 0:**
  - the cooker home against the wall with its foot beside or outside the bracket (looks installed, not secured), or
    the wrong rear foot in line;
  - the cooker stopped short with its pad against the bracket's side wall;
  - the cooker turned more than 3 degrees, lifted, leaning on the wall, still moving or still touched by a gripper;
  - the cooker tipped more than 15 degrees at any point, or the bracket dragged or turned, even if put back.

The cases in `near_misses.py` cover the 10 mm wall gap, the 6 mm centre-line limit, the 3 degree turn, the 5 cm and
5 mm limits of the lined-up tier on both sides, the straight push, the unsecured home position, the wrong foot, a
lifted and a leaning cooker, a sliding cooker, a tipped cooker and a dragged bracket (both followed by an installed
state that still scores 0), a bracket nudged 4 mm (no fault), and scene resets.

## Recorded runs

- Scripted demo, `--epochs 4`: 1.0 in each of lots A, B, C and D (task_score 1.0).
- Dream-1, 1,200 steps, `--epochs 4`: 0.0 in every lot (each trial ran to the step limit).
