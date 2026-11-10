# Pack six books in the case so its lid shuts

Six hardback books, each thicker along its spine than along its fore-edge, go into a book case with a hinged lid.
One or two are already in the case; the rest lie on the table with their spines toward the robot. The instruction
is only "pack them flat and close the lid". A stack of such books only stays level when the spines are shared
between the two sides of the case, three and three; with four or more on one side the stack leans and the lid comes
down on the high corner. Book handlers stack hardbacks spine to fore-edge for exactly this reason. The rule is not
stated.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts and the hinged lid) and `lots.py` (layout
lots), both in this folder; they import only NumPy and `yam_arena`. `near_misses.py` lists near-successes and wrong
outcomes with the score each must get; they are plain-Python checks of `score()` and `success()` and need no
simulator.

## Lots

There are three layouts, which differ only in what is already in the case:

- Lot A (the screenshot): the red book lies in the case, spine to the robot's left; five books on the table.
- Lot B: the red book's spine is to the robot's right.
- Lot C: the red book as in lot A, with the teal book on it, also spine left (the pair leans about 15 degrees);
  four books on the table.

Trial k uses lot k mod 3, lot A first, so a scored run needs `--epochs 3` (or a multiple); `TASK_LOT=A`, `B` or `C`
pins every trial to one lot. Each lot needs a different set of turns: lot A needs two more spines left and three
right, lot B three left and two right, lot C one left and three right. A fixed habit fails at least one lot: turning
every book the same way fails all three; replaying lot A's turns fails B and C; "alternate with the book on top"
passes A and B but fails C (it ends four spines left, two right). The lots stop a single fixed habit, not a memorised
replay of all three layouts.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Pack all six books flat in the case and close its lid."

# Dream-1, all three lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1600 --epochs 3 --store-frames --log-dir ~/yam-eval/books-dream

# Scripted demo, all three lots
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1600 --epochs 3 --store-frames --log-dir ~/yam-eval/books-demo
```

The run prints `task_score`, the mean over the trials. Scoring is `score()` and `success()` in `task.py`, described
in words in `task.json`.

- **Runtime per trial:** at most 80 s of simulated time (1,600 steps at 20 Hz), about 2 to 4 minutes of wall time
  on an L40S once Isaac Sim has started; a scored run is three trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## Scoring details the simulator applies

- **Inside the case:** every corner of each book's four boxes (its true outline, not a bounding box) lies between
  the inner faces of the two end rails and within the floor's width, each with 1 mm tolerance, and no lower than
  1 mm below the floor's top.
- **Below the lid:** no corner higher than the lid's underside when shut (114 mm above the case floor).
- **Level top:** the book whose centre is highest has its covers within 4 degrees of level.
- **Flat:** every book's covers within 30 degrees of level (a book on edge or end fails; the tallest balanced stack's
  books lean at most 23 degrees).
- **Lid shut:** the hinge within 1.5 degrees of its closed stop.
- **Let go:** a `parts.on_step` callback reads both arms' finger, gripper and wrist (link4, link5) poses every step and measures the
  shortest distance from their collision hulls (yam_arena.analysis) to every shape of every book and of the lid; it
  must exceed 5 mm. Offline (`near_misses.py`) there is no robot, so this check counts as clear.
- **Still:** every book moving under 1 cm/s and turning under 0.5 rad/s (`ObjectState.resting()`); success must
  hold 1 s.
- `score()` returns 1.0 for success, 0.5 for a packed, level, let-go stack under an open lid at the last step (books
  moving under 1 cm/s; the 1 s hold applies only to success), otherwise 0.

## The physics the task relies on

Each book is a 12 mm slab on two strips along its long edges, 10 mm deep under the spine and 2 mm under the
fore-edge, so it lies 7.7 degrees off level on a flat surface, which shows in the videos: the black spine strip
under one edge and the cream page-edge strip under the other. A book laid on another tilts by the one below's lean
plus its own, so the top of a stack leans 7.7 degrees times (spines left minus spines right). Only three and three
gives a level top. Offline, over all 64 spine orders, every three-and-three stack's highest point is at most 111 mm
above the case floor and fits under the 114 mm lid; every four-and-two stack reaches 116 mm or more (placed centred) and holds the lid
open, and every one has a top book leaning about 15 degrees, which the level check fails.

## Demo

`demo()` reads only the lot's layout (which the cameras show). The left arm takes each table book across its width
with the fingers along the book (gripper turned 90 degrees one way or the other), turns it a quarter turn so its
spine goes to the side the count needs, carries it in over the case's open left side and lets it drop from about
6 mm. The right arm then tips the upright lid past its balance point and the lid falls shut under its own weight.
The demo does not use `tilt.py`.
