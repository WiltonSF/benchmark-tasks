# Wind a torsion spring three quarter turns, passing the cone from hand to hand, and pin it

A spring winder stands on the table: a 27 cm winding cone on a fixed base, pulled back to its stop by a torsion
spring, with four upright bars (one blue, three orange) a quarter turn apart near its rim. A fixed hub covers the disc inside 7 cm, and two
fixed bridges cross over the rim, the near bridge on the robot's side and the far bridge opposite, 8 mm above the bar
tops. A gripper holding a bar cannot pass
under a bridge, so each arm can follow the cone only on its own side, at most about 140 degrees. Three quarter turns
therefore need at least one handover, and the demo makes three: one arm holds its bar while the other takes the bar
on its side, then the first lets go. A red set pin in the near bridge, pushed down, latches and stands in the bars'
path, so the spring rests a bar on it at each quarter turn.

The instruction does not say how to keep the cone from running back. A cone left unheld runs back to its stop: in the
failure replay below it fell from 52 degrees to 0 in about a second.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena, run through Inspect Robots), on the `station` scene. It uses `parts.py` (in this folder) to build the
winder: the cone is a hinge with a spring, the pin a spring slider whose catch is an `on_step` rule. `near_misses.py`
lists near-successes, wrong outcomes and fixed habits with the score each must get; it runs without a simulator.

## Run a trial

From this folder, on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Wind the spring three quarter turns and lock it with the red set pin."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames

# Scripted demo (ends at 1.0 after about 30 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames
```

The run prints `task_score`: 1.0, 0.5, 0.25 or 0.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 2 minutes of wall time on an
  L40S once Isaac Sim has started.
- **Seeds:** none needed. Submissions run with `-E object_jitter=0`; the winder is fixed and nothing else is loose.

## Scoring details

After every control step an `on_step` callback reads the cone's angle and the pin's travel and keeps a record over
the whole trial:

- **Run-back:** the most the cone has been wound so far. If the cone is ever more than 30 degrees below that, the
  trial scores 0 from then on. This counts any turn back, slow or fast. The record is cleared only by a scene reset
  (`on_reset`).
- **Pin:** latched once pushed 14 mm or more (the callback then moves the pin spring's target past its 17 mm stop,
  so it stays down); freed if pulled back above 10 mm. Unlatched, the spring holds it up against its top stop.
- **Released:** the robot's `gripper`, `tip_left` and `tip_right` collision hulls (`yam_arena.analysis`) against
  points on all four bars, the disc's open top and rim, and the pin's knob. The plane distance used is a lower bound on the true distance, so the
  check errs toward "touching". Under 3 cm counts as touching.
- **Still:** the cone turned less than 0.05 degrees since the last step (1 degree per second).

`score()`: 0 after a run-back; otherwise, with the pin latched, the robot clear and the cone still, 1.0 at
268 +- 3 degrees, 0.5 at 178 +- 3, 0.25 at 88 +- 3, and 0 at any other angle. These are the lock angles: with the
pin down and nothing holding the cone, it rests with a bar on the pin at 88, 178 or 268 degrees (or on its stop at
0). Success
(`score() == 1.0`) must hold for 3 s (`SUCCESS_HOLD_S`). A 1e-6 margin applies to every limit.

Offline (`near_misses.py`) each case is one step of a trial, run in order, so the run-back record carries from case
to case; a case at the start state begins a new trial. The release check is skipped there and counts as clear.

## Fixed habits, and why each fails

The layout is fixed, so each ordinary habit was checked against it (each is a case in `near_misses.py`):

- **One arm only, regrasping:** each arm can follow at most about 140 degrees; letting go to take a new grip lets
  the cone run back (score 0). GPU replay below.
- **Pin first:** pressing the pin before winding locks the cone at 0 (score 0) and blocks winding.
- **Stop at the first or second lock:** 0.25 or 0.5, not 1.0. Each is a quarter or half wound and safely pinned.
- **Turn the cone by pressing on the disc instead of a bar:** this also holds the cone (GPU replay below), but the
  pressing fingers cannot pass under a bridge any more than a grip on a bar can, and the hub closes the middle, so
  it gives no way round the handover; it is another way of holding the load.
- **Wind to the end stop, then pin:** the 300 degree stop is 32 degrees past the 268 degree lock, so letting it come
  back onto the pin is a run-back (score 0).
- **Either arm first, or any bar:** all four bars are alike for winding and either arm may start; only the turn
  angle is scored, so no choice of bar or arm is rewarded by itself.

A memorised replay of the one fixed layout cannot be ruled out without seeded layouts.

## Physics, as measured on the GPU

- **Demo (trace before the hub was added, and the final demo with it, both 1.0):** in the trace the cone was wound to 281 degrees through three handovers,
  each giving back 1 to 3 degrees. After the pin went down the right arm eased back and the cone came to rest on the
  pin at 268 degrees. The score reached 1.0 at 29 s.
- **Dream-1 (1,200 steps):** both arms moved toward the winder but never took a bar; the cone did not turn. Score 0.
- **Let-go replay** (`SPRING_DEMO=letgo`, the same command with the scripted policy): the left arm wound the cone to
  52 degrees and opened its fingers to take a new grip. The cone ran back to its stop at 0 within about 1.2 s, so
  the run-back rule zeroed the trial. Score 0.
- **Press replay** (`SPRING_DEMO=press`): the left arm's closed fingers pressed 4 mm into the disc top 10 cm out
  and dragged it round; the cone turned to 63 degrees and stayed there while pressed. The fingers cannot follow it
  past a bridge. Score 0 (the pin was never pressed).
- Earlier traces found the build problems this layout fixes: the pin sagged 6 mm under its own weight into the bars'
  path (now held up by its spring), and a gripper closing across the bar's path's radius let it slip 9 degrees
  (the grips now close along the path).

## Scripted demo

`demo()` uses fixed waypoints only, worked out from the starting layout and the planned cone angle, with the
gripper pointing straight down and turned only about the vertical (its fingers close along the bar's path). Bar angles
here are the ones in task.json's setup. The left arm takes the bar at 118 degrees and carries it to 170. The right
arm takes the bar then at 260, the left lets go, and the right carries it to 330. The left takes the bar then at
60 and carries it to 170; the right takes the one at 260 and carries it to 310, 282 degrees of wind. The left
arm presses the pin down and moves away. The right arm eases its bar back 20 degrees, so a bar comes to rest on the
pin, then lets go and moves away. The right arm's grips are aimed 8 mm toward the robot, an offset measured in a GPU
trace.
