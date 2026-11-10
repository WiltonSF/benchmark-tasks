# Load a folding-leg stretcher into a raised bay

A model ambulance stretcher stands on two locked leg pairs in front of a raised loading bay. Pushed straight in, its
front legs meet the bay's sill and it stops. The head end has to go in over the bay floor first, then the front legs
are released so the head end settles on its skids, then the stretcher is pushed on until the rear legs near the sill,
the foot end is held up while the rear legs are released, and it is pushed home and let go. A leg pair released
before its end is carried drops that end to the table, below the sill. The instruction does not say how.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts with hinges and sliders) and `lots.py`
(layout lots), both in this folder; they import only NumPy and `yam_arena`. `near_misses.py` lists near-successes and
wrong outcomes with the score each must get; they are plain-Python checks of `score()` and `success()` and need no
simulator.

## Lots

Two layouts. Lot A (the screenshot): the bay's sill 34.0 cm in front of the arm bases. Lot B: 37.0 cm. The stretcher
starts in the same place in both. Trial k uses lot k mod 2, lot A first, so a run needs `--epochs 2` (or a multiple);
`TASK_LOT=A` or `TASK_LOT=B` pins every trial to one lot. In lot B the head end is 45 mm short of the sill at the
start, so releasing the front legs where lot A's demo does (after 46 mm of push, less the arm's lag) drops the head
end before it is over the floor. Lot A's tab positions are 30 mm off in lot B, so a replay of either lot's waypoints in
the other presses beside or short of the right moment. This was reasoned from the geometry, not run on the GPU. The
lots stop a single fixed release point, not a memorised replay of both layouts. The bay is a loose 3 kg part only so
the lots can move it.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Load the stretcher into the raised bay and let go of it."

# Dream-1, both lots
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 2 --store-frames --log-dir ~/yam-eval/stretcher-dream

# Scripted demo, both lots (each reaches 1.0 in about 31 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 800 --epochs 2 --store-frames --log-dir ~/yam-eval/stretcher-demo
```

The run prints `task_score`, the mean over the trials. Scoring is `score()` and `success()` in `task.py`, described
in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started; a scored run is two trials.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## The mechanism, as the simulator runs it

- Each leg pair is one hinged link, turning about the deck's width at the deck's underside, from 0 (straight down)
  to 90 degrees (flat under the deck, its feet toward the foot end). Each tab is a sprung slider, 8 mm of travel.
- A `parts.on_step` callback sets each pair's joint limits every step: locked at 0 to 0.005 rad until its tab is
  pressed 4 mm or more; then free (0 to 90 degrees) until the pair reaches 75 degrees; from then on the catch holds it
  between 75 and 90 degrees for the rest of the trial. A spring (0.15 N m/rad, rest point past the 90-degree stop) and
  damping (0.45 N m s/rad) swing a freed pair up in about 3 s when nothing holds it down.
- `parts.on_reset` puts the stretcher back on its legs at its start and relocks both pairs before each trial (limits
  from the last trial would otherwise hold a folded pair up).

## Scoring details the simulator applies

- "Still" means the stretcher moves under 1 cm/s and turns under 0.5 rad/s (`ObjectState.resting()`) and no joint
  moved more than 0.005 rad or m in the last control step. Success must hold 1 s.
- **Let go:** the callback measures the shortest distance from both arms' finger and gripper collision hulls
  (`yam_arena.analysis`) to every shape of the stretcher, legs and tabs included; it must exceed 5 mm. Offline
  (`near_misses.py`) there is no robot, so this counts as clear.
- **On the bay floor:** the skids' bottom corners, in the bay's own frame, between 3 mm below and 3 mm above the
  floor, past the sill, short of the back stop and between the rails.
- **Past the sill:** the deck's end edge (its nearest corner) at least 20 mm past the sill line, in the bay's frame.
- **Fault record (whole trial):** the score is 0 for the rest of the trial if at any step the deck tilts more
  than 12 degrees (an end dropped to the table tilts it well past that; the demo peaked at 4.3 degrees in lot A and 2.6 in lot B on the GPU), even if the end is lifted back later, or its lowest point is more than 10 mm below the table top, or the bay moves more than 10 mm from its
  lot position or tilts more than 5 degrees. The record is cleared only by a scene reset: the stretcher back within
  0.5 mm of its start and at rest, after a jump of over 20 mm in one control step.

## Scripted demo

`demo()` uses only the layout the cameras show (the bay's place in the trial's lot) and fixed waypoints. The right
arm grips the handle's cross bar with the fingers closing front to back (so the pads hold the stretcher's pitch) and
pushes the head end in until the front legs are 5 mm short of the sill; the left arm comes in from the side and
presses the front tab and moves well clear, so the overview camera sees the front legs swing up and the head end
settle on the bay floor. The right arm pushes on until
the rear legs are 8 mm short of the sill and holds the foot end; the left arm presses the rear tab and moves clear while
the rear legs swing up. The right arm lifts the foot end 12 mm, pushes the stretcher home, lowers it and lets go. Its targets allow for
the 9 to 11 mm the stretcher lags behind the pushing arm.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases include: loaded with the foot end exactly 20 mm and 19 mm past the sill (1.0 and 0.5), a rear pair at
74 and 75 degrees, the stretcher still sliding, held 4 mm above the floor, pushed in on standing legs (0), the head
end in with the front pair folded (0.5), only 17 mm past the sill (0), both pairs released early on the table (0),
the head end in but the foot end dropped to the table (0), an end dropped and then lifted back and loaded (0:
the drop is judged over the whole trial), a pitch of 11.9, exactly 12 and 12.1 degrees on the way followed by a full
load (1.0, 1.0, 0), and the bay shoved 9 mm and 11 mm, and a 31-degree tip (faults that stay until a scene reset).
