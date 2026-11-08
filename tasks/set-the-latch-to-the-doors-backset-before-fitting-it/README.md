# Set each tubular latch to its door's backset before fitting it

Two doors lie flat side by side, each with an edge bore, a face bore and dark marks every 10 mm along its top. The
face bore is 70 mm in from the edge on one door and 60 mm on the other. Two identical adjustable tubular latches lie
in front of the doors, both set to 60 mm. The robot has to read each door's backset, slide the hub of the 70 mm
door's latch out to its 70 mm setting (it clicks there), push each latch home into its edge bore, and drop a spindle
through each hub. A latch fitted at the wrong setting leaves its hub 10 mm off the face bore, and the spindle cannot
pass.

There are two layouts (lots, `lots.py`): lot A has the 70 mm door on the left (the screenshot), lot B swaps the two
doors so the 70 mm door is on the right. Trial k of a run uses lot k mod 2, lot A first, so a run needs
`--epochs 2` and its `task_score` is the mean of the two. The doors are loose 3 kg bodies so a lot can move them;
scoring judges each door where its lot started it.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for
Isaac Lab-Arena), on the `station` scene. The doors, latches (with their sliding hubs) and holders are built with
`parts.py` (in this folder), and the lots by `lots.py`, which adds multi-shape parts, sliders and step callbacks to the task-file API at import
time; it uses only NumPy and the `yam_arena` package. `near_misses.py` lists near-successes, fixed habits and wrong
outcomes with the score each must get.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Fit a latch in each door edge bore at the length that door needs, then drop a spindle through each."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 900 --epochs 2 --store-frames --log-dir ~/yam-eval/latch-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 900 --epochs 2 --store-frames --log-dir ~/yam-eval/latch-demo
```

`--epochs 2` runs lot A then lot B; without it only lot A runs, where a fixed "extend the left latch" habit scores
1.0. To run one lot only, put `TASK_LOT=A` or `TASK_LOT=B` in front of the command. The videos and screenshot are
the first trial's, so they show lot A.

The run prints `task_score` (the mean over the trials). Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 45 s of simulated time (900 steps at 20 Hz); about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0` (as submitted): every object starts exactly where `task.py`
  puts it. With jitter on, seeded resets shift only the loose latches and spindles.

## How it is scored (simulator details)

- **The click:** each hub is a kit `Slider` (0 to 10 mm along the tube) with a 60 N/m spring. An `on_step` callback
  moves the spring's rest point to the nearer setting: past 5 mm it jumps to 10 mm (the 70 mm backset), below it to
  0 (60 mm). So a hub pushed past half-way clicks out and stays; one pushed less springs back.
- **Doors:** judged at the position their lot starts them (`door_layout()`), not where they end up. They are
  3 kg on the kit's grippy material (static friction 1.2), so pushing a latch home does not shift them in the traces.
- **Fitted:** the latch's faceplate back (its body origin) is at most 3 mm in front of the door edge and 1 mm past
  it, the latch is within 4 mm of the bore's line sideways and in height and within 3 degrees of it, the hub's hole
  centre (from the latch's pose and the slider position) is within 3 mm of the face bore's centre, and it is at rest. Pushed home, the faceplate settles about 2.4 mm short of
  the door edge in the simulator (contact offset), which is why the tolerance is 3 mm.
- **Spindle through:** within 5 degrees of upright, its centre within 8 mm of the face bore's centre (inside the
  hub's 16 mm hole), its lowest point below the hub's underside, and at rest. It can only get there by passing through the
  hub.
- **Released:** after every control step an `on_step` callback reads both arms' `gripper`, `tip_left` and `tip_right`
  link poses and measures their collision hulls against points every 4 mm or less over both latches (faceplate,
  tube, hub) and both spindles. Everything counts as released only when every hull is at least 5 mm away. Offline
  (`near_misses.py`, no simulator) the check is skipped and counts as released. The 1.0 condition must hold for
  3 s (`SUCCESS_HOLD_S = 3.0`).

## Scripted demo

`demo()` uses both arms at once, one per door, and reads which door is where from the current lot. For the 70 mm
door (left in lot A, right in lot B) that arm first brings its closed fingers down
behind the ridge on the hub's top and slides the hub out 8 mm, past half-way, then backs off; the hub clicks to 70 mm. Each
arm then pushes its latch by the faceplate along the bench and home into the edge bore (commanded past contact,
because the arm gives way under load), takes its spindle out of its holder, carries it over the face bore, pauses so it stops swinging and lowers
it slowly through the hub (aim points carry fixed offsets measured in traces for each arm's sag), letting go about 7 mm above the table. Both arms then park and wait so the 3 s hold runs with
the hands clear. The waypoints depend only on the lot; no step reads simulator state.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases include: both doors done (1.0), latches or spindles swapped between doors (1.0), a faceplate exactly 3 mm
proud (1.0) and 3.1 mm (0), a latch turned 3.5 degrees (0), both latches right but no spindles or one spindle or a
spindle tilted 6 degrees (0.5), a latch still moving (0), and the fixed habits: fit both as found at 60 mm (0), extend
both (0), extend the right one (0 in lot A), a wrong-length latch drawn 10 mm proud to line its hub up (0), and
doing nothing (0). These run in lot A, where "extend the left one" scores 1.0.

`habit_check()` in `near_misses.py` (run on import, so `factory/check.py` fails if it breaks) builds every end state
a policy could leave without reading the doors (each latch at 60 or 70 mm, each spindle aimed 60 or 70 mm in) as it
would come to rest in each lot, and scores it there. The best mean is 0.5: "extend the left one" scores 1.0 in lot A
and 0 in lot B, "extend the right one" 0 then 1.0; everything else averages 0.25 or 0.
