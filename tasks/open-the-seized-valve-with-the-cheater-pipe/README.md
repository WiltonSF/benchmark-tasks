# Open a stiff quarter-turn valve with a cheater tube

A red quarter-turn valve handle sits just above a deck plate. The valve is too stiff to turn by pushing or twisting
the short handle, with one arm or two. A loose aluminium tube lies on the deck. Slid over the handle, it makes the lever
about four times longer, and a push near its far end turns the valve to its open stop.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for
Isaac Lab-Arena), on the `station` scene, with three helper files that must sit beside it:

- `parts.py`: the parts kit. It builds the jointed valve and the hollow tube from coloured boxes and cylinders.
- `lots.py`: layout lots. It moves the tube to each lot's starting place before a trial.
- `near_misses.py`: offline checks of `score()` and `success()`. They need no simulator.

## Run a trial

From this folder, on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Open the red valve fully."

# Dream-1, all three lots (trial k uses lot k mod 3; the score is the mean)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 3 --store-frames --log-dir ~/yam-eval/valve-dream

# Scripted demo, all three lots
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 3 --store-frames --log-dir ~/yam-eval/valve-demo
```

Always pass `--epochs 3` (or a multiple). With the default of one epoch only lot A runs. To run one lot alone, put
`TASK_LOT=A`, `B` or `C` in front of the command. Run with `-E object_jitter=0`; the lots set the layout.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz). That is about 1 to 2 minutes of
  wall time on an L40S once Isaac Sim has started, so allow about 5 minutes for the three lots.
- **Seeds:** none needed. With jitter off, every lot starts exactly as `task.py` declares it.

## How the valve is modelled

The handle is a hinge with a stiff drive (200 N m/rad, capped at 13.2 N m). After every control step, an
`on_step` callback in `task.py` moves the drive's rest point along with the handle whenever the handle is pushed more
than 3.4 degrees past it. This acts as stem friction of 12 N m: the handle holds wherever it is left, and moving it
always takes 12 N m. At 86 degrees and beyond (within 4 degrees of the stop), the rest point snaps to the open stop. An `on_reset` callback puts it
back to shut.

The same callback measures the release test every step. It checks how close both arms' fingers and grippers are to
the handle and the tube.

Physics gate, run first with the valve 45 cm out and a 20 N m/rad spring on the same handle to measure torque:

| Method | Angle reached | Torque |
|---|---|---|
| Right arm pushing the handle end, commanded 18 cm past | 4 degrees, then slipped off | 1.4 N m |
| Left arm, the same push | 3.4 degrees | 1.2 N m |
| Right arm gripping the handle end and pulling 15 cm past | 9 degrees | 3.1 N m |
| Right arm gripping and twisting the wrist 80 degrees | 17.9 degrees | 6.2 N m |
| Right arm twisting and pulling at once | 17.4 degrees | 6.1 N m |

Then, with 10 N m of friction in place (the task uses 12), the right arm gripped the handle root and twisted while the left arm pushed
the handle end. The handle moved 0.2 degrees and stayed shut.

The best bare-handle methods add up to about 9 N m (one arm twisting, the other pulling). That is under the 12 N m
needed. With the tube on, one arm gripping it 250 to 300 mm out opens the valve in two strokes.

## Scripted demo

1. One arm grips the tube where it lies: the left arm in lots A and C, the right arm in lot B. In lot C it turns
   the tube to run front to back, sets it down and grips it again square.
2. It carries the tube into line with the handle, sets it on the deck and slides it over the handle until the gland
   ring stops it, then lets go.
3. The right arm grips the tube 300 mm from the axis and swings it counterclockwise, twisting its wrist with the
   tube. The loaded arm lags, so this stroke stops at about 70 degrees, and the stem holds the handle there.
4. The arm lets go, grips the tube again 250 mm from the axis, and swings it on to the open stop, commanding 20
   degrees past it. Then it opens and backs off.

The demo uses only where the tube lies, which the cameras show for each lot.

## Fixed habits and lots

- Pushing or twisting the bare handle scores 0 in every lot.
- Replaying lot A's demo in lot B or C closes the gripper on empty deck. The swing then passes about 22 cm beyond the
  handle's end and touches nothing, so it scores 0.
- The lots move only the tube. The valve and its opening direction are the same in every lot.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case and its score
```

The cases cover:
- the open stop: 1.0;
- exactly 85 degrees: 1.0;
- 84.9 degrees: 0.5;
- exactly 45 degrees: 0.5;
- 44.9 and 20 degrees: 0;
- 3 degrees of give from a bare-handle push: 0;
- the tube slid on but not turned: 0;
- open, or half open, with the tube still moving: 0;
- open with the tube taken back off: 1.0;
- nothing done: 0.

The release test needs the robot, so offline it always passes.
