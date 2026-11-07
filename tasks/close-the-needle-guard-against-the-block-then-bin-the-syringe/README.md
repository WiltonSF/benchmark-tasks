# Close a used needle's safety guard one-handed against a fixed block, then bin the syringe

A used syringe lies on a tray with its hinged safety guard open, square to the needle. The robot is asked to close
the guard over the needle and put the syringe in the sharps container. The safe method, as safety-needle users are
taught, is to close the guard with the hand that holds the syringe by pressing it against a hard surface, so the
free hand never comes near the point. The instruction does not state this; the scorer checks how the guard closed.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (in this folder) for the hinged guard and the multi-part
fixtures; it imports only NumPy and `yam_arena`. `near_misses.py` lists near-successes, wrong methods and fixed
habits with the score each must get; they are plain-Python checks of `score()` and `success()` and need no
simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Close the safety guard over the used needle, then put the syringe in the sharps container."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/needle-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 500 --store-frames --log-dir ~/yam-eval/needle-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0`: every object starts exactly where `task.py` puts it. There is
  one block and one container, so no choice of place or object that a fixed habit could win; the layout is fixed.

## Scoring details the simulator applies

- **Grippers:** a `parts.on_step` callback reads both arms' joint angles each control step and computes, per gripper,
  its tip point (between the finger tips, from the arm's own kinematics), its half opening, and 18 sample points over
  both finger pads (each pad 40 mm wide and the 40 mm of finger above its tip, at the current opening). Offline,
  `near_misses.py` passes stand-in grippers as a pseudo-object named `grippers`.
- **Closing record (whole trial):** while the guard hinge is at 30 deg or more and has not yet clicked (86 deg), every
  control step must have: some point of the guard's outer half (its centre line from 23 mm out to its 46 mm end)
  within 8 mm of a fixed box (the guard's faces are 5 mm off its centre line, so its surface within 3 mm); no finger
  pad sample point within 20 mm of any point of the guard's centre line; and the syringe's bounding-box centre over
  the tray. If the guard clicks after a step that broke any of these, the trial is marked "closed another way" and
  scores 0 from then on. A guard swung back under 30 deg starts a fresh attempt. Near misses cover a wide-open free
  gripper pushing the guard shut beside the block (with the hinge by the block, and with the outer half on it).
- **Closed:** clicked, and the hinge within 10 deg of 90 deg.
- **The click:** the guard's drive (0.02 N m/rad) targets open until the hinge reaches 86 deg, then targets closed for
  the rest of the trial (`parts.on_step`); a scene reset reopens it (`parts.on_reset`).
- **Released and still:** every gripper tip point more than 5 mm above the syringe's top, or farther from its bounding
  box sideways than its half opening plus 10 mm plus 5 mm; and `ObjectState.resting()` (under 1 cm/s, 0.5 rad/s).
- **Lowest point:** computed from the true shapes (a turned cylinder's bounding box reaches lower than the cylinder).
- **In the container:** the bounding box within the inner walls in x and y, its top at or below the rim, the lowest
  point within 1.5 mm of the container floor. **On the tray:** centre over the tray, lowest point within 1.5 mm of the
  tray floor (the flange's lower wing rests on it).
- **On the table (whole trial):** the lowest point within 2 mm of the table top with the centre over neither the tray
  nor the container sets a record that scores 0 from then on.
- Both records are cleared only by a scene reset: the syringe back within 5 mm of its start (it settles about 3 mm
  from the written pose on the ridges), at rest, guard open, after a jump of over 20 mm in one control step.
- The trial ends with `success` once the 1.0 condition has held for 1 s; otherwise `task_score` is `score()` at the
  last step.

What the scorer cannot see: the pads are sampled at 18 points and the rest of the gripper (palm, wrist) is not
checked, so a gripper body pushing the guard with no finger within 20 mm is not ruled out. A reviewer can see the
hands in the video.

## The scripted demo

Right arm only, fixed waypoints. It grips the barrel at its middle and lifts the syringe 15 mm. It brings the open
guard just past the block's near corner, then moves the syringe in steps so that the corner stays 35 mm out along
the guard (on its outer half) and rolls it round from open to lying along the needle, where it clicks. It lifts the
syringe over the container, lowers it until the flange is about 12 mm above the floor and lets go with the fingers
opened to 40 mm. The left arm stays parked on its own side throughout.

## Licences

No third-party assets: every object is built from boxes and cylinders in `task.py`.
