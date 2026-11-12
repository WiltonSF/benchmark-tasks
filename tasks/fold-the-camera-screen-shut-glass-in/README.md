# Fold a camera's vari-angle screen shut with its glass facing the body

A mirrorless camera stands on the table with its rear screen swung out to the side on its vari-angle mount, lying
flat. The robot is asked to leave the screen folded flush against the camera's back with the glass facing inward.
The screen turns on two axes: a side hinge (0 to 175 degrees) and a swivel about the screen's width (-90 to +180
degrees). The swivel only turns while the screen is swung well out, and the screen only folds in when it is square.
Folding it straight back leaves the glass facing out. In both lots the screen starts flat, glass up, and a quarter
turn the right way is needed before folding. Lot A (the screenshot) has the lens pointing away from the robot; in
lot B the camera stands turned round, so the right turn is the opposite one as seen by the robot: pressing down the
half nearest the robot, which is right in lot A, turns the screen glass-forward and on to its -90-degree stop in
lot B.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts with joints) and `lots.py` (layout lots),
both in this folder; they import only NumPy and `yam_arena`. `near_misses.py` lists near-successes and wrong
outcomes with the score each must get; they are plain-Python checks of `score()` and `success()` and need no
simulator. No third-party assets are used.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Leave the camera's rear screen folded flush against the body with its glass facing inward."

# Dream-1, both lots (trial k uses lot A for even k, lot B for odd k; the mean is task_score)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 900 --epochs 2 --store-frames --log-dir ~/yam-eval/camscreen-dream

# Scripted demo, both lots (1.0 in each lot, in about 17 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 900 --epochs 2 --store-frames --log-dir ~/yam-eval/camscreen-demo
```

Without `--epochs 2` only lot A runs. `TASK_LOT=A` or `TASK_LOT=B` in the environment pins every trial to one lot.
The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 45 s of simulated time (900 steps at 20 Hz), about 1 to 2 minutes of wall time on an
  L40S once Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0`.

## The camera and its screen

Sizes are in `task.py` and `task.json`. The camera is one loose articulated part (0.60 kg body and lens, a 10 g
hinge bracket, a 40 g screen): the bracket turns on the side hinge, and the screen turns on the swivel carried by
the bracket. `parts.py` here is the parts kit with these additions: a joint may hang from an earlier joint's link
(`parent=`, a chain); a loose part's articulation is rooted at its body; an `on_reset` hook that returns True gets a
0.5 s settle and a fresh first frame, with `after_first_step` hooks run after each of its first two steps; a task may turn off
link-to-link collisions (`SELF_COLLIDE`); and the `OFFLINE_CASES` flag for near-miss cases. After the first settle steps
the task stands the camera where its lot puts it (lot A as declared, lot B turned round). The declared pose is the folded, shooting position (both joints 0); an
`on_reset` hook sets the start (side hinge at its 175-degree stop, swivel at +90 degrees, flat, glass up) before the
first frame, and puts the body back at its declared pose (setting the joints of a loose articulation keeps its
centre of mass in place, which would otherwise shift the body about 6 mm). The kit's link-to-link collisions are
turned off for this part (`parts.SELF_COLLIDE = False`); the housing is enforced through the joint limits instead.

- **Friction hinges:** each joint has a spring (side hinge 1.5 N m/rad, swivel 0.5 N m/rad) whose rest point follows
  the joint, trailing it by at most 1.3 and 1.2 degrees. So a hinge holds against less than about 0.034 N m (side
  hinge) or 0.011 N m (swivel) and slips at that torque, and a screen left alone stays where it is. Within 4 degrees
  of shut the side hinge's rest point goes 2 degrees past shut, so it clicks closed and stays closed.
- **The housing:** links of one kit part do not collide with each other reliably, so the housing's interference is
  enforced through the joint limits every control step. While the side hinge is under 75 degrees out, the swivel is
  held where it is (within half a degree). While it is 75 degrees or more out, the side hinge cannot go below 75
  degrees unless the swivel is within 8 degrees of 0 or 180 (square to the body).

## Scoring details the simulator applies

- **Released:** every control step an `on_step` callback measures the distance from every moving link of both arms
  (finger tips, gripper, wrist and arm links; their collision hulls) to every box and cylinder of the camera, bracket
  and screen. No credit
  unless all are more than 5 mm away. Until the first measurement nothing is credited (fail closed). The offline near-miss cases are credited as released only through a flag that `near_misses.py` sets (`parts.OFFLINE_CASES`), and never once a simulator scene has been built.
- **Still:** both joints turning under 0.05 rad/s and the camera moving under 1 cm/s and turning under 0.5 rad/s.
  `score()` gives a tier only once that tier's state, released and still, has held for 1 s; `success()` ends the
  episode after it has held 1 s.
- **Upright:** the camera's base within 5 degrees of level and its lowest point within 3 mm of the table top.
- **Tiers:** 1.0 when the side hinge is within 3 degrees of shut and the swivel within 8 degrees of 180 (glass
  facing the body); 0.5 when the swivel is within 8 degrees of 180 and the side hinge more than 3 degrees out.
  Limits are inclusive (compared with a 1e-6 margin).

## The demo

`demo()` plays one fixed routine per lot, open loop from that lot's published start (where the camera stands and which
way it faces); it reads nothing from the running simulation. It is the same routine in the camera's own frame: with
its fingers closed, the arm on the screen's side (left in lot A, right in lot B) presses down the half of the flat
screen that lies on the camera's back side until the glass faces the robot's side of the screen, then pushes the
screen's back, which squares it against the swivel's 180-degree stop and swings it round on the side hinge until it
clicks shut against the camera's back, and moves out to the side, toward the robot and up.

## Near misses

`near_misses.py` covers the start (0), folded glass in (1.0), the 3-degree and 8-degree limits on both
sides, folded glass out (0), turned glass in but not folded (0.5 at two side-hinge angles), turned the wrong way or
left flat (0), and the camera sliding, lifted, tipped or knocked over (0). The checker scores each case; scoring does not depend on the lot.
