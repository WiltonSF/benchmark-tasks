# Fit an inside-bolt-circle gasket into a spread pipe-flange joint and put all four bolts in

A short flanged pipe joint is held open for a new gasket. The lower pipe stands on the table with its flange 20 mm
up. The upper flange, on its own length of pipe, rests on two loose spacer blocks (the spreader) that hold it 16 mm
above the lower one. A ring gasket that just fits inside the bolt circle, with a flat handle tab as thick as the
ring, waits on a block in front of the gap, and
four M10 x 45 size bolts stand on the table. The gasket has to end centred on the pipe, under the upper flange still
in place over the lower one, with all four bolts through both flanges. With the two near bolts in, it can't pass
between them. Bolts put through both flanges end 4 mm below the lower flange; with the spacers out and the joint
closed onto the gasket, 16 mm stands below it for a nut.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. `parts.py` next to it builds the lower pipe and flange with the gasket's support
block (one fixed part), and the upper flange with its pipe, the two spacer blocks, the gasket and the bolts (loose
parts). It patches `yam_arena`'s scene spawning and step and reset at import time, in the task's own process only,
and changes nothing on disk. `near_misses.py` holds near-successes, fixed habits and wrong outcomes with the score
each must get. It runs as plain Python without a simulator.

## Run a trial

From this folder, on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Fit the gasket between the flanges, centred on the pipe, leaving the upper flange in place; put all four bolts in and let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/gasket-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/gasket-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, and `task.json` states it in words.

- **Runtime per trial:** at most 70 s of simulated time (1,400 steps at 20 Hz). The trial ends at 1.0 once
  success has held for 5 s (`SUCCESS_HOLD_S`), so the video ends with both arms parked clear of the joint for
  about the last 3.5 s, showing the gasket's tab on the midline and all four bolt heads on the upper flange.
- **Seeds:** none. Run with `-E object_jitter=0`, so the layout is the same every trial.

## Scoring in the simulator

- **Score:** with the gasket in, 0.2 plus 0.2 per hole with a bolt in, in any order (1.0 with all four); with the
  gasket not in, 0. The gasket is in only while the upper flange is in place over it, so every tier, 0.2 included,
  needs the upper flange in place. Gasket in with both near bolts and one far bolt scores 0.8, the same as both
  far bolts and one near bolt.
- **Still (`_still`, a settle window):** the `on_step` hook keeps the last 10 poses (0.5 s) of the upper flange, the
  gasket and each bolt. A part is still when, across that window, its origin stayed within 1 mm of where it is now
  and it turned less than 0.5 degrees. Until a part's window is full (0.5 s after a reset) it is not still, so the
  check fails closed. A window, not an instantaneous speed, so that contact chatter in a seated joint does not count
  as moving.
- **Positions:** each part's `pos` is its origin: the centre of the gasket's underside, the end of a bolt's shank, the
  centre of the upper flange's underside. A seated bolt's origin is 16 mm above the table, with its head resting on
  the upper flange.
- **Upper flange in place (`_flange_over`, at the end):** still, released (no gripper within 5 mm), its centre within 3 mm of the pipe axis seen from
  above, turned at most 2 degrees about it and tilted at most 3 degrees, its underside 3 to 17 mm above the lower
  flange's top. That covers the joint open on its spacers (16 mm) and closed onto the gasket with the spacers out
  (4 mm).
- **Gasket in (`_ring_in`):** upper flange in place and never moved off; gasket underside within 1.5 mm of the
  lower flange's top, tilted under 3 degrees, centre within 4 mm of the axis, still, released. Its centre is the
  point 144.5 mm behind the tab's end along the tab's centre line (centred, the tab's end is 64.5 mm in front of the
  lower flange's near edge).
- **Upper flange never moved off (`track_flange`, every step):** the `on_step` hook checks it each step. Lifted
  more than 5 mm above its start, slid more than 10 mm off the axis, or tilted or turned more than 5 degrees at any
  step, and the trial scores 0 from then on, even if the flange is put back. A reset clears this record. Lowering
  it onto the gasket is not a lift. To close the joint, take both spacers out together (or lift the flange under
  5 mm first): pulling one spacer alone lets the flange tip toward the 5 degree limit.
- **Bolts through both flanges (`_bolt_in`):** upright within 8 degrees; the head's underside within 2 mm of the
  upper flange's top, measured in that flange's own frame (it is loose on its spacers); the shank's centre line,
  where it crosses the middle of each flange's thickness, within 2.5 mm (upper, 13 mm hole) and 4 mm (lower, 16 mm
  clearance hole) of the matching hole's centre in x and in y, that is the hole's clearance round the 10 mm shank
  plus 1 mm of contact give; and the shank's end below the lower flange's underside. A bolt through the upper
  flange only is not counted. If the spacers are taken out and the joint closes onto the gasket, bolts through both
  flanges still count.
- **Release check:** every step an `on_step` hook places the hulls of both arms' gripper and finger links at their
  simulated poses (fingers as they are). It measures their distance to every shape of the upper flange, the gasket
  and each bolt. A part counts only while no gripper is within 5 mm of it. Before the first measurement after a reset nothing is
  credited (the check fails closed). Offline there is no robot: `near_misses.py` sets the task's offline flag, and
  also checks the rule with gripper distances filled in by hand.
- **Holes and bolts:** the bolts are identical, so any bolt may fill any hole.

## Geometry

- **Gasket fit:** the gasket's outer radius (59.5 mm) is 0.5 mm inside the bolts' inner edges (65 mm bolt circle
  less the 5 mm shank radius).
- **Near bolts block the gasket:** they are 92 mm apart centre to centre, 82 mm clear, so a 119 mm gasket cannot
  pass between them.
- **Far bolts centre the gasket:** pushed against the two far bolts, it stops 0.7 mm past the axis when they sit in
  the middle of their 13 mm holes. It stops at most about 3.7 mm past when the push shoves them to the far side of
  their holes. Both are inside the 4 mm tolerance, and the near bolts then clear its edge.
- **Spacers clear the gasket:** their inner faces are 66 mm from the axis, 6.5 mm outside the gasket's edge.
- **Lower flange holes:** these are 16 mm clearance holes, so the upper flange's 13 mm holes locate the bolts; a
  seated bolt's shank runs through both, its end 4 mm below the lower flange.
- **Handling tab:** flat, 20 mm wide and 4 mm thick like the ring, reaching 85 mm out of the ring's edge. With the
  gasket centred its outer 64.5 mm stand outside the flange's near edge, where it is gripped; under the flange it
  is no thicker than the ring, so it does not hold the joint open. Nothing else stands between the flange faces
  once the spacers are out (`near_misses.py` checks this from the shapes).
- **Closing the joint:** with the spacers out, the upper flange rests on the 4 mm gasket, 12 mm lower than on its
  spacers; the bolts' ends then stand 16 mm below the lower flange and 4 mm above the table.
- **Closure checked on the GPU:** a separate test scene (not part of the task) with the spacers out, the gasket
  centred and the upper flange and four bolts released 2 mm above their closed positions, both arms parked. Under
  gravity the flange settled onto the gasket within the first 0.25 s: its underside ended 0.2 mm below the gasket's
  top (contact give), level within 0.1 degrees and centred within 0.1 mm. Scored by this task's `score()`, the closed joint read 1.0
  at every logged step from step 10 (when the 0.5 s settle window filled) and the trial ended at 1.0 at step 105,
  once success had held for 5 s.

## Habits that must not pass

The layout is fixed, so each ordinary habit was checked against it:
- **Nearest holes first** (the near holes are nearer the arms): the gasket then stops against the near bolts
  40 mm short, and the trial scores 0.
- **All four bolts first:** the gasket can't get in, and the trial scores 0.
- **Gasket first with no bolts,** then all four bolts: fair, if it is centred by eye to about 2 mm in each diagonal
  direction.
- **Gasket pushed in hard with no stops** ends 10 mm past the axis: 0.

## Scripted demo

`demo()` uses fixed waypoints and reads no simulator state. The arms take turns, so their grippers are never near
each other:
1. The left arm puts a bolt in the far-left hole.
2. The right arm puts a bolt in the far-right hole.
3. The right arm grips the gasket's tab and slides the gasket in to 3 mm past touching the far bolts, then lets go.
4. The left arm puts a bolt in the near-left hole.
5. The right arm puts a bolt in the near-right hole.

For each bolt the arm grips the head (fingers 5 mm past its faces) and carries it over the hole. It lowers the bolt
until the shank end is 2 mm above the upper flange and settles there. Then it lowers the bolt in until the head is
3 mm above its seat, and lets go. The demo never touches the spacers or the upper flange.

In demo-20261115T040504 (GPU run of the final scene and scorer, 1,186 steps), every bolt went in without aim offsets
and the trial ended at 1.0, so the upper flange stayed in place for the whole trial and all four bolts ended through
both flanges, let go. The 4 mm tab was gripped 1 mm above its underside and slid the gasket in. From frame 1110 to
the last frame (1185) both arms are parked clear and the top camera shows the gasket's tab on the midline and all
four bolt heads on the upper flange. In a GPU trace of the same scene and demo (trace-20261115T034743) the score
reached 1.0 at step 1090 and held to the end, with the gasket 2.4 mm from the axis.

## Dream-1

In dream-20261115T043025 (the final scene and scorer, 1,400 steps, 320 x 176 cameras) Dream-1 scored 0.0. It
raised both arms and held them up for the whole trial, never touching the gasket, the bolts or the flanges.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases cover:
- the gasket tolerance boundaries: centre at 4.0 and 4.1 mm, height at 1.5 and 2 mm, tilt at 2.9 and 3.1 degrees;
- partial credit in any order: the gasket alone (0.2), one far bolt (0.4), both far or both near bolts (0.6), both far
  plus one near and both near plus one far (0.8);
- bolts with no gasket in (0), and the habits above;
- the upper flange in place: slid 2 and 3 mm with its bolts (1.0), and the spacers out with the joint closed onto
  the gasket (1.0);
- the upper flange removed or shifted (0 in every case, the gasket alone included): lifted away onto the table with
  the gasket centred, with or without bolts in the lower holes; shifted 3.5 and 10 mm; turned 2.5 degrees; tipped
  3.5 degrees; held 2 mm high;
- the whole-trial record: the flange lifted 30 mm, slid 12 mm or tipped 6 degrees during the trial and put back (0),
  lifted 4 mm (1.0), and lowered onto the gasket (1.0);
- bolts through the upper flange only (not counted, 0.8 with the other three): head seated with the shank 4.5 mm off
  the lower hole, head seated with the shank leaning 8 degrees past it, and the end resting on the lower flange
  beside the hole;
- the bolt limits: a 3 degree lean (in), 9 degrees (out), head height at 2 and 2.5 mm, shank 2.5 and 2.6 mm off the
  hole centres;
- the release rule: a gripper 4.9 and 5.0 mm from a bolt, a gripper on the gasket, a gripper holding the upper
  flange in place or 4.9 mm from it (0), and no measurement yet;
- the settle window: all parts at rest for 0.5 s (1.0); the gasket or the upper flange slid 2 mm within the window
  (0); the flange crept 0.9 mm (1.0); a bolt slid 2 mm or turned 0.6 degrees (not counted) or turned 0.4 degrees
  (counted); a window not yet full for the flange (0) or a bolt (not counted);
- doing nothing.
