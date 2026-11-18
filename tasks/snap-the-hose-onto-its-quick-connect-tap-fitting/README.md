# Fit the hose end into the tap's sprung-sleeve coupler so it holds when tugged

A garden standpipe tap (its spout 82 cm above the floor, as on a post in a garden; here the post stands on the
table) has a brass sleeve-type quick coupler on its spout. The free end of a garden hose, with its brass male
connector, lies on the table. The robot is asked to attach it so that it cannot be pulled off, and to leave it with
nothing holding it. Doing that means pulling the coupler's sprung collar back toward the tap and holding it there,
lifting the hose end, lining the connector up with the bore and pushing it fully home, then letting the collar spring
forward over the locking pins. The instruction names only the goal. Whether it holds is judged the way a gardener
checks: by pulling on it.

## Credit

This idea was first submitted by GuraseesWasNotAvailable as PR 1868 (TK-GD5BJP, revised as TK-LP9ERQ: connect a
garden hose to a tap through a quick-connect coupler and let it lock). Its first review asked that full credit need
the 10 N pull test (or an equivalent engagement check in simulation) and that no rung call an unchecked fitting
locked. Its second review was "Video demo should show more detail, it is currently challenging to clearly discern the
operation". No revision arrived within the 48-hour hold, and the task board then lets another team take the idea on
if it credits the original. This version was written from scratch; no code or text was reused. How each point is met:

- **Locking is scored by a pull.** Full credit needs the connector to stay home under a 10 N pull along the bore,
  applied by the scorer with nothing touching it (section "Scoring as implemented"). There is no rung for a fitting
  that only looks seated.
- **The pins cannot miss the groove.** The bore leaves 0.5 mm of clearance round the shaft, and the pins stand 2.0 mm
  into it (section "The mechanism").
- **The videos show the coupling closely.** Besides the top and wrist cameras, the demo and Dream-1 videos show two
  close cameras on the coupler (side-on to the collar, and into the mouth), a zoomed panel of the collar and pins, and
  the seating replayed at half speed (section "Videos").

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It needs `parts.py` (multi-shape parts with hinges and sliders) and `tilt.py` (a
tilted grip for the scripted demo), both in this folder; they import only NumPy and `yam_arena`. `near_misses.py`
lists near-successes and wrong outcomes with the score each must get; they are plain-Python checks of `score()` and
`success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Attach the loose hose end to the coupler on the tap so that it cannot be pulled off, and leave it with nothing holding it."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir logs/dream --epochs 2

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir logs/demo --epochs 2
```

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 2 to 3 minutes of wall time on
  an L40S once Isaac Sim has started. The demo's trial ends on success at about 20 s.
- **Seeds:** none needed; run with `-E object_jitter=0`. The hose end is the only loose object. `--epochs 2` runs
  two identical trials and reports their mean, to show the result repeats.
- **Close cameras (optional, videos only):** with `HOSE_VIEWS_DIR=<dir>` in the environment, two more cameras are
  added and their frames saved to that directory (section "Videos"). Without it the scene, the observations and the
  scoring are exactly the same.
- **Layout:** one fixed layout. There is one coupler and one hose end, so there is no choice for a fixed habit to
  win and no layout lots. A memorised replay of the demo is not ruled out.

## The mechanism

All of it is ordinary contact physics between rigid shapes; nothing is switched by a script.

- **Coupler** (a fixed articulated part, brass, friction 0.3): a round bore 17.0 mm across (12 slabs, inscribed
  circle) behind a 45-degree lead-in chamfer from 22 mm. Two locking pins are spring sliders (20 N/m, 3 g, 2.5 mm
  travel) moving radially, front and back of the bore (horizontal, so gravity does not bias them). At rest their
  round noses stand 2.0 mm into the bore (6.5 mm from the axis). The collar is a third slider along the bore (0 to
  11 mm toward the tap; 273 N/m with its rest point 4.4 mm past the forward stop: 1.2 N at rest, 4.2 N at the stop),
  round (a 12-sided ring 39 mm across), with its own friction (1.0, knurled). Its inner land, 0.3 mm outside the
  pins' outer ends, stops them moving out; 6 mm of collar travel brings its relief over them.
- **Connector** (a loose articulated part, rooted at the connector, friction 0.5): shaft 16.0 mm, so 0.5 mm of
  clearance all round in the bore; nose stepped 14/15/16 mm; groove 6.4 mm wide with a 12.0 mm core, 6.2 to 12.6 mm
  in front of the flange, so the pins (6.5 to 11.5 mm behind the mouth) drop into it with the flange up to 1 mm short
  of the mouth. The hose is the last 48 cm of a 25 mm PVC hose: eight 60 mm, 15 g sections on soft hinges
  (alternately bending sideways and up-down, 0.5 rad each), its far end lying free on the table.
- **What follows from the geometry:** with the collar forward the pins cannot move out, so the nose stops against
  them (about 6 mm in). With the collar held back, the nose cams them out and they ride on the shaft. If the collar
  is let go before the connector is home, its land meets the pins (which are out on the shaft) and it stays back at
  6 mm: the connector is not held, and slides out. Only with the groove under the pins do they drop back in, and the
  collar returns over them. Pulled, the groove's shoulder pushes the pins outward against the land, and it holds.
- **The reviewed construct problem (a narrow shaft in a wide bore missing its groove)** does not arise here: the
  clearance is 0.5 mm, the pins are horizontal so the shaft's sag (downward) does not move it away from either pin,
  and the pins stand 2.0 mm in, past the shaft's surface by 1.5 mm with the shaft fully off-centre toward the other
  pin. In every trace the shaft sat within 0.7 mm of the axis.
- **`parts.py` changes** from the stock kit (beyond those of the kit copy it starts from: chained joints, per-part
  contact offset, shapes without colliders): box shapes may also be turned about the part's x and y axes (`roll=`,
  `pitch=`, for the bore, chamfer and collar around a horizontal axis); a joint's link may have its own friction
  (`friction=`, the collar); a loose part's articulation is rooted at its body. Without the last, PhysX chose a
  middle link of the connector's chain as the floating root, and the reset placed that link, not the connector, at
  the declared position.

## Scoring as implemented

`score()` and `success()` in `task.py`, in the coupler's frame (its axis runs left-right, the mouth facing left).

- **Measures:** `f`, how far the flange face is into the mouth (0 home, negative in front); the connector's body
  origin (the flange face's centre) offset from the axis; its axis's angle to the bore's; the collar's travel.
- **0.25** (whole trial): at some control step the nose (f + 24 mm) was at least 14.5 mm in, within 2 mm and
  6 degrees of the axis. That is past the pins' far edge, which needs the collar held back.
- **0.5** (whole trial): at some step the connector was home: f at least -2.0 mm, within 1.5 mm and 5 degrees of the
  axis. (The groove's play lets a locked connector rest 0.7 to 1.4 mm out, and the bore's 0.5 mm play lets it rest
  about 3.3 degrees askew under the hose's weight: measured in the traces.)
- **No rung for an unchecked fitting.** The 0.75 rung of the earlier version of this idea ("seated with the collar
  forward") is gone: no credit says "locked" without the pull test.
- **1.0, locked** (`_pull_step`, `_apply_pull`): once the connector is home, the collar within 1 mm of its forward
  stop, nothing touching it and the connector at rest (under 1 cm/s and 0.5 rad/s, pose unchanged within 0.2 mm and
  0.2 degrees) for 20 control steps (1 s), an `on_step` callback pulls it straight out of the bore along the
  coupler's axis: 1, 2, ... 10 N over 10 steps (0.5 s), 10 N for 10 steps (0.5 s), then nothing for 10 steps.
  Pass: it came out at most 1.5 mm, and at least 0.02 mm (a pull under which nothing moved at all is treated as no
  test), is home with the collar forward and at rest at the end, and the callback recorded the full 10 N on at
  least 9 of the 10 full-force steps (fail closed: a force that was not set is no test). An unlocked connector is
  pulled out of the coupler: once it has come 3 mm the test fails and the force stops. Anything coming within 5 mm
  during the test, or the connector moving afterwards, sets the test back to the start.
- **Nothing touching it:** an `on_step` callback measures every step the shortest distance from both arms' collision
  hulls (`yam_arena.analysis`: forearm `link3`, `link4`, wrist `link5`, gripper and both fingers) to every shape of
  the hose end (connector and all hose sections), the collar and both pins; all must exceed 5 mm. Each hull is
  measured over its faces, sampled about every 3 mm with the corners included, not only at its corners: some hull
  faces are over 100 mm across, so a face lying on the collar can have every corner far from it. So neither a
  gripper nor a wrist can brace the hose. Until it has measured all six links of both arms, or if it cannot, the
  state is unknown and the pull test does not start (fail closed). near_misses.py checks the unknown and the held
  cases, and a flat 120 mm face 0.5 mm over the collar with no corner near it (no full credit; it would pass a
  corners-only check), and the same face 6 mm up (clear).
- **0:** the hose end's body more than 2 cm below the table top at the end (dropped off), whatever was reached.
- **Success** is the 1.0 state, and must hold 1 s. History is cleared by the simulator's reset, and also when the
  hose end is back within 0.5 mm of its start, at rest, after a jump of over 20 mm in one control step.
- **Real trial:** a rule along the bore for the depths and the collar; a spring scale hooked on the ferrule, pulled
  along the bore up to 10 N and held for half a second, with a rule to read the movement.

## Checked on the GPU (Isaac Sim 6.1)

- **Final demo** (`--epochs 2`, task_score 1.0, both trials succeeded), with the arms' clearance measured over
  their hulls' faces: the collar went back to its 11 mm stop, the connector went home at about 14 s, the pins
  dropped into the groove and the collar sprang forward to 0, both arms backed off, and the pull test passed: each
  trial ended on success at about 20 s. (In an earlier trace with the same scene and demo, the connector rested
  0.8 mm out and came out to 1.4 mm under the 10 N pull, the pins' play in the groove, and stayed home.)
- **Dream-1** (the command above, `--epochs 2`): both trials ran the full 1,200 steps and scored 0.0. In the first
  the left arm went down near the hose end and the right toward the tap; in the second both arms stayed above the
  table. Neither trial grasped the hose end or the collar. The Dream-1 video is the first trial.
- **Unlocked probe** (an earlier build with a 3-section hose; a copy of the folder with the pins moved outside the
  bore, so nothing can lock; not part of the submission): the same demo pushed the connector home and the collar
  came forward, but the pull drew the connector out of the coupler and it came to rest on the table 17 cm in front of
  the mouth. Score 0.5, no success. So the pull separates a locked connector from one only resting home.
- **Collar stuck on a pin:** in one trace with a 5.6 mm groove, the connector stopped 0.6 mm short, a pin rode on
  the groove's shoulder and the collar jammed 6 mm back on it. That is why the groove is 6.4 mm and full credit also
  requires the collar forward.
- Earlier traces: pushed without the collar back, the nose stopped 9 mm in, against the pins. With the collar back
  but the push 4 mm short, the connector stopped 3.9 mm out and the collar, let go, stayed back at 6.0 mm on the pins.

## Scripted demo

`demo()` uses fixed waypoints from the published start layout only (no object state is read).

- **Right arm:** over the collar, fingers closing across it (wrist turned +90 degrees), leaning 30 degrees toward the
  mouth; pinches its flats 8 mm past their faces, pulls it 34 mm toward the tap (it stops at 11 mm; the arm is soft,
  about 190 N/m at the fingertip), holds it until the connector is home, opens, slides back toward the mouth, rises
  and moves away.
- **Left arm:** grips the ferrule (25 mm behind the flange, pads 5 mm inside its faces, firm enough that the hose's
  weight does not turn the connector in the pads) with the finger tips leaning 30 degrees toward the nose, lifts,
  turns it onto the bore's line 40 mm out, settles, pushes the nose in and on to 8 mm past home (soft arm), holds
  while the collar springs forward, lets go and moves away. Its target height is 4 mm above the axis: the left arm
  settles about 4 mm low with a light load near x 0.30 (measured earlier). The leans keep the two arms apart
  (checked offline with `yam_arena.analysis.hull_set_distance`; closest 9 mm, fingertips, while both back off).
- `tilt.py` drives the tilted grips by wrapping `yam_arena.demo.grip_rotation` in the task's own process; it affects
  only the scripted demo, never Dream-1 or scoring. `DEMO_TOUCHES = ("coupler",)`: the offline checker treats the
  fingers as closed, which would put them inside the collar; on the 39 mm collar they stay open to its width.

## Videos

The board's videos are built from the run's own camera frames; nothing is drawn or edited in.

- **Top row, whole trial at real speed:** the top camera (what the policy sees), `side_cam` and `mouth_cam`.
  - `side_cam`: level with the bore, 24 cm beyond the coupler, looking back toward the robot. The collar, the mouth
    and the connector are seen side-on, and the bore's axis runs across the picture.
  - `mouth_cam`: 10 cm out from the mouth along the bore, 14 cm to the far side and 2.5 cm above the axis, looking
    into the mouth 54 degrees off the axis and from below the grippers, so the hose and the fingers holding it do
    not hide the mouth.
- **Bottom row:** the two wrist cameras, and a 2x crop of `side_cam` (rendered at 1280 x 720) on the collar, the
  pins' line and the flange.
- **Then the seating at half speed:** `side_cam` and `mouth_cam` from just before the nose enters until after the
  collar springs forward.
- The two close cameras are added only when `HOSE_VIEWS_DIR` is set (`parts.views()`, in the section "close cameras"
  of `parts.py`). They are not in the observation: the embodiment returns only `top_cam`, `left_cam` and
  `right_cam`, so Dream-1 sees exactly what it sees without them. The screenshot is the top camera's first frame.

## Licences

No third-party assets: every object is built from boxes and cylinders in `task.py`.
