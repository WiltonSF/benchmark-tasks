# Take the battery pack off a cordless drill and put it in the tray

A cordless 18 V drill stands upright on its battery pack on a home workshop bench. The pack is latched to the drill's
foot: it only slides off along the foot's rails while both release buttons, one on each side of the pack, are held
in at the same time. It then has to be drawn out against the rails' friction, which drags the drill along unless the
drill is held. The pack goes into a blue tray (where it waits to go on the charger) and the drill is left standing on
the bench.

This is a household chore most people with cordless tools do every time a pack goes on charge: chargers take the
pack, not the tool, and one pack moves between the household's tools of the same brand.

**Credit.** This task takes up the idea of PR 622 on this repository (TK-5CPBS2, revised as TK-WZDTSE and
TK-3VNC9C), whose 48-hour revision hold lapsed. The scene, code, wording and scoring here are new; the review
points raised on that pull request are answered below.

## Review points on the original, and how this version answers them

1. *"Revise the full-score predicate so the pack must be released, resting on the tray floor, and fully inside the
   tray walls; retain partial credit for a removed pack that has not been placed successfully."* Full credit needs
   all of: let go (every sampled point of both grippers' hull faces more than 5 mm from the pack, its buttons and
   the drill, every control step), resting on the tray floor (the four lowest corners of the pack's body within
   1.5 mm of the floor), wholly inside the tray (all eight body corners inside the inner walls), and the whole state
   unchanged for 1 s. A pack that is off the drill but held, in the air, on the bench, leaning on a wall or tipped
   scores 0.5 (`near_misses.py` covers each).
2. *"Change the category to societal relatability and explain the familiar household context of removing a cordless
   drill battery for charging or replacement."* The category is Societal relatability and the context is the household
   one (the pack comes off to go on its charger, and moves between the household's tools). No jobs claim is made.
3. *"Name an available drill and battery pair with the same release and sliding action ... including its spring and
   sliding resistance."* The model follows a purchasable pair: the Milwaukee M18 compact brushless 1/2 in
   drill/driver 2801-20 (any current M18 drill takes the same packs) with the M18 REDLITHIUM CP2.0 pack 48-11-1820.
   Milwaukee's manuals give the removal as pressing the release buttons on the pack and pulling it away from the
   tool; the pack slides along rails under the tool's foot. Sizes and masses used: drill 165 mm long, 1.13 kg without
   its pack (maker's listing, 2.5 lb); pack 118 x 79 x 55 mm, 430 g (maker's and retailers' listings). The button
   springs and rail friction in the model are estimates, listed under Fidelity with how to measure them on the real
   pair.
4. *Flag: "Use an inert battery pack or protect the cells for repeated trials."* For real trials use a pack that
   cannot short or heat: a dead CP2.0 pack with its cells discharged and its terminal slots taped over, or an empty
   pack shell. The latch, buttons and rails are in the plastic housing and work the same.

## Run it

The task runs in yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena) on the `station` scene. It
needs the three helper files next to `task.py`: `parts.py` (multi-shape parts with sliding joints: the two buttons),
`lots.py` (the two layout lots) and `tilt.py` (used only by the scripted demo, to lean both grippers in under the
drill's motor housing; it does not affect Dream-1 or scoring).

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Take the battery pack off the drill and put it in the blue tray. Leave the drill standing on the bench."

# Dream-1, both lots (trial k uses lot A for even k, lot B for odd k; task_score is the mean)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1600 --epochs 2 --store-frames --log-dir ~/yam-eval/pack-dream

# Scripted demo, both lots
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1600 --epochs 2 --store-frames --log-dir ~/yam-eval/pack-demo
```

Without `--epochs 2` only lot A runs. `TASK_LOT=A` or `B` in the environment pins every trial to one lot. Run with
`object_jitter=0` (as submitted): the pack is seated in the drill's rails with 0.5 to 1 mm clearance, so jitter of
the two loose parts separately would start them interpenetrating. At the start of each trial the task file writes
the drill, pack and tray exactly at their lot's pose and lets them settle. One trial is at most 1,600 control steps
(80 s at 20 Hz); the scripted demo reaches full score at about 27 s in each lot. Offline checks need no GPU:
`python near_misses.py` from this folder (it raises on any wrong score).

## Layout

Coordinates: x forward from the line through the arm bases, y to the robot's left, from the midpoint between the
bases; z up from the bench. Drill and pack share one frame at the seat: origin on the bench under the pack's middle,
+y toward the chuck (the way the pack slides off), x across.

- **Pack** (loose, 0.43 kg): black body 118 (along the slide) x 79 x 42 mm with a red top band; on top a rail tongue
  70 mm long, open at the chuck end: a 48 mm wide, 5 mm neck under a 60 mm wide, 8 mm head (the flanges the drill's
  rails hook under). Two red release buttons, one on each 118 mm side face, 18 x 12 mm, centred 41 mm from the pack's
  middle toward the chuck and 34 mm up; each stands 4 mm proud and travels 4 mm (flush). Each button carries a latch
  catch (6 x 8 x 4.5 mm) standing up from the top of the head into the drill's sole. Button springs: 6 N at rest,
  12 N pressed flush. A catch clears the drill's latch pocket once its button is 1.5 mm in, so the pack only slides
  while both buttons are 1.5 mm or more in.
- **Drill** (loose, 1.13 kg): red housing, black grip. Foot 118 x 72 mm, 34.5 mm tall from 42.5 mm above the bench
  (it stands on the pack's head; its underside 0.5 mm above the pack's body). Two black L-section rails run 70.5 mm
  along the foot from the chuck end and hook under the pack's head (0.5 to 1 mm clearance); behind them the foot is
  closed and stops the pack going further in. In the sole above the head: round each catch a pocket closed toward
  the chuck, and 2 mm farther in a channel that runs out of the front. Grip 34 x 50 mm, 106 mm tall, its middle
  14.5 mm toward the chuck from the pack's middle; motor housing a 56 mm cylinder along y from 45 mm behind to 75 mm
  in front of the pack's middle, top 239 mm above the bench; steel chuck 42 mm across, 45 mm long, nose 120 mm in
  front of the pack's middle.
- **Tray** (loose, 0.30 kg): blue plastic, inside 200 x 150 mm (200 along x), 5 mm walls 45 mm tall, 5 mm floor.
- **Lot A** (the screenshot): drill and pack at (0.28, -0.06), chuck pointing to the robot's left (+y); tray centred
  at (0.26, 0.25). **Lot B**: everything mirrored across the bench's centre line: drill and pack at (0.28, +0.06),
  chuck pointing to the robot's right; tray at (0.26, -0.25). Trials alternate A, B.

## Scoring

The seat is where the pack sits on the drill at the start, measured in the drill's own frame wherever the drill is.
"Both buttons pressed together" means both 1.5 mm or more in at the same control step, at any time in the trial.

| Score | When |
|---|---|
| 1.0 | both buttons were pressed together; the pack's middle is 8 cm or more from its seat; the pack's four lowest body corners are within 1.5 mm of the tray floor and all eight inside the inner walls; the tray within 5 degrees of flat; the drill upright (within 10 degrees) with its foot's underside within 3 mm of the bench; every sampled gripper point more than 5 mm from the pack, its buttons and the drill; all of this unchanged (no pack corner or drill origin moved more than 2 mm) for 1 s |
| 0.5 | both buttons were pressed together and the pack's middle is now 8 cm or more from its seat, without the full-score state (held, in the air, on the bench, leaning or tipped in the tray, or the drill not stood up) |
| 0.25 | both buttons were pressed together and the pack's middle is 1 cm or more from its seat, still on the rails |
| 0 | otherwise, and for the whole trial if the drill ever tipped past 45 degrees, the drill or pack fell off the bench, or the simulator could not report the grippers or the parts at any step |

How it is measured: `observe()` in `task.py` runs every control step (20 Hz). It records the buttons, the fall rule
and the distance from both grippers to the pack and to the drill: each gripper link's collision hulls are sampled at
their corners and about every 3 mm across every face, so a flat face lying on the pack counts even with no corner
near it. The 1 s starts when every full-score condition holds and restarts if any fails or anything moves more than
2 mm. If any measurement is missing, the trial scores 0 (fails closed). The record is cleared only by the
simulator's reset at the start of a trial. Cylinders are treated as their bounding boxes in the distance check,
which only makes "let go" stricter.

With a photo and a tape measure: the pack lies flat in the tray (a 1.5 mm feeler gauge does not pass under any
bottom corner) with no part over a wall, the drill stands on its foot on the bench, neither arm touches anything,
and in the video nothing moves for the last second. The video also shows whether the drill was knocked over.

Unintended strategies and their scores (all in `near_misses.py`): pulling without both buttons in does nothing (the
latch holds); buttons pressed one at a time and then the pack slid: 0; dragging the drill along with its pack, or
carrying both into the tray together: 0 (the pack never leaves its seat); knocking the drill over at any point, even
if it is stood up again: 0; dropping the pack off the bench: 0; pack held 2 mm above the tray floor, leaning on a
wall, or 1 mm over a wall: 0.5.

## Fidelity

Rigid bodies, 240 Hz physics, 20 Hz control. The drill and pack are built from boxes and cylinders at the listed
sizes and masses; their mass is spread evenly through their shapes (the real drill's motor makes it a little
front-heavy). The latch is geometric: each button slides on a sprung joint and carries the catch that sits in a
pocket in the drill's sole, so the pack is held by contact, not by a scripted lock. In the real CP2.0 the buttons
work the catches through the pack's housing; here catch and button are one piece. Estimates, not measured on the
real pair: button spring 1,500 N/m with 6 N preload (12 N flush), 4 mm travel, catch release at 1.5 mm (of 4 mm travel: the catch overlaps its pocket wall by 1.5 mm, a typical small
snap-latch engagement, not a measured M18 figure); rail
friction coefficient 0.35 (plastic on plastic), latch catches 0.2, grip 1.2 (rubber overmould). On the real pair, measure the button
force with a push gauge at the button's centre and the slide-off force with a spring scale pulling the released
pack along the rails. The tray is a plain plastic tray.

The drill's 1.13 kg is above the roughly 1 kg the YAM arm usually carries, and it shows in the traces. While the
holding arm carries the bare drill by its grip, the arm sags: a 50 mm lift command left the drill about 8 mm below
the height it had on the pack, so the arm gave about 4 cm. The drill also leaned up to about 10 degrees in the
fingers. The demo allows for both. It never needs the drill raised far: it lifts the drill only enough to let the
pack slide out from under it, stands it back on the bench at the grip's true height before opening the fingers (so
the arm does not spring up with the drill still in them), and backs away low and level before rising. A rubber grip
(friction 1.2) and a firm 50 N pinch keep the drill from turning in the fingers. On real hardware this step is the
one most likely to fail at the arm's limits.

The scripted demo is open-loop: every waypoint comes from the start layout, and the only thing it reads is which way
the chuck points in the trial's layout, which the cameras show. Each trial's demo first takes both arms to their rest pose, so a lot B trial behaves the same
whether or not a lot A trial ran before it. It carries fixed trims measured in earlier traces: the leaning right arm is aimed 3.5 mm
nearer in x at the buttons, the carry grip is aimed 5 mm farther out in x, and where the pack is set in the tray
allows for each arm drawing the pack out a different distance (about 8 mm short for the left arm, 31 mm for the
right). Lots stop a single fixed habit (for example "the
left arm always pulls toward the robot's left") but not a memorised replay of both layouts.

## Results

Both runs use `--epochs 2` (trial 0 lot A, trial 1 lot B) at the 1,600-step limit.

- Scripted demo: 1.0 in lot A and 1.0 in lot B (task_score 1.0). In each lot both buttons go in about 4 mm, the pack
  slides off, the drill is stood back on the bench and let go, and the pack is set in the tray; the full-score state
  holds from about 27 s.
- Dream-1: 0.0 in both lots (task_score 0.0); both trials ran to the step limit.
