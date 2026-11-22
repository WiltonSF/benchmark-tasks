# Hold a mango cheek steady, score it into a grid down to but not through the skin, then push the skin up to fan the cubes

**Instruction:** Steady the mango cheek, score its flesh into a crosshatch grid down to the skin with the paring knife
without cutting through the skin, press the skin side onto the upturned ramekin so the cubes fan out, then leave the
cheek and the knife on the board and let go.

One cheek of a ripe mango, already cut off the stone, lies skin side down on a cutting board. The left gripper pinches
its narrow near end to keep it from sliding. The right gripper picks up a paring knife by its handle and, holding it
point down, draws seven cuts through the flesh, four across the cheek and three along it, each running down to just
above the skin, which curves up from 24 mm deep at the middle to nothing at the edge. It stands the knife back on its
handle and lets go; the left gripper lets go of the cheek. The right gripper then picks the cheek up by both sides of
its rim and presses the middle of the skin onto the base of an upturned ramekin, which pushes the skin up and fans the
scored cubes apart (the "hedgehog" cut), sets the cheek back on the board and lets go.

## Why this task

- It is the core of the standard way to cube a mango. The National Mango Board's steps: cut the cheeks off beside the
  seed, then "Score (slice) the flesh in a crosshatch pattern, without cutting through the skin", then push the
  centre of the skin upward to fan the cubes outward
  ([mango.org, How to Cut a Mango](https://www.mango.org/how-to-cut-a-mango/)).
  The first step (cutting the cheek off the stone) is done by the assessor at reset; this task is the scoring and
  turning out of one cheek, which fits the 120 s limit.
- The hard part is depth along a hidden, curved boundary. The cut face is flat but the skin under it is a cup: the
  flesh is 24 mm thick in the middle and thins to nothing at the edge, so a knife held at one height either stops
  short in the middle (the cubes stay joined and do not fan out) or goes through the skin at the edge. The point has
  to follow the cup down and up again on every cut, against a stated cutting drag; at cutting speed (above about
  6 mm/s) the drag is more than the cheek's board friction, and at any speed a cut counts only while the other hand
  pinches the cheek.

## Scene (true size, one fixed starting layout per lot)

| Item | What it is | Size and source |
|---|---|---|
| Cutting board | OXO Good Grips Carving & Cutting Board 11272900, polypropylene | 535.9 x 368.3 mm (maker); 16 mm thick (estimate); fixed, centre (0.29, 0) m, long side along y, top at z = 0.7635 m |
| Mango cheek | One cheek of a ripe Ataulfo (Honey) mango, cut about 6 mm from the centre plane beside the flat seed (real-trial cut below), skin side down | cut face 112 x 72 mm, 25.5 mm deep at the middle (24 mm flesh on 1.5 mm skin), 115 g; the skin's outer surface is half an ellipsoid with semi-axes 56, 36 and 25.5 mm (estimates from a 280-350 g fruit; checked against each specimen, see "Real trial") |
| Paring knife | Mercer Culinary Millennia M23900P, 3 in (76 mm) high-carbon steel blade, black handle with finger guard | blade 76 mm, 18 mm tall at the heel, the last 20 mm toward the point modelled 8 mm tall; handle 100 x 22 x 24 mm; 50 g (handle size, blade height and mass estimated from maker photos). Stands on the flat underside of its handle and guard, edge on the board, blade pointing -y (right); in a real trial a pea-sized dab of reusable putty under the handle keeps the rounded handle upright and lifts off with it |
| Ramekin | White porcelain ramekin, 2 fl oz, upside down, loose | 60 mm across the rim, 48 mm across the base, 32 mm tall, 100 g (generic catalogue size, estimated) |
| Ruler | Steel rule, 10 mm ticks, for photos | 300 x 20 x 2 mm, beyond the board at (0.49, 0) |

Coordinates are metres in the station frame (x away from the robot, y to its left). The cheek's long axis points
away from the robot and to its right; its narrow near end is toward the robot's left arm.

- **Lot A (screenshot):** cheek cut-face centre (0.310, -0.090), long axis pointing 45 degrees right of straight
  ahead; knife handle centre (0.240, -0.135), blade pointing -y; ramekin centre (0.410, -0.210).
- **Lot B:** cheek (0.300, -0.072), long axis 33 degrees right of straight ahead (12 degrees turned from A); knife
  (0.215, -0.140); ramekin (0.400, -0.200).
- No jitter, no seeds. Each simulated trial checks that the cheek, knife and ramekin start within 4 mm and 3 degrees
  of the lot's layout; otherwise the trial is invalid (scored 0).

## Score

**Score 0 for the trial, whatever happens later, if at any moment:**
- the knife cuts through the skin: the edge goes more than 1 mm into the 1.5 mm skin anywhere more than 2 mm in from
  the cheek's edge (within 2 mm of the edge the skin turns up to meet the cut face and is not judged);
- the blade touches either gripper, or comes within 10 mm of a gripper that is not holding the knife by its handle;
- while a gripper holds the cheek, the skin is pushed more than 35 mm in from its resting shape, past a full
  inside-out turn (split skin).

**Otherwise it is judged at the end: both grippers at least 5 mm from the cheek, the knife and the ramekin; the
cheek skin side down, the knife and the ramekin resting on the board; no part of the blade in or within 2 mm of the
cheek; nothing moving more than 1 mm or 2 degrees for 1 s. If these end conditions are not met, the score is 0.**

A **cube** is a piece of the cut face that cuts reaching within 6 mm of the skin separate from every other piece
(flesh within 7 mm of the cheek's edge is rim and belongs to no piece), with a top of 80 to 800 mm2 and no two
points of its top more than 30 mm apart.

| Score | Condition | How it is read in a real trial |
|---|---|---|
| 1.0 | At least 10 cubes, and after they were cut the skin's middle was pushed up at least 12 mm onto the ramekin while a gripper pinched the cheek, opening the cuts (a full turn-out is about 25 mm) | top photo with the ruler after the push: count cubes standing apart (gaps open to near the skin), ruler on each; side video with a 1 mm scale level with the ramekin's base for the push |
| 0.75 | At least 10 cubes, not pushed out | top photo with the ruler; spread the cuts gently with the knife tip to see they reach the skin |
| 0.5 | 5 to 9 cubes | same |
| 0.25 | One connected run of cut flesh (within 6 mm of the skin, outside the 7 mm rim) whose bounding box is 30 mm or more corner to corner, but fewer than 5 cubes | same; ruler on the cut |
| 0 | Anything else | |

Only the highest level reached counts. Lots A and B are averaged.

A gripper pinches the cheek when its opening is 10 to 90 mm and both fingertips are within 3 mm of the cheek with their
contact points at least 15 mm apart across the cut face (the rule checks position, not force). A cut counts only
while the knife is held by its handle in one gripper, the cheek is pinched by the other, the blade
is within 45 degrees of upright, and the edge moves within 45 degrees of the blade's own plane (drawn or pushed point
first, not dragged sideways), with the cutting drag acting (below). Cuts made after the push do not count.

**How the simulation scores it (`task.py`).** `update()` runs every control step. The cut face is a grid of 1 mm
cells; for each cell it keeps the lowest point the edge of the knife reached there during counted cuts, sampled
every 0.5 mm of edge motion relative to the cheek and every 1 mm along the edge. A cell is cut when that point is
within 6 mm of the skin. Cubes are the 4-connected groups of uncut cells inside the 7 mm rim band. The skin test
uses the whole edge and the spine of the blade at the same sampling, whether or not a cut counts. The push is the
height of the ramekin's flat base above the cheek's undisturbed outer skin surface at the base's centre, which must
be within 15 mm of the cheek's centre and below the cut face, with the ramekin base up and the cheek upright and
pinched by a gripper; 35 mm anywhere on the base, while the cheek is pinched, is the split limit. Hand distances use the grippers' collision-hull faces sampled every 3 mm. `score()` returns the
level; `success()` is score 1.0.

## Cutting drag (declared material response)

While any part of the edge is in the flesh, the simulation applies a drag to the knife along the blade's horizontal
direction, against the edge's motion along it, and a feed resistance upward while the edge moves down. Each grows
with the edge's speed relative to the cheek (its motion over the last 0.1 s) and reaches 1 N at 10 mm/s (the demo
cuts at 4-14 mm/s); the equal and opposite force acts on the cheek. Both act at the bodies' centres of mass; the moment
about the knife grip is not simulated. A cut counts only on steps when this load was acting, with the edge, averaged
over the last 0.25 s, moving at 1 mm/s or more. (A load taken from the 0.25 s average lagged the knife's motion and
set it bouncing about 5 mm up and down during cuts across, so the load uses the shorter 0.1 s window.)
The values are estimates for a sharp paring knife in ripe mango flesh: firmness probes with an 8 mm plunger record roughly 5-15 N for ripe mango (a pressure of
0.1-0.3 MPa), and a thin edge with about 2 mm2 of edge in the flesh needs well under 1 N to part it, plus some side
friction. Above about 6 mm/s the drag is more than the unheld 115 g cheek's board friction (about 0.6 N), so an
unheld cheek slides; at any speed a cut counts only while the other hand pinches the cheek.

## Real trial

**Fruit.** One Ataulfo (Honey) mango per trial, 280 to 350 g, ripe: a 10 N press with a 20 mm flat-ended probe on
its shoulder (fruit on a kitchen scale reading 1 kg) dents it 2 to 4 mm, read with a ruler; no soft spots, bruises or
cracks. Keep at room temperature (18 to 24 C) for the trial.

**Cutting the cheek (assessor, at reset, about 2 minutes).** Stand the mango on its stem end, narrow side toward
you. With a chef's knife, cut down about 6 mm to one side of the centre line, just clearing the flat seed, as in
the mango.org steps. Keep that cheek if its cut face is 105 to 120 mm long and 66 to 78 mm wide, it is 23 to 28 mm
deep at the middle (ruler across the face, depth gauge or skewer to the skin) and the cut face is whole (no seed
fibres torn out deeper than 3 mm); otherwise cut the other side or use a new mango. Weigh it (1 g scale) and
photograph the cut face with the ruler. Lay it skin side down at the lot's place with its narrower end (the fruit's
tip, away from the stem) toward the robot's left arm.

**Before the run.** Knife, ramekin and ruler as listed. Fit clean, food-safe, cut-resistant covers (EN 388 cut level
C or higher) over the gripper pads; their thickness counts in the grasp width. Two cameras: one at the side, level
with the ramekin's base, with a 1 mm scale; one above the board. Measure the cheek's depth at the middle and at both
ends of each planned cut line (skewer marked in mm) before the run.

**After the run (about 5 minutes).** Do not touch anything until the video has ended. Then:
1. Photograph the cheek from above with the ruler. If the skin was pushed up, the cubes stand apart with open gaps.
2. Count the cubes: pieces separated on every side by cuts (or by the 7 mm rim band), 80 to 800 mm2 on top and no
   more than 30 mm corner to corner (ruler).
3. Check depth: on each cut, part the flesh gently with the knife tip and push a skewer marked in mm to the bottom
   of the cut at both ends and the middle; compare with the depth measured there before the run. A cut counts where
   it reaches within 6 mm of the skin.
4. Check the skin: turn the cheek over and hold it against a bright light; any slit through the skin more than
   2 mm in from the edge is a cut through (score 0). (The simulation's rule, more than 1 mm into the 1.5 mm skin, is
   a stricter stand-in for a visible slit.) Read how far the middle of the skin was pushed from the side video
   against the 1 mm scale. Step through the side and top videos frame by frame for the 10 mm gap between the blade
   and the steadying gripper, using whichever view shows it clearly.

**Reset.** A new mango and cheek, the board, knife and ramekin washed and dried, new pad covers. All fruit is
thrown away.

**Safety.** The knife is a real sharp knife. Run real trials only with the rig's stop and guard procedures and
nobody within the arms' reach. The 10 mm blade rule protects the steadying gripper.

## What is simplified (fidelity)

- The cheek is one rigid 115 g body built from 58 stacked boxes in the shape of half an ellipsoid (drawing:
  `fixture/cheek.svg`). It does not bend, so the push is a declared rule: the ramekin passes into the rigid cheek
  (their collision is switched off), and 12 mm or more of overlap at the skin's middle counts as the skin pushed up
  that far, which flattens the cup and opens the cuts (a full inside-out turn is about 25 mm). The push is read only
  while the base's centre is below the cut face, so it reads at most 25.5 mm; past 35 mm anywhere on the base while
  the cheek is held counts as split. The cheek does not change shape on screen. Completed cuts are drawn as dark 4 mm squares on the cut
  face, shown as soon as the scorer records a cut reaching within 6 mm of the skin under them; they are a drawing
  only and play no part in scoring.
- PhysX cannot cut. The blade has no collider (the handle and guard do); scoring is the declared cell rule above,
  with the stated drag and feed forces so the knife hand has to push through and the other hand has to hold the
  cheek. The cubes are not separate bodies.
- The skin is 1.5 mm thick everywhere; real skin is 1 to 2 mm. Ripeness, flesh strength and the forces are estimates;
  no mango or knife was measured.
- The knife stands on its handle so a top-down gripper can take it; in a kitchen it would lie flat.

## Known limits (found, not yet fixed)

An independent review of the final source found three faults in `task.py`. They are recorded here and not yet fixed;
fixing any of them changes `task.py`, so the demo and Dream-1 results below would have to be run again.

- **Split check: a ramekin held above the cheek reads as a push.** `ramekin_push` measures each point of the
  ramekin's base against the cheek's outer skin with no upper bound. While a gripper holds the cheek, a base more
  than about 10 mm above the middle of the cut face (or a cheek held cut face down over the ramekin) therefore reads
  as more than 35 mm and zeroes the trial as split skin. The fix is to count a split only with the ramekin base up
  relative to the cheek and its rim below the cut face. The scripted demo never holds the ramekin over the cheek, so
  its scores do not depend on this.
- **Full credit needs only a 12 mm push.** A full turn-out is about 25 mm; at 12 mm the skin is still cupped and the
  cuts have only started to open, short of the instruction's "so the cubes fan out". The fix is to raise the 1.0
  push to 20 mm and press the demo's cheek 22 mm onto the ramekin.
- **The skin test samples only the blade's edge and spine.** A blade with its edge outside the fruit under the rim's
  overhang and its spine in the flesh could cross the skin between the two without being caught. The fix is to add
  rows of sample points across the blade face, for the skin test only.

## Demo

The scripted demo (`demo()`) plays fixed waypoints worked out from each lot's published layout and the frozen
corrections below; it reads no object state during a trial.

- The left gripper leans 45 degrees toward the far end and pinches the cheek's narrow near end across its width, at
  the top 4 mm of its sides, 50 mm from the middle of the cut face.
- The right gripper takes the knife by its handle (15 mm behind the handle's centre), tips it 45 degrees point down
  and makes four cuts across the cheek (at 38, 24, 8 and -6 mm along it, `CROSS_X`) and three along it (at -16, 0
  and 16 mm, `LONG_Y`, from the far edge to 11 mm past the middle toward the held end). Each cut starts 3.5 mm inside
  the edge (the cuts across also end 3.5 mm inside it) and moves point first, so the blade behind the point stays over
  deeper flesh: 14 mm/s, slowed to 4 mm/s over the last 12 mm of each cut across and to 7 mm/s over 8-25 mm and the
  last 10 mm of each cut along, where the point otherwise trails its waypoints. The point is aimed 3 mm above the
  skin (4 mm over the first 8 mm, 2 mm over the last 12 mm), following the cup down and up again, and never sent more
  than 0.5 mm below the skin's inner surface at the commanded spot itself. At the end of each cut the knife lifts
  straight out. Between the two sets of cuts the knife is levelled and turned beside the board. After the last cut
  the point backs 50 mm away from the held end and the knife is carried off high.
- It stands the knife back on its handle where it started and lets go; the left gripper lets go of the cheek and
  moves away.
- The right gripper pinches both sides of the cheek's rim at the middle, lifts it 45 mm, lowers it onto the
  upturned ramekin until the skin's lowest point is 16 mm below the ramekin's base, holds 0.8 s, lifts it off, sets it
  back where it was and lets go.

`CAL` in `task.py` holds fixed corrections for each lot and each set of cuts: the knife arm sits a little off, lags
behind along the cut and rides up under the stated cutting load, so the scoring waypoints lead along the cut by about
5 mm at full speed (scaled with speed, tapering to nothing over the last 12 mm) and aim a millimetre or so lower. In
both lots the last waypoint of each cut across is raised (2.5 mm in lot A, 1.5 mm in lot B, rising from the waypoint
8 mm before it): those cuts end close to the rim, and in traced runs the arm's stored lag sprang the point 2-3 mm on
into the rising skin when the stroke stopped. The corrections were measured from the task's own logs of traced runs and then frozen; the
demo never reads a live pose.

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Steady the mango cheek, score its flesh into a crosshatch grid down to the skin with the paring knife without cutting through the skin, press the skin side onto the upturned ramekin so the cubes fan out, then leave the cheek and the knife on the board and let go." \
  --scorer task_score --max-steps 2400 --epochs 2 --store-frames
```

`--epochs 2` runs lot A, then lot B; the score is their mean. `TASK_LOT=A` or `B` pins one lot. For Dream-1 use
`--policy dream`.

## Results

| Run | Lot A | Lot B | Mean |
|---|---|---|---|
| Scripted demo, final source | 1.0 | 1.0 | 1.0 |
| Dream-1, final source | 0.0 | 0.0 | 0.0 |
| Dream-1, earlier version 1 (before the cut-end and load-timing changes) | 0.0 | 0.0 | 0.0 |
| Dream-1, earlier version 2 | 0.0 | 0.0 | 0.0 |

Scripted demo, one trial per lot: all seven cuts made without cutting the skin, at least 10 cubes, the skin pushed up
onto the ramekin while pinched, the knife back on its handle and both hands clear and still; full score held at
111.5 s (lot A) and 110.9 s (lot B) of the 120 s limit.

Dream-1 on the final source ran to the 2,400-step limit in both lots without reaching a scoring level; in lot A the
left gripper went to the cheek and the right gripper to the knife handle, then both drew back. In the two earlier
complete runs the arms drew back without touching the knife or the cheek. Two further Dream-1 launches on earlier
versions were stopped during their first trial, unscored, when the task was changed.

## Shortcuts and what they score

`near_misses.py` drives the rules offline with moving knife and hand states (`python near_misses.py`):

| Shortcut | Result |
|---|---|
| Full grid, skin never pushed up | 0.75 |
| Grid cut 4.5 mm deeper than the planned path (1.5 mm into the skin) | Skin cut through: 0 |
| Grid cut 8.5 mm above the skin | No cut reaches within 6 mm of the skin, cubes stay joined: 0 |
| Full grid with no hand pinching the cheek | No cut counts: 0 |
| Knife left standing in the fruit | End condition fails: 0 |
| Skin pushed 26 mm onto the ramekin (a full inside-out turn) | 1.0 |
| Skin pushed 38 mm onto the ramekin | Split skin: 0 |
| Ramekin lifted and carried over the cheek, never pushed | 0.75 |
| Skin pushed only 8 mm | Not pushed far enough: 0.75 |
| Skin pushed up first, grid cut afterwards | Cuts after the push do not count: 0 |
| Blade lying flat, dragged along the lines | 0 (the flat blade's edge goes through the skin; a flat blade also earns no cut) |
| Blade turned across the lines, pushed flat side first | 0 (no cut counts, and the turned blade passes within 10 mm of the steadying gripper) |
| Cuts across only, none along | Strips, not cubes: 0.25 |
| Shallow scratch along the thin rim | Not inside the 7 mm rim band, so not a cut: 0 |
| Extra cut next to the steadying gripper | Blade within 10 mm of it: 0 |

## How this differs from close tasks

- TK-V7P9X9 (slit a carton seam short of the product 6 mm below): one straight cut at one depth over a flat layer;
  here the depth limit is a hidden cup, so the point has to rise and fall along every cut, and seven cuts must close
  cubes on all sides.
- TK-N9C8DA (hold a loaf while sawing a slice) and TK-53WVPU (cut a held cord): cut through the whole object; here
  the cut must stop just short of the skin.
- TK-VT2J8N (peel a cucumber) and TK-SANNN9 (scratch off a coating): remove an outer layer; here the outer layer
  must stay whole while the flesh above it is divided.
- TK-JRVAY3 (grate cheese): no knife, no depth limit.

## Files

`task.py` (scene, scoring, demo), `near_misses.py` (checker cases and offline controls), `fixture/cheek.svg`
(dimensioned drawing of the modelled cheek and the demo's cuts), `parts.py`, `lots.py`, `tilt.py` (shared task kit;
`task.py` replaces the kit's rotation-to-quaternion helper with a standard form that keeps half-turn rotations
right). Code MIT; text CC BY 4.0. No maker CAD, photos or logos are used.
