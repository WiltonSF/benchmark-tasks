# Free a UR5e's Festo DHEF gripper cap from its inner coupling: turn the catch, evert the silicone, push the release ring

A Universal Robots UR5e stands powered off at the far left of the bench, its wrist folded low so that the Festo
DHEF-20 adaptive shape gripper on its tool flange lies level and points across the bench toward the right-hand arm.
The gripper's air is exhausted. Its worn silicone inverting cap is held on its nose by a knurled bayonet catch ring.
The inside of the cap's tip carries a knob that is held, inside the cap, by the coupling on the end of the gripper's
piston rod. The robot must take the cap off, with its catch ring, and lay it on the grey service mat without
puncturing the silicone.

## Why it matters

- The DHEF grips by turning its silicone cap inside out around a part, so the cap is its wear part. Festo's operating
  instructions (DHEF, 8102537 [8102539], 2019-01) section 9: "Inspect the inverting cap for wear every day."
  Section 10.1, Replacing inverting cap: "1. Exhaust adaptive shape gripper (including inverting cap). 2. Open bayonet
  catch and evert inverting cap. 3. Press the inverting cap tip into the coupling and at the same time push the
  releasing ring towards the piston rod. This releases the inverting cap from the coupling." (copy:
  https://docs.rs-online.com/d47e/A700000014068131.pdf). The cap's own assembly sheet (DHAS-GA-B22-S, 8104615
  [8104617], https://docs.rs-online.com/4763/A700000013469651.pdf) shows the reverse: evert the cap, press its tip into
  the coupling until "the releasing ring snaps back audibly", turn the catch about 45 degrees until it locks.
- O*NET 17-3024.01, Robotics Technicians, core task: "Perform preventive or corrective maintenance on robotic
  systems or components."
- The gripper is sold ready for UR arms (DHEF-20-A-RA1, part 8119114: "compatible with ... Universal Robots
  UR3e/UR5e/UR10e/UR16e", catalogue 2021/06 p. 4). Changing its cap on the arm, the arm left in place, is a
  recurring chore of running such a cell; a robot that can do it takes one more human step out of keeping robots
  working.

## What the robot has to do, and why each part is hard

1. **Turn the catch open.** The black catch ring (57.5 mm across) turns about 45 degrees on the gripper's nose, from
   the white "locked" mark to the "open" mark (counterclockwise seen from the cap's tip). With the gripper lying
   level, that is a turn about a horizontal axis: a hand pinching the ring's sides must roll about the gripper's own
   axis, keeping the ring centred on it.
2. **Turn the cap inside out (evert it).** The cap's tip is held by the coupling at the end of the rod, deep inside
   the cap: nothing can reach it from outside. The rod is already fully forward, so drawing the opened catch ring
   forward along the axis rolls the silicone over itself: the tip stays where the rod holds it while the wall rolls
   back through the ring's bush, until, after about 130 mm of draw, the tip and the coupling with its blue releasing
   ring stand behind the catch ring, in the open (about 14 mm of wall stays as a cup in front of the ring: its rim is
   stretched over the bush's collar, as Festo's Fig. 6 shows). The draw is long (about 130 mm, more than the arm's own
   lag of about 25 mm under the rolling resistance) and the cap and ring hang on the rod's coupling all the way; in
   the model the tip and the ring stay on one axis, and pulled on past full eversion the silicone stretches.
3. **Release the coupling, two hands at once.** One hand holds the cap's carrier still in front of the coupling while
   the other pinches the 22 mm releasing ring, 70 mm away, and pushes it 1.5 to 3 mm toward the rod. While it is held
   there the coupling lets the knob go and the cap can be drawn off; let go of the releasing ring and the coupling
   closes again. The two hands work 70 mm apart on one axis, so their wrists must be kept out of each other's way.
4. **Lay the cap on the mat** (its place changes between lots), let go and move both hands away.

## Answers to questions a reviewer may ask

- **Is this the maker's procedure?** Yes, steps 1 to 3 of section 10.1 above, with the air already exhausted by the
  setter (step 1). The manual says to press the tip into the coupling while pushing the releasing ring; in this task
  the hand holding the carrier keeps the tip seated (the rod is at its forward stop) while the other pushes the ring.
  Fitting a new cap (the reverse, which needs the new cap's knob pressed into the coupling until it clicks) and
  unscrewing the knurled nut that holds the catch ring on the cap's bush (a bench step) are not part of the task:
  removal is the step that has to be done on the robot, and the one that needs the cap everted to reach a hidden
  coupling.
- **Why is the eversion more than a trivial flip?** The tip's knob cannot be reached any other way: the coupling sits
  inside the cap, behind its tip, and there is no opening. The robot has to make the cap roll inside out by drawing
  the catch ring a long way along the axis while the rod holds the tip, then find the exposed releasing ring. A pull
  before the catch is turned is blocked by its lugs; a pull on past full eversion only stretches the silicone and
  leaves the cap held (at most 0.5); a finger pushed into the silicone to get at the coupling counts as a puncture
  (0). Collar-release tasks on the board start with the collar in the open; here reaching it is half the job.
- **Is two-handedness forced?** The coupling closes again whenever the releasing ring is let go, so the ring must be
  held while the knob is drawn out. One hand cannot do both: the two places are 70 mm apart and pull in opposite
  directions. Any other route that frees the cap intact counts the same (for example holding the carrier and drawing
  the coupling away instead).
- **Is the silicone simulated?** Not as a soft body (see Fidelity). The cap wall is drawn rolled to where its tip is
  and is not collided; it resists rolling with about 4 N, stretches past full eversion at 2,000 N/m, and a finger
  3 mm or more inside it is scored as a puncture. These are stated rules with estimated numbers.
- **What does the real trial cost?** The gripper (DHEF-20-A, part 8092533, $1,336.06 at
  [Tameson](https://tameson.com/products/festo-adaptive-shape-gripper-66mm-stroke-size-20-double-acting-8092533) or
  EUR 1,319.92 at [Landefeld](https://www.landefeld.com/artikel/en/dhef-20-a-8092533-adaptive-shape-gripp/OT-FESTO085544),
  checked Nov 21; the RA1 UR kit adds the flange and valves) and a spare cap (DHAS-GA-B22-S, part 8097634, EUR 57.75
  at [Landefeld](https://www.landefeld.com/artikel/en/dhas-ga-b22-s-8097634-inverting-cap/OT-FESTO085842)). The UR5e is
  the cell's own robot, left powered off. A lab without a spare UR5e can bolt the gripper by its ISO 9409-1-50-4-M6
  flange to a rigid bracket at the same pose (axis level 80 mm above the bench, mounting face at the same place);
  with a steel bracket made to its mounting (about $30, an estimate) the parts cost about $1,450 in all;
  the arms' access to the cap is the same. Nothing is consumed per trial; the cap is refitted between trials. The
  gripper is the equipment being serviced, as the Franka arm is in the accepted hand-guiding task (TK-ZW8U3V): a
  printed stand-in cannot replace it, because the coupling, its latch and releasing ring inside the cap are
  unpublished Festo parts, and the task is about reaching and releasing exactly those.

## Nearest tasks, and how this differs

- **TK-462BUT (accepted), release a push-fit fitting and pull the pipe out:** a collet on an open, fixed fitting is
  held while a rigid pipe is drawn. Here the coupling is inside a closed silicone cap; reaching it needs the cap
  rolled inside out first, and the part drawn off is the soft cap on a carrier.
- **TK-GPLTLT (accepted), lock a bayonet cap into its socket:** keyed push-and-turn on a vertical socket. Here the
  catch is only the first quarter of the task, turned about a horizontal axis on a mounted gripper.
- **TK-NT76KH (accepted), fit an O-ring onto a hydraulic plug:** stretching an elastomer ring into a groove. Here the
  elastomer is a closed cap that must be rolled through its own bush; pulled on past full eversion it only
  stretches and the coupling still holds it (at most 0.5).
- **TK-ZKXT33 (accepted), replace a worn soft gripper finger:** a finger swapped with screws on a spare gripper on the
  bench. Here the gripper stays on the robot and the wear part is held by a bayonet and an internal coupling.
- **TK-8NDRZH (accepted), disconnect a pneumatic line from a push-to-connect port:** a collar held while a tube is
  drawn from an open port. Same release idea as TK-462BUT; no hidden coupling and no eversion.

## Run a trial

From this folder, in the Isaac Lab-Arena environment of the program's simulation walkthrough (yam-isaac-lab-arena,
the program's YAM embodiment for Isaac Lab-Arena). Trial k uses lot k mod 2, so `--epochs 2` runs both lots and
`task_score` is their mean:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the worn inverting cap, with its bayonet catch, off the Festo gripper on the robot arm and lay it, intact, on the grey mat." \
  --scorer task_score --max-steps 3600 --epochs 2 --store-frames --log-dir ~/yam-eval/dhef-cap-dream
```

- The scripted demo: the same command with `--policy yam_scripted -E cam_width=640 -E cam_height=360`. It uses
  `tilt.py` (in this folder) for the right hand's lean about the gripper's axis; that affects only the scripted demo.
- `DHEF_DEMO=yank` in front of the scripted command plays a shortcut instead of the demo (a check): the catch is
  turned and the cap drawn on 120 mm past where the demo stops, without releasing the coupling, then let go. The cap
  must stay held by the coupling: at most 0.5.
- `TASK_LOT=A` or `TASK_LOT=B` in front pins every trial to one lot (debugging only).
- Runs use `object_jitter=0`. The UR5e and gripper are fixed; only the mat's place changes between lots.
- The simulation prints one `[dhef]` line a second: how far the cap has rolled (`evert`, mm), the rod's and the
  releasing ring's travel, whether the coupling is open, the cap's pose and speed, the tier reached, the deepest
  finger into the silicone so far, and whether the hands are clear.

## Scoring

Every trial is scored by `score()` in `task.py`; `task.json` has the short text. Lengths in mm.

**Whole trial (checked every control step):** 0 for the trial if

- a robot finger pushes into the silicone: any point of either hand's gripper or finger collision hulls (faces
  sampled every 4 mm) lies 3 mm or more inside the cap wall's outline, rolled to the cap's current eversion
  (`puncture_depth()`); in a real trial (yes or no), a finger seen in the video pressing a visible dent into the cap
  wall, or a hole or tear in the removed cap held against a lamp. The wall has no collider in the simulation, so a
  finger there would have passed through the real cap; the 3 mm is the simulation's allowance for a finger grazing
  the wall, which in a real trial leaves no visible dent;
- the cap or the mat leaves the table (a corner more than 10 mm below the table top, or off its edge);
- the hands or the parts cannot be read at any step (the scorer fails closed).

**Let go and still:** every score needs both hands' gripper and finger hulls at least 30 mm from the cap assembly and
from the gripper's front (catch region, rod, coupling, releasing ring, keepers), each taken as a solid cylinder round
its axis, and the cap at rest (under 10 mm/s and 0.5 rad/s), for 20 control steps (1 s) in a row.

| Score | Simulation (`task.py`) | Real trial |
|---|---|---|
| 1.0 | the back of the tip's knob 5 mm or more in front of the coupling's front face (or 15 mm off its axis), and every corner of the carrier (ring, bush, collar) over the mat's outline, its lowest point within 3 mm of the mat's top, the knob above the mat | overhead video frame and photo: cap and ring lying inside the mat's edges, nothing touching them |
| 0.75 | off the coupling as above, anywhere else (on the table, hanging, propped) | video: the cap is free of the gripper but not on the mat |
| 0.5 | still on the coupling and everted 122.5 mm or more: the cap's tip 20 mm or more behind the ring's back face | side photo against the rule: the tip at least 20 mm behind the ring's back edge, the blue releasing ring in view |
| 0.25 | the ring drawn 10 mm or more forward along the axis from its start (its lugs have passed the lip), or 15 mm off the axis | side photo against the rule: ring's back edge 10 mm or more in front of the body |
| 0 | otherwise, or any whole-trial rule broken, or a hand within 30 mm or the cap moving at the end | video and photos |

Trials run on lot A, then lot B; `task_score` is their mean.

### What each way of doing it scores (cases in `near_misses.py`)

| Strategy | Score |
|---|---|
| Turn the catch open and stop | 0 |
| Pull the catch ring without turning it | 0 (physics: its lugs stay behind the lip; `near_misses.py` checks the overlap) |
| Turn and draw the ring 9.9 mm / 10 mm | 0 / 0.25 |
| Roll the cap 122.4 mm (tip 19.9 mm behind the ring) / 122.5 mm | 0.25 / 0.5 |
| Evert the cap and pull on hard without releasing the coupling (`DHEF_DEMO=yank`, GPU) | at most 0.5: the cap stays held (see Results) |
| Reach in through the silicone to the coupling (any finger 3 mm or more into the cap) | 0 |
| Evert, release, leave the cap hanging free of the coupling | 0.75 |
| Push the rod back so the coupling leaves a held cap's knob by 5 mm | 0.75 |
| Cap off, dropped on the table beside the mat, or 4 mm above the mat (held up) | 0.75 |
| Cap off and on the mat, ring 1 mm over the mat's edge | 0.75 |
| Cap off, lying on the mat, hands clear, still 1 s (any turn of the ring) | 1.0 |
| Any of the above with a hand still within 30 mm, or still for under 1 s | 0 |
| Cap or mat knocked off the table at any moment | 0 |

`near_misses.py` also checks the declared mechanisms on import: the rolling force never exceeds 4 N before full
eversion, the stretch force is 2,000 N/m past it, a point about 4 mm inside the cap wall counts as a puncture, a point
on the catch ring's outside does not, at full eversion the left hand's pinch on the releasing ring stays out of the
silicone, and the locked lugs lie behind the lip while the turned ones line up with its gaps.

## The scripted demo

`demo()` drives both arms open loop from what the cameras show: the gripper, its catch ring and the mat's place (lot
A or B decides only where the cap is laid). It never reads the cap's joints, the latch state or exact poses.

1. The left arm first backs straight out from beside the UR5e's wrist and waits, parked.
2. The right hand comes down over the catch ring, its fingers closing across the gripper's axis, and pinches the
   ring 3 mm into each side.
3. It leans 50 degrees about the gripper's axis (fingertips toward +x), turning the ring until its lugs reach the
   open stop at 45 degrees.
4. Still leaning, it draws the ring 162.5 mm along the axis toward itself over 8 s. The rod is at its forward stop, so
   the cap rolls inside out: the tip stays on the coupling while the ring and the cap wall pass over it, until the
   tip, the coupling and the blue releasing ring stand behind the ring. The arm lags its target by about 25 mm, so the
   ring ends near full eversion (132.5 mm) without stretching the cap.
5. The left hand comes down over the releasing ring (fingers across the axis) and pinches it, 70 mm from the right
   hand, then pushes it 20 mm (commanded) toward the rod: the ring moves to its 3 mm stop and the latch opens.
6. Holding still, the right hand draws the cap 35 mm further: the knob leaves the open coupling.
7. The left hand lets go and parks; the right hand lifts the cap assembly, straightens up, lays it on the mat (6 mm
   drop), lets go and parks.

## Fidelity

| Value | Source |
|---|---|
| DHEF-20: 224 mm long, flange 62.5 mm, catch ring 57.5 mm, cap 82.5 mm long, 41 and 25 mm across; stroke 66 mm; 475 g; cap 18 g, VMQ | Festo catalogue DHEF 2021/06, pp. 8, 9, 11, 12 |
| Catch turns about 45 degrees; cap tip pressed into the rod's coupling; releasing ring pushed toward the rod to release | Festo operating instructions 8102539 s. 10.1 and cap sheet 8104617 Figs. 6, 7 |
| Permissible transverse load on the rod 2.3 N (at full stroke, in operation) | catalogue p. 8 (not modelled as damage: see below) |
| UR5e joint centres: d1 162.5, a2 425, a3 392.2, d4 133.3, d5 99.7, d6 99.6 mm; base 149 mm | Universal Robots' published DH parameters |
| Bayonet lugs, lip and stops; nose 38 mm; bush bore 30 mm; rod 10 mm; coupling 19 mm; releasing ring 22 mm, 3 mm travel; the knob's head 13 mm; carrier 65 g, tip 8 g | estimates (inside the parts; not published) |
| Rolling the cap: 4 N; stretching past full eversion: 2,000 N/m; exhausted rod's seals: 6 N to move | estimates (VMQ wall about 1 mm, 30 mm across; Festo seal friction of a 20 mm cylinder); a spring scale on the ring checks the first two in a minute |
| Latch opens with the releasing ring 1.5 mm or more toward the rod, closes under 1.0 mm | stated rule |

Simplified: the cap wall is not a soft body. It is drawn as one of 17 meshes, rolled to where its tip is (the fold
halfway between the tip and the wall's length), and has no collider; its resistance is a stick-slip joint between the
carrier and the tip (4 N to roll, 2,000 N/m past full eversion), and fingers are kept out of it by the puncture rule.
The tip and the carrier stay on one axis (a real everted cap can sag a little). The latch is two keepers in front of
the knob's head, opened by a stated rule from the releasing ring's travel (a real ball coupling's balls drop into the
ring's relief). The rod and everything else are rigid: a real 10 mm steel rod does not bend under the 20 to 40 N the
arms can exert, so rod damage is not scored. The knurled nut on the cap's bush stays fastened (fitting a new cap is the
bench step). The UR5e is fixed (powered off, brakes on). The layout is fixed per lot: two lots stop a single fixed
habit (laying the cap at one place), not a memorised replay of both.

## Real trial

- Hardware: a Festo DHEF-20-A on a rigid bracket at the pose in `task.json` (about $1,450 with a spare cap; see
  above), or a UR5e with a DHEF-20-A-RA1 jogged to that pose and switched off with its brakes on; the gripper's three
  air ports exhausted (the cap unpressurised), rod pushed fully forward by hand.
- A steel rule (300 mm) taped to the bench under the gripper's axis, zero at the mounting face, read in a side
  camera's video; overhead video at 20 fps or more.
- After each trial: hold the removed cap against a lamp and look for holes or tears; refit it (Reset notes).
- Safety: the UR5e is off and braked; the gripper unpressurised; nothing is sharp, hot or live.

## Results

All runs on one NVIDIA L40S, yam-isaac-lab-arena on Isaac Lab-Arena, `object_jitter=0`, both lots (`--epochs 2`)
unless noted.

| Run | What | Lot A | Lot B | Notes |
|---|---|---|---|---|
| Final demo | `yam_scripted`, 3,600-step limit | 1.0 | 1.0 | catch turned to its open stop, cap rolled 134.5 mm (2 mm past full eversion, then back to 132.5 when released), releasing ring pushed to its 3 mm stop, cap drawn off and laid on the mat, released and still; each trial ended in success at 38 s (765 steps); no finger ever inside the cap wall |
| Dream-1 | `dream`, 3,600-step limit | 0.0 | 0.0 | both trials ran to the 3,600-step limit without refusing; the arms moved (joints swept up to about 1.4 rad) but never came within 30 mm of the cap or the gripper's front, and the cap was never touched |
| Shortcut check | `DHEF_DEMO=yank`, lot A, 1,000 steps | 0.5 | not run | the catch turned and the cap drawn on 120 mm past the demo's stop: the cap stretched 10 mm past full eversion (about 20 N) and the coupling held it; let go, the cap hung everted on the coupling (0.5: everted, still held) |

How the demo got there (traces, same scene):

- First trace: 0 in lot A. The right hand turned the catch only 41 degrees and the ring's lugs, drawn as flat boxes,
  caught on the edge of the lip's gaps, so the hand slid off the ring. The gaps were widened to 50 degrees and the lugs
  narrowed to 18 (the turn now has 10 degrees of play either way) and the stops moved out of the lugs' way.
- Second trace: 1.0 in lot A, but the cap rolled only 119 mm (the arm lags its target by about 25 mm against the 4 N
  rolling resistance) and the releasing ring reached only 1.8 mm. The draw and the push were lengthened.
- The UR5e was moved 100 mm further back and the gripper 20 mm lower, so the left arm's start pose and its first move
  clear the UR5e's upper arm and wrist; the left arm now backs straight out before parking.
- Third trace (both lots): 1.0 and 1.0, cap fully everted, releasing ring at its stop. The final demo is the same plan
  on the final code.

The simulation logs one `[dhef]` line a second; in the final demo the catch ring turned 45.5 degrees (its stop), the
rod stayed at its forward stop while the cap rolled, the latch opened when the releasing ring reached 2.98 mm, and the
cap ended lying on the mat on a flat of its ring, with the hold counted from the release.

## Licences

No third-party meshes or assets in this folder: every shape is a box, cylinder, prism or triangle mesh written in
`task.py` from the makers' published dimensions and stated estimates. `parts.py`, `lots.py` and `tilt.py` are helper
modules shipped with the task. The YAM arm models, the station scene, its cameras and the `yam_arena` embodiment come
from the program's yam-isaac-lab-arena environment, used under the licences it ships with (the YAM description: MIT;
Isaac Lab and Isaac Lab-Arena: BSD-3-Clause; Isaac Sim: NVIDIA's Isaac Sim licence).
