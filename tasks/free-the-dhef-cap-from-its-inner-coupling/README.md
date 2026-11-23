# Free a UR5e's Festo DHEF gripper cap from its inner coupling: turn the catch, evert the silicone, push the release ring

**Simulation scene:** a Universal Robots UR5e stands powered off at the far left of the bench, its wrist folded low so that the Festo
DHEF-20 adaptive shape gripper on its tool flange lies level and points across the bench toward the right-hand arm.
The gripper's air is exhausted. Its worn silicone inverting cap is held on its nose by a knurled bayonet catch ring.
The inside of the cap's tip carries a knob that is held, inside the cap, by the coupling on the end of the gripper's
piston rod. The robot must take the cap off, with its catch ring, and lay it on the grey service mat without
puncturing the silicone.

## Changes in this revision

Earlier verdict (TK-HWDN73), in full:

> **Not accepted** (TK-HWDN73).
>
> Feasibility (not met): The real trial requires a roughly $1,340 specialist gripper with no route costing only a few
> hundred dollars.
>
> What to change: The specified real-trial gripper costs about $1,340, beyond the program's practical equipment
> budget. Please provide a repeatable acquisition route within a few hundred dollars, or a low-cost reproducible
> equivalent with the files or parts needed to rebuild the cap, bayonet and coupling while preserving the actual
> removal mechanics.
>
> To submit a revision, push your changes to this pull request (no need to open a new one). Your idea stays
> reserved for 48 hours from this review; if no revision arrives by then, the hold lapses and another team can
> claim the idea.

What changed, point by point:

1. *"The real trial requires a roughly $1,340 specialist gripper with no route costing only a few hundred
   dollars."* The real trial no longer uses a DHEF gripper, a UR5e or compressed air. It uses a printed replica of the
   gripper's front end with a genuine Festo cap (below, "What does the real trial cost?" and "Real trial"). Its parts
   and tools have US$302.48 in listed purchases plus US$86.40 estimated commercial printing: **US$388.88
   before other shipping and tax** (`fixture/BOM.csv`, sources opened 2026-11-23). The estimate includes a new
   genuine cap, mat, clamps, measuring tools and assembly hand tools; no printer purchase is needed. It is a
   budget, not a supplier quote or a delivered-price ceiling. The earlier expensive gripper route is removed.
2. *"The specified real-trial gripper costs about $1,340, beyond the program's practical equipment budget."* The
   gripper's price is no longer part of the task; the simulation still shows the gripper on a UR5e, as a cell has it.
3. *"Please provide a repeatable acquisition route within a few hundred dollars, or a low-cost reproducible
   equivalent with the files or parts needed to rebuild the cap, bayonet and coupling while preserving the actual
   removal mechanics."* The proposed acquisition and rebuild route is:
   - **Cap:** the genuine Festo DHAS-GA-B22-S (part 8097634) is bought as a spare part (US$89.49, Zoro; Landefeld alternative EUR57.75). A
     printed mould and knob insert for casting a cap in platinum silicone are included as a fallback.
   - **Bayonet and coupling:** `fixture/` holds `make_replica.py`, which builds every printed part from the dimensions
     in `task.py`, the STLs, drawings with the key dimensions (`drawing.pdf`), the parts list (`BOM.csv`), build steps
     (`BUILD.md`) and the sizing and acceptance checks (`ENROLMENT.md`). The parts are the nose with its three-lug
     bayonet lip and stops, the catch ring with the cap's bush, the rod and its forward stop, the coupling with
     keepers, and the releasing ring that opens them, plus a bracket that holds the axis at the task's pose.
   - **Proposed removal sequence:** the catch must be turned until its lugs align with the lip gaps, near the 45-degree open mark, before the ring can be drawn off (locked, its lugs are
     behind the lip); the cap must be rolled inside out to reach the coupling, which is inside it; the coupling lets
     the knob go only while the releasing ring is pushed toward the rod, and a spring closes it again when the ring
     is let go. The CAD encodes the turn/evert/hold-sleeve/withdraw sequence; this has not established preservation of the actual loaded removal mechanics. Exact Festo internals and forces are not verified.
     The manual's tip-press requirement is not independently enforced in simulation. Exhausting the air has no
     counterpart on the unpowered replica.
   - **Nominal design numbers shared with the simulation:** the generator reads `task.py` and checks the meshes and design values
     against it: catch ring, lugs (18 degrees), lip gaps (50 degrees), the 45 degree turn, nose, bush, rod (10 mm,
     55 mm travel), coupling (19 mm), keepers (3.5 mm from the axis, 4 mm travel), releasing ring (22 mm, 3 mm
     travel), knob, and the axis at 80 mm. 80 checks pass, none fail (`fixture/CHECKS.txt`). The return spring is a
     catalogue part whose published rate is the simulation's 1.00 N/mm. The rod's 6 N drag is an enrolment target, to be set with the brake screw
     and measured with a spring scale.
   - **What is not the same:** the latch's inside and the bayonet's internals are a design, because Festo has not
     published them (`task.py` already calls them estimates). The replica releases the knob from 1.5 mm of ring
     travel, like the simulation, but holds it again as soon as the ring is back under 1.5 mm, where the simulation
     keeps it open until 1.0 mm. A spacer (14 mm across, 33.5 mm long) and the return spring (12 mm across) sit on
     the replica's rod in front of the nose. Refitting a cap on the replica needs the releasing ring held pushed (the
     knob cannot snap in as on the gripper). The cap's knob is not in Festo's data: the replica is sized to the cap in hand by caliper, and the trial
     report states any difference from `task.py`'s estimate.
   - **Status:** nothing has been printed, assembled or measured. The forces are design values or estimates
     (`fixture/ENROLMENT.md` gives their derivation and the checks that measure them).
4. `task.py`, helpers, scene, scorer and demo are unchanged. The instruction now states the ordered manual steps,
   including simultaneous tip seating and sleeve operation. The reused runs below used the earlier goal-only
   instruction; neither establishes the additional tip-seating requirement. Text now states the implemented
   table-origin rule exactly, corrects historical trace precision, links the sources, and separates CAD reproduction
   from genuine-cap fitting and physical qualification.

**What the replica reproduces:** nominal exterior stations, bayonet lugs/gaps/stops, axial rod travel and a proposed
passive keeper/sleeve mechanism derived from `task.py` dimensions. It does not reproduce measured Festo internals.
The genuine cap's head diameter, neck diameter, head length, usable undercut and unstretched rim ID must be measured
with a caliper; those inputs set the fitted pocket, keeper clearance, cam slope and rim seat (`fixture/ENROLMENT.md`).
The supplied STLs use the unmeasured 13/6/10 mm nominal knob. Neither those STLs nor the casting fallback establish
genuine-cap compatibility. The first fitted, force-tested, ten-cycle specimen is still required.

| Removal step | Replica | Status |
|---|---|---|
| Turn the catch 45 degrees, then draw it off (blocked when locked) | same lugs, lip gaps and stops as `task.py` | CAD-checked; not printed |
| Roll the cap inside out to reach the coupling | the genuine Festo cap | the cap is the maker's; its knob and rim are sized by caliper on arrival |
| Hold the releasing ring pushed while the knob is drawn out; it closes when let go | designed latch, same travel (opens from 1.5 mm), catalogue spring at 1.00 N/mm | CAD-checked; fit to the genuine knob unverified (fallback: cast cap with `task.py`'s knob) |

## Why it matters

- The DHEF grips by turning its silicone cap inside out around a part, so the cap is its wear part. Festo's operating
  [operating instructions](https://docs.rs-online.com/d47e/A700000014068131.pdf) (DHEF, 8102537 [8102539],
  2019-01), section 9, require daily wear inspection; they do not prescribe daily replacement. Section 10.1 calls
  for exhausting the gripper, opening the bayonet and everting the cap, then simultaneously seating its tip into
  the coupling and pushing the release ring toward the rod. The cap's [assembly sheet](https://docs.rs-online.com/4763/A700000013469651.pdf)
  (DHAS-GA-B22-S, 8104615 [8104617]) shows the reverse seating and bayonet-locking operation.
- [O*NET 17-3024.01, Robotics Technicians](https://www.onetonline.org/link/summary/17-3024.01), lists preventive or
  corrective robot maintenance. This supports the general duty, not a claim that 500,000 workers perform this exact
  cap-removal step or that it is routine assembly work.
- The gripper is sold ready for UR arms (DHEF-20-A-RA1, part 8119114: "compatible with ... Universal Robots
  UR3e/UR5e/UR10e/UR16e", [Festo catalogue](https://ftp.festo.com/Public/PNEUMATIC/SOFTWARE_SERVICE/Documentation/2024/TR/DHEF_EN.PDF),
  2021/06 p. 4). This scene removes a worn cap from an installed, braked arm. It demonstrates end-effector
  maintenance, not building or refitting an end effector during assembly and commissioning. The retained
  `Impact on AI timelines` category therefore remains unsupported under a robot-building-only criterion.
  [TK-ZKXT33 / PR 800](https://github.com/robocurve/benchmark-tasks/pull/800) is an accepted nearby finger-replacement
  task on a spare gripper; that precedent does not establish that this installed-arm maintenance task qualifies.

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
   there the coupling lets the knob go and the cap can be drawn off; releasing the ring requests closure
   (spring return in the replica; driven keepers in simulation). The two hands work about 70 mm apart on one axis, so their wrists must be kept out of each other's way.
   Simulation closure has a 1.5/1.0 mm hysteresis band; the physical cam re-engages below 1.5 mm.
4. **Lay the cap on the mat** (its place changes between lots), let go and move both hands away.

## Answers to questions a reviewer may ask

- **Is this the maker's procedure?** It follows the turn/evert/sleeve-push/withdraw sequence in section 10.1 above, with the air already exhausted by the
  setter (step 1; the real-trial replica has no air, so only steps 2 and 3 apply to it). The manual says to press the tip into the coupling while pushing the releasing ring; the simulation does not independently require or measure that simultaneous tip press: keeper drives follow
  sleeve travel. The replica's cam carries the hand's sleeve force to the keepers; tip seating unloads their faces.
  Physical release force, fit and durability still need the enrolment checks.
  Fitting a new cap (the reverse, which needs the new cap's knob pressed into the coupling until it clicks) and
  unscrewing the knurled nut that holds the catch ring on the cap's bush (a bench step) are not part of the task (on the replica, a cap is refitted with the releasing ring held pushed):
  removal is the step that has to be done on the robot, and the one that needs the cap everted to reach a hidden
  coupling.
- **Why is the eversion more than a trivial flip?** The tip's knob cannot be reached any other way: the coupling sits
  inside the cap, behind its tip, and there is no opening. The robot has to make the cap roll inside out by drawing
  the catch ring a long way along the axis while the rod holds the tip, then find the exposed releasing ring. A pull
  before the catch is turned is blocked by its lugs; a pull on past full eversion only stretches the silicone and
  leaves the cap held (at most 0.5); a finger pushed into the silicone to get at the coupling counts as a puncture
  (0). Collar-release tasks on the board start with the collar in the open; here reaching it is half the job.
- **Why does the demo use two hands?** The coupling closes again whenever the releasing ring is let go, so the ring must be
  held while the knob is drawn out. The demo uses two hands on places about 70 mm apart, pulling in opposite
  directions. This is a demonstrated coordination strategy, not proof that every one-handed strategy is impossible. Any other route that frees the cap intact counts the same (for example holding the carrier and drawing
  the coupling away instead).
- **Is the silicone simulated?** Not as a soft body (see Fidelity). The cap wall is drawn rolled to where its tip is
  and is not collided; it resists rolling with about 4 N, stretches past full eversion at 2,000 N/m, and a finger
  3 mm or more inside it is scored as a puncture. These are stated rules with estimated numbers.
- **What does the real trial cost?** The kit budget is **US$388.88**, comprising US$302.48 in listed purchases
  and US$86.40 of estimated printing. This includes one new genuine Festo cap
  ([Zoro US$89.49](https://www.zoro.com/festo-inverting-cap-dhas-ga-b22-s-dhas-ga-b22-s/i/G702750279/)),
  a 10 x 200 mm polished steel rod (US$10.28), collar pack (US$13.89), one Lee Spring LC 036G 06 M (US$8.31),
  hardware, bracket/base, clamps, grey Desco 42554 mat (US$44.24), caliper, spring scale, rule, lubricant,
  marker, and the iron, keys, screwdriver and knife used for assembly. All prices/URLs were opened on 2026-11-23;
  full pack costs, quantities, availability limitations and alternatives are in `fixture/BOM.csv`.
  Large PETG parts are budgeted at US$65 (500 g x US$0.12/g + US$5 setup,
  [Triple G Workshop](https://www.triplegworkshop.com/home/pricing)); the seven small tough-resin parts at
  US$21.40 (20 g x US$0.07/g + 4 h x US$5/h,
  [Protoats](https://www.protoats.com/3D/printing.html)). Those weights and resin hours are design estimates,
  not sliced measurements; no quote has been obtained and the shops have not confirmed these tolerances.
  Shipping and tax are extra except the US$9.95 scale shipping allowance shown by its seller. A table, the two
  test arms, cameras and a lamp for inspection are existing trial infrastructure, excluded from this kit budget.
  Cap delivery is not stated by Zoro; the [Landefeld alternative](https://www.landefeld.com/artikel/en/dhas-ga-b22-s-8097634-inverting-cap/OT-FESTO085842)
  exposes an in-stock EUR57.75 offer (no currency conversion included). Nothing has been ordered. The cap is reused
  until damaged; reset is estimated at 2–3 minutes. No full DHEF, UR5e or compressed air is required.

## Nearest tasks, and how this differs

- **[TK-462BUT / PR 662](https://github.com/robocurve/benchmark-tasks/pull/662) (accepted), release a push-fit fitting and pull the pipe out:** a collet on an open, fixed fitting is
  held while a rigid pipe is drawn. Here the coupling is inside a closed silicone cap; reaching it needs the cap
  rolled inside out first, and the part drawn off is the soft cap on a carrier.
- **[TK-GPLTLT / PR 34](https://github.com/robocurve/benchmark-tasks/pull/34) (accepted), lock a bayonet cap into its socket:** keyed push-and-turn on a vertical socket. Here the
  catch is only the first quarter of the task, turned about a horizontal axis on a mounted gripper.
- **[TK-NT76KH / PR 1576](https://github.com/robocurve/benchmark-tasks/pull/1576) (accepted), fit an O-ring onto a hydraulic plug:** stretching an elastomer ring into a groove. Here the
  elastomer is a closed cap that must be rolled through its own bush; pulled on past full eversion it only
  stretches and the coupling still holds it (at most 0.5).
- **[TK-ZKXT33 / PR 800](https://github.com/robocurve/benchmark-tasks/pull/800) (accepted), replace a worn soft gripper finger:** a finger swapped with screws on a spare gripper on the
  bench. Here the gripper stays on the robot and the wear part is held by a bayonet and an internal coupling.
- **[TK-8NDRZH / PR 1784](https://github.com/robocurve/benchmark-tasks/pull/1784) (accepted), disconnect a pneumatic line from a push-to-connect port:** a collar held while a tube is
  drawn from an open port. Same release idea as TK-462BUT; no hidden coupling and no eversion.

## Run a trial

From this folder, in the Isaac Lab-Arena environment of the program's simulation walkthrough (yam-isaac-lab-arena,
the program's YAM embodiment for Isaac Lab-Arena). Trial k uses lot k mod 2, so `--epochs 2` runs both lots and
`task_score` is their mean:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Turn the DHEF catch, evert the cap, seat its tip while holding the release ring pushed, withdraw it intact onto the grey mat, let go, and clear both hands." \
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
- any corner of the cap's rigid carrier or tip collision boxes falls more than 10 mm below the table top; or the
  cap origin or mat centre has x <= -180 mm, x >= 900 mm, or |y| > 640 mm; or the mat centre falls more than 10 mm
  below the table top. Coordinates use the setup frame. These are the code's origin bounds, not geometric table-edge
  containment: mat corners, horizontal cap corners and the visual silicone outline are not checked. A mat with
  its centre at x=850 mm can overhang the actual table edge and still pass this rule;
- the hands or the parts cannot be read at any step (the scorer fails closed).

**Let go and still:** every score needs both hands' gripper and finger hulls at least 30 mm from the cap assembly and
from the gripper's front (catch region, rod, coupling, releasing ring, keepers), each taken as a solid cylinder round
its axis, and the cap at rest (under 10 mm/s and 0.5 rad/s), for 20 control steps (1 s) in a row.

| Score | Simulation (`task.py`) | Real trial |
|---|---|---|
| 1.0 | the back of the tip's knob at least 5 mm in front of the coupling's front face, or at least 15 mm off its axis; every rigid carrier-box corner inside the mat outline; carrier lowest point from 1 mm below to 3 mm above the computed mat top; tip-box lowest point at least mat top minus 1 mm. This checks the rigid carrier, not the whole silicone outline. | overhead video frame and photo: detached cap assembly resting on the mat, the rigid carrier inside its edges, nothing touching it; the whole silicone outline is not a simulation containment check |
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
| Hold the releasing ring pushed (latch open) and push the rod back, instead of drawing the cap, until the coupling leaves the held cap's knob by 5 mm (with the latch closed the keepers drag the knob along) | 0.75 |
| Cap off, dropped on the table beside the mat, or 4 mm above the mat (held up) | 0.75 |
| Cap off and on the mat, ring 1 mm over the mat's edge | 0.75 |
| Cap off, lying on the mat, hands clear, still 1 s (any turn of the ring) | 1.0 |
| Any of the above with a hand still within 30 mm, or still for under 1 s | 0 |
| A cap rigid-box corner below -10 mm, cap/mat origin outside the stated x/y bounds, or mat centre below -10 mm, at any moment | 0 |
| Mat partly overhanging the actual table but centre within the bounds; detached cap supported on it, hands clear and still | can score 1.0; uncorrected geometry-containment limitation |

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
| DHEF-20: 224 mm long, flange 62.5 mm, catch ring 57.5 mm, cap 82.5 mm long, 41 and 25 mm across; stroke 66 mm; 475 g; cap 18 g, VMQ | [Festo catalogue DHEF 2021/06](https://ftp.festo.com/Public/PNEUMATIC/SOFTWARE_SERVICE/Documentation/2024/TR/DHEF_EN.PDF), pp. 8, 9, 11, 12 |
| Catch turns about 45 degrees; cap tip pressed into the rod's coupling; releasing ring pushed toward the rod to release | [Festo operating instructions 8102539](https://docs.rs-online.com/d47e/A700000014068131.pdf) s. 10.1 and [cap sheet 8104617](https://docs.rs-online.com/4763/A700000013469651.pdf) Figs. 6, 7 |
| Permissible dynamic transverse load on the rod 2.3 N at maximum cantilever | [Festo catalogue](https://ftp.festo.com/Public/PNEUMATIC/SOFTWARE_SERVICE/Documentation/2024/TR/DHEF_EN.PDF) p. 8; this is NOT a sleeve spring or release force, and is not modelled as damage |
| UR5e joint centres: d1 162.5, a2 425, a3 392.2, d4 133.3, d5 99.7, d6 99.6 mm; base 149 mm | [UR DH parameters](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/) for joint offsets; [UR5e specification, 08/2023, base-flange drawing](https://www.universal-robots.com/media/1807465/ur5e_e-series_datasheets_web.pdf) for the 149 mm base |
| Bayonet lugs, lip and stops; nose 38 mm; bush bore 30 mm; rod 10 mm; coupling 19 mm; releasing ring 22 mm, 3 mm travel; the knob's head 13 mm; carrier 65 g, tip 8 g | estimates (inside the parts; not published); the real-trial replica is designed to the same sizes (`fixture/CHECKS.txt`; not yet built), its knob/keeper insert sized to the cap in hand; adapted dimensions must be logged separately from the nominal simulation |
| Rolling the cap: 4 N; stretching past full eversion: 2,000 N/m; exhausted rod's seals: 6 N to move | estimates (VMQ wall about 1 mm, 30 mm across; Festo seal friction of a 20 mm cylinder; derivation in `fixture/ENROLMENT.md`); use a spring scale to record rolling force versus draw; measurement duration has not been timed; the stretch is the genuine cap's own; the replica's rod drag is set to 6 N |
| Latch opens with the releasing ring 1.5 mm or more toward the rod, closes under 1.0 mm | stated rule in simulation (a drive moves the keepers when the ring's travel crosses these values); on the nominal replica the ring's drawbars open the keepers through cam slots, clear of the knob from 1.5 mm, closed again under 1.5 mm, return spring 1.00 N/mm plus 0.3 N preload (`fixture/BUILD.md`) |

Simplified: the cap wall is not a soft body. It is drawn as one of 17 meshes, rolled to where its tip is (the fold
halfway between the tip and the wall's length), and has no collider; its resistance is a stick-slip joint between the
carrier and the tip (4 N to roll, 2,000 N/m past full eversion), and fingers are kept out of it by the puncture rule.
The tip and the carrier stay on one axis (a real everted cap can sag a little). The latch is two keepers in front of
the knob's head, opened by a stated rule from the releasing ring's travel: in simulation the keepers are driven open
while the ring is pushed 1.5 mm or more (the elastic spring term is 1.5 N at 1.5 mm, plus damping/contact effects; keeper-drive force is not mechanically charged to the sleeve); on the real-trial replica
the ring itself opens them through cam slots. The rod and housing are rigid; bending, lateral-load damage and fatigue are not scored. No measured safe-load claim is made. The knurled nut on the cap's bush stays fastened (fitting a new cap is the
bench step). The UR5e is fixed (powered off, brakes on). The layout is fixed per lot: two lots stop a single fixed
habit (laying the cap at one place), not a memorised replay of both.

## Real trial

**Design package, not a physical demonstration.** The printed apparatus substitutes for the gripper and support arm;
its shared exterior stations preserve the front-end approach, while the surrounding UR5e collision geometry is
absent. The real and simulated workspaces are therefore not identical. Keep those trial results separate.

- Hardware: the DHEF-20 front-end replica in `fixture/` (printed nose, bayonet, catch ring and bush; steel rod with
  its forward stop and a 6 N brake; printed coupling, keepers and releasing ring with a catalogue return spring) with
  a genuine Festo DHAS-GA-B22-S cap, on its bracket screwed to a board clamped at the table's edge, the axis at the
  pose in `task.json`. Parts, prices and sources: `fixture/BOM.csv`; build: `fixture/BUILD.md`. One-time work:
  printing and about 2 hours of assembly (estimates), then the sizing and acceptance checks in
  `fixture/ENROLMENT.md` (knob and rim sized to the cap by caliper; bayonet, rod drag, latch force and travel, rolling
  force recorded with a 20 N spring scale). A DHEF-20-A can supply a comparison trial, with its actual forces recorded separately.
- A steel rule (300 mm) taped to the bench under the gripper's axis, zero at the mounting face, read in a side
  camera's video; overhead video at 20 fps or more.
- After each trial: hold the removed cap against a lamp and look for holes or tears; refit it (Reset notes; about 2
  to 3 minutes, not yet timed).
- Not yet done: nothing has been printed or measured. Until the checks are run, the replica's fit to the genuine cap,
  its latch forces and its wear over repeated trials are unverified; the trial report gives the enrolment values.
- Trial state: no robot arm other than the two test arms, no air, no hot assembly tools in the workspace.

## Results

Reused simulation evidence from 2026-11-21; physics, scorer and demo code are unchanged. The runs used the earlier
goal-only instruction, not the revised ordered instruction. The old demo has no separate tip-seating command;
neither run validates that added instruction step. The replica CAD is not exercised by these runs.

Dream-1 with the revised instruction: run dream-20261123T231155 (2400 steps, 2 lots): 0.0 in lot A and 0.0 in lot B; both lots reached the step limit, no errors, no refusal.

A fresh Dream-1 run with the ordered instruction is in progress with a 2,400-step (120 s) limit per lot, two lots, at 20 Hz. It has no completed aggregate score yet. The historical Dream-1 run below used 3,600 steps (180 s) per lot and a different instruction; these runs are not identical evaluation conditions.

The fresh run uses `--max-steps 2400 --epochs 2`. Its `score.txt` is not yet available; the coordinator must fill in its completed results. The historical scripted demo remains `demo-20261121T110625`, 1.0/1.0, 764 steps per lot, on unchanged `task.py`. The existing media ID refers to the original bundle; no fresh-video upload or media association is established here.

The historical runs below used one NVIDIA L40S, yam-isaac-lab-arena on Isaac Lab-Arena, `object_jitter=0`, both lots (`--epochs 2`)
unless noted.

| Run | What | Lot A | Lot B | Notes |
|---|---|---|---|---|
| Historical demo | `yam_scripted`, 3,600-step limit | 1.0 | 1.0 | sampled eversion about 134.6 mm (about 2.1 mm past full eversion, then back to 132.5 when released); releasing ring pushed to its 3 mm stop; cap off and on mat, released and still. Each trial: 764 steps / 38.2 s. No recorded puncture; no independent tip-seating check |
| Historical Dream-1 | `dream`, 3,600-step limit | 0.0 | 0.0 | both trials reached 3,600 steps without a recorded refusal. One-second samples show hands clear by the scorer's 30 mm test and zero eversion; this is sampled state evidence, not continuous contact-sensor evidence or a run of the revised instruction |
| Shortcut check | `DHEF_DEMO=yank`, lot A, 1,000 steps | 0.5 | not run | the catch turned and the cap drawn on 120 mm past the demo's stop: the cap stretched 10 mm past full eversion (model elastic term about 20 N) and the coupling held it; let go, the cap hung everted on the coupling (0.5: everted, still held) |

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
rod stayed at its forward stop while the cap rolled, and the first sampled open-latch state was at 2.68 mm sleeve
travel at t=23.05 s (both lots). The code opens at 1.5 mm; the sampled 2.68 mm and later 2.98 mm readings are not
measured release thresholds. The
cap ended lying on the mat on a flat of its ring, with the hold counted from the release.

## Licences

No third-party meshes or assets in this folder: every shape is a box, cylinder, prism or triangle mesh written in
`task.py` from the makers' published dimensions and stated estimates. `fixture/` (the real-trial replica) was written
for this task: `make_replica.py` under the MIT licence, its STLs, drawings and notes under CC BY 4.0; no Festo CAD is
used. `parts.py`, `lots.py` and `tilt.py` are helper
modules shipped with the task. The YAM arm models, the station scene, its cameras and the `yam_arena` embodiment come
from the program's yam-isaac-lab-arena environment, used under the licences it ships with (the YAM description: MIT;
Isaac Lab and Isaac Lab-Arena: BSD-3-Clause; Isaac Sim: NVIDIA's Isaac Sim licence).
