# Grease both Robotiq Hand-E racks into their guide-flange steps while keeping the tapped faces dry

**Instruction:** Grease both racks with the brush for reassembly: every surface except the face with the tapped
holes (the small end faces may stay bare), keeping the brush off the opened gripper and its parts. Then leave the
racks in the tray and the brush back across it.

**Category:** Other — documented precision-part lubrication procedure.

## Why it matters

Robotics technicians service the Robotiq Hand-E rack-and-pinion mechanism to restore smooth finger travel:
the maker specifies maintenance every 2 million cycles, sooner in dirty conditions, and grease on the removed racks
before reassembly. The sliding faces and recessed guide-flange steps need coverage while the tapped mounting face
stays dry. This is a documented equipment-service procedure under Other. O*NET 17-3024.01 identifies Robotics
Technicians as the relevant occupation.

[Robotiq's maintenance procedure](https://blog.robotiq.com/knowledge/preventive-maintenance-on-the-hand-e-gripper)
identifies the two removable racks, degreasing and drying, and greasing before reassembly. Its greasing step says:
"Apply grease on all surfaces of the racks except the face with the tapped holes."
The specified lubricant is Mobilith SHC 1500; Robotiq supplies it as ACC-LUB-SHC1500. The task starts after
removal and cleaning. The scored outcome is coverage of the sliding surfaces without contaminating the mounting
faces; the two small end faces are excluded explicitly in the instruction and scoring.

What makes it hard for a robot: the racks are small (14 x 16 x 62 mm) and light (about 70 g); the faces that slide
in the housing include the narrow steps beside the guide flange; the back lies on the tray floor; and the tapped face
must stay dry. A side pressed into 22 mm bristles is greased up its steps only if the rack is held in a height band
5.3 mm wide: lower, the bristle tips wrap over its top edge onto the tapped face; higher, the upper stem's side stays
bare. The rack has to be held at that height, pressed into the bristles and moved along them, on both sides and
then from below, without the fingers touching greased faces. A rack lying in the tray cannot be brushed on its sides without the brush's tips
reaching over its top (a bristles-up brush resting on the tray floor stands 34 mm tall; the rack is 16 mm).

## Scene (dimensions used by the build; estimates are marked)

Millimetres from the midpoint between the two arm bases (x forward, y to the robot's left), heights above the table.

| Item | Product and size | Where (lot A) |
|---|---|---|
| Bench mat | grey anti-static bench mat, 457 x 610 x 2 mm (18 x 24 in) | centre x 300, y 0 (fixed) |
| Tray | Miltex non-perforated Mayo instrument tray, stainless steel, 254 x 165 x 18 mm (10 x 6-1/2 x 23/32 in); floor 1 mm, walls modelled 1.5 mm thick | centre x 300, y 0, long side along x (fixed) |
| Two racks | Robotiq Hand-E racks, steel. Section from the maker's Fig. 5-11 (Racks in rack housing): a lower stem 6.1 wide x 4.8 tall (its underside is the back), a guide flange 14.0 x 6.0, an upper stem 6.1 x 5.3 whose top is the tapped face, with three M3 tapped holes and two 2 mm dowel holes at 4.5 mm pitch; teeth along one side of the lower stem. Length 62 mm and the stem and flange heights are read off the drawing at its scale (estimates; check: calipers on a rack). 71 g each (estimate from 9.0 cm3 of steel; check: kitchen scale) | lying along y on the tray floor, tapped face up: rack 1 at x 270, y 30; rack 2 at x 210, y 30 |
| Brush | Dimensioned nylon brush replica with a tile-and-grout-style handle, 248 mm long, bristles 22 mm long in a 20 x 56 mm block on a 25 x 65 x 12 mm head, handle 22 x 183 x 12 mm (authored dimensions; check: ruler), 60 g (estimate; check: kitchen scale). Bristles loaded with about 1 g of Mobilith SHC 1500, which is red | bristles up across the tray's two long walls at x 320: head over the +y wall (head middle at y 60), handle toward -y over the -y wall |
| Opened Hand-E body | rack housing off: 75 mm base, 62 mm body, 83 mm tall (Fig. 5-1; 83 is an estimate), pinion stub on top | standing on the mat at x 490, y 170 (fixed) |
| Rack housing | removed, lying channels up, 75 x 16 mm (estimate) | x 490, y 68 (fixed) |
| Coupling | GRP-ES-CPL-062, 75 x 16.9 mm, with its electrical contact | x 490, y -35 (fixed) |
| Grease syringe | Robotiq ACC-LUB-SHC1500, about 22 x 120 mm (estimate) | lying along y at x 470, y -150 (fixed) |

Lot B mirrors lot A across y = 0: the brush turned end for end (head over the -y wall, handle toward +y) and both
racks at y -30. Trials are run on lots A and B and averaged; the screenshot shows lot A. No seed changes the scene;
run with `-E object_jitter=0`. All shapes are boxes and cylinders built in the task file (`parts.py`); no third-party
assets or meshes are used.

## Reusable real-trial kit

The worked specimens are two steel racks or replicas. The opened body, housing, coupling and syringe define
areas the bristles must avoid, so dimensioned obstacle replicas suffice. The fixture is the service tray and mat
shown in the scene; one hand steadies the brush on the tray rims while the other works each rack by its end faces.

| Item | Procurement or one-time preparation | Budget, US$ |
|---|---|---:|
| Pair of reusable steel rack replicas | Machine once from low-carbon steel to the three-tier section in the scene table: 62 mm long, centred widths 6.1/14.0/6.1 mm, tier heights 4.8/6.0/5.3 mm. Drill and tap three M3 top holes; smooth and deburr all edges. Allowance, not a shop quote. | 150 |
| Reusable brush replica | Nylon strip-brush stock trimmed and arranged to a 20 x 56 mm footprint with 22 mm free bristles, mounted on a 25 x 65 x 12 mm head and 22 x 183 x 12 mm handle. Materials allowance; make once and reuse. | 25 |
| Stainless tray | Miltex 3-926M, 254 x 165 x 18 mm; [supplier listing](https://relinkonline.com/products/integra-miltex-3-926m-mayo-tray-non-perated-10-x-6-1-2-x). Budget allowance. | 49 |
| Grey bench mat | Bertech 18 x 24 in; [Zoro listing](https://www.zoro.com/bertech-esd-anti-static-table-mat-kit-18-in-x-24-in-gray-1059-18x24gkt/i/G210027023/), listed $66.15. | 66.15 |
| Red Mobilith SHC 1500 | 13.4 oz cartridge; [Speedway Motors listing](https://www.speedwaymotors.com/Mobil-1-122115-1-Mobilith-SHC-Series-1500-Grease-13-4oz-Tube-Ea,491429.html), listed $10.99. | 10.99 |
| Fixed obstacle replicas and syringe | Printed or wooden blocks/cylinders at the scene dimensions, including the body pinion stub; 20 mL plastic syringe. Materials allowance. | 6 |
| Degreaser and lint-free white wipes | Stock consumables; allowance. | 10 |
| Disposable finger-pad covers | Smooth nitrile covers on both hands; allowance. | 10 |
| Transparent 1 mm grid and spatula | Stock scoring/reset aids; allowance. | 5 |

Total allowance **about $335**, before tax and shipping, excluding robot, cameras, calipers, a gram scale and a
small force gauge. Prices are indicative; fabricated items are budget estimates, not supplier quotes. The steel
specimens and brush are reused after cleaning; no specimen manufacture or curing is needed between trials.
Real Hand-E racks may replace the replicas after the same enrolment checks; record their actual dimensions.

The brush geometry is a replica specification. [Libman's current #18 specification](https://www.libmanpro.com/products/tile-grout-brush)
lists a different scrubbing footprint and PET fibers. The dimensioned nylon replica preserves the build's bristle
envelope and handle access. Its dimensions are specified above so a physical trial does not depend on a retail revision.

**Enrolment checks before a physical series:** measure and record rack length, each tier width/height and mass;
target the listed geometry within 0.2 mm and mass 71 +/- 5 g. Check both racks have smooth, burr-free end-grip
faces and no old grease on any face (fresh white tissue shows no red). Check the brush footprint and free bristle
length against the dimensions above, and photograph the initial layout. With a spare clean replica, check that a
side pass reaches the flange steps without marking the tapped face. Record the force needed for that pass with a
force gauge; the simulator's 0.2 N scoring threshold is not a measured physical requirement. Report the measurements
and any differences from the replica specification with physical results.

**Reset between trials:** allow 3-5 minutes as a planning estimate; reset time has not been measured. Wipe the
racks, tray, brush handle and pad covers with compatible degreaser and dry. Check all rack faces, end-grip areas,
handle and fixed obstacles with fresh white tissues; continue cleaning until no red transfers. Replace contaminated
pad covers. Wipe excess old grease from the bristles and work in a fresh 1 g load, measured by mass. Put the racks
and brush back in the selected A or B layout. Grease, wipes and contaminated covers are the only consumables.
Keep used wipes in a waste container and verify the shared grippers are clean after removing their covers.

## How to run

From the task folder:

    inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 --epochs 2
    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 --epochs 2

`--epochs 2` runs lot A then lot B (trial k uses lot k mod 2); the score is their mean. `TASK_LOT=A` or `TASK_LOT=B`
pins every trial to one lot. Run limit 2,400 steps (120 s at 20 Hz) per trial; the scripted demo ends on success at
95 s of simulated time. Simulator: yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena).
`parts.py` and `lots.py` are helper modules next to `task.py` (multi-colour rigid parts with contact reports; layout
lots). The task file's sha256 is in the Scores section.

## Scoring

The text in task.json, with the exact simulation rules (all in `task.py`).

**Reading a real trial.** Overhead and wrist videos, then, after the trial, square-on photos of each rack's back,
both sides and tapped face, with a transparent 1 mm grid held beside (not on) the rack. The supplied 1:1 rack stencil
lays out the windows: 16 along the rack (3.875 mm each) and, across each face, the strips listed below. The back photo
taken at 45 degrees from each side also shows the flange's undersides and the lower stem's sides; each side photo
shows the flange side, and at 45 degrees from above the flange top and the upper stem's side. A window is greased if
red grease shows in it. A white tissue pressed on each tapped face shows any grease there. Mobilith SHC 1500 is red,
so it shows on bare or black steel.

**Score 0 for the whole trial if, at any moment:**
- any grease reaches either rack's tapped face (simulation: one brushing step that reaches any of its three strips,
  or a greasy finger on it; see below);
- a rack lands on the mat or the table outside the tray (simulation: its centre outside the tray's outline and its
  lowest corner within 4 mm of the mat, or anything below the table top; `rack_landed_outside`);
- the bristles touch the opened gripper body, its rack housing, its coupling or the syringe (simulation: the
  bristles' outline within 1 mm of any of them; `bristles_near_gripper`);
- (simulation only, fail closed) the contact forces or the hand positions cannot be read.

**Windows and zones.** Each rack's surface, end faces aside, is cut into 16 rows along it and, around it: the back
face in three strips (2.03 mm each), each lower stem side, each flange underside, each flange side, each flange top,
each upper stem side, and the tapped face in three strips. The back zone: the back face, the lower stem's two sides
and the flange's two undersides (112 windows). Each side zone: that side's flange side, flange top and upper stem
side (48 windows). The two cross-shaped end faces are not scored: they touch nothing in the housing (the racks slide
out of its ends) and a rack may be held by them. A zone is greased when at least 90% of its windows are.

**Ladder** (every score also needs, for the last 1 s: both racks lying in the tray, i.e. all eight corners inside the
walls and the lowest within 3 mm of the floor; every point on both hands' gripper and finger hulls, sampled every
3 mm, at least 30 mm from the racks and the brush; the racks and the brush moving under 1 cm/s and 0.5 rad/s):

| Score | Condition |
|---|---|
| 1.0 | all six zones greased, and the brush back across the tray's two long walls, bristles up (tilted 10 degrees or less, both ends beyond the long walls, its underside within 3 mm of their tops, both ends at least 15 mm inside the short walls along x) |
| 0.75 | all six zones greased |
| 0.5 | all three zones of one rack greased |
| 0.25 | at least two zones greased |
| 0 | otherwise |

Trials are run on lots A and B and averaged.

**The greasing rule** (`record_brush`, called every control step by `_on_step`). The bristles are a 20 x 56 x 22 mm
block. Their packed core, 10 x 46 x 17 mm, is what the simulator collides: the rack is stopped by it. The outer 5 mm
gives way, so a pressed rack sinks into the bristles' outline. The simulator reports the contact force between the
brush and each rack. A window gains the distance the bristles moved against that rack since the last step (along the
face or into it: a stroke or a dab both lay grease) when: some part of the window lies inside the bristles' outline;
the brush pressed that rack with 0.2 N or more on this step and the last; the bristles moved at least 0.6 mm against
the rack over the last 0.2 s (3 mm/s; a held press that only jitters counts nothing); and the step is under 12 mm
(not a jump). A window is greased at 2 mm. On a tapped face any amount counts.

What this means for the rack's shape: pressed against the bristles' long side, a flange side sinks 5 mm in, so the
bristles reach the flange's top and underside and, 3.95 mm in, the stems' sides; the back pressed down onto the tips
reaches the back face, the lower stem's sides and the flange's undersides 4.8 mm up. Because the outline is 22 mm
tall, a side held too low puts the tips over the rack's top edge onto the tapped face, and one held too high leaves
the upper stem's side bare: the rack's back must be between 5.9 and 11.2 mm above the brush head. The middle third of
the back face is reached only from below.

**The finger rule** (`record_hands`, every control step). The hands' gripper and finger surfaces are sampled every
3 mm. A point within 1 mm outside a window's face and over the middle 80% of its length touches it (points within
0.5 mm of an end face are left out: a rack may be held by its ends). A greased window a hand touches is wiped bare, and that hand is
greasy from then on; a greasy hand touching a tapped face puts grease on it (0).

**Why the naive ways score low.** Brushing a rack where it lies in the tray: the brush's head rests on the tray
floor, so its bristle tips stand above the rack's top and reach the tapped face (0). Greasing the sides but not the
back leaves the back face's middle third bare (two zones a rack, 0.25). Holding a greased rack by its flanges wipes
them.

**Shortcuts and what they score** (each is a case in `near_misses.py`):

| Shortcut | Score |
|---|---|
| nothing done | 0 |
| sides brushed where the racks lie in the tray | 0 |
| both sides of both racks held up, no back pass | 0.25 |
| side passes with the tips 0.4 mm above a rack's top edge | 0 |
| everything, then one pass along a rack's top, or a short dab on a tapped face first | 0 |
| bristles moved all round with no contact force (hovering) | 0 |
| bristles 6 mm off the faces, force reported | 0 |
| contact on one rack credited to the other | 0 |
| dabs without sliding; a held press jittering 0.05 mm a step for 2,000 steps | 0 |
| a creep at 2.9 mm/s | 0 |
| jumps of 13 mm a step | 0 |
| everything, then a rack pinched across its greased flanges at both ends | 0.5 |
| everything, then a finger greased that way touches a tapped face | 0 |
| everything done but a hand within 30 mm, or still for under 1 s | 0 |
| everything done, a rack landed on the mat on the way | 0 |
| everything done, the bristles touched the gripper or its parts | 0 |
| rack left on the mat, held 1 cm above the floor, or lying across the tray's wall | 0 |
| brush left on the tray floor or bristles down | 0.75 |

## How this differs from close tasks

- **Brush a denture, TK-EFZTFV / PR #2998 (accepted):** both tasks coordinate a held part and a brush. Denture care
  removes marks from a tooth arch. This task adds a coating around recessed flange steps on opposite faces,
  preserves finished coverage while gripping, and keeps the adjacent tapped face entirely dry. Extra brushing can
  irreversibly contaminate that face rather than improve the score.
- **Grease a pivot pin, TK-BYRCR3 / PR #1243 (accepted):** that task seals a coupler and pumps a lever. This task
  tests presentation of a loose part to bristles, coverage of stepped sliding faces and control of grease transfer.
- **Finish painting a panel, TK-7P97ME (accepted):** that task follows boundaries on one fixed flat surface. Here a
  hand exposes a rack's underside and both recessed sides while the other stabilizes the brush. A side-only
  pass leaves the middle of the back bare, and gripping finished flanges removes coverage.
- **Patch nail holes, TK-4T97N9 (accepted):** that task changes blade angle to fill and strike a stationary wall.
  Here coverage depends on the part's height against the bristle envelope and access around its guide flange.
- **Dust a 2F-85 gripper, TK-AX2KMM / PR #3415 (closed, final):** that task wipes pads on an installed gripper.
  Here loose racks are lubricated before reassembly, with recessed surfaces to coat and a mounting face to keep dry.
- **Assemble a rack-and-pinion gripper, PR #1898 (under revision):** that task seats teeth at a matching index and
  checks coupled finger travel. This task covers the removed sliding faces; assembly and tooth indexing are not scored.

## Fidelity

Dimensions follow the authored replica specification. The rack section is based on Robotiq's Fig. 5-11 (14.0 x 16.1 mm, cross-shaped); its 62 mm length,
the stem and flange heights and its 71 g are estimates. The brush is a dimensioned nylon replica. Bristles bend; here they are a stiff core
inside a soft outer 5 mm that a pressed part sinks into (a stated rule standing for bristles closing round the part;
check: press a rack's side 5 mm into a loaded brush and look for red on the steps above and below the flange). Grease
is not a fluid: it does not reach the tray where a greased rack is laid down. Fingers wipe greased windows bare and
carry grease on. The rack's teeth and holes are drawn, not modelled. The tray's walls are modelled 1.5 mm thick
(real: about 0.8 mm sheet). Greasy nylon on steel is given a friction of 0.15. Rigid bodies in Isaac Sim (PhysX),
20 Hz control. The 0.2 N contact threshold, bristle compliance rule and grease-transfer behavior have not been
calibrated against a physical trial.

## The demo

Scripted, open loop, from the camera-visible layout only (`demo()` in `task.py`; it reads the lot's start positions,
never a simulated state). The arm on the brush's handle side takes the handle and keeps the brush on the tray's rims
while the other arm works; it stands clear while a rack is taken from or laid in the tray. The other arm takes each
rack by its two end faces (fingers 70 mm apart round it, then closed to 59 mm on its 62 mm length), holds it over the
bristles and:

1. lowers its back onto the bristles and slides it 6 mm each way along its length, twice;
2. lifts it, brings each side against a long side of the bristles with its back 10.5 mm above the brush head (the
   tips reach 0.7 mm up the upper stem's side and stay 4.6 mm below the tapped face; the held rack sags about 2 mm,
   which this allows for), presses it in and slides it 3 mm each way along its length, twice;
3. lays it back where it lay, back face down, and lets go.

Both arms then park clear. Each trial ends on success after 1,904 control steps (95 s) of the 2,400 allowed.

## Scores

These are recorded simulation results. No physical trial or measured robot-hardware result is reported.

Dream-1 run: 0.0 (mean of lots A and B, 0 in each; both trials ran to the 2,400-step limit without refusing; the arms stayed low near their bases and never reached the tray, so nothing was touched); demo: 1.0 in each of lots A and B (all six zones greased, both tapped faces dry, racks back in the tray, brush on the rims, hands clear and still; each trial ends on success at 1,904 steps).

task.py sha256: 6572b8a33f18356f87252da551dddf8884f70a356e5a6763aef425c4c21b27b5 (the file both runs used).
