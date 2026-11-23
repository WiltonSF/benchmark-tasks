# Fit the CSF-14 wave generator into a robot joint's mounted flexspline

**Instruction:** "Fit the wave generator into the gear until it is fully home, then let go."

**Category:** Impact on AI timelines.

## Why it matters

Robot-joint assembly technicians fit wave generators to mounted spline pairs when building strain-wave joints.
This task isolates one fitting step on a Harmonic Drive CSF-14-50-2A-R: fit the wave generator into the
mounted circular spline and flexspline without creating an eccentric tooth mesh. A correctly seated mesh is needed before the new joint can be driven;
asymmetric engagement can cause vibration and early failure. The scene contains the joint housing, output flange,
assembly cradle, motor and cover at their assembly-bench positions.

The [manufacturer's component-set catalogue, printed page 29](https://www.harmonicdrive.net/_hd/content/catalogs/pdf/csf-csg.pdf)
specifies that the two splines are mounted before the wave generator, and gives two instructions:

- "Insert Wave Generator as you rotate it."
- "Avoid assembling with excessive force on Wave Generator bearing."

For scale, the [HDS Report 2025, printed pages 11-12](https://www.hds.co.jp/Portals/0/files/csr/HDSreport/2025/HDSREPORT2025_EN_01Introduction.pdf)
describes five to six speed reducers per compact or collaborative robot. The
[maker's SCARA application note](https://legacy.harmonicdrive.net/reference/applicationhandbook/display.php?id=ir_srpa)
places strain-wave gears in robot primary axes. These sources establish repeated use of the mechanism across robot
joints; neither counts CSF-14 sets or manual insertions. Motor coupling, cover fitting and powered joint testing
are subsequent operations outside this task.

The manipulation is aligned rotation under axial load throughout insertion. The generator expands the thin
flexspline into its mating circular spline; forcing it can leave an eccentric mesh even at the correct final height.
Withdrawal and a careful refit can recover the simulated mesh error. Final height alone earns no credit for an
eccentric mesh. The category concerns building a new robot joint. The demo uses the right hand;
the cradle supports the assembly and the left hand stays clear.

## Scene (lot A is the screenshot)

Millimetres; positions from the midpoint between the arm bases (x forward, y to the robot's left), heights above
the bench. Sizes are recorded in `dimensions.json`; `joint_housing.scad` is an incomplete housing/cradle drawing.
The physical mounting defects are listed under specimen enrolment below.

| Object | What it is | Where |
|---|---|---|
| Gear | Harmonic Drive CSF-14-50-2A-R component set, 0.09 kg (catalogue). Circular spline 50 pilot, 8 x M3 on a 44 circle (catalogue); flexspline cup 28.5 deep (catalogue), bore 31.0 before the generator is in, 1.0 lead-in at the rim (estimates) | in the housing, open end level with the housing's top face, 60 up |
| Joint housing | printed PETG, 72 across, circular spline bolted in its pilot, flexspline bolted to the output flange on a 6805-2RS bearing | standing output side down in the cradle, axis at x 290, y -90 |
| Assembly cradle | printed PLA, 120 x 120 x 10; the housing's base sits in its 4 mm locating pocket (one body in simulation) | under the housing; housing and cradle 1.6 kg with the gear (estimate) |
| Wave generator | 17.6 tall (catalogue H): steel plug 30.4 across, 8.0 tall, 1.2 chamfer, red index mark; black Oldham disc 23 across, 3 tall; steel hub 16 across, 6.6 tall, 6 bore (catalogue); 30 g (estimate) | plug down on the tray floor |
| Parts tray | aluminium, 140 x 100 x 15, 4 floor | lot A centre x 195, y -225; lot B x 385, y -215 (generator turned 90 degrees) |
| Stepper motor | NEMA 17, 42 x 42 x 48, on a printed 60 x 60 x 8 adapter plate (fixed, background) | x 540, y 20 |
| Joint cover | printed, 72 x 20 (fixed, background) | x 540, y -90 |

The housing is never moved between lots. Trials alternate lots A and B and are averaged.

## Scoring

Tools for a real trial: overhead and side video at 30 fps, a platform scale under the cradle (tared), a depth gauge
with 0.1 mm resolution, a dial indicator on a magnetic base and straight 6 mm shaft, a small square, a 10x loupe,
and tape marks round the cradle. Convert scale kilograms to newtons with N = kg x 9.81; 30 N is about 3.06 kg.

**Score 0 for the whole trial if, at any moment:**

1. the press on the generator exceeds 30 N: any reading over 30 N on the tared scale, whose display is in view of the
   side camera (simulation: the press on the plug while it sits on the gear axis, averaged over 0.25 s); 30 N is the benchmark overload threshold, not a published bearing rating;
2. a gripper enters the flexspline below the circular spline's face (it would mark the teeth; fresh marks on the rim
   under a 10x loupe after the trial, and the side video);
3. the generator falls more than 10 mm with no gripper on it anywhere but inside the gear, for example back onto the
   tray or the bench (side video; simulation: falling faster than 0.4 m/s with no gripper within 5 mm);
4. the housing moves over 5 mm, tilts over 5 degrees or turns over 5 degrees about its axis (tape marks, side video).

**Score 0 at the end for the benchmark eccentric-mesh check:** after the trial the setter puts a 6 mm shaft in the hub
and turns it slowly by hand through two turns with the dial indicator on the shaft 10 mm above the hub; the mesh is
assigned the eccentric-mesh outcome if the shaft runs out over 0.3 mm. The cutoff and this runout diagnostic
are unvalidated benchmark choices, not maker acceptance criteria or a proven diagnosis of dedoidal mesh. In simulation the eccentric mesh is set when the plug, on the
gear axis, is pushed at 20 N or more for 0.25 s while turning slower than 0.3 rad/s, or pressed at 8 N or more while
tilted over 3 degrees. Lifting the plug out of the rim lets the flexspline spring back round and clears it, so a generator that is
taken out and fitted again properly can still score in full.

**Otherwise**, with the generator's axis within 1 mm of the gear's and within 30 degrees of upright (home also needs
it within 3 degrees of the gear axis: the hub square to a small square on the circular spline face):

| Score | End state | How it is read |
|---|---|---|
| 1.0 | home: hub top 9.7 mm or less above the circular spline face (9.2 mm when fully home), even mesh, both grippers at least 20 mm from the generator, nothing moving for 1 s | depth gauge; overhead video |
| 0.75 | home and even, but still gripped or not yet still for 1 s | depth gauge; video |
| 0.5 | plug at least half way into the teeth zone: hub top 12.65 mm or less above the face | depth gauge |
| 0.25 | plug at least 1 mm into the teeth zone: hub top 15.1 mm or less above the face | depth gauge |
| 0 | anything else, including a generator resting on the rim or lifted back out | |

The simulation uses the following observable proxies: `observe()` in `task.py` records the mesh front, the plug's depth,
offset and tilt, the press (from the mesh stop's deflection), the turn, the hands (face-sampled gripper hulls) and
the hold; `score()` applies the table. A trial ends at 2,400 steps or at full score. The code allows 0.5 mm plug-depth tolerance and also requires the
mesh front to have reached each credited depth: the nominal 15.1 and 12.65 mm partial-credit hub heights therefore
have tolerated limits of 15.6 and 13.15 mm. For the final one-second hold, generator and housing marker motion must
stay within 1 mm, with linear speed below 20 mm/s and angular speed below 0.5 rad/s. Real-trial height and runout
checks are specified measurements; the simulated eccentric-mesh flag is a history-based proxy for that runout.

### The declared mesh rule (what the simulator cannot do)

PhysX bodies are rigid, so the flexspline cannot bend round the elliptical plug. The teeth zone is a stop under the
plug that the rule moves:

- the plug's chamfer and the rim's lead-in go in freely for 1.5 mm; there the plug rests on the teeth zone;
- while the plug presses on it at 1.5 N or more, square (tilt 3 degrees or less) and on the axis, each turn of the
  generator about its own axis, either way, lets it 4 mm deeper, up to home at 8.4 mm (catalogue insertion depth
  0.4 mm below the open end, plus the 8.0 mm plug);
- a straight push (20 N for 0.25 s while turning under 0.3 rad/s) or a press of 8 N while tilted over 3 degrees
  forces the eccentric mesh, and the plug then drops home;
- lifting the plug out of the rim (above 0.5 mm) resets the stop to the mouth and clears the eccentric mesh.

The 1.5, 8, 20 and 30 N thresholds, and the 4 mm/turn feed, are benchmark estimates. They have not been measured
on a physical specimen. A physical trial reads force and runout directly; the maker's warning supports the need
for controlled insertion but supplies no numerical insertion-force or damage threshold. Enrolment records the
selected specimen's fit before physical evaluation; it does not establish these simulation thresholds as calibrated.

## Bypasses and near misses

`near_misses.py` checks 4 fixed end states with no history (all 0) and replays 23 trials through the scorer with 33
checks (`python near_misses.py`), including:

| Strategy | Score |
|---|---|
| pushed straight down without turning (25 N) | 0 (eccentric mesh), even though it ends home |
| pushed firmly (15 N) without turning | 0: stalls on the rim |
| turned without pressing (1 N) | 0: no progress |
| pressed while tilted 4 degrees | jams; at 9 N, 0 (eccentric mesh) |
| pressed at 28 N while turning | allowed (under the 30 N limit) |
| pressed at 35 N | 0 for the trial (benchmark overload threshold), even after a proper finish |
| fingertip into the flexspline on the teeth | 0 for the trial |
| generator dropped on the way, or dropped 30 mm back onto the tray and then fitted | 0 for the trial |
| lowered fast (0.6 m/s) while gripped | not a drop: 1.0 |
| housing knocked 8 mm, or turned 8 degrees | 0 for the trial |
| plug 1.5 mm off the axis on the rim | 0: no progress |
| generator upside down, hub first | 0 |
| eccentric mesh, then lifted out and fitted properly | 1.0 |
| home but still gripped | 0.75 |
| released 0.9 / 1.1 / 3.3 / 3.5 mm into the teeth zone | 0 / 0.25 / 0.25 / 0.5 |

## How this differs from close tasks

The comparisons concern the manipulation, not the product name or starting layout. Board searches and
public PR verdicts were checked on November 23, 2026; the local board cache supplies the accepted task IDs.

- **Strain-wave flexspline unit, [PR 1890](https://github.com/robocurve/benchmark-tasks/pull/1890), final:**
  the reviewer found that turning a toothed unit to find phase while seating repeats TK-YNPRML; the rigid ten-tooth
  cup also failed construct validity. That task moves the flexspline unit into the circular spline. Here both splines
  are already mounted and the separate wave generator goes inside them, with continued loaded turning and a
  recoverable home-but-eccentric failure. This is the claimed procedural difference. The present rigid proxy still
  omits flexible tooth engagement and torque; a different part name alone would not resolve that review.
- **Three-gear train, TK-YNPRML / [PR 893](https://github.com/robocurve/benchmark-tasks/pull/893), accepted:**
  find the phase of rigid spur gears and lower them onto shafts. This task requires accumulated rotation under
  axial load throughout the insertion depth in its declared rule, with a separate eccentric-mesh failure even when
  the generator is home. Physical flexspline engagement has not been validated.
  [PR 719](https://github.com/robocurve/benchmark-tasks/pull/719) received an overlap final for another spur-gear
  phase-and-seat task; merely changing the gear would not distinguish the skill.
- **Robot gearbox cartridge stack, [PR 3278](https://github.com/robocurve/benchmark-tasks/pull/3278), final for
  overlap:** search for an input-spline angle, then square the housing's separate register. Here there is no final
  angular register: loaded turning advances the mesh through its depth, and a forced insertion can be fully home
  yet eccentric. Hidden alignment alone is not the claimed difference.
- **Servo horn, [PR 1864](https://github.com/robocurve/benchmark-tasks/pull/1864), revisable:** choose a spline
  tooth at a visible zero mark, then press home. Here rotation continues during seating rather than selecting a
  final tooth angle. [PR 2708](https://github.com/robocurve/benchmark-tasks/pull/2708) received an oversimplification
  final and [PR 3112](https://github.com/robocurve/benchmark-tasks/pull/3112) a duplicate final for servo-horn tasks.
- **Jaw coupling, TK-V7QS9F / [PR 2484](https://github.com/robocurve/benchmark-tasks/pull/2484), final on
  November 23 for failing to establish an occupation-scale core task:** turn jaws into the spider's pockets,
  then slide the hub to its gap. Here continuing loaded rotation, rather than one phase match followed by sliding, controls insertion.

## Real-trial kit and specimen enrolment

The specimen is a ready-made matched component set: circular spline, flexspline and wave generator. The
[CSF-14-50-2A-R product page](https://www.harmonicdrive.net/products/component-sets/cup-type/csf-2a/csf-14-50-2a-r)
identifies the catalogue model and its 0.09 kg mass. Buy the component set, with a removable wave generator, rather
than a sealed gear unit. The printed pieces are workholding; the actual steel set supplies the flexible mesh.

Planning budget in USD, November 2026; allowances below are estimates before tax and shipping, not supplier quotes:

| Item | Budget |
|---|---:|
| One CSF-14-50-2A-R matched component set, obtained by manufacturer/distributor quote | $1,000 allowance |
| Intended PETG housing and PLA cradle, after completing the mounting design; 6805-2RS bearing (25 x 37 x 7 mm), output flange and M3 fasteners; ballast to bring the supported assembly to 1.6 kg | $80 allowance |
| Aluminium parts tray; unpowered NEMA 17 motor, adapter plate and joint cover matching the scene | $35 allowance |
| Platform scale with at least 5 kg capacity and 1 g resolution, depth gauge with 0.1 mm resolution, dial indicator/base with 0.01 mm resolution, straight 6 mm shaft, square, loupe and tape | $120 allowance |
| Dedicated replacement pads with the rig's standard profile/material; wipes and reset consumables | $25 allowance |
| Total for one reusable specimen and fixtures, excluding robot and cameras | $1,260 allowance |

For a public price reference, a [CSF-14-50-2A-R-SPS033 surplus listing](https://www.ebay.com/itm/298673092772)
was advertised at $916. Its special suffix is not established as interchangeable with the catalogue model; it
is a price reference, not an enrolled substitute. A quote for the specified catalogue set determines the actual
kit price. One spare matched set adds the same component-set allowance if damaged specimens must be replaced.

The physical fixture is incomplete. In `joint_housing.scad`, the 49 mm through-bore removes both the 37 mm
bearing-seat volume and the material at the circular-spline M3 holes on the 44 mm circle. The output flange
and its attachment are also unspecified. Printing this file does not supply the described bearing-supported,
bolted mount. These features need a completed, checked design before physical trials; the recorded simulator
uses one rigid housing body instead. The intended assembly has a 50 mm pilot, eight M3 bolts on the 44 mm circle
and a bearing-supported output flange, with the flexspline mouth coplanar with the circular-spline face, 60 mm
above the support surface. Check the printed fits against the actual component dimensions before mounting; do not change the trial geometry to accommodate
an unverified substitute. Use the maker's lubrication and fastening instructions for the selected component set.

Once the fixture is built, allow 5-10 minutes to fit the dedicated pads, mount, inspect and enrol a ready-made
matched specimen before its first trial (planning estimate, unmeasured). No batch casting, curing or printing
of specimens is needed. Record its model/suffix, component dimensions, mounting, freely turning output
and two-turn hub runout on the straight shaft. Check that a gentle rotating insertion reaches the stated home height,
then withdraw and record the scale trace and heights. Enrol only an undamaged, matched set that fits the stated
fixture and passes the 0.3 mm runout check. These are proposed physical enrolment checks; no physical trial or measured
preparation or reset time is reported.

**Between trials: allow 2-3 minutes for an undamaged specimen.** Read the final mesh/runout first, lift the generator
out by its hub, check for fresh tooth marks, bearing roughness or looseness, and check that the output turns freely.
Keep all three serialized components together; replace a damaged matched set rather than mixing individual parts.
Leave the two splines mounted, return the generator plug down to the next lot's tray centre and index direction, check the tape marks, zero the depth gauge and tare the scale. There is no per-trial
fabrication, casting or curing. Nothing is powered during assembly or scoring.

### Shared gripper pads and reset kit

Before a physical trial, remove and store the shared rig's pads. Fit a dedicated sacrificial replacement set with
its standard contact profile and material; do not add a sleeve that changes the hub grasp or tooth clearance.
The replacements take any incidental metal contact. Inspect the hub, rim and tray for burrs or sharp damage;
reject damaged specimens. Wipe grease from the hub and external grasp surfaces while retaining the maker's required
internal lubrication. Use the disconnected motor only as a background assembly part. These are physical-trial
preparations, not tested protection results or additions to the recorded simulation.

Stage the scale, depth gauge, indicator/base, straight 6 mm shaft, square, loupe, tape, wipes, specified lubricant
and replacement pads as the reset kit. At each reset, inspect the dedicated pads for cuts, embedded metal or grease
and replace contaminated or cut pads before reuse. Inspecting and cleaning belongs in the 2-3 minute reset estimate;
replacement and re-enrolment time must be logged separately if needed. First-time fixture printing, machining,
flange fitting and pad replacement are not included in that estimate. No measured physical preparation or reset
time is available.

## Fidelity

True size from the CSF-14 catalogue outline (pilot, bolt circle, flexspline length, generator height, hub bore,
insertion depth); the plug, rim and hub diameters, the masses and the frictions are estimates. Bodies are rigid;
the flexspline's bending and tooth mesh are the declared rule above. Mesh drag torque is not modelled. The Oldham coupling inside the generator is
one rigid body. The output flange is fixed in the simulation. Physical flexspline compliance, running torque and durability have
not been validated by the recorded simulation runs.

## Run

From the task folder in the Isaac Lab-Arena environment:

```
inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py \
  -E object_jitter=0 -E control_hz=20 --instruction "Fit the wave generator into the gear until it is fully home, then let go." \
  --scorer task_score --max-steps 2400 --epochs 2
```

- Simulator: yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena). `<policy>` is `yam_scripted`
  for the scripted demo or `dream` for Dream-1.
- `--epochs 2` runs lot A then lot B (trial k uses lot k mod 2) and averages them. `TASK_LOT=A` or `TASK_LOT=B` pins
  one lot. `-E object_jitter=0`: the layout is fixed; no seed changes the scene.
- `DEMO_ROUTE=straight` makes the scripted demo push the generator straight down without turning (the naive route).
- The demo takes about 30 s of simulated time per trial (under 700 of the 2,400 steps); a trial takes a few minutes
  of wall time on one L40S GPU.

## Recorded simulation scores

The recorded runs use the unchanged instruction above and the same two lots. These are simulator results, not
measurements on a physical robot.

- Scripted demo (`yam_scripted`, `--epochs 2`): 1.0 in lot A and 1.0 in lot B. The right hand lifts the generator by
  its hub, squares it over the rim, presses it onto the teeth zone at about 3 to 4 N and turns it back and forth
  through ten 120-degree strokes; it is home at about 21 s, released, and the trial ends at full score at about 29 s
  (about 580 of the 2,400 steps). Peak press about 4 N, well under the 20 N and 30 N limits.
- Dream-1 (`dream`, `--epochs 2`): 0.0 in lot A and 0.0 in lot B. Both trials ran to the 2,400-step limit without
  refusing; the right hand moved between the tray and the housing, but the generator was never fitted.
