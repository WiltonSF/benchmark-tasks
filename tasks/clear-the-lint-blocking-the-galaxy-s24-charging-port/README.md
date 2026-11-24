# Brush the lint out of a Galaxy S24's charging port with a soft toothbrush, without packing it, and wipe it off

A Samsung Galaxy S24 stands upside down in an adjustable aluminium desk phone stand on a kitchen counter, so its USB-C
charging port is on top. Four small balls of lint sit inside the port, one at each end of the two 0.93 mm gaps
beside the flat tongue in its middle, 4.5 mm down. A soft toothbrush lies bristles-down on the counter and a folded
paper towel lies beyond it, held flat by two tape tabs outside the wiping area. The robot must brush the lint out of the port, wipe it off the bristles onto the towel
and lay the toothbrush down.

Instruction: "Brush the lint out of the phone's charging port with the toothbrush, wipe the lint off onto the paper
towel, then lay the toothbrush down."

## Changes in this revision (answers to the review of 2026-11-22)

The review, quoted in full:

> **Not accepted** (TK-NHZVVP).
>
> Flags: Note for the team: Specify a repeatable lint blend and packing procedure to reduce variation between
> resets.
>
> Construct validity (not met): The rigid bristle field passes through the non-colliding frame and tongue,
> eliminating the deflection and contact forces needed to brush inside the real port.
>
> Scoring (not met): A side video of a mark on the outside of the bristle field cannot reliably reveal the depth of
> bristles hidden inside the narrow port.
>
> Feasibility (not met): The full-score demo relies on a rigid bristle field penetrating the phone's non-colliding
> frame, so it does not demonstrate the real contact motion.
>
> What to change: Model the brush and port so the demonstrated motion includes realistic bristle deflection and
> contact with the frame and tongue, and provide a full-score demo using that model. Replace the obscured
> bristle-depth check with a reliably observable real-trial measurement or an outcome-based check for the relevant
> damage or packed lint.
>
> To submit a revision, push your changes to this pull request (no need to open a new one). Your idea stays reserved for 48 hours from this review; if no revision arrives by then, the hold lapses and another team can claim the idea.

Point by point:

1. **"Specify a repeatable lint blend and packing procedure to reduce variation between resets."** The dryer lint is gone. The lint is now one
   commercial stock, Quilter's Dream Natural 80/20 crib batting (80 % cotton, 20 % polyester), cut into 2 x 2 mm
   squares on a printed 2 mm grid, each rolled dry into a ball about 2 mm across. Each ball is pressed into its place
   once with a fixed tool: a blunt 0.5 mm nylon pusher with a rigid 6 mm diameter plastic collar leaving exactly 3.5 mm protruding,
   pressed once until the collar bridges the mouth. No metal enters the port. A square-on macro photo before the trial enrols the four
   balls; a ball that is loose above the mouth or not visible is redone (see "Reset").
2. **"The rigid bristle field passes through the non-colliding frame and tongue, eliminating the deflection and contact
   forces needed to brush inside the real port."** The port end now collides: the
   frame round the opening, the end walls, the cavity floor and the tongue are all colliders. The rigid bristle box is
   gone. The bristles are 40 tufts, each three jointed segments with two passive bending swings per segment, set from
   the bending stiffness of the nylon filaments (next section). The joints permit bending and buckling on the frame; tufts over
   the opening are intended to enter the 0.93 mm gaps and bend against the gap walls and the tongue. Both-gap material fixtures pass the overlap guard; full robot cleaning remains under test (Results). Their native contact forces push on
   the phone (which rests loose in a loose stand) and back on the brush and the hand.
3. **"A side video of a mark on the outside of the bristle field cannot reliably reveal the depth of bristles hidden
   inside the narrow port."** The pen line and the 6.5 mm
   depth rule are deleted from the scoring, the code and the text. The real-trial score uses no bristle-depth mark or
   hidden-tip observation. Scoring now reads the outcome: square-on flashlight macro photos of the opening at the end
   show whether lint is still in each quarter. Lint that a jab pushed onto the port's floor is packed there and is
   still in its quarter at the end. It also fails the post-photo plug-seating check, so packing scores 0 (the verdict's "outcome-based check for ... packed lint").
4. **"The full-score demo relies on a rigid bristle field penetrating the phone's non-colliding frame, so it does not
   demonstrate the real contact motion."** The old rigid-field demo is withdrawn and is not used as evidence. The
   scripted demo on this revised source (jointed bristles, colliding port) does not yet complete the task: it scores
   0.0 in lot A and 0.0 in lot B. In both lots the bristles went into the colliding port within the 0.1 mm overlap
   guard, but at about 28 s, when the lint resistance first acted, a tip force sensor read 10-22 micronewtons above
   the native force cap. That exceeds the 10 micronewton allowance, so the trial was marked invalid before any lint
   ball was hooked. The cause of that excess is not yet explained. The controller targets one gap, strokes along the
   slot, withdraws and wipes, then does the same for the other gap (Results).
5. **"Model the brush and port so the demonstrated motion includes realistic bristle deflection and contact with
   the frame and tongue, and provide a full-score demo using that model."** The model part is items 2 and 4. The full-score demo
   part is not done: no run on this model has scored above 0 (item 4). The bending stiffness is derived below from a
   published filament modulus; the filament count and diameter are reference figures from a different brush.
6. **"Replace the obscured bristle-depth check with a reliably observable real-trial measurement or an outcome-based
   check for the relevant damage or packed lint."** Item 3: the end state is read from square-on macro photos (a named
   clip-on macro lens at a stated working distance, with a 0.5 mm rule; an unreadable quarter scores 0), and a plug
   seating check after the photos catches packed lint. Neither has yet been tried on a real phone.

Other changes: the toothbrush is now an Oral-B CrossAction All-in-One Soft, whose tufts are angled along the head
(the old model's tufts were straight; angled tufts are what lets a pressed head splay on a smooth frame in the
simulation, see below). The stand is now loose on the counter (it was fixed). In lot B the phone and stand stand
20 mm further right, instead of the brush lying elsewhere.

## Why it matters

- Samsung's [Basic charging tips for your Samsung Galaxy](https://www.samsung.com/ca/support/mobile-devices/galaxy-device-not-charging-or-charging-slowly/)
  identifies debris in the port as a common charging problem and advises flashlight inspection. Its directions
  exclude metal tools and liquids.
- Samsung UK, [What to do if your phone is not charging](https://www.samsung.com/uk/support/mobile-devices/what-to-do-if-your-phone-is-not-charging/),
  notes that debris can interrupt charging for protection, and says: "Remove debris using a soft brush."
- Lint that has just collected in the port is loose and a soft brush lifts it out. Lint pushed down onto the port's
  floor, by a jab with a tool or by the plug, packs there, and then the cable no longer seats (that packed lint remains
  after these gentle strokes is this task's stated assumption). So the job is to clean the port early, gently, and without
  pushing the lint further in.

## What the robot has to do, and why each part is hard

1. **Pick up the toothbrush by its head** (11 mm wide) where it lies bristles-down, and carry it bristles-down.
2. **Find the port and line a row of bristles up on one of its gaps.** The opening is 8.34 x 2.56 mm on the phone's
   7.6 mm top edge, about 154 mm above the counter, with the phone upright. Each gap beside the
   tongue is 0.93 mm wide.
3. **Press the bristles in.** The tufts over the frame have to buckle before the tufts over the gap reach the lint,
   4.5 mm down. Pushed too far, a tuft in the gap passes the ball and packs it onto the floor (5.75 mm).
4. **Sweep the lint out with a stroke along the slot, and stop.** Pushing straight in and pulling straight out moves
   nothing: the tips have to travel 2 mm through a ball, sideways, against its resistance. A stroke carried on to the
   end of the slot scrapes the lint off there, where it stays. Then the bristles come straight out. Then the other
   gap.
5. **Get the lint off the brush.** Lint carried on the bristles goes back into the port if the loaded bristles are
   pushed in again, so it has to be wiped off: drawn at least 10 mm across the towel with the tufts pressed on it.
6. **Lay the toothbrush down** clear of the phone and move both hands away.

## Answers to questions a reviewer may ask

- **Why a toothbrush?** Samsung prescribes "a soft brush" and rules out metal objects and liquids. A clean, dry,
  soft toothbrush is the soft brush most homes have, and its head (11 mm) can be held by the arm's parallel
  gripper.
- **Why is the phone in a stand, upside down?** The port then faces up under the light, which is how it is
  inspected; the stand is an ordinary desk phone stand. Neither is fixed: the phone rests in the stand and the stand
  rests on the counter, so the bristles' push reaches both. The trial scores 0 if the phone is knocked (moved more
  than 6 mm or tipped more than 4 degrees).
- **One hand or two?** The current demo brings a left finger to a lower side corner while the right brushes. The phone and stand stay loose. An earlier pickup/alignment diagnostic (at an earlier solver setting, before the current source) measured up to 0.31 N left-finger contact on the phone without a hard-contact, movement or material-validity fault. Full cleaning and an unbraced comparison remain unproven.
- **Is the lint simulated?** No. It is a stated rule on the simulated bristle tips (next section), drawn as four
  small off-white ellipsoids (0.8 x 2 x 2 mm compressed across the gap, nominally 2 mm across outside it) that ride out on the bristles and stay on the towel. Its declared resistance is applied as a force in the
  simulation to the bristle tips moving through it and as the exact opposite force and moment to the phone.
  No lint-removal or packing credit is allowed without previously applied resistance.
- **What is not covered?** Charging, the electrical contacts, damage to the contacts below the gross level a macro
  photo shows, and other cleaning methods (air, liquids, picks).

## The bristles and the lint (simulation)

**Bristles.** 40 tufts in four rows and ten columns, 11 mm free length and about 1.15 mm diameter, obtained by phone from a phone-repair technician (Nov 2026). Each tuft has three equal segments and two passive bending swings at every joint. All shaft boxes collide with the frame, tongue and floor. A flat 0.5 x 1.15 mm bundle represents separated filaments; its thickness is estimated and individual-filament splay is omitted. Neighbouring columns lean in opposite directions: Oral-B's reference CrissCross angle is 16 degrees, with an estimated 4 degree natural tip curve.

The reference stiffness uses nylon-612 E=4.0 GPa (DuPont Tynex brochure, Table 1, 50% RH), d=0.15 mm and 72 reference filaments from a different brush patent, EP4295727A1. For n segments, each swing spring is k=N*EI/L*(n+1)*(2n+1)/(2n), preserving the cantilever tip stiffness 3*N*EI/L^3 = 16.1313 N/m. At n=3, k=0.00303628 Nm/rad. Effective tip damping 0.661157 Ns/m is an estimate. These are reference properties, not measured Oral-B stiffness or filament count: 72 such circular filaments occupy 1.27 mm², exceeding the measured tuft's 1.04 mm² outline. The higher reference resistance is retained. Product stiffness remains a measurement needed.

Reference shaft mass comes from the same filament geometry and DuPont's 1060 kg/m³ density: 14.8355 mg per tuft, 0.5934 g across the field. The remaining 15.4066 g belongs to the plastic handle/head; total 16 g remains an estimate. No inflated filament masses, gripper attachment or world fixture is present in the robot task. Self-collision within the tuft field is omitted. The task-local actuator configuration also adds 1e-8 kg m² numerical armature per bending DOF; this is not measured filament inertia. The reference stiffness is a quasi-static property, and product transient response remains unmeasured.

**CPU PhysX is required** (`-E device=cpu`), using PGS at 1920 Hz and 128/4 position/velocity iterations. A complete three-segment field with two degrees of added roll passed insertion, strokes and withdrawal in both gaps in an unscored material fixture. CUDA complete-field tests and the finer six-segment candidate failed and are not qualification evidence. The effective friction coefficients remain 0.20 static/0.15 sliding via minimum combining; these are unmeasured estimates. The numerical solver setting changes neither simulator nor physical dimensions. Brush/phone contact-generation margins are 100 micrometres (rest offsets remain zero); these are numerical early-contact settings, not enlarged physical gaps. Speculative angular CCD is enabled on the 120 shaft bodies to anticipate fast rotation in contact generation; it may generate early contacts, and the actual 0.1 mm geometry guard remains unchanged.

An independent oriented-box audit checks native shaft/phone poses at every physics step while a tip is within 30 mm of the port or carrying lint, and at every 20 Hz control sample elsewhere. Overlap above 0.1 mm latches simulation failure. Joint coordinates are audited at 20 Hz; missing, nonfinite or out-of-limit motion fails. Native force-observer errors also latch failure. Recovery cannot undo any invalidity. These guards generate no forces or lint progress and are not real-trial hidden-depth observations. A material fixture does not establish full robot cleaning.

**Port.** Opening 8.34 x 2.56 mm and tongue 6.69 x 0.70 mm from USB-IF Type-C R2.0 (August 2019),
Figure 3-1, pages 42-44 (0.70 mm is a reference dimension). The approximately 8.3 x 2.6 mm opening and 7.5 mm depth were obtained by phone from a technician who works with this product (Nov 2026), measured on a production part. The tongue's top 2.0 mm below the mouth is an estimate for this phone. All of it collides.

**Lint.** Each control step, `record()` in `task.py` reads the tip of every tuft from its deflected end segment and
expresses it in the port's frame (u through the phone's thickness, v along the opening's 8.34 mm side, d depth below
the mouth). The four balls sit at u = +/-0.815 (the middle of each gap), v = +/-2.2, d = 4.5 mm. For each ball:

- a tip is **in it** when it is in the ball's gap, within 1 mm of its centre along the slot and within 1 mm of its
  depth (3.5 to 5.5 mm down);
- a touching tip activates native velocity resistance capped at 0.0301 N per axis per lint contact, with equal phone reaction; actual observed force below the 0.03 N gate earns no capture credit;
- only after at least 0.03 N delivered resistance remains after subtracting 10 micronewtons of numerical uncertainty in the preceding interval, sideways tip travel through the
  ball is weighted by the opposing component of that force. Assisting force earns nothing and credit never
  exceeds actual travel. The largest eligible contribution per step is accumulated; jumps over 3 mm count
  nothing. At 2 mm (60 microjoules at the nominal force) the ball is **hooked** onto the nearest tip;
- a tip in its gap, within 1 mm of it along the slot, pushed deeper than 5.75 mm under the same applied resistance **packs** it onto the floor, where
  it stays;
- while still retained in the port, a hooked ball continues to resist movement with the same native capped drives and equal phone reaction. First exit must follow an actually resisted interval, with no jump over 3 mm, and cumulative net resisting work since capture must be at least 0.03 N times its outward depth change; assisting work is deducted; an unloaded or teleport-sized exit invalidates the simulation. Hooking alone cannot give a force-free extraction;
- a hooked ball dragged to within 0.25 mm of the slot's end below the mouth is **scraped off** and is in the port
  again; it is **out** once it is 0.5 mm above the mouth, and carried back into the opening after that (0.5 mm
  down) it is left there again;
- a carried ball within 1 mm above the towel, over it, whose end segment has at least 0.02 N native towel contact within 1 mm of its tip, and
  that slides 10 mm across it (counted between consecutive steps, 3 mm a step at most) **stays on the towel**.

**Native lint resistance.** Lint contact activates three native capped velocity drives between each touching tuft endpoint and the phone. All six relative coordinates remain free, position stiffness is zero, and target velocity is zero. The drives oppose relative movement; they cannot pull a tip toward a target pose or attach the brush to the gripper. The per-axis cap is 0.0301 N per lint contact, slightly above the unchanged 0.03 N force gate. Forty permanent fixed sensor frames at the existing tuft endpoints read the delivered external force from native joint reactions and inertia, without subtracting phone/towel contact forces. Each collisionless frame has 1e-9 kg, deducted from the plastic body so total mass stays 16 g. They add no moving DOFs and change no shaft or port collider. The scorer subtracts 10 micronewtons of numerical uncertainty from force and work credit (a reading more than 10 micronewtons above the native cap invalidates the trial), retains all assisting-work debt, and still requires 2 mm of resisted capture travel and paid extraction work. These sensors are simulation instrumentation, not real force measurements.

The 0.03 N, 0.02 N, 2 mm, 10 mm and 5.75 mm are stated estimates, not measurements. The retention estimate is
of the same order as the 0.032 N transverse force predicted to bend a 72-filament tuft by 2 mm (16.13 N/m tip
stiffness). It is a chosen model load, not an empirical property of the batting. Capture credit uses net resisting work: assisting work found from native force and tip-displacement observations remains a debt and must be repaid before further credit; positive credit is capped by one tip's across-slot movement. Tiny zero-net-work vibration cannot accumulate capture credit. Doubling it predicts about 3.7 mm
free tuft deflection, so real retention and brush stiffness measurements could materially change the demo.
Carried lint must actually intersect a phone solid before the return/scrape rule applies resistance and leaves it behind; entering an empty part of the opening alone does nothing. The extracted proxy expands from 0.8 to a nominal 2 mm across u (an estimate of fibre re-expansion, not a simulated material). A robot finger, or the brush's
head, neck or handle making native contact on the phone's top 14 mm scores 0 for the trial (a hard
object at the port: Samsung's "without using metal objects").

## Scoring

Every trial is scored by `score()` in `task.py`; `task.json` has the short text.

**Score 0 for the whole trial, at any moment (it stays 0):**

- a hard object at the port: a robot finger, or the brush's head, neck or handle, touching the phone's top edge
  (side video; simulation: native contact point in the top 14 mm and nonzero normal force, with 0.0001 N
  numerical tolerance);
- the phone knocked: moved more than 6 mm or turned more than 4 degrees from its place in the stand (side video
  against tape marks on the stand);
- the phone or the toothbrush falls off the counter;
- the scene cannot be read (the scorer fails closed).

**Every score also needs, for the last 1 s:** the toothbrush let go and lying on the counter or the towel (its lowest
point within 4.5 mm of the counter top) at least 5 mm from the phone and the stand; both hands at least 30 mm from
the phone and the toothbrush; the phone and the toothbrush still (under 10 mm/s and 0.5 rad/s).

**Counting lint (the outcome).** After the robot has let go and before anyone touches the phone, the opening is
photographed square-on under a flashlight with a 0.5 mm rule beside it, as at enrolment. It is read in four quarters
(each side of the tongue, each end), one per ball. A quarter still holds lint if any fibre 0.5 mm or longer is seen
in it, at any depth, including on the floor (packed lint). If light, focus or occlusion prevents reading
any quarter, the trial scores 0; a dark or hidden quarter is never counted as clean.

**Plug seating, after the photos.** Every score also requires the same unpowered standard USB-C cable plug to seat to its clean-port baseline witness mark at the phone rim, checked in a macro photo against the rule. Establish the mark on the mechanically intact, empty port before enrolment; do not assume an arbitrary insertion length for a cable shell. Do not force a plug that stops short. A failed or unreadable seating check scores 0. Take the lint outcome photos first, so the check cannot improve the photographed outcome. In simulation, any PACKED ball fails this check conservatively; the connector is not physically simulated. This tightens the former per-quarter packing penalty in response to strict review B4. A real baseline and packed-port check remain unperformed.

| Outcome at the end | Score |
|---|---:|
| No quarter holds lint, and all 4 balls are on the paper towel (photo of the towel) | 1.0 |
| No quarter holds lint (some still on the bristles, or wiped off elsewhere) | 0.75 |
| 1 or 2 quarters hold lint | 0.5 |
| 3 quarters hold lint | 0.25 |
| 4, or any rule above broken | 0 |

Lots A and B are averaged.

## Shortcuts and naive moves, and what they score

All of these are run by `near_misses.py` (behaviour cases and end states; results recorded below). They test the
scoring logic with tip positions placed by hand, not the physics:

| Attempt | Score |
|---|---:|
| Tips pushed 4.5 mm into both gaps and pulled straight out, no stroke | 0 |
| Strokes with the tips 3.0 mm deep (above the balls) | 0 |
| Brushing across the outside of the mouth | 0 |
| Short strokes, 0.4 mm each way | 0 |
| Jumps of 4 mm a step (not brushing) | 0 |
| Both gaps jabbed 6.5 mm deep and stroked (all four balls packed) | 0 |
| One gap jabbed 6.5 mm (two packed), the other cleaned: plug cannot seat | 0 |
| Only one gap brushed | 0.5 |
| Strokes carried on to the slot's end (two balls scraped off) | 0.5 |
| All the lint out, then one loaded row pushed back in (two balls go back) | 0.5 |
| All the lint out but left on the brush when it is laid down | 0.75 |
| Loaded bristles set on the towel without sliding, slid only 8 mm, or slid 25 mm without pressing (0 N or 0.01 N) | 0.75 |
| Lint wiped on the counter beside the towel | 0.75 |
| Full clean, but the brush still held, or still for under 1 s | 0 |
| Full clean, but the brush left lying across the phone or against it on the stand | 0 |
| Full clean, but the phone knocked 8 mm or tipped 6 degrees | 0 |
| Full clean, but the brush's head or a finger touched the port end | 0 |

## Nearest tasks, and how this differs

- **Pop a phone's SIM tray out with the eject pin (TK-MLJF7T, accepted):** a rigid pin pushed straight into a 1 mm
  hole on a docked phone to trip a latch. Here a soft tool's bristles must bend into a slot, move sideways inside
  it, and carry debris out.
- **Scrub an IV connector's hub before flushing (TK-F37LCS, accepted):** a pad rubbed on the outside of a hub. Here
  the dirt is inside a 2.56 mm slot, past a tongue, and pushing too far packs it.
- **Clean an SC fibre connector's ferrule on a cassette cleaner (TK-6Z2YW3, accepted):** a connector end wiped
  across a fixed cleaner. Here the tool goes into the socket and the debris must come out with it.
- **Clean a car battery's tapered post with a terminal brush (TK-4Z8LF9):** brushing the outside of a large post
  with a brush made for it.
- **Scrape stuck-on debris off a griddle into its tray (TK-63RTRA, accepted):** an open flat surface; here the debris
  is hidden in a narrow socket and is lifted out, not pushed along.
- **Take a screenshot on a phone in a desk stand (TK-QJFSFZ, accepted):** the same kind of scene (a phone in a desk
  stand) for pressing buttons; here the skill is tool use inside the port.

## Run a trial

From this folder, in the Isaac Lab-Arena environment of the program's simulation walkthrough (yam-isaac-lab-arena,
the program's YAM embodiment for Isaac Lab-Arena). Trial k uses lot k mod 2, so `--epochs 2` runs both lots and
`task_score` is their mean:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E device=cpu -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Brush the lint out of the phone's charging port with the toothbrush, wipe the lint off onto the paper towel, then lay the toothbrush down." \
  --scorer task_score --max-steps 1800 --epochs 2 --store-frames
```

- The scripted demo: the same command with `--policy yam_scripted -E cam_width=640 -E cam_height=360`. Its plan
  lasts 90 s of the 90 s limit; a trial ends earlier, once the full score has held for 1 s.
- `TASK_LOT=A` or `TASK_LOT=B` in front pins every trial to one lot (debugging only). Runs use `object_jitter=0`.
- The simulation prints one `[lint]` line every 0.5 s: each ball's state (0 in the port, 1 on the bristles, 2 on the
  towel, 3 packed), the sideways travel and towel slide counted for each, the deepest tip in the opening, each tip
  in the opening (tuft, u, v, d in mm, and its contact force on the phone in N), the bristles' total contact force on
  the phone and separately on the towel, the lint's resistance applied, the largest hinge angle, the rule flags and the
  score.
- `python near_misses.py` runs the scorer's behaviour cases offline.

## Reset and real-trial kit

The exact reset is in `task.json` reset_notes. Use a single ENCB batting pack and record its lot, four full-loft
2 x 2 mm coupons per reset, one press per ball, and the hard-stop pusher. Print `fixture/reset-template.svg` at
100% and check its 10 mm calibration bar. The pusher is a blunt 0.5 mm nylon rod/line with a rigid 6 mm diameter, 1 mm thick plastic
collar fixed outside the insertion end; exactly 3.5 mm protrudes. Check this against a 0.5 mm rule before every
reset. Its diameter bridges both rims even at either gap centre. The stopped face is 3.5 mm down; the
nominal 2 mm ball centre is 4.5 mm down, matching the model. Compression and springback are estimates needing
a real reset check. The collar bridges the mouth. Metal, adhesive and a depth-marked wire do not enter the port.
Photograph all quarters empty, then all four enrolled balls under the same square-on light and scale. Reject a
missing ball, a ball loose above the mouth, an unreadable quarter or a collar that cannot seat flat. Keep both
photos. The 3-5 minute reset is an estimate; no timed physical trial has been performed.

Prices displayed on pages opened November 23, USD before tax/shipping:

| Item | Source and displayed price | Model use |
|---|---|---|
| Used Samsung Galaxy S24 | [Back Market](https://www.backmarket.com/en-us/p/galaxy-s24), $299.00 | Powered off; 147.0 x 70.6 x 7.6 mm, 167 g |
| Oral-B CrossAction All-in-One Soft, 4 pack | [Target](https://www.target.com/p/-/A-14770185), $12.99 | Tuft diameter, length and layout: technician phone figures (Nov 2026); other dimensions, mass and material grade estimated |
| Lamicall DP13 adjustable phone holder | [Lamicall shop](https://lamicallshop.com/products/height-adjustable-phone-holder), $16.99 | Upright placement required. Minimum usable ledge height and mass are not verified against the simulated 76 x 76 mm, 120 g stand and 7 mm ledge |
| Quilter's Dream Natural 80/20 crib batting ENCB, 46 x 61 in | [Fat Quarter Shop](https://www.fatquartershop.com/natural-dream-80-20-crib-quilt-batting-package), $15.48 | 80% cotton, 20% polyester; one pack/lot for all resets |
| APEXEL Classic 100mm macro lens APL-HB100MM | [APEXEL](https://www.shopapexel.com/collections/macro-microscope/products/100-mm-macro-lens-for-cell-phone), $35.99 (opened Nov 24) | Maker working range 40-70 mm; use 50 mm, camera square to the port |
| Bounty towel/tape tabs, blunt nylon pusher/collar, paper template, 0.5 mm rule, flashlight and USB-C cable | $20-40 combined allowance, estimate rather than quoted prices | Already-owned camera excluded; images must pass the readability check |

Listed items total $380.45; with the small-item allowance, estimated parts total $400-420. Nothing was purchased.
A mechanically intact USB-C port is required even if the powered-off phone is cosmetically worn. Use the APEXEL lens at 50 mm working distance; aim the flashlight about 2 degrees from the port axis along the slot, alternating its two ends if needed. The distance lies within the maker's range; the light angle is a proposed geometry, not a measured optical guarantee. Enrolment must resolve the 0.5 mm rule and all four quarters. Native rendered lighting probes do not establish that this real kit can read every floor fibre; the real check remains outstanding.

Stiffness sources actually opened: [DuPont Tynex brochure](https://brushtec.com/wp-content/uploads/2023/01/Filaments_Brochure_web.pdf),
Table 1, printed page 4; [EP4295727A1](https://patents.google.com/patent/EP4295727A1/en), first-tuft example (72 +/-4,
0.15 mm, for a different brush); [Oral-B manual brush](https://oralb.com/en-us/products/toothbrushes/manual-toothbrushes/pro-health-manual-toothbrush/),
16 degree CrissCross tufts. Port drawing: [USB-IF specification](https://www.usb.org/sites/default/files/USB%20Type-C%20Spec%20R2.0%20-%20August%202019.pdf),
Figure 3-1, printed pages 42-44. Port opening and depth were obtained by phone from a technician who works with this product (Nov 2026), measured on a production part. Brush length, layout and tuft diameter were obtained by phone from a phone-repair technician (Nov 2026). No published figure was found for this batting's retention force; it remains a declared estimate.

## Results

Scripted demo on this source (CPU PhysX, PGS at 1920 Hz, 128/4 iterations, angular CCD, 100 micrometre contact margin), task instruction, 1800 steps per lot: **0.0 in lot A and 0.0 in lot B.** No lint ball was hooked, removed or wiped in either lot. In lot A the brush inserted into the colliding port without breaking the 0.1 mm shaft-overlap guard (peak 0.081 mm up to the failure below) and reached 5.0-5.8 mm deep. When the lint resistance first acted, at about 28 s, a fixed tip sensor read 10.1 micronewtons above the native 0.0301 N cap. That is more than the 10 micronewton numerical allowance, so the trial was invalidated. In the strokes before that, the work rule had recorded 0.26 mm of assisting-work debt on one ball and no capture credit. Lot B failed the same way at about 28 s (22 micronewtons over the cap, 0.15 mm of debt on another ball), and excessive tip travel in one control step followed at 33 s. After each lot was already invalid, sampled overlap rose to 0.26 mm (A) and 0.54 mm (B). The robot demo has therefore not yet shown cleaning on the revised model.

Dream-1 on the same source and instruction, 1800 steps per lot: **0.0 in lot A and 0.0 in lot B.** The brush never reached the port (no tip entered it, no force on the phone).

Earlier, a native PGS 1920 Hz fixture with 40 tufts, three segments and two degrees of added roll completed insertion, strokes and withdrawal in both gaps (peak per-physics overlap 0.049/0.043 mm, net resisting sampled work); that is a material fixture, not a robot result. Known +/-0.04 N loads checked the virtual force sensors' sign and frame; these are not real batting or brush measurements. Earlier rigid-field successes do not apply.

`python near_misses.py`: 43 behaviour cases and 12 end states pass (scoring logic only). Real lint forces, scoring readability, plug-seat checks and upright holder matching remain unverified.

## Licence

Task code MIT; text CC BY 4.0. Robot assets keep their upstream licences.
