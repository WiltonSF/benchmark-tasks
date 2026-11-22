# Ream a robot finger's pivot bore through both knuckle ears without bell-mouthing it or marking its routing-pin seat

A 3D-printed robot-hand finger (the Yale OpenHand Model T42 pin-flexure finger, from the maker's own STL) has just
had its pad and flexure cast. Its pivot bore, two 3 mm ears either side of an open spring pocket, is printed tight:
the 6.35 mm joint pin does not go in. The robot takes a ratcheting T-handle tap wrench holding a 0.2510 in (6.375 mm)
reamer from its stand, reams both ears on the bore's own axis with ratchet strokes of the housing against the cutting
torque, withdraws it still turning forward, and sets
the wrench down. Turning the
reamer off-axis bell-mouths an ear beyond the maker's 6.70 mm design clearance (the finger then wobbles on its pin
more than designed), pushing it hard splits a thin ear, and
turning it on the 3.3 mm routing-pin seat 12 mm away ruins that press fit.

In real trials the cast finger is reused: each ear holds a replaceable press-in ABS bushing, 3 mm long like the ear,
whose bore is printed tight in the same way, so a trial consumes two bushings (0.21 g of ABS) and the reset takes about
5 minutes (see "Real-trial finger" below and `fixture/README.md`).

**Category:** Impact on AI timelines. Robot hands for research labs are still built and finished by hand.

## Run

```
inspect-robots run --embodiment yam_arena --policy <policy> -E task=tasks/ream-the-finger-pivot-bore-without-enlarging-its-pin-seat/task.py \
  -E object_jitter=0 -E control_hz=20 --instruction "<instruction from task.json>" --scorer task_score \
  --max-steps 2400 --epochs 2
```

- Simulator: yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena).
- `--epochs 2` runs lot A then lot B (trial k uses lot k mod 2) and averages them. `TASK_LOT=A` or `TASK_LOT=B`
  pins one lot. Without `--epochs 2` only lot A is run.
- `-E object_jitter=0`: the layout is fixed; no seed changes the scene.
- Runtime: about 4 minutes per trial on an L40S-class GPU (scorer included); the scripted demo's waypoints take
  110 s of the 120 s limit.
- `--policy yam_scripted` plays `demo()`: open loop from the current lot's published layout (the stand and vise
  positions in `task.py`) plus two static offsets per arm in `fixture/calibration.json`, measured once from a trace.
  It never reads object poses, joint states or the cut record during a trial.

## Scene (lot A; every object at true size)

| Object | Product / source | Size, material, mass |
|---|---|---|
| Finger | Yale OpenHand Model T42 pin-flexure finger, original STL `finger_pf_torsion_t42.STL` (openhand-hardware commit b10197d), printed in ivory ABS | 117.6 x 30.6 x 31.75 mm; pivot bore CAD 6.70 mm through two 3 mm ears (z 24.25-27.25 and 4.50-7.50) with a 16.75 mm open spring pocket between; 3.325 mm routing-pin seat 12.4 mm from the pivot. Cast pad (Vytaflex 30, black) and flexure (PMC-780, amber) per the maker's HDM process; cavity fill estimated. Real trials: a reusable cast carrier of this STL whose two ears hold replaceable press-in ABS bushings (9.40 mm OD x 3.00 mm, flush) with the same tight bore |
| Load plate (real trials) | Phidgets KIT4007_0 weighing-scale frame (156 x 156 mm anodised aluminium top and bottom plates) with its 0-25 kg FRC4116_0 load cell, read by a Phidgets DAQ1500_0 bridge interface at 50 samples/s through a HUB0007_0 USB hub | under the vise; everything else raised by its height so its top is the bench level; logs the downward push on the finger (not simulated as a part: the simulator reads the reamer's push directly) |
| Drill-press vise | LittleMachineShop.com 2.5 in drill press vise (63.5 mm jaws, 38.1 mm jaw depth, 63.5 mm opening, 2.0 kg) | cast-iron body, steel jaws; base 107 x 66 x 20 mm and bed height 32 mm estimated from the listing |
| Parallel spacer | 6 x 25 x 36 mm ground steel parallel | between the fixed jaw and the finger's back face |
| Ratcheting T-handle tap wrench | McMaster-Carr 2544A5 (1/4-1/2 in taps, 0.188-0.313 in jaws, 4-1/4 in long) | ratchet housing 18 mm dia x 80 mm with the sliding T-bar (90 x 6.4 mm) through its top: this is what the gripper holds and swings; chuck nut 21 mm dia x 22 mm below it is the ratchet's output and holds the reamer; direction knob set to clockwise; sizes estimated from the listing |
| Reamer | McMaster-Carr 8307A513, 0.2510 in carbide, 6 straight flutes, 1-1/8 in flutes, 3-1/2 in overall | 73 mm protrudes below the chuck nut (estimate); wrench and reamer 0.16 kg (estimate) |
| Tool stand | custom, mild steel, `fixture/README.md` | slotted top plate 80 mm above the bench; the wrench hangs by its chuck nut, reamer tip 7 mm above the bench |
| Bench items (fixed, not used) | printed T42 hand base (grey ABS, 95 x 90 x 40 mm, at x 0.51-0.61, y -0.045-0.045 m); two printed mold halves (blue PLA, 60 x 125 x 20 and 45 x 125 x 20 mm, at x 0.52-0.64, y 0.075-0.20 m); parts tray (100 x 120 x 6 mm) with a finished finger (same STL) at x 0.52-0.62, y -0.20 to -0.08 m | sizes estimated; they show the hand being built and limit where the arms can put the wrench down |

The Yale finger guide names a 0.2510 in reamer, McMaster 2791A33 (6 in long); 8307A513 is the same cutting diameter
in a 3-1/2 in length, which keeps the wrench low enough to lift over the knuckle.

**Real-trial finger: a reusable carrier with replaceable bore bushings (reproducibility).** A finger is printed flat
in ABS (the STL's z, the pivot axis, vertical) on an FDM printer at 0.2-0.25 mm layers and cast per the maker's HDM
steps once, then reused for every trial (a second one is kept as a spare). Its only change from the maker's STL is
that each ear's 6.70 mm bore is opened to a 9.30 mm socket through the 3 mm ear; the ears keep their faces,
thickness, 16.75 mm spacing and axis. Each socket takes a press-in ABS bushing (`fixture/bushing.stl`, made by
`fixture/bushings.py`): a tube 9.40 mm OD x 3.00 mm long, flush with both ear faces, drawn with the maker's 6.70 mm
bore and printed with the axis vertical in the same ABS at the same settings, so its bore comes out tight in the same
way as a one-piece ear. The maker's CAD bore carries a 0.35 mm free-clearance allowance "assuming the part is printed
perfectly"; small vertical holes in FDM ABS commonly come out 0.2-0.5 mm under size (estimate), which is why the guide
reams them. The scene models a 6.25 mm print (estimate). The reamer cuts the same 3 mm ring of the same printed ABS,
held all round by the press fit as the solid ear holds it, so the cutting torque and push should match a one-piece
ear (reasoned from the same material, print orientation, length and cut; not measured). Bushings are printed in
batches ahead of time (about 40 a build plate, 0.11 g each); a trial uses two.

The whole reset, bushing swap and enrolment included, takes about 5 minutes per trial; the steps, tools and drawings
are in `fixture/README.md`. The spring pocket between the ears is open on one side, so the lower bushing goes in and
the used upper one comes out sideways through it. Enrolment, so every trial starts with the same reaming task in both
ears:
1. each bushing, pushed on its top face at 50 N (5.1 kg on a kitchen scale) in the reaming direction with nothing
   under it, does not move, and sits flush with both faces of its ear (no step felt with a fingernail): it holds at
   least twice the 25 N limit and, by the same friction on its 4.7 mm radius, about 0.2 N m of torque, above the
   0.15 N m cutting torque estimate;
2. the 6.350 mm go pin does not enter either bushing at 5 N (both ears start tight);
3. one 6.20 mm pin passes through both bushings in a single push at 5 N or less (both bores are at least 6.20 mm and
   their axes in line within about 0.15 mm);
4. under a 10x loupe, no cut, burr or chamfer on either ear face, the scaffold's top face or the routing-pin seat's
   mouth, and no cut or dent in the pad or flexure. (The carrier's routing-pin seat is checked once, when it is made:
   the finger's 3.175 mm dowel pin goes in only as a press fit.)
A bushing that fails 1-3 is discarded and replaced; a carrier that fails 4, or whose socket lets two fresh bushings
in a row fail 1, is replaced by the spare. A paint-pen witness line is drawn across each bushing and its ear face.

Lot A: vise with its pivot bore at (0.290, -0.085) m (x away from the arm bases, y to the robot's left), knuckle
overhanging toward the right arm, finger body toward the robot's left; tool stand at (0.200, -0.200) m. Lot B: the
vise turned 180 degrees about the pivot bore (the bore stays at the same point; the knuckle now overhangs toward the
robot's left and the finger body, jaws and routing-pin seat lie on the right-arm side); tool stand at
(0.335, -0.235) m, beyond the vise, so the wrench is fetched from a different place and must be carried around the
vise body. Every other object is the same in both lots. The screenshot shows lot A. (The vise carries the jointed
cutting stop on the bore axis, so it is only turned about that axis, never moved: moving a jointed part at reset
leaves it millimetres off its start pose.)

## Scoring

**Score 0 for the trial, at any moment, even if everything is finished afterwards:**
- an ear is started (first clockwise turn under load) with the reamer more than 0.5 mm off the bore axis (its lead
  is cutting the ear face, not the bore);
- an ear is bell-mouthed: while it is being cut, the reamer's axis wanders enough that the bore it sweeps at the
  ear's mouth (0.5 mm and 2 mm in) is 6.70 mm or more across, so the 6.70 mm no-go pin (the CAD free-clearance bore,
  0.35 mm over the joint pin) would enter 2 mm; or the reamer is levered more than 5 degrees off the axis while
  cutting (a carbide reamer levered that hard chips or side-cuts);
- the reamer is turned backwards by its chuck nut (the ratchet's output, which does not slip) more than 20 degrees in
  total over the trial while it is in a started ear;
- any part of the reamer is more than 4.5 mm from the bore axis inside the knuckle's material (a gouged knuckle wall);
- the reamer is turned while its tip rests on the scaffold's top face, including the routing-pin seat's mouth;
- the reamer, the wrench or a gripper enters the cast pad or flexure, or the wrench or a gripper enters the knuckle;
- the axial force on the reamer, averaged over any 0.25 s, exceeds 25 N (a 3 mm ear splits);
- the vise is moved more than 30 mm or tilted more than 10 degrees;
- the reamer is still inside the knuckle when the trial ends (the bore cannot be gauged).

In a real trial these zero rules are read as: the 6.70 mm no-go pin entering either ear 2 mm at 2 N (bell-mouth);
a cut or countersink on an ear face outside the reamed bore's edge, on the knuckle wall, the scaffold face or the
routing-pin seat's mouth under a 10x loupe (the thin burr that normal reaming leaves at a bore's exit edge does not
count); a cut or dent in the urethane; more than 25 N on the load-plate log under the vise as the mean of any 13
consecutive samples (0.26 s at 50 samples a second; the log minus the tare taken with the vise and finger on the
plate is the push on the finger, which in a trial comes from the reamer), or a bushing found pushed out of flush
after the trial (it needed more than its 50 N hold, which would split a one-piece ear); a lean of the wrench over
5 degrees against a machinist's square in either of two side videos taken 90 degrees apart; a backward turn of the
chuck nut read from four paint stripes 90 degrees apart on its side in the side videos; the vise's position against
a pencil outline; the reamer left in the bore. A bushing whose witness line is offset while it is still flush turned
in its socket instead of being cut: that trial is void and is run again with a new pair (the simulated ear cannot
turn).

**Otherwise**, read after the trial with class ZZ gauge pins pushed by hand: go 6.350 mm (0.2500 in) at up to 5 N,
no-go 6.70 mm (0.2638 in) at 2 N:

| Score | Condition | How it is read |
|---|---|---|
| 1.0 | the 6.350 mm go pin passes through both ears and the 6.70 mm no-go pin does not enter either more than 2 mm; the wrench is released at least 20 mm from the finger and has not fallen off the bench (on its stand, the bench or the vise); both grippers at least 20 mm from finger and wrench; finger, vise and wrench still (under 1 mm of motion) for 1 s | gauge pins; overhead video stepped frame by frame against the 63.5 mm jaw width |
| 0.75 | both ears pass the go pin, but the wrench is still held, within 20 mm of the finger, off the bench (dropped to the floor), or not yet still for 1 s | gauge pin; video |
| 0.5 | the go pin passes the upper ear and stops on the lower ear (19.75 mm in) | gauge pin and a steel rule |
| 0.25 | the go pin enters the upper ear at least 1.5 mm | gauge pin marked at 1.5 mm |
| 0 | anything less, or any rule above | |

Trials are run on lots A and B and averaged. Progress is counted from the as-printed bushing bores, which the go pin
does not enter (enrolment checks before each trial), so doing nothing scores 0. In a real trial "an ear" is the ear
with its bushing: the gauge pins are read through the bushings.

### How the simulator measures it (task.py)

- `observe()` runs after every control step (`parts.on_step`). It reads the vise, wrench and both hands (gripper and
  finger-tip collision hulls, faces sampled at 3 mm), and keeps a record for the whole trial; a scene reset clears it.
- Turning: the wrench's rotation about the bore axis, in the vise's frame. The wrench is one rigid body and the
  ratchet is a rule: only clockwise turns (from above) count; back-swings of the housing are pawl slips (in the video
  the whole wrench swings back; on the real tool the chuck nut and reamer stay still). A gripper on the chuck nut
  bypasses the ratchet, so a backward turn made while holding the nut counts as turning the reamer backwards.
- Axial force: the uncut, as-printed material is a 6.25 mm ring (`front`) on a 5,000 N/m slider at the cut front;
  the reamer's 45-degree lead seats in it, and the force is the spring's deflection minus the ring's own weight.
- Lateral guidance is physical: each ear has hidden ring colliders at its CAD bore (6.70 mm) under the original mesh,
  so the reamer is held within 0.16 mm of the axis once it is in an ear, as a real bore holds it.
- `score()` applies the zero rules first, then the tiers. It fails closed when hand or object states are missing.

## Declared cutting model (what is simulated and what is a stated rule)

PhysX cannot cut. The ears' CAD bores are colliders; the printed undersize and its removal are this stated rule:

- Cutting load. While the reamer's lead bears on uncut ABS and turns clockwise, a torque of 0.15 N m (scaled down
  below 1.5 N of bearing load and over the first 0.3 rad/s of turning) resists the turn, applied to the wrench about
  its axis, and the same torque acts on the vise as the reaction. Basis (estimate): about 0.03 N m of cutting (ABS
  at roughly 0.1 J/mm3 on a 0.06 mm x 1.0 mm/rev chip at a 3.2 mm radius) plus about 0.12 N m of margin rub as the
  printed ABS springs back. The robot's ratchet strokes have to turn the wrench against it; the back-swing does not
  (the pawl slips). The feed resistance is the stop itself: it gives way only as material is removed, so the reamer
  must be pressed with 1.5 N or more and the arm feels the push.
- An ear is cut where the reamer's tip stands on the uncut front, pressed with at least 1.5 N, and actually turns
  clockwise against that torque: the front moves down 1.0 mm per revolution (0.17 mm per flute; estimate for a sharp carbide reamer taking about
  0.06 mm per side, 6.25 to 6.375 mm, out of ABS). It never moves past the tip. Pushing without turning, turning without pressure,
  and turning backwards remove nothing.
- An ear counts as finished when the tip is 0.4 mm (the 45-degree chamfer) past its far face; the go pin then passes.
  When the upper ear is finished the stop drops to the lower ear's top face, 16.75 mm further down.
- An ear can only be started within 0.5 mm of its axis (0.16 mm CAD clearance plus the 0.4 mm lead). Once started,
  the ear's CAD bore holds the reamer, but the hold can still make it wander: the scorer records the reamer's axis at
  0.5 mm and 2 mm below the ear's mouth at every cutting step, and the reamed bore there is the 6.375 mm reamer plus
  the largest circle inside the area its axis has covered. When that reaches 6.70 mm at both depths, the no-go pin
  would enter: the ear is bell-mouthed. Levering the reamer more than 5 degrees while cutting also counts. Beyond
  4.5 mm from the axis the flutes are in the knuckle wall.
- Only the ears' bores collide; the rest of the knuckle has no colliders, so any wrench or gripper point inside its
  material scores 0.
- The lower ear is cut with the reamer running through the finished upper ear, which guides it as a bushing, so the
  two reamed bores are coaxial and the go pin passes both. Each ear's bell-mouth check uses the reamer positions
  recorded while that ear was being cut; the upper ear's 0.16 mm guide-ring play during the lower-ear pass is not
  counted as cutting (a real 6.375 mm reamed bore leaves the reamer no such play).
- Real forces in hand reaming of a 3 mm printed ear are a few newtons; 25 N averaged over 0.25 s is a stated estimate
  for splitting it. The stop is a 5,000 N/m spring, so the reamer tip sits past the cut line by the push divided by
  that stiffness: 0.6 mm at the demo's 3 N, 5 mm at the 25 N limit. Only the cut line counts for the gauge reading.
- The 6.70 mm no-go pin is the maker's design bore: the scorer zeroes a bore reamed larger than designed (the
  "bell-mouth"), not one that merely passes the go pin.
- Real trials ream a press-in ABS bushing in each ear (`fixture/README.md`). The simulator draws each ear and its
  bushing as one piece: same 3 mm length, faces and axis, same as-printed undersize (6.25 mm) and cutting load, so the
  cut, the guidance and the gauge readings are the same. A reamer more than 4.5 mm from the axis is cutting the
  bushing's outer rim (it is 9.40 mm across) or the carrier's socket wall, a visible gouge in both cases.

## Bypasses (each checked in `near_misses.py`, 51 checks)

| Shortcut | Result |
|---|---|
| Do nothing; lift the wrench and put it back | 0 |
| Push the reamer down hard without turning | nothing cut; over 25 N splits the ear: 0 |
| Turn without pressing, or above the ear | nothing cut: 0 |
| Turn the housing only backwards | nothing cut (ratchet): 0 |
| Grip the chuck nut and twist it back and forth in a started ear | reamer turned backwards: 0 |
| Start an ear 0.51 mm off axis; ream it while the axis circles 0.2 mm (swept bore 6.78 mm, the no-go enters); ream it levered 5.1 degrees | 0 for the trial, even after both ears pass (a steady 0.3 mm offset or a 0.12 mm circle is not a bell-mouth) |
| Push the wrench body through the knuckle | 0 |
| Drive the reamer into the knuckle 1.4 mm or more beside the bore | 0 (gouge) |
| Turn the reamer on the routing-pin seat or the scaffold face | 0 |
| Press the cast pad or flexure with the tool or a gripper | 0 |
| Leave the reamer in the bore | 0 |
| Finish both ears but keep holding the wrench | 0.75 |
| Lot A's motions in lot B | the gripper closes on empty air at lot A's stand position: 0 |
| Turn the wrench with no grip on it (it does not turn against 0.15 N m) or let the grip slip | nothing cut where it did not turn |

One hand is enough by design (the vise holds the finger, as at a real bench); the other arm stays clear.

## Why it matters

- O*NET 17-3024.01 Robotics Technicians, core task: "Align, fit, or assemble components, using hand tools, power
  tools, fixtures, templates, or microscopes."
- Yale OpenHand Finger Fabrication Guide, step 8 (0.2510 in reamer): "Joint clearances should be cleaned out with an
  oversized reamer." Step 7 cleans the press-fit seats with undersized reamers, which is why the routing-pin seat
  must not be touched by this one.
- Open-source hands (OpenHand, LEAP, ORCA) are printed and finished in labs by hand; robots that can do the
  finishing steps shorten that loop.

## How this differs from close tasks

- TK-NY6WTD (file a steel block to a scribed line): material removal to a mark; here the tool must follow an
  existing hidden axis through two separated ears, and a side load is the failure.
- TK-XTCCB5 (remove tree supports from a printed robot part): breaking off supports; here a rotating tool must be
  kept coaxial under light pressure, with a ratchet stroke.
- TK-84X8VV (telescoping gauge in a bearing housing): measures a bore; here the robot changes it and an assessor
  gauges it.
- TK-S242XN (tighten a bit in a keyed drill chuck): tool setup, not cutting.
- TK-VFWZ47 (fasten a bracket bolt with a ratchet in confined access): ratchet strokes against a thread with no
  alignment limit; here the strokes remove material and the score depends on staying on an existing axis.
- TK-T6GWHJ (feed a hex shaft through two bearings): coaxial insertion through two supports; here the tool must cut
  its way through two separated ears under a force limit, beside a seat it must not touch, and nothing is inserted
  that stays.

## Reset (by hand, about 5 minutes)

1. Unclamp the finger and stand it the right way up on a kitchen scale with its knuckle on the 4.5 mm seating block,
   the block's hole under the lower ear. Push the reamed upper bushing down into the spring pocket with an 8 mm flat
   punch and take it out of the pocket's open side; push the reamed lower bushing down into the block's hole with the
   punch reaching through the empty upper socket.
2. Fit two new bushings from the printed stock, chamfered end first: the lower one put into the pocket sideways onto
   the lower socket and pressed until it stops on the block's solid end (flush), then held at 50 N with the block's
   hole under it; the upper one pressed flush through an M6 washer, then held at 50 N on its own face. Draw the
   witness lines and run the four enrolment checks above (a failed bushing is replaced; a marked carrier is replaced
   by the spare).
3. Clamp the finger in the vise: back face on the parallel against the fixed jaw, knuckle 19.5 mm past the jaw end.
   Set the vise, standing on the load plate, with the bore at (0.290, -0.085) m: lot A knuckle toward the right arm,
   lot B the vise and plate turned 180 degrees about the bore (knuckle toward the left; plate centres in
   `fixture/README.md`). Tare the plate and start its log.
4. Hang the wrench by its chuck nut in the stand's slot, T-bar at 60 degrees to the slot, ratchet knob on
   clockwise; set the stand at the lot's position (A: 0.200, -0.200 m; B: 0.335, -0.235 m), slot opening toward
   the robot.
5. Arms at their ready pose.

Consumed per trial: two reamed bushings (0.21 g of ABS together, under 0.01 USD). The finger, its cast pad and flexure
are reused.

## Real-trial equipment and cost

Besides the robot, three cameras (overhead and two sides 90 degrees apart; phones will do), a laptop and an FDM
printer. A price without "estimate" is the list price on the named seller's page in November 2026; the rest are
estimates.

| Item | Source | Price (USD) |
|---|---|---|
| 2.5 in drill-press vise | LittleMachineShop.com SKU 4140 | 74.95 |
| 6 x 25 mm ground parallel | any toolmaker's parallel or ground flat stock | about 15 (estimate) |
| Ratcheting T-handle tap wrench | McMaster-Carr 2544A5 | about 50 (estimate; comparable 1/4-1/2 in ratcheting T-handles list at 47-69) |
| 0.2510 in carbide reamer, 3-1/2 in long | McMaster-Carr 8307A513 | about 90 (estimate; 6-flute carbide reamers near this size list at 64-160) |
| Class ZZ gauge pins: 6.350 mm (0.2500 in) go, 6.70 mm no-go, 6.20 mm enrolment | e.g. Vermont Gage via Zoro, 2.99 each, sold in packs of 8 | 72 (three packs) |
| Load plate | Phidgets KIT4007_0 (40) + DAQ1500_0 (35) + HUB0007_0 (26) | 101 |
| Shims and spacers raising arm bases, stand and bench items by the plate's height | plywood or printed | about 10 (estimate) |
| Kitchen scale, 10 kg | any | about 15 (estimate) |
| Tool stand | mild steel, drawing in `fixture/README.md` | about 15 (estimate) |
| Reset tools | 8 mm flat-ended pin punch, M6 flat washer, printed 4.5 mm seating block | about 10 (estimate) |
| Inspection | 10x loupe, machinist's square, 150 mm steel rule, paint pens | about 30 (estimate) |
| Two carrier fingers, made once | ABS print plus the maker's casting step (Smooth-On VytaFlex 30 and PMC-780 trial units, 0.9 kg each) and the finger's own 3.175 mm dowel pin | about 80 (estimate) |
| Bushings and printed bench items | ABS filament, 1 kg | about 20 (estimate) |
| **Total** | | **about 580**, then under 0.01 per trial |

## Fidelity

See `fidelity` in task.json. Licences: the only third-party asset shipped in this folder is the finger STL, from the
Yale OpenHand project (openhand-hardware, commit b10197d), Creative Commons Attribution-NonCommercial 3.0, used
unmodified with attribution in a non-commercial research benchmark (the real-trial carrier opens its two ear bores
to sockets, as `fixture/README.md` describes). The robot, table, lights and cameras come from
the program's yam-isaac-lab-arena embodiment and are not redistributed here. Everything else is built from boxes and cylinders in
`task.py`.

## Results

- Scripted demo (`yam_scripted`, `--epochs 2`, 2,400 steps, cutting torque on): 1.0 in lot A and 1.0 in lot B
  (mean 1.0).
- Lot B trace of this source: the reamer bore on uncut ABS against the 0.15 N m torque (mean 0.14 N m over 240
  sampled steps) at 1.6-3.6 N of feed; the upper ear was cut from 21 to 41 s and the lower ear from 51 to 71 s; the
  reamer's axis stayed within 0.20 mm of the bore and leaned at most 0.5 degrees while cutting; the bore it swept at
  the ears' mouths was 6.55-6.61 mm against the 6.70 mm no-go; score 1.0 from 105.2 s (plan ends at 110 s).
- Lot A has no trace of this exact source; its final demo scored 1.0. A lot A trace of the preceding source (same
  cutting moves, 17 instead of 21 lower-ear strokes) showed the reamer leaning up to 3.2 degrees at the start of the
  upper ear (the cutting torque leans the compliant wrist; the limit is 5 degrees), the axis within 0.21 mm, a swept
  bore of 6.50-6.54 mm, and the cutting reaction turning the free-standing vise by 0.08 degrees.
- Dream-1 (`--epochs 2`, same instruction, this source): 0.0 in lot A and 0.0 in lot B (mean 0.0); both trials ran to
  the 2,400-step limit with no refusal. It moved the right arm to the tool stand and tipped the wrench over in its
  slot; it never brought the reamer to the bore, so nothing was cut. Two earlier Dream-1 runs on earlier versions of
  this task (before the cutting torque) also scored 0.0 in both lots, at the step limit, with no refusal.
