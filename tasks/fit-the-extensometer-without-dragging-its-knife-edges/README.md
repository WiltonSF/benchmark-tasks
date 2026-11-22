# Fit the extensometer without dragging its knife edges

**Instruction:** Clip the extensometer from its case onto the specimen in the test machine, knife edges at the gauge marks without sliding them on the specimen, then let go.

**Category:** Other (materials-testing laboratory set-up).

## Why it matters

Every tensile test that reports modulus or yield strain (ASTM D638 for plastics, D3039 for composites, E8 for metals)
starts with someone clipping an extensometer onto the gripped specimen; it is the step after loading the specimen into
the grips (accepted task TK-K3R94E), and automated tensile testing and self-driving materials labs need a robot to
do it. Clip-on extensometers measure the stretch between two knife edges pressed on the specimen face by
wire clips. The knife edges are sharp on purpose: dragged across the face they leave a scratch, and on plastics a
scratch at the gauge section can start the break there and spoil the result. The Instron 2630 manual (M20-52630-1,
printed pages 29-31) describes the one-hand method this task follows: hold the extensometer with the gauge-length
latch set, hook the wire clips behind the specimen, keep the knife edges clear while it is brought in line, then let
the clips draw the knife edges gently onto the face, and its warning is short: "Do not slide the knife edge against
the specimen."

O*NET 17-2131.00 Materials Engineers, core task: "Conduct or supervise tests on raw materials or finished products to
ensure their quality."

What makes it hard for a robot: a 56 g instrument is carried by its housing over the specimen and lowered so its
clips straddle the upper edge while the knife edges stay clear of the face (about 1.5 mm in the demo); then both
10 mm knife edges must meet the face squarely, within 2 mm of the marks, without leaving a track longer than 0.5 mm
on it, and the hand must let go and leave without knocking it. Both knife edges must touch: an extensometer turned
1 degree about the vertical touches with one knife edge while the other is still 0.4 mm off.

## Parts and scene (true size)

| Item | Product | Size and mass | Status |
|---|---|---|---|
| Test stand | Mark-10 ESM303H horizontal motorized stand with controller (listed at US$4,050 on the maker's legacy page) | frame 750.3 x 341.1 mm, 168.4 mm high, 348 mm with controller; 28.3 kg | maker size; fixed |
| Grips | two Mark-10 G1061 wedge grips, jaw faces vertical | 115 mm between jaw mouths | grip body offsets are drawing estimates |
| Specimen | ASTM D638 Type I, natural (unpigmented) HDPE, already clamped | 165 x 19 x 3.2 mm, 57 x 13 mm narrow section, 76 mm shoulder radius | standard size; fixed in the grips |
| Gauge marks | two black ink lines on the specimen's upper edge | 0.6 mm wide, 25 mm apart, centred on the narrow section | drawn on |
| Extensometer | Instron 2630-105 clip-on extensometer (price on request) | 25 mm gauge length, catalogue 100 x 52 x 39 mm, 56 g; housing 44 x 52 x 39, arms 6 x 6 x 51, knife edges 10 long, clips 1.6 mm wire | catalogue size and mass; housing, arm and clip shapes estimated |
| Case | the extensometer's open carry case, grey moulded plastic | 235 x 165 mm base, 12 mm rims, lid standing open along its left long side; foam insert shown as the flat floor; 0.45 kg | size estimated from the manual's case drawing |
| Cable | the extensometer lead to the machine | 3 mm | drawn only (no collision, no pull) |

Sources: [Instron 2630-100 series manual](https://www.instron.com/wp-content/uploads/2024/07/2630-100-series-clip-on-extensometer-reference-manual.pdf)
(pages 19 case, 22-23 clips, 25-31 attaching); [Instron 2630 data sheet](https://www.instron.com/wp-content/uploads/2024/07/2630-series-strain-gauge-extensometers-1.pdf)
(2630-105: 25 mm, 100 x 39 x 52 mm, 56 g); [Mark-10 ESM303H](https://mark-10.com/products/legacy-products/esm-303h/)
and [manual](https://mark-10.com/downloads/product-downloads/manualESM303.pdf); [Type I dimensions](https://www.shimadzu.com/an/products/materials-testing/uni-ttm-system/tensile-test-methods-for-plastics-astm-d638/spec.html).
`fixture/dimensions.svg` is a 1:1 top view of layout A and a 4:1 section through the fitted extensometer;
`fixture/dimensions.json` lists every primitive (`python fixture/draw.py` regenerates both).

Real-trial equipment: the robot never operates the stand, so a real trial needs only a rigid holder that clamps the
specimen's ends 115 mm apart, 86 mm above the bench, broad faces vertical (two bench vises or toggle clamps on a
plate do this). No strain signal is read, so the instrument can be a dummy built to `fixture/dimensions.svg`: a 56 g
3D-printed housing 44 x 52 x 39 mm, two 6 mm steel arms 25 mm apart, knife edges ground from blade stock and 1.6 mm
music-wire clips bent to the drawing. Specimens are Type I blanks cut from 3.2 mm HDPE sheet. Forces stay under 2 N.

Approximate prices (checked Nov 21): two De-Sta-Co 215-U horizontal toggle clamps, about $13 each
([Zoro](https://www.zoro.com/de-sta-co-toggle-clamp-horiz-153-in-550-in-215-u/i/G1963227)), on two wooden blocks
screwed to a plywood plate (about $10); natural HDPE sheet 1/8 in (3.2 mm) x 12 x 8 in, about $11
([Zoro](https://www.zoro.com/usa-industrials-hdpe-plastic-sheet-12-l-x-8-w-x-18-thick-bulk-ps-pe-353/i/G7798610/)),
enough for about ten specimens; K&S 504 music wire, 0.062 in (1.6 mm) x 36 in, about $5; 6 mm square steel key
stock and a utility-knife blade for the arms and knife edges, about $10; PLA for the housing and a printed copy of
the case (235 x 165 mm base, 12 mm rims, lid hinged open), about $5; a dry-erase marker, about $2; and a 10x
measuring loupe with a linear scale, $138
([Edmund Optics 30868](https://www.edmundoptics.com/p/10x-peak-measuring-loupe-w-linear-scale/30868/)). About $200 in
all. For comparison, the ESM303H was listed at $4,050, each G1061 grip is about $690
([TestEquity](https://www.testequity.com/product/10034243-G1061)) and the 2630-105 is priced on request.

## Layout (fixed; two layouts)

Millimetres from the midpoint between the two arm bases, x away from the robot, y to the robot's left; heights above
the bench. The specimen's axis runs along y with its broad faces vertical; its centre is at x 330, y 20, 86 above the
bench, so its near face is at x 328.4. The extensometer lies flat (housing on the case floor, arms horizontal and
pointing +x, knife edges 25.5 mm above the bench).

| | Layout A (screenshot) | Layout B |
|---|---|---|
| Case centre | x 185, y 160 | x 150, y 120 |
| Extensometer housing centre | x 163, y 150 | x 128, y 110 |
| Machine and specimen | as above | the same |

Trial k uses layout k mod 2; run with `--epochs 2` (or `TASK_LOT=A|B`), `object_jitter=0`. The case and extensometer
move together, so the extensometer always lies in the same place in its case. Each trial checks its start: if the
extensometer or case starts more than 4 mm from its layout position, or the extensometer is turned more than 3
degrees, the trial scores 0 and the log says so (a setup fault; rerun it). In the runs below both layouts started
within 0.1 mm.

## Score 0 if (whole trial; putting it right afterwards does not undo it)

- a touching knife edge leaves a track longer than 0.5 mm on the specimen: a drag mark. Real trial: before the trial
  the near face of the narrow section is wiped with a thin dry-erase-marker witness coat (it does not bond to
  polyethylene, so a sliding knife edge wipes a clean track through it); after the trial read each knife edge's line
  with a 10x reticle loupe; a track counts as its length along the face beyond the knife edge's own seat line.
  Simulation: the extent of the knife edge's path along the face while it is within 0.3 mm of the specimen and the
  specimen reports any contact force (below);
- a knife edge dents the specimen (a seat line wider than 0.3 mm in the coat) or strikes the machine or grips
  (simulation: more than 8 N on the specimen or the machine);
- the grip crushes the housing (real trial: gripper force limit set to 40 N; simulation: more than 40 N on it);
- once the clips hold it, a hand shifts the extensometer on the specimen (a second seat line or a track in the coat;
  simulation: a net hand force above 2 N, about where the clips' hold would let the knife edges slip, an estimate);
- the extensometer falls before it is fitted (its lowest point under 4 mm above the bench, or tilted more than 45
  degrees).

## Fitted: when the clips hold it

The extensometer counts as fitted, and from then on stays where it is, when at one moment:

1. the extensometer is upright (within 45 degrees) and both wire clips are over the specimen's upper edge (their
   top wires above it) with their hooks behind its far face (below the edge, between the jaws);
2. every point along both knife edges touches the near face (real trial: a 0.3 mm feeler gauge cannot enter at
   either end of either knife edge; simulation: within 0.3 mm), inside its width and between the jaw mouths;
3. the extensometer presses the specimen with at least 0.1 N (real trial: the clips hold it when the hand lets go).

A knife edge that only reaches the face by sliding along it has already scored 0 under the drag rule.

## Ladder (the lowest tier shown at every control step of the last 1 s)

| Score | State | Real-trial reading |
|---|---|---|
| 1.0 | Fitted within the gauge tolerance (below), released, every hand surface at least 10 mm from the extensometer and no hand contact for the whole last second | top and wrist video for the hands; front photo with a ruler for the tolerance |
| 0.75 | Fitted within the gauge tolerance, but a hand still holds or touches it or is within 10 mm | same |
| 0.5 | Fitted, outside the gauge tolerance | front photo with a ruler against the ink marks |
| 0.25 | Not fitted, but both clips over the upper edge with the hooks behind the far face and the extensometer still (under 5 mm/s and 0.1 rad/s), held or hanging | side video |
| 0 | Anything else, including the extensometer left in its case or a trial under a Score 0 rule | |

Gauge tolerance: the midpoint between the two knife edges within 2 mm of the midpoint between the ink marks along the
specimen's axis; both knife-edge centres within 1.5 mm of the specimen's centre line across its 13 mm width; the
knife edges leaning no more than 3 degrees from square to the axis. Doing nothing scores 0.

## How the simulation scores it (task.py)

- After every 240 Hz physics substep, `physics_update()` reads the extensometer's pose and the PhysX contact forces on
  it from the specimen, the machine, the case and the six hand links (gripper body and both fingers of each arm).
- Drag record: three points along each knife edge. A track starts in the substep a knife edge is within 0.3 mm of
  the specimen and the specimen reports any contact force at all (no lower force limit); it lasts until the knife edge
  leaves the 0.3 mm tolerance, and its length is the extent of the positions the knife edge's centre took along the
  face (what a loupe sees). Over 0.5 mm scores 0. The whole specimen counts, shoulders included. Scrubbing +-0.3 mm
  leaves a 0.6 mm track (0); +-0.2 mm leaves 0.4 mm, as it would under the loupe.
- Fit: the three conditions above, checked every substep. From then on the simulator writes the fitted pose back
  after every physics substep, as the clip springs would hold it.
- After the fit, a net hand force over 2 N (any substep) scores 0, so the hand cannot nudge or prise the fitted
  extensometer for free. The demo's hand stays under 0.35 N after the fit.
- Every control step, `control_update()` records the tier; the hands-clear test samples both arms' gripper and finger
  hulls every 3 mm against every extensometer shape, and any hand force in any substep of that control step counts
  as touching. `score()` returns the lowest tier recorded over the last 20 control steps (1 s) and now, and 0 under
  any Score 0 rule. The 1.0 tier therefore needs a full second released and clear; a touch in any substep of that
  second resets it.
- `near_misses.py` drives these same functions substep by substep: the clean fit (both layouts), a 0.4 mm slide (1.0)
  and a 0.6 mm slide (0), scrubbing +-0.3 mm (0) and +-0.2 mm (1.0), lowering the knife edges down the face while
  touching (0), 3 mm off the marks (0.5), knife edges leaning 4 degrees (0.5), 1.0 mm and 1.8 mm off the centre line
  (1.0, not fitted), one knife edge touching (0.25 held, 0 when let go), upside down (0), 9 N on the specimen, 9 N on
  the machine, a 45 N grip, 2.5 N and 1.5 N pushes after the fit (0 and 1.0), released for only 0.5 s (0.75), still
  held (0.75), left in the case and dropped (0).

## Bypasses checked

| Shortcut | Result |
|---|---|
| Lower the knife edges straight down the face, or touch first and then slide them onto the marks | drag mark: 0 |
| Come straight in from the front | the rigid clips hit the specimen's near face; no fit |
| Fit it upside down, clips under the lower edge | not fitted: 0 |
| Press hard so the arm's sag does not matter | over 8 N: 0 |
| Touch with one knife edge and let go | not fitted; it falls onto its clips: 0 |
| Keep holding it in place on the face | at most 0.75 |
| Fit it anywhere on the specimen | outside the gauge tolerance: 0.5 |
| Fit it, then nudge it along with the hand | over 2 N: 0; under 2 N it stays where the clips took it, so the tier is fixed at the fit |
| Lay it on the grips or wedge it between them | not fitted: 0 |

One-hand check: the maker's manual gives a one-hand method, so one hand is enough and allowed; the demo uses the left
arm only. A second hand may steady the extensometer; the model's clips are rigid, so they are not opened by hand.

## Fidelity and what is simplified

- The extensometer is one rigid 56 g body. Its real arms pivot +-2.5 mm, but during fitting the gauge-length cone
  latch holds them at 25 mm; releasing that latch after fitting is not part of this task.
- The wire clips are rigid, in their pulled-open shape (6.2 mm between knife edge and hook for a 3.2 mm specimen),
  and collide with everything. In the manual's method the hand eases off and the clip spring draws the knife edges
  onto the face; here the hand eases them on at a similar slow speed (about 2 mm/s), and the clip spring's hold is the
  declared fit rule: once fitted, the extensometer stays where it was fitted.
- Limits that are estimates: 0.5 mm track (readable with a 10x reticle loupe in the witness coat), 8 N dent or
  blunting, 2 N slip of a fitted extensometer, 40 N housing crush. Friction 0.5/0.4 (extensometer) and 0.35/0.3 (HDPE).
- Uniform density puts the centre of mass 2.6 mm ahead of the housing centre; the demo grips over it.
- The grips hold the specimen rigidly: no slip, no bending (56 g on the 115 mm free span would bend it about 0.6 mm),
  no fracture. The cable is drawn but has no collision or pull. No strain signal is simulated.

## Notes on timing, damage and reset

- Release and hold timing: hand contact and hand forces are read every physics substep, the tier every control step,
  and the score is the lowest tier over the full final second, so a touch, push or oscillation anywhere in that second
  stops the 1.0.
- Damage measure: the drag track is recorded every substep over the whole specimen with no lower force limit and
  is permanent for the trial.
- Reset: the extensometer is a single rigid body; it and its case are placed together, and each trial checks its
  start pose (4 mm, 3 degrees). Both layouts started within 0.1 mm in every run.
- Clip clearance: the clips are drawn and collided as 1.6 mm wire with a stated 6.2 mm opening; nothing deflects.

## How this differs from close tasks

- TK-K3R94E (accepted: load the dogbone into the wedge grips): ends where this task starts. Here the gripped
  specimen gets the strain sensor.
- TK-SZTAYG (pulse oximeter onto a finger) and TK-TYWZHY (candy thermometer onto a pan rim): spring clips pushed onto
  a part. Here the contact itself is the risk: the knife edges must land square, within 2 mm of marks, without
  sliding 0.5 mm.
- TK-KU9N36 (binder clip onto paper): opened by its handles and slid over an edge. Here the clips go over the edge
  while the measuring edges are kept off the face, the reverse of sliding on.
- TK-DKQNQY (circlip with pliers) and TK-3QCBW3 (clamp meter jaw round a lead): a sprung tool closed round a part. No
  surface there can be marked by the approach.

## Run

```bash
I='Clip the extensometer from its case onto the specimen in the test machine, knife edges at the gauge marks without sliding them on the specimen, then let go.'
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 --instruction "$I" --scorer task_score --max-steps 1200 --epochs 2 --store-frames --log-dir demo-log
inspect-robots run --embodiment yam_arena --policy dream -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 --instruction "$I" --scorer task_score --max-steps 1200 --epochs 2 --store-frames --log-dir dream-log
python near_misses.py
```

Run limit 1,200 steps (60 s at 20 Hz) per layout. The demo is open loop from the visible layout: it reads no state,
force or score. It grips the housing over its centre of mass, lifts it 50 mm above the specimen's centre line,
lowers the clips over the upper edge with the knife edges 1.5 mm off the face, eases in 4.5 mm (commanded 3 mm past
touch; the arm gives about 0.2 N per mm, so the knife edges press at most 1.1-1.2 N), opens the fingers and lifts the hand
away. Its two corrections were measured in traces: the simulated arm, loaded, turns the extensometer about 0.85
degrees about the vertical and drops about 1 mm as the push starts, so the demo turns the grip -0.85 degrees and aims
1.8 mm higher and 0.8 mm along the axis.

## Reset

Leave the specimen clamped unless it is marked. Read both knife-edge lines in the witness coat with a 10x reticle
loupe; replace a marked specimen at the same 115 mm jaw spacing, centred, and redraw the two ink marks 25 mm apart;
wipe the old coat off and re-coat the near face of the narrow section. Unclip the extensometer,
check neither clip is bent, set the cone latch, and lay it in the case; put the case and extensometer at the
layout's positions.

## Results

Traces (both layouts each): (1) no turn correction: one knife edge touched while the other was still 0.37-0.44 mm
off (outside the 0.3 mm contact tolerance), so the clips did not take it; the hand let go and the extensometer fell
onto its clips: 0 and 0. (2) turn and height corrected: 1.0 and 1.0
(knife-edge centres 1.18 and 1.25 mm off the centre line). (3) height corrected further: 1.0 and 1.0 (0.67 and
0.90 mm off the centre line, 0.96 and 1.48 mm along the axis; specimen force peak 1.24 and 1.08 N; hand force after
the fit at most 0.35 N; drag 0).

Final demo (both layouts, final code and instruction): 1.0 and 1.0. Fitted with
the knife-edge midpoint 0.41 and 0.78 mm from the marks' midpoint along the axis, knife-edge centres 0.60 and 0.80 mm
from the centre line, knife edges leaning 0.25 and 0.24 degrees; full score at 18 s (361 control steps) in each.

Dream-1 (same code and instruction): 0.0 and 0.0. Both layouts ran to the 1,200-step limit without refusing; the
left hand moved over the case but never picked up the extensometer, and no Score 0 rule fired.
