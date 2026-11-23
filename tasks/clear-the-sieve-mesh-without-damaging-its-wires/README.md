# Recover wedged grains from a Gilson V3SF No. 16 test sieve without damaging its mesh

Materials-testing laboratory technicians recover grains lodged in test-sieve openings before weighing the
retained fraction. This affects the measurement, as well as the next use of the sieve: lost particles change
the reported grading, and damaged openings change which particles pass. The task isolates that recovery step
on a catalogue-size Gilson V3SF No. 16 sieve, with its pan and TSA-170 brush.

[Indiana DOT's AASHTO T 27 procedure, Fine Aggregates, p. 4](https://www.in.gov/indot/div/mt/aashto/testmethods/aashto_t27.pdf)
requires trapped material to be removed and included in the retained weight, using gentle brushing. The procedure
serves aggregate design, production control and acceptance testing. Related sieve measurements cover
[soil gradation (ASTM D6913/D6913M)](https://store.astm.org/d6913_d6913m-17r25.html) and
[pharmaceutical powders (historical USP <786> harmonization text)](https://www.usp.org/sites/default/files/usp/document/harmonization/gen-chapter/g01_pf_30_6_2004.pdf).
For sector scale, [AASHTO re:source](https://aashtoresource.org/specifiers) reports roughly 2,000 accredited
construction-materials laboratories (its page says nearly 2,000 in the text and 2,100+ in the banner).
That is a laboratory-sector count, not a count of users of this sieve, this operation or this benchmark.
The cited DOT procedure commonly uses 8- or 12-inch sieves. This task isolates its recovery step using a commercial
3-inch sieve; the 20-grain exercise is not a complete T27 grading test and does not measure laboratory throughput.

**Instruction:** Clear the grains stuck in the sieve's mesh into its pan without damaging the wires, then set the sieve
and brush down on the counter.

**Category:** Other. **Run limit:** 2,400 steps (120 s at 20 Hz) per trial; two trials (lots A and B), averaged.

## Procedure and manipulation

**Category: Other.** This is a laboratory measurement procedure shared across wire-cloth instruments. The
sources establish the testing context and the recovery step, not a 500,000-worker occupation or a measured
automation benefit. Preserving the instrument does not demonstrate a risk of harm to a person.

Gilson's [sieve-care guide](https://www.globalgilson.com/blog/how-to-clean-a-sieve) says, "After each use, sieves
should be carefully brushed to recover all the sample material" and prescribes light underside brushing and
backlit inspection. It also cautions against forcing embedded particles out: use only replicas that pass the
gentle-release enrolment below. The [TSA-170](https://www.globalgilson.com/fine-sieve-cleaning-brush) is intended
for No. 16 and finer sieves. Instrument cleaning is a core duty of
[Chemical Technicians, O*NET 19-4031.00](https://www.onetonline.org/link/details/19-4031.00); that listing does not
establish how many technicians perform this particular step.

One hand holds the free sieve over the pan while the other slides the brush across its underside. This couples
three physical requirements: maintain bristle contact sufficient to release wedged particles, keep hard tool parts
and fingers away from the wire cloth, and keep the falling particles inside the receiver. The 31.75 mm brush head
must cover sites spread across 36.2 x 32.58 mm, within the sieve's raised frame. The support hand must counter
brushing forces without blocking access to those sites. The demo holds the sieve's rim against the pan's rim;
the sieve is never clamped or fixed to the pan.

## How this differs from the nearest tasks

The physical ability is loaded underside release of lodged particles while positioning a free mesh over a
receiver, with collection and preserved openings both required. A change of object alone would not distinguish it.

- **BEHAVIOR-1K [clean_a_sieve](https://behavior.stanford.edu/knowledgebase/tasks/clean_a_sieve-0.html):**
  starts with an adhesive-covered kitchen sieve and provides soap, a sponge and running water; its goal is
  removal of the coating. Here success also requires recovery of the freed particles into a pan and preserved
  measuring apertures. The underlying sieve-cleaning activity overlaps; those added requirements do not by
  themselves establish a distinct core skill, so duplication remains a review risk.
- **Denture brushing, TK-EFZTFV, accepted ([PR 2998](https://github.com/robocurve/benchmark-tasks/pull/2998)):**
  shares hold-and-brush coordination. Its outcome is loaded coverage across a tooth arch; this outcome is
  unseating and recovering individual trapped particles through a mesh. Surface coverage earns no credit here.
- **Griddle scraping, TK-63RTRA, accepted ([PR 2019](https://github.com/robocurve/benchmark-tasks/pull/2019)),
  and dustpan sweeping, TK-QVV8CJ, final for overlap with dexhand: Sweep Dustpan
  ([PR 2633](https://github.com/robocurve/benchmark-tasks/pull/2633)):** move residue or loose debris along solid
  surfaces; the griddle scraper shears adhered residue from a fixed plate. Here a compliant brush works on a held
  porous instrument, and grains must be released from its opposite face into
  a separate pan. **Print-support removal, TK-XTCCB5, accepted
  ([PR 2321](https://github.com/robocurve/benchmark-tasks/pull/2321)),** shares held-part recovery over a bin,
  but breaks supports off a print rather than preserving the original openings while releasing lodged particles.
- **Hand-sieve separation, TK-52U378 ([PR 2427](https://github.com/robocurve/benchmark-tasks/pull/2427)), and
  soil sifting, TK-KPLDY8 ([PR 684](https://github.com/robocurve/benchmark-tasks/pull/684)), both final:** agitate
  loose material to separate size fractions. This task starts after separation, with 20 already lodged grains.
  The final pan photo supplies the count; no scorer must reconstruct hidden aperture crossings from moving video.
  Those verdicts also show that a sieve alone does not establish importance; the laboratory use above is the claim.
- **Earbud cleaning, TK-UFPU8X, final ([PR 2954](https://github.com/robocurve/benchmark-tasks/pull/2954)):**
  shares mesh cleaning, but removes surface marks and replaces a tip. This task recovers a retained specimen and
  preserves a measuring instrument. It is not offered as an everyday household activity.

## Scene (lot A; lot B moves the sieve and the pan)

Millimetres from the midpoint between the two arm bases (x forward, y to the robot's left), heights above the
counter.

| Item | Product | Size used | Where (lot A) |
|---|---|---|---|
| Test sieve | [Gilson V3SF #16](https://www.globalgilson.com/3-inch-sieve-all-stainless-full-height-number-16), stainless, $58.50 | 76 across, 44.45 tall, 1.18 openings; 0.63 wire at 1.81 pitch (ASTM E11 typical); wall 1.2, wire plane 7.2 above the frame's bottom edge and 90 g are estimates | upside down, centre (260, 130); wires 37.2 up |
| Pan | [Gilson V3SFXPN](https://www.globalgilson.com/3-inch-sieve-pan-stainless-full-height), stainless | 76 across; 44.45 tall, 1 mm floor and 100 g are estimates | upright, centre (250, -30) |
| Brush | [Gilson TSA-170 fine sieve brush](https://www.globalgilson.com/fine-sieve-cleaning-brush), $20.50 | 146.1 long, 31.75 natural-bristle head (26.1 long); wood handle 16 x 100 and nickeled steel ferrule 22 x 20 are estimates; 40 g estimate, 8 g of it bristles | standing on its bristles, (300, -190) |
| Grains | dry limestone, near-size fraction | 20 grains, 1.26 to 1.36 across (0.08 to 0.18 wider than the openings), about 3.5 mg each (estimate) | one wedged in each of 20 numbered openings ([map](fixture/native-layout.svg)) |
| Shaker | [Gilson SS-3 3-inch vibratory sieve shaker](https://www.globalgilson.com/gilson-performer-iii-3-inch-sieve-shaker), switched off | 203 x 279 x 559 | fixed, centre (570, 240) |
| Counter | lab counter with back wall and shelf | 1100 x 1300 top; wall at x 770 to 790, shelf 580 to 600 up | fixed |

Lot B: sieve at (220, 170) and pan at (270, -50); the brush, shaker and counter stay. No custom parts; the
[fixture notes](fixture/README.md) list the estimates to check with a caliper and scale.

## Proposed real-trial kit and reset

The simulation and its recorded results use the original limestone-grain model. For repeatable physical trials,
use manufactured glass spheres as replica grains, with the same 1.26-1.36 mm size band and the enrolment checks
below. [Kremer 59835](https://shop.apfitzpatrick.co.uk/kremer-59835-glass-beads-150g-2390-p.asp) is commercial coarse
soda-lime glass sphere stock, nominally 1.25-1.55 mm; its 150 g retail pack is listed at GBP6.90
including UK VAT. Select the narrower size band once, then reuse the same 20 beads. The replica changes the
specimen supply, not the requirement to brush a particle lodged in real wire cloth into the pan.

| Kit item | Price or budget allowance |
|---|---|
| Gilson V3SF No. 16 sieve | US$58.50, catalogue price |
| Gilson V3SFXPN pan | US$45.80, catalogue price |
| Gilson TSA-170 brush | US$20.50, catalogue price |
| [Gilson V3SF No. 14 sieve](https://www.globalgilson.com/3-inch-sieve-all-stainless-full-height-number-14), for initial grading | US$58.50, catalogue price |
| [Plugable USB2-MICRO250X-DEAL microscope](https://plugable.com/products/usb2-micro250x-deal), for calibrated close-ups | US$50.99, listed price |
| LED backlight and caliper | US$40 combined allowance |
| 0-5 N force gauge, for specimen enrolment | US$40 allowance |
| Ruler, microscope scale, catch tray, soft-tipped tweezers and soft wooden probe | US$10 combined allowance |
| Cardboard or plywood shaker replica, 203 x 279 x 559 mm | US$15 allowance |
| Matching sacrificial replacement gripper pads; lidded bead vial and rubber tap block | US$20 combined allowance |
| Kremer 59835, 150 g | GBP6.90, listed pack price |

Budget about **US$365 plus GBP6.90**, excluding robot, cameras, computer, delivery and local tax. Catalogue and
retail listings checked November 23; allowances are estimates. The switched-off shaker supplies only an obstacle
outline, so its full-size box replica needs no motor or operating controls.

**Protect the shared rig:** before physical trials, remove and store its original gripper pads; fit sacrificial
spare pads of the same dimensions, material and mounting pattern. Check fastening and dry grip on the empty
sieve's smooth rim and the wooden brush handle. Inspect for burrs, exposed wire ends and loose pads; reject
damaged parts and replace worn spares. Do not put covers, adhesive or guards on the wire cloth or bristles;
contact and brushing resistance must remain. A wire-contact score penalty alone does not protect pads. This
replacement-pad protocol has not been tested on hardware and is not represented in the recorded simulation.

**Initial specimen enrolment:** allow **8-12 minutes for one person per 20-bead set**, including grading, diameter checks,
seat/release tests and reseating; this is an unmeasured planning estimate, not an observed preparation time.
Record supplier, product and lot; grade through No. 14 and retain on No. 16, then measure candidate diameters
with a calibrated microscope and keep 1.26-1.36 mm. Use intact spheres only. Seat each from the top face with
a soft wooden probe at one of the 20 mapped openings, without adhesive, until retained and visible from below.
Accept a set only if all 20 stay seated when inverted and after one gentle frame tap from 10 mm onto rubber,
and release under sliding TSA-170 contact below 3 N, measured with the force gauge, without changing the wires.
Record release loads and before/after mesh images. Reseat and repeat retention before the trial. This checks
lodged-particle release; it does not calibrate the simulated friction or establish 3 N as a safe wire limit.
If spheres cannot pass both checks, the proposed physical specimen is not qualified; do not glue them in or
change the score threshold. Successful physical wedging remains unverified.

**Between trials:** recover and count the same 20 beads into a lidded vial; inspect and photograph the backlit
mesh; reseat at the numbered sites with the soft probe; repeat retention and photograph the occupied map.
Empty the pan and restore the lot coordinates with a ruler, brush upright on its bristles. Replace a chipped
or lost bead only after the full enrolment checks; reject a damaged sieve. Budget **5-8 minutes per routine
reset by one person**, including count, full-mesh inspection, seating, retention check and placement. This remains a planning
estimate; log elapsed preparation and reset time, rejected candidates and replacements in the physical pilot.
No curing, printing or fresh natural specimen is required per trial. Microscope calibration and spare-pad fitting
are kit commissioning, not specimen preparation; neither has a measured time. The kit and time estimates are
proposed; the reported scores and videos remain from the original limestone simulation.

**Proposed tolerances and pre-trial rejection:** reuse the same numbered set rather than draw a new batch.
Keep a per-site diameter record; any replacement must be 1.26-1.36 mm and within 0.02 mm of that site's original
bead, from the same product, then pass the complete retention/release enrolment. Use the same sieve and mapped
openings; set lot A or B centres within 2 mm of the listed coordinates and the brush within 2 degrees of vertical.
These are proposed physical setup tolerances, not a demonstrated robustness range; simulated starts use exact poses.
Before each trial, reject the set if any bead is missing, chipped, on the wrong site or fails inversion/the 10 mm
frame tap; reject the sieve for a broken/loose wire or >=0.05 mm opening growth. After qualifying release, reseat
and repeat retention before starting. Photograph all 20 occupied sites and the empty pan. Never start a failed
specimen and count it as policy failure. Record replacements and elapsed reset time, including rejected attempts;
the minutes estimate is not yet demonstrated.

For before/after wire inspection, calibrate the microscope against a scale at the wire plane, capture overlapping
backlit tiles covering the entire cloth at 0.01 mm/pixel or finer, and compare matching openings. At 1600 pixels
across, each tile must cover at most 16 mm. Keep exposure and scale fixed; do not infer a 0.05 mm opening change
from an overview photograph. Photograph the pan separately to count the 20 possible recovered beads.

## Scoring

Read from the overhead and wrist videos, backlit macro photos of the whole mesh before and after (0.01 mm per pixel
or finer, with a scale in view), a photo of the pan from above, and a ruler.

**Score 0 for the whole trial if, at any moment:**

- a wire breaks or comes loose, or an opening widens by 0.05 mm or more, on the backlit macro photos (macro lens or
  USB microscope);
- the wooden handle, the steel ferrule or a robot finger touches the wire cloth (wrist video);
- the sieve, pan or brush falls off the counter (below its top or past its edge; lifting them is fine).

Simulation wire damage is represented by an uncalibrated proxy rule: more than 3 N on the bristle
head while it is on the wires, or the handle, ferrule or a finger at the wire plane.

**Score 0 at the end if** any freed grain is anywhere but lying in the pan: on the counter, on the brush, hidden
under something, or wedged back in the mesh. A lost grain spoils the weighed sample; a grain still wedged stays with
the sieve and can be cleared later, so it only lowers the count.

**Otherwise count the 20 original grains lying in the pan, below its rim and visible from above:**

| Score | Grains in the pan |
|---|---|
| 1.0 | all 20 |
| 0.75 | 18 or 19 |
| 0.5 | 15 to 17 |
| 0.25 | 10 to 14 |
| 0 | fewer than 10 |

Every score above 0 also needs, for the last 1 s of the trial: the sieve resting on its rim on the counter (either
face up, tilted 10 degrees or less, not on the pan), the pan and brush on the counter, nothing held, both hands at
least 30 mm from all three, and everything still (parts under 3 mm/s and 0.03 rad/s, grains under 3 mm/s).

In the simulation, `task.py` applies these rules: a grain counts when it lies inside the pan below its rim and has a
clear vertical line of sight up past every part (and, through the stillness gate, moves slower than 3 mm/s); part
speeds come from the change in pose between control steps; the hands are checked with point clouds sampled over the
finger and gripper surfaces. Final grain counts use the pan close-up; overview video establishes handling and final placement.

## How the simulation models the grains and the mesh

Sieve frame, wires, pan and brush are PhysX bodies at true size. The wires are straight 0.63 mm cylinders at 1.81 mm
pitch (the real cloth is woven). The brush's bristle head is one rigid 31.75 mm disc on an 8 mm spring slide
(500 N/m, an estimate), so the force on the wires is 500 N/m times the measured compression.

Grains use a separate local contact model:

- Under brushing, a grain moves while the pressed bristle face is over it and pushing from the open (underside) face, with
  its share of the load (by area) above its wire grip of 0.45 to 0.69 mN (an estimate). It then moves with the
  bristles and comes free once it has moved 1.22 mm, half an opening plus the wire's radius, out of its seat.
- A freed grain falls under gravity and bounces off the wires, the frame wall, the pan, the counter, the bristles
  and the other grains, every 1 ms. Nothing is moved into the pan by rule.
- Frame acceleration can also overcome retention; the lowest estimated threshold is about 13 g.
  Tapping is permitted and scores by its resulting collection and damage state.
- Damage is a stated rule too: over 3 N of bristle load on any wire, or the handle, ferrule or a finger at the wire
  plane. Once marked, it stays for the trial and the frame turns red.

The local grain solver reads sieve and brush motion and the spring load, but does not feed grain-contact
reaction forces back into the PhysX sieve or brush. The bristle spring still supplies the simulated brush/wire
contact load; grain retention is a separate, one-way coupled approximation. Hard-tool and finger damage checks
sample geometry at 20 Hz, so they do not certify continuous contact history between samples. The wires remain
rigid; damage is a persistent flag, not simulated wire deformation or fracture.

Retention forces, bristle stiffness and the 3 N damage threshold are model estimates. No hardware trial or
force calibration has been performed; the glass-replica kit is a proposed physical protocol. All reported
results below are from the unchanged simulation.

## Shortcuts and what they score

| Shortcut | Result |
|---|---|
| Brush the sieve where it lies on the counter | freed grains land on the counter inside the frame: 0 |
| Brush the grain-side (top) face | pushing from that side cannot unseat a grain: 0 |
| Press hard, or jab with the handle or ferrule | damage: 0 for the trial |
| Gentle shaking below the model's retention threshold | grains remain wedged: 0; higher acceleration is scored by actual collection and damage |
| Leave the sieve on the pan, or the brush over the pan | sieve not on the counter, grains not visible: 0 |
| Hold the sieve or brush at the end | not released: 0 |
| Tip the pan or knock grains out | lost grains: 0 |

Scoring uses the resulting collection, damage history and final release. The demonstrated method coordinates
both hands; the scorer does not impose an arm-count rule.

## Demo and results

The scripted demo runs open loop from the fixed lot positions. The left hand leans 65 degrees, closes across the
upside-down sieve's 76 mm diameter, lifts it and sets its rim on the pan's rim, and keeps holding it there. The right
hand picks the brush up by its handle, holds it straight down and presses the bristles flat on the mesh (commanded
4.5 mm past first touch), then makes four 23 to 28 mm strokes across the openings, lifting between strokes. The
grains drop into the pan. The brush goes back on its bristles where it stood, the sieve goes back upside down where
it lay, and both hands move clear.

| Trial | Score | What happened |
|---|---|---|
| Demo, lot A | 1.0 | first grain free at 19.9 s, all 20 in the pan at 34.9 s; peak bristle load 1.59 N; full score held from 63.2 s (1,265 steps) |
| Demo, lot B | 1.0 | first grain free at 20.2 s, all 20 in the pan at 37.1 s; peak bristle load 1.66 N; full score held from 63.2 s (1,265 steps) |
| Dream-1, lots A and B | 0.0, 0.0 | Same source and instruction. Both trials ran to the 2,400-step limit without refusing. The arms drew back toward their bases and hovered; no grain was freed and nothing was damaged ([lot A frames](evidence/dream-A-sheet.png)). |

[Per-lot results](evidence/demo-results.json), [lot A frames](evidence/demo-A-sheet.png),
[lot B frames](evidence/demo-B-sheet.png).

## Run it

Run from the task folder with the stock yam_arena embodiment of Isaac Lab-Arena and Inspect Robots:

```bash
I="Clear the grains stuck in the sieve's mesh into its pan without damaging the wires, then set the sieve and brush down on the counter."
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=360 --instruction "$I" --scorer task_score --max-steps 2400 --epochs 2 --store-frames --log-dir demo
inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$I" --scorer task_score --max-steps 2400 --epochs 2 --store-frames --log-dir dream
python near_misses.py
```

Trial 0 is lot A, trial 1 is lot B; no random seeds (object jitter is 0). `tilt.py` lets the scripted demo lean
the support hand; it changes nothing else. The demo is open loop from the fixed lot positions.

Code MIT; text and drawings CC BY 4.0. All geometry is generated from primitives in `task.py`.
