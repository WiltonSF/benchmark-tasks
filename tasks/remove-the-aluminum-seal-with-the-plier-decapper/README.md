# Plier-decap a serum bottle: squeeze and twist the crimp seal off, then ease the grip so the seal drops into the beaker

**Instruction:** Remove the aluminum crimp seal from the serum bottle with the plier decapper, drop the seal in the
waste beaker and put the decapper down on the counter. Touch the seal only with the decapper's jaws, and move both
hands away at the end.

**Category:** Other (laboratory sample handling).

## Why it matters

Chromatography and headspace-GC laboratories, pharmaceutical QC and anaerobic-culture labs close samples in glass
vials and serum bottles under 20 mm aluminum crimp seals. To re-sample, empty or recycle the glass, the seal is
removed with a hand decapper. WHEATON's vial guide (printed page 21) gives the whole method for its 224373 plier
decapper in one line: "secure the seal between the jaws, squeeze the handles and twist the seal". A person usually
steadies the glass bottle with the other hand, squeezes the tool hard enough that its jaws do not slip, turns it,
and then lets the torn metal go over the waste without dropping the tool.

O*NET 19-4031.00 Chemical Technicians, core task: "Set up and conduct chemical experiments, tests, and analyses,
using techniques such as chromatography, spectroscopy, physical or chemical separation techniques, or microscopy."

What makes it hard for a robot: a two-lever tool on a snug rivet must be carried with its jaws open, closed on a
7.6 mm-tall seal (not on the glass below it), squeezed with at least 10 N at each jaw while it is turned (a lighter
grip slips and the seal does not turn), then eased, not released, so the seal falls into the beaker while the
plier stays in the hand.

## Parts and scene (true size)

| Item | Product | Size and mass | Status |
|---|---|---|---|
| Serum bottle | WHEATON 223747, 100 mL clear glass, 20 mm crimp finish | 52 mm diameter, 95 mm tall with seal; 110 g | maker size; mass an estimate |
| Stopper | WHEATON W224100-405 grey butyl, 20 mm | flange 19 mm, under the seal | maker part; size estimated |
| Seal | WHEATON 224178-01, 20 mm open-top aluminum seal | 21.5 mm outside, 7.6 mm tall, 13 mm centre hole, about 1 g | maker part; sizes and mass estimates |
| Plier decapper | WHEATON 224373 (steel, grey plastic-coated handles), listed at US$158.67 when the idea was researched | 157 mm long, 160 g; jaws curved to the seal; jaw tips 32 mm apart at the start | maker part; length, mass and jaw shape estimates |
| Waste beaker | PYREX 1000-250, 250 mL low form | 68 mm outside x 90 mm; 110 g | maker size (Corning catalogue); mass an estimate |
| Counter and shelf | lab preparation counter (the station table), white reagent shelf at the back | shelf 800 mm wide, 420 mm tall, front at x 495 | fixed |
| Per trial | one new 224178-01 seal (sold in packs; price not checked); a 20 mm hand crimper (WHEATON W225303 E-Z Crimper) to seal the bottles | | consumed |

Sources: [224373 product page](https://www.dwk.com/na/wheaton-plier-decapper-removes-20-mm-seals-224373);
[WHEATON Vial Guide](https://d3h4jppqn0j59k.cloudfront.net/pub/media/mageplaza/resource/pdf/WHEATON-Vial-Guide-Brochure.pdf)
(page 21: plier decapper; serum bottles and seals); [serum bottle flyer](https://d3h4jppqn0j59k.cloudfront.net/pub/media/mageplaza/resource/pdf/w/h/wheaton_crimp_serum_bottles_vials_flyer_en.pdf);
[DWK sealing components, page 11](https://d3h4jppqn0j59k.cloudfront.net/pub/media/mageplaza/resource/pdf/d/w/dwk_sealing_components_for_vials.pdf#page=11)
(224178-01 open top); [Corning 1000-250](https://ecatalog.corning.com/life-sciences/b2c/US/en/General-Labware/Beakers/Beakers%2C-Glass/PYREX%C2%AE-Beakers%2C-Low-Form%2C-Double-Scale%2C-Graduated/p/1000-250).
`fixture/dimensions.svg` gives the modelled plier, seal and bottle finish with every estimated dimension marked.

## Layout (fixed; two lots)

Millimetres from the midpoint between the two arm bases, x forward, y to the robot's left; heights above the counter.

| | Lot A (screenshot) | Lot B |
|---|---|---|
| Bottle (base centre) | x 290, y 50 | x 270, y 80 |
| Plier rivet (lying flat, jaws toward +y, open 0.55 rad) | x 250, y -120 | the same |
| Waste beaker (base centre) | x 370, y -45 | x 365, y -15 |

The reagent shelf stands at the back (front face x 495, 800 wide, 420 tall, one shelf at 260). Trial k uses lot k
mod 2; run with `--epochs 2` (or `TASK_LOT=A|B` for one lot). `object_jitter=0`. Each trial checks the start pose:
if the bottle, plier rivet or beaker starts more than 4 mm from its lot position, or the plier more than 3 degrees
turned, the simulator scores that trial 0 and logs it as a setup fault; rerun it.

## Score 0 if (whole trial; putting things right afterwards does not undo it)

- a robot finger touches the aluminum seal (only the plier jaws may; read from the side camera level with the seal;
  simulation: more than 0.5 N of finger contact on the seal at any control step);
- the plier chips or cracks the glass (10x loupe after the trial; simulation: more than 20 N of plier-on-glass
  contact at any step);
- the bottle or the beaker falls over (tilts more than 45 degrees at any step).

## Ladder (end state; every level needs the bottle standing and everything still for the last 1 s)

"Standing": tilted 10 degrees or less with its base within 3 mm of the counter. "Still": the bottle, seal, beaker
and plier all under 5 mm/s and 0.1 rad/s for the last 1 s (overhead video stepped frame by frame).

| Score | End state | Real-trial reading |
|---|---|---|
| 1.0 | The seal lies inside the waste beaker, below its rim; the plier lies on the counter (lowest point within 4 mm of it, tilted 15 degrees or less); both hands at least 30 mm from the bottle, seal, plier and beaker for the last 1 s | overhead and side photos of the beaker; video for the hands; ruler |
| 0.75 | The seal in the beaker; the plier still held, or not lying on the counter, or a hand within 30 mm | same |
| 0.5 | The seal off the bottle (no part of it within 10 mm of the glass or stopper; simulation: an 11.5 mm sphere round the seal's centre kept 10 mm clear, so straight up it must be lifted about 18 mm) but not in the beaker | photo of the bottle neck and the seal against a 1 mm rule |
| 0.25 | The crimp torn (the seal loose: it turns on the neck at a touch) but still on or within 10 mm of the bottle | close photo; turn it with a pen after the trial |
| 0 | Anything else, including a crimp that was never torn | |

Doing nothing scores 0. The order is forced by the physics (a seal cannot reach the beaker without being torn off),
so no order caps are needed.

## How the simulation scores it (task.py)

- `update()` (called every control step by the simulator callback with the states and the contact forces it reads)
  keeps the whole-trial rules, the crimp and two timers: everything still, and both hands 30 mm clear.
- Contact forces come from PhysX contact views: seal against each plier half and against the six hand links of
  both arms; bottle against each plier half.
- The crimp (`update`, `TEAR_TURN`, `GRIP_N`, `TURN_MIN`): until it is torn, the callback holds the seal on the
  finish at the angle it has been turned to. Each control step in which both jaws press the seal with 10 N or more,
  the seal may turn with them and the turn is added up (either direction; steps under 0.5 degree are solver jitter
  and are not counted); once it reaches 90 degrees the crimp counts as torn and the seal becomes a loose rigid part
  held only by the jaws. A seal pushed by one jaw, by a light grip, or by a finger does not turn (it is put back),
  and nothing lifts a crimped seal. In two development traces a still 1.5 s squeeze added 2 and 7 degrees before the
  deadband was added; with it, the last full trace added 1.8 and 0 degrees (lots A and B).
- The rivet (`RIVET_K`, `RIVET_PLAY`): the plier's hinge holds its angle up to 0.48 N m and then slips (the hinge
  drive's set point follows the joint once the deflection passes 0.08 rad). A light squeeze carries the plier open;
  a hard squeeze closes the jaws on the seal; easing the squeeze lets the jaws spring back by up to 0.08 rad, which
  frees the torn seal so it falls.
- `level()` reads the end state and `score()` returns it once the 1 s still timer has run; the 1.0 level also needs
  the 1 s hands-clear timer. The hands are face-sampled every 3 mm on both arms' gripper and finger hulls.
- `near_misses.py` drives `update()` step by step for every rule (squeeze without turning, one jaw, 9.99 N, 89
  degrees, 180 degrees of 0.45-degree creep, a 0.51 N finger touch, a 20.1 N glass contact, glassware knocked over
  and stood back up, hands clear for only 0.5 s, still for only 0.9 s) and checks 15 end states, with the exact
  limits on both sides.

## Bypasses checked

| Shortcut | Result |
|---|---|
| Pull or pry the seal off with the fingers | 0 for the trial (finger contact over 0.5 N) |
| Lift the plier with the jaws closed on the seal but not turned | crimp holds; seal stays on; 0 |
| Turn the plier while barely touching the seal, or with one jaw | the seal does not turn; 0 |
| Squeeze the jaws on the glass neck below the seal | over 20 N: 0 for the trial |
| Knock the bottle or beaker over, then stand it back up | 0 for the trial |
| Tear the crimp and leave the seal on the neck | 0.25 |
| Drop the seal on the counter or leave it in the jaws | 0.5 |
| Put the seal in the beaker but keep holding the plier | 0.75 |

One-hand check: the model does not pass the crimp's twisting resistance to the bottle (see fidelity), so in
simulation a careful one-handed route is not ruled out; it is allowed, and it earns the same score only if the
unheld 110 g bottle is never knocked over or chipped and ends standing on the counter. The demo steadies the bottle with
the other hand, as a person does.

## How this differs from close tasks

- TK-Z97WAY (child-resistant medicine cap): a reversible push-and-turn cap opened by hand. Here a two-lever tool on
  a rivet must hold a crimped metal seal while it is turned, then be eased, not opened, to let the seal go.
- TK-8CK8TG (crown cap on a wall-mounted opener): the opener is fixed and levers the cap. Here the tool is handheld,
  its squeeze sets whether the jaws grip, and the bottle must be steadied by the other hand.
- TK-EMWW6H (locking pliers past their lock point) and TK-WBHW69 (tongue-and-groove pliers to the last groove): the
  pliers are worked on their own. Here the plier works on a part held by the other arm and must be carried open,
  squeezed hard, turned and partly released in sequence.
- TK-DKQNQY (circlip pliers): opens a ring into a groove on a loose shaft. Here the jaws close and turn a seal off
  glass that must not be chipped, and the removed metal is disposed of.
- TK-XJDMXF (rotary can opener): one hand keeps the clamp shut while the other turns the key. Here the squeeze and
  the turn are one hand's motion and the other hand steadies the glass.

## Fidelity and what is simplified

- True size from maker data: the bottle (52 x 95 mm, 20 mm crimp finish) and the beaker (68 x 90 mm). The plier,
  seal and stopper dimensions and every mass are estimates; all parts are boxes and cylinders (drawing:
  `fixture/dimensions.svg`).
- The seal is collided as one solid 21.5 x 7.6 mm cylinder (skirt, bead and stopper flange as one rigid piece) and
  shown as its skirt and annular top. The stopper stays in the bottle's neck. The bottle's bead and stopper flange
  are shown but not collided, because the seal fills them.
- The crimp is a declared rule, not a metal model: the seal stays on the finish until it has been turned 90 degrees
  in all while both jaws press it with 10 N or more; then it is a loose 1 g part. The skirt does not tear or deform,
  and the crimp's twisting resistance is not passed to the bottle (the bottle feels only the jaws' sideways
  contact), so steadying the bottle is allowed but not required. The 10 N grip and the quarter turn are estimates,
  not measurements; a spring scale on the real handles would set them.
- Rivet friction is a declared rule: the hinge holds its angle up to 0.48 N m, then slips, and springs back up to
  0.08 rad (about 2 mm at the jaw cradles) when the squeeze eases; this lumps handle flex and seal spring-back.
- Glass chipping is a contact rule (over 20 N of plier-on-glass force), not a fracture model.
- Two fixed layouts stop one memorised replay from passing both, not a replay of both.

## Real-world trial

Lay out the parts as in the layout table (tape outlines). One fresh crimped bottle per trial (reset notes). Run the
policy for 120 s; then, without touching anything, photograph the beaker from above and the side, the bottle neck
from two sides against a 1 mm rule, and check the finish under a 10x loupe. Read the score from the ladder. The
bottle is empty and nothing is hot or pressurised; the torn seal can have sharp edges, so people handle it with
the plier or gloves, never bare fingers.

Cost. Bought in the lab-supply packs named above, the set costs about $1,640 (prices checked Nov 21): bottles in
cases of 144 ($581.53), stoppers and seals in cases of 1,000 (about $382 and $105), and the E-Z Crimper (about
$412-509). A smaller set keeps the procedure and what is measured, for about $450: the WHEATON 224373 plier,
$148.71 ([DWK via CalPac](https://www.calpaclab.com/wheaton-plier-decapper-removes-20-mm-seals-case-1/dwk-224373));
Supelco 33110-U 100 mL clear serum bottles with a 20 mm crimp finish, 51.7 x 94.5 mm (within 0.5 mm of the
modelled bottle), about $150 for 36 ([Sigma-Aldrich](https://www.sigmaaldrich.com/US/en/product/supelco/33110u)),
reused between trials; Supelco 27230-U open-centre 20 mm aluminum seals, $28.60 for 100
([Sigma-Aldrich](https://www.sigmaaldrich.com/US/en/product/supelco/27230u)); Supelco 27232 grey butyl 20 mm
stoppers, $61.30 for 100 ([Sigma-Aldrich](https://www.sigmaaldrich.com/US/en/product/supelco/27232)); a generic
stainless 20 mm hand crimper, about $35-50 ([eBay](https://ebay.com/p/24039173713)), in place of the E-Z Crimper;
a PYREX 1000-250 beaker, $5.15
([Carolina](https://www.carolina.com/lab-beakers/pyrex-griffin-beaker-glass-low-form-measuring-250-ml/721209.pr));
and a 10x loupe, about $10. These seals and the generic crimper may hold a little differently from the 224178-01
and the E-Z Crimper, so reset step 1's check (a crimped seal must not turn by hand) applies to them too.

## Run it

yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), from this folder:

```sh
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Remove the aluminum crimp seal from the serum bottle with the plier decapper, drop the seal in the waste beaker and put the decapper down on the counter. Touch the seal only with the decapper's jaws, and move both hands away at the end." \
  --scorer task_score --max-steps 2400 --epochs 2
```

2,400 control steps (120 s at 20 Hz) per trial; trial k uses lot k mod 2 (`TASK_LOT=A` or `B` pins one lot). No
seeds change the scene. The scripted demo (`demo()`, policy `yam_scripted`) plays fixed waypoints from the visible
layout of each lot; the left hand's tilted grip uses the included `tilt.py`, which affects only the scripted demo.

## Results

- Scripted demo (`yam_scripted`, `--epochs 2`, 2,400 steps): 1.0 in lot A and 1.0 in lot B (mean 1.0). In both, the crimp tore at about 18 s and the trial reached full score at about 39 s of the 120 s limit; the scorer then ends the trial.
- Dream-1 (same instruction, `--epochs 2`, 2,400 steps): 0.0 in lot A and 0.0 in lot B (mean 0.0). Both trials ran to the 2,400-step limit; it did not refuse. The right hand moved over the plier but never closed the jaws on the seal; no crimp was torn and no rule was broken (the runner logged 100 request-expiry retries).
- What the demo does: the right hand lifts the open plier by its grips with a light squeeze (about 8 N per pad, under
  the rivet's slip point), lowers the open jaws round the seal while the left hand steadies the bottle, squeezes
  (about 30 N per pad; in the traced run of the same source the jaws pressed the seal with 10 to 24 N each, the plier touched the glass with at most 1.2 N, and no finger touched the seal), turns the plier 45 degrees about the bottle and back twice (the crimp tears after
  90 degrees of turning), the left hand lets go, the right hand lifts the seal off, carries it over the beaker,
  eases the squeeze to about 4 N so the jaws spring back and the seal drops in, and lays the plier back on the
  counter.
- Development runs on the same task (all traces, both lots unless stated): a converter bug (no result); the light
  grip on the splayed open handles let the plier pitch and fall (fixed by straight grip sections and a firmer carry
  squeeze); a thin skirt collider let the jaws close through it onto the glass and drag the bottle (fixed by the
  solid seal); a per-trial move of the jointed plier put it 19 mm off in lot B (fixed by leaving the plier in place
  in both lots; each trial now checks its start pose); a left fingertip brushed the lifted seal (fixed by gripping
  lower and letting go before the lift).

## Licence

Code and the procedural geometry: MIT. Text and the drawing: CC BY 4.0. No third-party meshes are included; the
maker documents are linked, not copied.
