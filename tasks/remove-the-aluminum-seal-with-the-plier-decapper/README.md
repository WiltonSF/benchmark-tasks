# Plier-decap a serum bottle: twist off the crimp seal, drop it in the beaker and put the tool down

**Revision status:** NOT-READY. The restored 0.25 N m tear-torque estimate and 25 N per-jaw engagement floor
were tested in a native 2,400-step A/B DEMO: A0.0/B0.75 (mean0.375). Full-score disposal was not demonstrated;
Same-source Dream-1 results are pending. Earlier lower-resistance results are not final evidence. Simulation readouts are not bench measurements.

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
and then disposes of the torn metal over the waste beaker.

O*NET 19-4031.00 Chemical Technicians, core task: "Set up and conduct chemical experiments, tests, and analyses,
using techniques such as chromatography, spectroscopy, physical or chemical separation techniques, or microscopy."

What makes it hard for a robot: a two-lever tool on a snug rivet must be carried with its jaws open, closed on a
7.6 mm-tall seal (not on the glass below it), squeezed with at least 25 N at each jaw while it is turned (a lighter
grip slips and the seal does not turn), then opened over the beaker so the seal falls inside; the decapper
must finish on the counter. The demo releases the squeeze over the beaker, supports the tool near its rivet,
repositions the held tool, attempts a separate grip on handle A before spreading B, and returns the tool to the counter. The bottle must be steadied: the estimated
tear torque exceeds the approximately 0.011 N m base-friction torque of the empty bottle.

## Parts and scene (true size)

| Item | Product | Size and mass | Status |
|---|---|---|---|
| Serum bottle | WHEATON 223747, 100 mL clear glass, 20 mm crimp finish | 52 mm diameter, 95 mm tall with seal; 110 g | maker size; mass an estimate |
| Stopper | WHEATON W224100-405 grey butyl, 20 mm | flange 19 mm, under the seal | maker part; size estimated |
| Seal | WHEATON 224178-01, 20 mm open-top aluminum seal | 21.5 mm outside, 7.6 mm tall, 13 mm centre hole, about 1 g | maker part; sizes and mass estimates |
| Plier decapper | WHEATON 224373 (steel, grey plastic-coated handles), US$139.99 on the opened Zoro page, Nov 23 | 157 mm long, 160 g; jaws curved to the seal; jaw tips 32 mm apart at the start | maker part; length, mass and jaw shape estimates |
| Waste beaker | PYREX 1000-250, 250 mL low form | 68 mm outside x 90 mm; 110 g | maker size (Corning catalogue); mass an estimate |
| Counter and shelf | lab preparation counter (the station table), white reagent shelf at the back | shelf 800 mm wide, 420 mm tall, front at x 495 | fixed |
| Per trial | one new 224178-01 seal (sold in packs; price not checked); a 20 mm hand crimper (WHEATON W225303 E-Z Crimper) to seal the bottles | | consumed |

Sources: [224373 product page](https://www.dwk.com/na/wheaton-plier-decapper-removes-20-mm-seals-224373);
[WHEATON Vial
Guide](https://d3h4jppqn0j59k.cloudfront.net/pub/media/mageplaza/resource/pdf/WHEATON-Vial-Guide-Brochure.pdf)
(page 21: plier decapper; serum bottles and seals); [serum bottle
flyer](https://d3h4jppqn0j59k.cloudfront.net/pub/media/mageplaza/resource/pdf/w/h/wheaton_crimp_serum_bottles_vials_flyer_en.pdf);
[DWK sealing components, page
11](https://d3h4jppqn0j59k.cloudfront.net/pub/media/mageplaza/resource/pdf/d/w/dwk_sealing_components_for_vials.pdf#page=11)
(224178-01 open top); [Corning
1000-250](https://ecatalog.corning.com/life-sciences/b2c/US/en/General-Labware/Beakers/Beakers%2C-Glass/PYREX%C2%AE-Beakers%2C-Low-Form%2C-Double-Scale%2C-Graduated/p/1000-250).
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
- The crimp (`_author_crimp`, `CRIMP_TAU`, `CRIMP_K`, `TEAR_TURN`, `GRIP_N`) is a native bottle-to-seal revolute
  joint, authored with the scene before simulation starts. It locks relative translation and tip, leaving axial
  twist free. Its native angular force drive has estimated stiffness 5 N m/rad, damping 0.001 N m s/rad and
  torque cap 0.25 N m (reached at 0.05 rad relative to its target). The scorer uses a larger 0.1 rad reference
  deadband before crediting slip; this is conservative bookkeeping, not the physical elastic limit. Beyond it,
  the reference target follows the measured relative angle. Absolute credited slip counts only while both jaws press
  with at least 25 N; 90 degrees accumulated credited slip disables the joint. The seal then moves through contact
  and gravity. No in-trial seal or bottle pose/velocity writes are used. Prior readouts and current validation status are reported below;
  configuring a drive alone is not evidence that its torque reached the bottle.
- The rivet uses native static/sliding joint friction, 0.48 N m (an unmeasured estimate). There is no spring,
  viscous drive, moving angle target or automatic spring-back. Friction dissipates work and cannot open the jaws.
  The demo removes the squeeze and lets the tool settle against the beaker rim, then supports it near the rivet
  then attempts to grip A separately and spread B through robot contact. The native DEMO scored A0.0/B0.75; tool disposal did not reach full score.
- `level()` reads the end state and `score()` returns it once the 1 s still timer has run; the 1.0 level also needs
  the 1 s hands-clear timer. The hands are face-sampled every 3 mm on both arms' gripper and finger hulls.
- `near_misses.py` drives `update()` step by step for every rule (squeeze without turning, one jaw, 24.99 N, 89
  degrees of plastic turn, elastic deflection without yield, slow plastic turning, a 0.51 N finger touch, a 20.1 N glass contact, glassware knocked over
  and stood back up, hands clear for only 0.5 s, still for only 0.9 s) and checks 15 end states, with the exact
  limits on both sides.

## Bypasses checked

| Shortcut | Result |
|---|---|
| Pull or pry the seal off with the fingers | 0 for the trial (finger contact over 0.5 N) |
| Lift the plier with the jaws closed on the seal but not turned | the joint holds the seal on the finish; 0 |
| Turn the plier while barely touching the seal, or with one jaw | the jaws slip, or the turn does not count toward the tear; 0 |
| Squeeze the jaws on the glass neck below the seal | over 20 N: 0 for the trial |
| Knock the bottle or beaker over, then stand it back up | 0 for the trial |
| Tear the crimp and leave the seal on the neck | 0.25 |
| Drop the seal on the counter or leave it in the jaws | 0.5 |
| Put the seal in the beaker but keep holding the plier | 0.75 |

A prior native probe at the restored 0.25 N m setting sustained approximately 0.247-0.249 N m contact torque and
released after 90.65 degrees of credited slip. Those are simulation readouts, not bench measurements. Its disposal
failed; it is not a full-score result. The current source tightens the per-jaw engagement floor from 10 to 25 N and
uses a B-handle capture validated in lot A, followed by an isolated A regrasp and physical B-handle spreading attempt. A matched native one-hand control turned the bottle 65.0 degrees while relative seal yaw stayed below 0.43 degrees;
both jaws stayed above 35 N and credited tear travel remained 0. The control differs only in unused late two-handed
waypoints: the physical model and complete one-hand reference plan are identical. The frozen v44 native DEMO scored A0.0/B0.75; the scripted feasibility gate failed; same-source Dream-1 results are pending.

## How this differs from close tasks

- TK-Z97WAY (child-resistant medicine cap): a reversible push-and-turn cap opened by hand. Here a two-lever tool on
  a rivet must hold a crimped metal seal while it is turned, then be opened to let the seal go.
- TK-8CK8TG (crown cap on a wall-mounted opener): the opener is fixed and levers the cap. Here the tool is handheld,
  its squeeze sets whether the jaws grip, and the bottle must be steadied by the other hand.
- TK-EMWW6H (locking pliers past their lock point) and TK-WBHW69 (tongue-and-groove pliers to the last groove): the
  pliers are worked on their own. Here the plier works on a part held by the other arm and must be carried open,
  squeezed hard, turned and partly released in sequence.
- TK-DKQNQY (circlip pliers): opens a ring into a groove on a loose shaft. Here the jaws close and turn a seal off
  glass that must not be chipped, and the removed metal is disposed of.
- TK-XJDMXF (rotary can opener): one hand keeps the clamp shut while the other turns the key. Here the squeeze and
  the turn are one hand's motion and the other hand steadies the glass.

## Crimp torque: where the number comes from

The crimp's resisting torque is the single replaceable constant `CRIMP_TAU = 0.25` N m near the top of `task.py`.
It is an engineering estimate, not a measured release torque on the specified WHEATON products. Mathaes et al.
reported residual seal forces of 7-115 N across capping settings and closure combinations
([PDA 70(1):12-29
abstract](https://www.ebi.ac.uk/europepmc/webservices/rest/search?query=DOI:10.5731/pdajpst.2015.005876&resultType=core&format=json)).
Using an assumed 50 N reference preload, assumed aluminum/butyl friction 0.6 and mean annular radius 8.3 mm gives
0.6 x 50 x 0.0083 = 0.249 N m, rounded to 0.25 N m. These assumptions are stated; the paper did not measure this
plier's tear torque. The separate phone description of a light squeeze concerns tool handling, not seal fracture,
and is not used to reduce tear resistance. A real start-turn/tear torque measurement can replace `CRIMP_TAU`.

To replace the estimate, clamp the empty bottle and keep both decapper handles squeezed together on the seal.
Use a torque meter, or pull a spring scale tangent to a circle around the bottle-neck axis while maintaining the
squeeze. Measure the perpendicular distance from that axis to the scale's line of force: torque in N m is force in
N times that distance in metres. Record the start-turn peak, the peak while tearing and approximate tear angle for
3–5 fresh seals at one recorded crimper setting; report the median and specimen product. Measure handle-opening
or pivot effort separately. No such product torque reading has been obtained for this revision.

### Other thresholds: derivations and limits

- **25 N per jaw:** the Coulomb torque bound is 0.25/(2 x 0.5 x 0.01075)= 23.26 N per jaw, rounded upward to 25 N.
  Radius 10.75 mm and sliding friction 0.5 are model estimates. This tightens the prior 10 N engagement floor;
  it does not claim a published jaw-force measurement. Native friction must still transmit the actual torque.
- **90 degrees credited slip:** the maker specifies squeeze-and-twist but gives no angle. The model assumes a
  circumferential path of two estimated skirt heights: 2 x 7.1/ 10.75 = 1.32 rad (75.7 degrees), rounded upward to
  a quarter-turn. This is an unmeasured conservative path assumption, not an angle reported by the maker.
- **Reference stiffness 2.5 N m/rad:**0.25 N m / 0.1 rad assumed compliance. The 0.1 rad allowance is about 1/16 of a
  quarter-turn. Native stiffness is twice that reference (5 N m/rad); native torque caps at 0.05 rad. Credited slip
  starts only beyond 0.1 rad, so the score never treats the full physical deflection as irreversible work. Neither
  stiffness nor compliance has been measured on a seal.
- **20 N glass contact:** a stress-scale proxy, 25 MPa x assumed 0.8 mm2 edge-contact patch = 20 N. The 25 MPa reference
  comes from [SCHOTT BOROFLOAT 33 mechanical data, printed p12](https://www.sydor.com/wp-content/uploads/SCHOTT-BOROFLOAT-33-Display-Glass.pdf#page=12).
  That source is flat borosilicate glass, not a WHEATON bottle-chip test. Patch area and mapping bending stress to
  edge damage are assumptions; this remains a proxy, not a calibrated chip force. The real-trial rule is visible
  glass damage under a 10x loupe, not a claim that every bottle chips at 20 N.

These estimates need a product torque/angle reading and material-contact validation. No technician measurement
is claimed for the grip, angle, stiffness or glass threshold.

## Fidelity and what is simplified

- True size from maker data: the bottle (52 x 95 mm, 20 mm crimp finish) and the beaker (68 x 90 mm). The plier,
  seal and stopper dimensions and every mass are estimates; all parts are boxes, cylinders and one convex prism (drawing:
  `fixture/dimensions.svg`).
- The seal is collided as one solid 32-sided convex prism with a 21.5 x 7.6 mm envelope (skirt, bead and stopper flange as one rigid piece) and
  shown as its skirt and annular top. The stopper stays in the bottle's neck. The bottle's bead and stopper flange
  are shown but not collided, because the seal fills them.
- The crimp is a native joint, not a metal model: the bottle-seal joint resists the seal's turn on the finish with up
  to 0.25 N m and passes the same torque to the bottle; it lets go after the seal has slipped 90 degrees against it
  while both jaws pressed with 25 N or more. The skirt does not tear or deform, and the jaws grip the seal by
  friction only (real decapper jaws also bite into the soft aluminum). 0.25 N m, the 90-degree tear turn and the
  25 N grip are estimates, not measurements (see "Crimp torque: where the number comes from").
- Rivet resistance is native static/sliding friction of an estimated 0.48 N m, with no viscous drive. There is no
  spring or moving angle target; no stored energy opens the jaws. This resistance has not been measured on the
  named tool.
- Glass chipping is a contact rule (over 20 N of plier-on-glass force), not a fracture model.
- Two fixed layouts stop one memorised replay from passing both, not a replay of both.

## Real-world trial

Lay out the parts as in the layout table (tape outlines). One fresh crimped bottle per trial (reset notes). Run the
policy for 120 s; then, without touching anything, photograph the beaker from above and the side, the bottle neck
from two sides against a 1 mm rule, and check the finish under a 10x loupe. Read the score from the ladder. The
bottle is empty and nothing is hot or pressurised; the torn seal can have sharp edges, so people handle it with
the plier or gloves, never bare fingers.

Cost (US dollars, product pages opened Nov 23; before tax and shipping). Two complete routes avoid a generic
marketplace crimper:

| Item | Ready-sealed route | Reusable-bottle route | Opened source |
|---|---:|---:|---|
| WHEATON 224373 plier | $139.99 | $139.99 | [Zoro G5865036](https://www.zoro.com/wheaton-pliers-hand-operated-20mm-aluminum-seals-224373/i/G5865036/) |
| ALK/Allergy Labs SV10020SILVER-25, 25 empty pre-sealed 100 mL vials with grey stoppers and silver seals | $155.00 | — | [IVPACKS](https://ivpacks.com/sealed-sterile-vials/865-100ml-clear-sterile-vials-silver-seals-pk-of-25.html) |
| Type 1 clear 100 mL serum bottles, 52 x 94 mm, 42-pack | — | $98.75 | [IVPACKS](https://ivpacks.com/clear-serum-vials/82-100ml-clear-serum-vial-52x94mm-ream-of-42.html) |
| Kebby 20001-00-C05A crimper for standard all-aluminum 20 mm seals | — | $145.00 | [IVPACKS](https://ivpacks.com/vial-crimpers-decrimpers/382-20mm-all-aluminum-vial-seal-crimper-kebby-crimpers.html) |
| Natural silver 20 mm open-hole seal rings, bag 1,000 | — | $50.00 | [IVPACKS](https://ivpacks.com/20mm-hole-punched-vial-seals/573-20mm-hole-punched-vial-seals-natural-silver-bag-1000.html) |
| 73828A-RBUS-100 20 mm round-bottom chlorobutyl stoppers, 100-pack | — | $15.00 | [IVPACKS](https://ivpacks.com/20mm-vial-stoppers/953-20mm-vial-stopper-usa-manufactured-round-bottom-chrlorobutyl-rubber.html) |
| PYREX 1000-250 beaker, minimum two at $8.39 | $16.78 | $16.78 | [Zoro G3300544](https://www.zoro.com/corning-pyrex-250-ml-low-form-griffin-beaker-do-1000-250/i/G3300544/) |
| Carson LL-10 10x loupe | $8.99 | $8.99 | [B&H](https://www.bhphotovideo.com/c/product/437227-REG/Carson_LL_10_LL_10_LumiLoupe_Craft_Loupe.html/overview) |
| Total | **$320.76** | **$474.51** | prices may change |

A lab counter, camera/phone, tape and millimetre rule are ordinary existing trial equipment. A torque meter or spring
scale for the calibration protocol is additional equipment; its price is not included or claimed here. The
opened Zoro page displayed $139.99 on the latest Nov 23 check; earlier checks differed. Prices are
observations, not quotations.

These are explicitly named substitute specimen routes, not claims that ALK/IVPACKS seals equal WHEATON 224178-01.
The ready-sealed product page names the product and closure but does not publish its outside dimensions. Before
admitting a substitute lot, check 20 mm crimp finish, bottle diameter 52 +/- 1 mm, overall sealed height 95 +/- 1 mm,
intact glass and seal that does not turn by hand; reject any mismatch. The reusable bottle's published height is
1 mm below the model. Check a few seals with the torque protocol and record the product, crimper setting and actual
reading; the simulation remains an estimate until this is done. Allow about a minute (a setup estimate) to
inspect and place a ready-sealed bottle; reusable bottles need a new seal crimped at one recorded setting. No sterilization is needed
for this empty-bottle manipulation trial. Twenty-five ready-sealed specimens cost $6.20 each, then the empty glass
can be reused with the crimper route. The full WHEATON catalogue-pack route from the original specification costs
substantially more and is not the low-cost route claimed here.

## Run it

yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), from this folder:

```sh
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Remove the aluminum crimp seal from the serum bottle with the plier decapper, drop the seal in the waste beaker and put the decapper down on the counter. Touch the seal only with the decapper's jaws, and move both hands away at the end." \
  --scorer task_score --max-steps 2400 --epochs 2
```

2,400 control steps (120 s at 20 Hz) per trial; trial k uses lot k mod 2 (`TASK_LOT=A` or `B` pins one lot). No
seeds change the scene. The scripted demo (`demo()`, policy `yam_scripted`) plays fixed waypoints from the visible
layout of each lot; the recovery grip uses the included `tilt.py`, which affects only the scripted demo.

## Revision response and results

> Construct validity (not met): The simulated crimp supplies no twisting resistance to the bottle, so the robot
> need not apply the torque required to tear a real seal.
>
> What to change: Add a crimp-release model that resists twisting and transfers the reaction torque to the bottle,
> calibrated against the specified decapper and seals. Demonstrate that the revised task reaches full score within
> the run limit.

1. **Resistance and reaction:** a scene-authored native force-driven revolute joint replaces the pose attachment.
   It applies equal/opposite resistance to seal and bottle. Native held-A work samples average 0.248277 N m; the matched unheld control rotated the bottle 65 degrees with relative seal yaw below 0.43 degrees and no credited tear. These are simulation diagnostics; full-score disposal remains unproven.
2. **Release:** disable the native joint only after 90 degrees credited slip beyond the conservative reference
   deadband while both jaws apply at least 25 N. No rule writes a part's in-trial pose or velocity.
3. **Calibration:** 0.25 N m is the study-anchored engineering estimate derived above, not a measurement on WHEATON
   224373/224178-01. The grip, slip path, stiffness and glass proxy derivations and assumptions are explicit.
4. **Full score within 2,400 steps:** unmet. Frozen v44 native DEMO scored A0.0/B0.75 (mean0.375), driver success with no errored trials. Same-source Dream-1 results are pending. The run directory is `demo-20261124T070902`; earlier successful scores are not evidence for this source.

In this native v44 run, A did not establish a two-pad A-handle grip: the opening command changed whole-tool yaw
by 10.45 degrees while the hinge changed by only 0.00083 rad. The seal remained clamped and the still timer stayed
at zero, yielding the official A0.0. In B the seal settled inside the beaker, but the tool remained tilted about
63.8 degrees against it, yielding B0.75. Neither is a full-score demonstration.

Historical results (previous pose-write crimp, Nov 21): scripted demo A1.0/B1.0; Dream-1 A0.0/B0.0, step limit,
no refusal. The previously linked media show that model and do not validate this revision. Intermediate pure-damping
revision also reached A1.0/B1.0, but its measured bottle reaction was inadequate; it is not offered as final evidence.

Prices may change. Alternative seller: [Thomas Scientific 224373](https://www.thomassci.com/p/pliers-decappers),
$185.40 per item on the opened Nov24 page; the ready-sealed kit is $366.17 using that seller instead of the
listed Zoro quote.
