# Squeeze the lemon half with the folding citrus press

Load a fresh half into a free-held Dreamfarm Fluicer Lemon, coordinate its original handles under resistance, collect juice in the bowl, and leave the spent half on the saucer and the press inside the white mat outline. This is an implemented simulation with declared material rules. No hardware trial or force calibration has been performed.

Instruction: “Squeeze the lemon half into the bowl with the citrus press, then leave the spent half on the saucer and the press inside the white mat outline.”

## Score 0 if

At each20Hz control observation, a failure persists for the rest of the trial: any modeled expressed juice misses the bowl; the back hinge separates more than 3 mm; the handle leaves its native intact range (closure −0.03 to2.8rad, with5mrad numerical tolerance); the half escapes the loaded basket while closure exceeds 0.35 rad; the cut face tilts more than 20° from vertical during loaded pressing; the bowl tips more than 10° after pressing; any scored object falls below the counter; or required history is unavailable, nonfinite, starts after0.050001s, runs backward in time, or is more than0.050001s after its prior20Hz observation. Duplicate reads of one timestamp are idempotent. A later healthy sensor frame cannot erase an invalid frame. Cut-face and basket-retention failures are the declared seed/rind-contamination proxy. Seed particles and tearing are not simulated.

Every positive tier requires one continuous loaded stroke with two distinct hands making opposed contact on the original handles, the half seated cut-side down, and the tool more than 5 mm above all support surfaces. Both hands must then be more than 5 mm from every scored solid, with a one-second still hold.

| Score | Qualified handle stroke | Collected volume | Final condition |
|---|---:|---:|---|
| 0.25 | ≥45° | ≥2 mL | Released and still |
| 0.50 | ≥90° | ≥8 mL | Released and still |
| 0.75 | ≥120° | ≥15 mL | Released and still |
| 1.00 | ≥150° | ≥20 mL | Spent half supported on saucer; press on mat; released and still |

Mechanical stroke is the largest reduction in handle opening within a continuous qualified contact interval. Later static contact cannot credit an earlier stroke. Material production and spills are recorded during physical loaded compression regardless of hand eligibility. Empty preclosure gives no loaded stroke.

Use overhead video and a protractor for handle travel, synchronized side video and a millimetre ruler for support clearance, and a graduated vessel resolving 1 mL for volume. A dry contrasting tray reveals expressed spills. Any visible expressed liquid outside the bowl is a physical failure; below-resolution wetting must be reported. The simulation conservatively rejects any positive spilled inventory. Verify the final 5 mm hand gap in both views against a gauge after stopping; an occluded gap makes the physical trial unjudgeable. Track physical vessel rim/corner markers, handle ends, panel/floor corners and hinge pins against the ruler for stillness. At each20Hz observation, the implementation reads each physical native link's instantaneous world linear/angular velocity, rejects spin>0.15rad/s or any boundary-point speed>12mm/s, and also uses all collision-box corners and64 points per cylinder cap, excluding non-colliding decoration; these are physical-boundary witnesses rather than40mm surrogate points.

Still means every tracked point stays within 2 mm of its hold anchor for 1 s, with body speed below 12 mm/s and spin below 0.15 rad/s. For 1.0, all press corners project inside the marked400×440mm parking outline with 1 mm tolerance; its lowest point is at table +6 ±2 mm and its normal has vertical dot product >0.94. The half's root is within 25 mm in each horizontal axis of the saucer centre. Its lowest collision surface is 2–6 mm above the saucer root, whose floor is at +4 mm; the saucer is upright within the same >0.94 normal criterion. Either stable rind orientation is permitted on the saucer. Cut-side-down remains mandatory during pressing. Layout A/B scores are averaged.

## Mechanism and declared material model

The maker sheet shows four separate hinges. The model uses the loop C–A–D–B–C: common handle hinge C, handle/basket hinges A and B, and rear basket hinge D. Estimated hinge pitches are C–A/C–B =50 mm and A–D/B–D =100 mm. Open coordinates relative to the tool root are C=(0,0,9), A=(0,50,9), B=(0,−50,9), D=(86.60,0,9) mm. The estimated original handles are 200 mm long with 18 ×18 mm sections. Initial opening is 180°; relative closure is limited to 2.8 rad (160.43°). Four physical revolute joints close the loop; the back joint is excluded from the articulation tree. No main-handle trajectory is imposed by the environment.

The polypropylene strainer is approximated by an 80 ×104 mm rigid elliptical support on a bounded follower hinge and 26 mm slider. Targets depend on measured handle angle and rear-pin position. Springs are 4 N m/rad, capped at 0.30 N m, and 500 N/m, capped at 3 N. These stand in for a plastic fold. The support does not narrow, and its collisions with the main linkage are filtered. Main-link masses are 50/50/65/65 g and follower masses 10/30 g, totaling 270 g. Panels are estimated 2.5 mm thick and 40 mm high, elevated for handle clearance. Detailed pitches, sections and offsets come from product views, not measured CAD. Equivalence of this support to the real folding floor remains unvalidated.

The 70 g half has a 60 mm diameter, 30 mm high hemispherical contact envelope made from 5 mm bands. Hand, floor and table contacts remain physical. Panel/fruit contact is filtered and replaced by compression wrenches. The rendered rind narrows after loaded closure; its collision envelope stays 60 mm. No fruit attachment or grasp weld exists. The half remains 70g and the bowl 130g throughout: the scalar juice inventory does not transfer mass or momentum. This is a disclosed material-model approximation, not physical fluid transport.

Every 20 Hz control step applies resistance through `set_external_force_and_torque` at 180 mm purchase on each original handle body. For a seated half and closure q>0.15 rad, target force per handle is

`min(8, 3 + 3*clip(q/2.8,0,1) + 2*max(qdot/2,0)) N`, smoothed over 0.15 s.

Point loads become body-centre wrenches, with equal force/moment reaction at the counterheld base. A 180:50 mm lever estimate maps 3–8 N per handle to about 11–29 N per basket side. This is an engineering estimate of approximately 20–50 N total fruit compression, not a maker specification or measured fruit curve.

Pressure is mean previously applied handle force ×3.6 divided by the 60 mm cut-face area. Loaded stroke >0.10 rad integrates `max(pressure−5000 Pa,0)*dt`. Each 120,000 Pa s requests25mL, limited by a finite25mL inventory and by available compression work; reopening never refills it. Material accounting runs independently of grasp eligibility. Capture uses the outlet at the root, 9 mm below C: 5–110 mm above the bowl rim, bowl tilt ≤5°, and the whole 8 mm-radius/8° stream cone, including ballistic lateral motion, inside the 69 mm effective opening. Otherwise inventory spills permanently. This is a declared pressure-time rule, not fluid dynamics or a calibrated constitutive law.

The work ledger combines positive external hinge-load work (`previous mean force×0.180m×measured relative travel`) with a conservative lower bound on the existing native handle damper's work, `0.02×delta_q²/dt` joules. The native implicit hinge damper was already0.02Nm s/rad; it is unchanged. Its work is counted only with the half seated cut-down after loading, so empty hinge motion cannot precharge the material budget. This is a composite virtual material/hinge energy account, not a hardware calorimetry measurement. Returned external work debits the store. Each modeled millilitre consumes0.060J irreversible loss, an uncalibrated estimate (25mL costs1.5J).

Before EVERY wrench publication—including startup and static holds—each force is bounded by `energy/(0.180m×0.151rad)`. Thus any valid next opening step can return at most the energy available after production. Production also retains0.21744J (=8N×0.180m×0.151rad) for sampled unloading. The existing tighter remaining-opening cap still applies once unloading is observed. A static hold adds no work; the native damper funds the initially small activation forces during actual hinge motion, with no initial energy credit. Zero-work instantaneous geometry jumps cannot produce juice.

This20Hz reduced controller has a declared validity domain of observed hinge travel≤0.15rad per50ms (`abs(delta_q)/dt≤3rad/s`, a sampled average rather than a continuous peak bound);0.151rad includes cadence/roundoff margin. A faster observed step, or cumulative energy deficit>1microjoule, permanently clears the load and marks `model_valid=False`. The scalar API falls back to0, but such attempts are INVALID/UNSCORED, not scored robot failures. Retain their count and reason; exclude them from a numeric A/B mean and report the evaluation incomplete. A valid completed Dream record requires both epochs model-valid as well as ordinary runtime completion. This numerical domain/diagnostic is not a hardware task-failure threshold. Hardware uses the observable geometric, release, spill and volume criteria above. This account covers the known native handle damper and declared external load; it excludes other contacts/follower work and is not a complete physical material simulation. Parameters require calibration together.

Seating requires the half within 14 mm along and 12 mm across the current basket midpoint, root height 2–14 mm above the tool root, tool tilt ≤15°, and cut-face tilt ≤20° from vertical. Opposed pad samples must reach lateral18mm handle surfaces within1mm, at least1.5mm from horizontal caps; rounded-end normals must oppose along the closing direction with absolute horizontal component>=sqrt(0.5). Top-only contact cannot qualify. This contact-envelope check is geometric; it is not a fingertip force sensor.

Hardware camera resolution, shutter, synchronization and measurement bandwidth must be qualified against the ruler/protractor before timed trials. Video-derived velocities have uncertainty and cannot establish the simulator's instantaneous link velocities exactly. Faster or sub-pixel motion can differ between these measurements; report the observation rate/uncertainty and invalidate occluded or unjudgeable physical trials. This proposed hardware procedure has not been enrolled or shown equivalent to the native velocity guards.

## Reproducible setup and real kit

[Dreamfarm Fluicer Lemon DFFU3451](https://us.dreamfarm.com/fluicer/) publishes 250 mm length, 104 mm width, 270 g, polypropylene/stainless steel. The maker describes two-handed sideways operation and says “cut side facing toward you.” The collecting pose here rotates that face downward. The [maker sheet](https://housewares.blob.core.windows.net/hc365v2/original/a5d0f21e-be52-426c-85cc-b795b8ccdbd1.pdf) establishes the linkage topology. Detailed dimensions remain estimates requiring physical enrollment.

All coordinates are world millimetres; z is relative to the tabletop. The silicone sheet is400×560×6mm, physically centred(280,40). Its marked400×440mm parking outline stays centred(280,100), exactly the predecessor parking bounds. The larger forward support keeps the bowl flat; it does not enlarge the scoring zone. Reuse a600×400 stock sheet trimmed once to560mm and mark the outline before trials. The tool root is (270,100,+7) in both layouts. A fresh half starts cut-down on the mat at (190,250,+6.5). The rigid polypropylene saucer is Ø130 ×8 mm, 120 g, with 3 mm walls and root (130,0,+6). The blue polypropylene bowl is Ø160 ×55 mm, 130 g, with 3 mm walls and root (355,−130,+6). Layout B moves only the bowl to x375 and saucer to y-20. Fruit and articulated tool keep the same declared start. The drawing is [fixture/setup.svg](fixture/setup.svg). Settling can produce small contact offsets; inspect the recorded start before accepting a trial.

The simulator resets arm actuator buffers, joint positions/velocities/targets to READY, primes the cleared controllers, then restores every loose body, hinge, velocity and external-force state before settling. This happens only between trials. Earlier second-epoch carryover is preserved in the build history as invalid setup evidence.

| Obtainable item | Price or allowance |
|---|---:|
| [Original Fluicer Lemon](https://www.everythingkitchens.com/dreamfarm-fluicer-fold-flat-easy-juicer-lemon.html) | $19.99 |
| [OXO 5 lb scale](https://www.oxo.com/5-lb-food-scale-with-pull-out-display.html), retail allowance | $37.99 |
| [Room Essentials bowls](https://www.target.com/p/-/A-94962235), two | $1.00 |
| [Bel-Art 25 mL graduated vessel](https://www.usplastic.com/catalog/item.aspx?itemid=35633), 0.5 mL marks | $5.35 |
| [Two BS-SCALE5 20 N spring scales](https://www.homesciencetools.com/product/spring-scale-2000-g-20-newton/) | $11.90 |
| Lemons, saucer, support sheet, tray, tape and ruler allowances | $60.00 |
| Knife, cutting board, thermometer, calipers, protractor and attachment cord (allowance) | $35.00 |
| Two USB webcams and clamp mounts, planning allowance | $50.00 |
| Total before tax/shipping | $221.23 |

Existing table, computer and one phone for the overhead view; the two budgeted webcams record simultaneous side views. Record a visible clap/flash in all three streams and align frames before judgment. Ancillary-tool prices are planning allowances, not checkout quotes. Tie each spring-scale hook to the original handle at180mm using the cord, oriented tangentially; the second scale counterholds the other handle. Use two synchronized side views and a fixed overhead view with the ruler/protractor in frame. If the pressing contact, hinge gap or final clearance is occluded, invalidate and repeat the trial with a fresh specimen; never assign a passing score from an unjudgeable view. Time batch qualification separately from the estimated under2min fresh-half preparation. The original press and spring scales were listed in stock at the continuation check; delivery is not guaranteed. Commodity vessels are candidates requiring dimensional enrollment, not verified exact replacements for the simulated shapes. Other listed prices remain retail allowances or earlier research anchors. No hardware has been purchased.

Fresh specimens are bought ready; fresh-half preparation/reset is estimated at60–120seconds, not measured. Enroll60±5mm diameter,60–80g half,20–24°C, cut less than5minutes earlier. Qualification is separate from that per-trial preparation estimate. For a sacrificial sister half, match the evaluated schedule: close through150–155° of measured physical hinge travel (not commanded wrist travel) over7.5s, ramp force during closing and reach approximately6±1N tangential force on each original handle at180mm from C at the measured150–155° endpoint, then hold40s, then reopen over4s. Collect at least20mL. Apply two additional closures with exactly that same150–155°/7.5s/6±1N/40s/4s schedule into a second vessel; their combined additional yield must be<1mL. Reject a batch that fails either qualification criterion. This screens batches; it does not guarantee a fresh half's yield. Every enrolled robot trial receives a fresh unpressed half and remains counted even when its yield is low. Its score uses absolute volume and observed stroke, without a residual-yield denominator. No qualification, individual-half repeatability, preparation-time or force calibration measurements have been performed. Clean and dry the reusable kit; no printing, casting or drying per specimen. Trial food is not eaten; each trial uses up one half.

## Neighbouring activities and bypasses

BEHAVIOR-1K includes [make_lemonade](https://behavior.stanford.edu/knowledgebase/tasks/make_lemonade-0.html), using bottled juice, and [make_lemon_or_lime_water](https://behavior.stanford.edu/knowledgebase/tasks/make_lemon_or_lime_water-0.html), including raw lemon. The proposed distinction is loaded four-bar handle coordination and directed collection. It has no mixing or recipe steps. RoboCasa includes juicing a sliced fruit on a reamer. There the fruit is pressed down onto a fixed cone; here the tool has no base, both hands hold it in the air, and the skill is closing two handles against resistance while keeping the fruit seated and aiming the outlet.

| Neighbour | Comparison |
|---|---|
| TK-NKL487/PR1137, closed final | Stationary reaming versus free-held folding linkage; extraction outcome overlaps. |
| TK-R547UD/PR3174 and PR2460 | Garlic presses: solid extruded and scraped; no liquid collection. |
| TK-W5XE32/PR3381 | Fixed tortilla press; another loaded pressing operation. |
| TK-CXPYGC/PR1059, accepted | Two handles closing around loaded material already exists in hedge shears. Basket loading/retention and collection are the proposed additional constraints. |
| TK-YTH6M4/PR1436, accepted | Opposed deformation and collection already exists in an ice tray; this model uses loaded compression and directed extraction. |
| TK-QF4WMY/PR991, accepted | Loaded lever counterholding and release judgment already exists in tube bending. This task does not test hidden springback adaptation. |
| PR1899/PR1830, accepted | Cucumber peeling and empty spring-tongs locking respectively. |

| Bypass | Result |
|---|---|
| Empty closure or inserting after preclosure | No loaded mechanical stroke: 0 |
| One-hand squeezing or table bracing | No qualified mechanical stroke: 0; material/spills still recorded |
| One-hand compression then static two-hand contact | No retrospective stroke credit: 0 |
| Pads hovering 4 mm outside handles | Fails 1 mm contact envelope: 0 |
| Press elsewhere, then move over bowl | Permanent spill: 0 |
| Escape, incorrect pressing orientation or broken hinge | Permanent 0 |
| Keep holding a scored part | Release/stillness gate: 0 |
| Too little stroke or volume | Corresponding tier caps score |

## Run and evidence

Only procedural assets and local `parts.py`, `lots.py`, `tilt.py` are required. The scripted demo uses fixed waypoints from the visible layout. The right arm waits clear while the half is loaded into the open press on the sheet; the left hand then grips the base handle before the right approaches. It grips the original handles at185/85mm, carries/compresses/holds/reopens, then retains both closed grips while translating over the left-front saucer. The left wrist pivots from45degrees to straight while the right hand counterholds, without opening either grip. The press tips80degrees with a simultaneous10degree side lean, holds3s for gravity release, levels, restores the left wrist and parks. The relative wrist orientation requests159.3degrees, below the160.43degree hinge stop. Fixed tangential TCP leads of1mm on each hand at full closure compensate deflection measured in an earlier native run; the resulting centre-distance-implied closure is160.27degrees, also below the160.43degree stop. Actual native hinge travel, contact and intact limits remain independently judged. The release root is45mm right of the saucer centre and80mm above the table. The lower diagonal tip was evaluated in the recorded native scripted trials; see the execution results below. No scoring tolerance was enlarged. Compression hold remains40s; total plan116.9s. No simulator state feedback controls the motion. Offline IK and kinematic-tool gravity probes are planning checks; the execution results below come from native runs.

```bash
I="Squeeze the lemon half into the bowl with the citrus press, then leave the spent half on the saucer and the press inside the white mat outline."
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 --instruction "$I" --scorer task_score --max-steps 2400 --epochs 2 --store-frames --log-dir demo
inspect-robots run --embodiment yam_arena --policy dream -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "$I" --scorer task_score --max-steps 2400 --epochs 2 --store-frames --log-dir dream
python near_misses.py
```

Trial k cycles A/B. The20mm target shifts have overlapping acceptance regions; these runs check two declared placements and reset consistency, not proof that every fixed habit fails. `TASK_LOT=A` or `TASK_LOT=B` pins one layout. Both final evaluations must use two epochs and identical functional source. Predicates are evaluated at20Hz; behavior between observations is not a continuous-time guarantee. Record hardware video at least20frames/s and judge every visible sampled failure; report its observation rate. The final scripted set-down is3s and final still waypoint5s, leaving time for the one-second gate within the120s cap. Standard combined media show the first layout; per-epoch records establish the other layout.

Native scripted A=1.00, B=1.00, mean=1.000. Dream-1 A=0.00, B=0.00, mean=0.000 (completed, model-valid). Dream produced no object progress in either layout. Its adapter logged 98 expired and 4 pool_full prediction retries, then accepted 50 action chunks per layout with dry-run disabled; these retries are not evidence of a task refusal. Both evaluations used the same frozen functional source and instruction, 20Hz control, zero object jitter and a 2400-step limit per layout. Model-invalid or runtime-error attempts are unscored and are not averaged as numeric zero. Standard combined media show the first layout; per-epoch records retain the second. No hardware trial, force/material calibration, specimen qualification or preparation-time measurement has been performed.

Code/procedural assets: MIT. Documentation: CC BY 4.0. Maker images are linked research references and are not redistributed assets.
