# Disperse a premeasured scoop of thickener into a 200 mL drink without lumps

Care aides prepare drinks with powdered thickeners during meal assistance. The benchmark measures delivery and observable dispersion of a supplied dose and the existing tool put-away goals. It does not certify a prescribed consistency, an IDDSI level, complete hydration or suitability to serve. Flow readings are separate diagnostics.

## Run

    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=tasks/thicken-the-drink-to-its-prescribed-consistency-without-lumps/task.py -E object_jitter=0 \
      -E control_hz=20 \
      --instruction "Thicken the drink with the thickener in the scoop so it has no lumps, then put the whisk back in its cup and the scoop down on the mat." \
      --scorer task_score --max-steps 2400 --epochs 3 --store-frames

Use `--policy dream` for Dream-1. `--epochs 3` runs lots A, B and C (trial k uses lot k mod 3) and `task_score` is
their mean; `TASK_LOT=A|B|C` in the environment pins every trial to one lot, and `TASK_DEBUG=1` prints the powder's
state every half second. Submissions run with `-E object_jitter=0`. A trial runs at most 2,400 control steps
(120 s). The revised scripted runs scored 1.0 in A/B/C at 1,848 / 1,847 / 1,848 steps (92.40 / 92.35 / 92.40 s). The first control step of a run also samples the gripper's collision
hulls for the hands-clear check, which takes about 40 s of wall time once.

Files: `task.py` (scene, powder model, scoring, demo), `parts.py` (multi-shape parts, visual-only meshes), `lots.py`
(layout lots), `tilt.py` (tilted grips for the scripted demo only), `near_misses.py` (offline checks of the scorer
and the powder model). All are plain Python on the official yam-isaac-lab-arena (the program's YAM embodiment for
Isaac Lab-Arena) stack; nothing on disk is patched. `parts.py`, `lots.py` and `tilt.py` patch the embodiment and the
demo's grip rotation in the task's own process only.

## Objects (true size)

| Object | Size | Mass | Notes |
|---|---|---|---|
| Placemat, navy | 45 x 60 cm, 2 mm | fixed | centred 30 cm forward on the centre line |
| Coaster, cork | 9 cm across, 4 mm | 6 g | the tumbler's mark |
| Tumbler, clear polycarbonate, 300 mL | 8.6 cm across (8.0 inside), 6.5 cm tall, 5 mm base | 91 g estimated + 200 g juice | 200 mL apple juice, 4.0 cm deep |
| Utensil cup, white ceramic | 5.2 cm across, 3.6 cm bore, 6.0 cm tall, 8 mm base | 200 g | holds the whisk |
| Mini whisk | 16 cm: steel wire head 6 cm long, 2.6 cm across at most; red handle 1.6 cm across, 10 cm long | 30 g | stands head down in the cup |
| Flour scoop, red polypropylene | flat floor 8 x 6 cm, open front, back wall 4.5 cm, side walls 4.5 cm at the back falling to 2.0 cm at the front, 2 mm walls; hollow handle (an inverted U, 1.4 x 1.4 cm) 10 cm long, its top level with the back wall's top | 30 g | lies flat, open front toward the robot |
| Thickener, Thick-It Original (starch) | 30 mL = 2 level tablespoons (lot A), 45 mL = 3 (lots B, C) | | in the scoop against its back wall |

Use **Mott's 100% Original Apple Juice, 64 fl oz bottle**, the clear pulp-free juice from concentrate, and **Thick-It Original Food & Beverage Thickener**, not Original Concentrated or Clear Advantage. Record both labels, batch/expiry identifiers and ingredients; use the same formulations and batches across a comparison. No cider, cloudy juice or altered formula is substituted. Measure 200.0 mL juice with a graduated cylinder. Juice, powder and utensils start at **20-22 degrees C**, checked with a thermometer; record starting and final temperatures. This is a chosen benchmark control.

Lots A/B/C use 30/45/45 mL of leveled powder measured with a 15 mL tablespoon. These are benchmark doses, not a claim that either volume produces a prescribed clinical consistency. Weigh each actual dose; do not assume a grams-per-tablespoon conversion.

A removable dry waterproof catch liner,600 x750 mm centred at x300,y0 mm, surrounds the existing mat. Powder outside it makes accounting ungradable. Secure impervious covers over robot bases/electronics outside arm paths; protect exposed pads if a wet tool can transfer liquid. Inspect/clean covers and pads between trials and replace contaminated or damaged covers. These physical protective accessories are omitted from simulation and preserve the published surface heights.

## Lots

Positions are of centres, in cm forward from the line between the arm bases (x) and to the left of their midpoint
(y).

| Lot | Tumbler and coaster | Utensil cup and whisk | Scoop's front edge (middle) | Dose |
|---|---|---|---|---|
| A (screenshot) | x 22, y -3 | x 30, y -18 | x 27, y +10 | 30 mL |
| B | x 22, y +3 | x 30, y +18 | x 27, y -10 | 45 mL |
| C | x 19, y +4 | x 30, y -18 | x 27, y +10 | 45 mL |

Lot B is lot A's mirror image, so the hands swap jobs. Lot C moves the drink 7.6 cm from lot A's place: a replay of
lot A's motions pours the thickener onto the mat and drives the whisk into the tumbler's rim. Lots stop that fixed
habit, not a memorised replay of all three layouts. The demo is an open-loop recipe using the published visible
positions and yaw from the layout helper. It reads no live object state, hidden dose or material inventory; it does
not demonstrate visual inference or adaptation to an unseen layout.

## The powder model (what is simulated)

The simulator is rigid, so the thickener and the juice are tracked volumes moved by `step_powder()` each control
step (20 Hz). Every rule:

1. **The scoop.** Its inside is cut into 2.5 mm cubes. It keeps the powder that lies below a plane through the
   lowest point of its rim (the open front's edge, in any normal pour), rising at the powder's angle of repose
   (40 degrees) toward the scoop's inside. Anything above that plane runs out over that point at up to 200 mL/s, so
   a scoop tipped over empties in a fraction of a second. The scoop holds 112 mL level and nothing once its floor is
   tipped 40 degrees; tipping it slowly from about 25 to 40 degrees pours about 3 mL per degree.
2. **Where it lands.** Powder falls plumb from that rim point. It lands on the drink if that point is above the
   juice and at least 3 mm inside the tumbler's inner wall, with the tumbler leaning under 20 degrees; otherwise it
   is spilled (counted, and drawn as cream spots on the mat on a 4 cm grid).
3. **The drink** is 7 regions of equal area (a centre disc and 6 sectors), each 28.6 mL of juice holding dry
   powder (floating, not yet wetted) and wetted powder (dispersed and taking up water).
4. **Stirring** is measured from the whisk's motion, not named: 30 points on its head (6 heights, 5 points across
   each) are tracked relative to the tumbler over a sliding 0.25 s window. A point's speed is the distance it
   travelled across the whisk's axis in that window divided by 0.25 s, at most 0.2 m/s, and it counts only if it got
   at least 3 mm from where it was at the window's start (a tremor or jiggle is not a stroke). Each point under the
   juice sweeps (its share of the head's width) x (1 cm) x (that speed) per second, shared equally among the regions
   whose centres are within 3 cm of it. A region's **local stirring** g is the swept volume it gets per second
   divided by its volume. The **swirl** follows the whole head's swept volume / 200 mL with a 1.5 s lag.
5. **Wetting** is done by the whisk itself: each step, dry powder in a region is wetted at the rate 40 x g (per
   second), but at most 5 mL/s in any one region, however hard it is stirred. The swirl left behind after the whisk
   stops wets nothing.
6. **Lumps.** Dry powder above 1.5 mL in one region (a layer over 2 mm deep on 7.2 square cm) becomes lumps at once.
   Dry powder in a region whose local stirring stays under 0.02 per second for 3 s also becomes lumps. Lumps never
   break up or dissolve (stricter than real starch, where hard whisking breaks some up). Together with the 5 mL/s
   cap, powder landing faster than one region can take it up lumps whatever the whisk does.
7. **Evening out.** Wetted powder moves between regions toward their mean at the rate 2 x (g + swirl) per second,
   so a drink stirred only briefly stays patchy.
8. **Juice.** The juice is part of the tumbler's mass. It spills if the tumbler ever leans more than 25 degrees (a
   level surface 2.0 cm below a rim 4.0 cm out reaches the rim at 26.8 degrees). Splashing is not modelled; whisk
   speeds above 0.2 m/s earn no more stirring.

The rates and thresholds (repose angle, 200 mL/s, 0.25 s and 3 mm, 0.2 m/s, 3 cm, 40, 5 mL/s, 1.5 mL, 0.02 and 3 s,
2, 1.5 s) are chosen to give the behaviour described, not measured. The powder's weight is not applied to the scoop; its physical dose mass must be weighed for every trial.

Drawing: the powder in the scoop as a cream slab (9 sizes), a falling stream where it lands, each region of the
juice in 4 shades from clear amber to cloudy by how much wetted powder it holds against an even share, a cream
patch for floating dry powder, white beads for lumps, and cream spots on the mat for spilled powder.

## Scoring and physical measurements

The revised primary score uses **delivery**, not a percentage wetted/dissolved. The quality screen has a declared 2 mm observable-defect scale; it is not a clinical texture standard.

Use a checked scale with 0.01 g display resolution. Record the empty dry scoop, loaded scoop, dry scoop residue and all dry off-cup powder, using tared recovery containers. Let M be initial powder mass, R dry scoop residue, and S all dry powder recovered outside the drink, including the liner, mat, coaster, holder, cup exterior and other surfaces. **D=(M-R-S)/M**. Powder on a tool or outside the drink must not be silently credited. All mass differences use the same scale and a documented conservative uncertainty u, at least 0.02 g per weighing or the larger error observed with a reference mass. Propagate individual weighing intervals through the mass differences: D_low=1-(R_high+S_high)/M_low and spill_high=S_high/M_low. Award 50% or 95% only when D_low reaches that boundary; permit a spill only when spill_high is strictly below 5%. A straddled delivery boundary earns the supported lower tier; a straddled safety/validity gate earns zero. M_low must be positive.

Wet residue or wet recovered powder cannot be weighed as dry thickener. Incomplete recovery, unaccounted material transfer, liquid-contaminated scoop residue, an obscured quality examination or an unmeasurable amount of undispersed powder on utensils makes the trial ungradable and scores zero. Do not invent a drying or juice-solids correction.

At the scored end state, film/photograph the undisturbed drink from above and through its wall, with a millimetre reference and sufficient magnification to resolve 2 mm. A dry-looking island/raft or exposed dry core with smallest visible width at least 2 mm fails. Before any destructive examination, record tool placement/release and the hold. After diagnostic sampling (if performed), inspect the bottom/wall with a spoon, then pour the entire drink, each diagnostic aliquot and its effluent through the **same flat stainless woven sieve with 2.0 mm clear square apertures**. Gravity-drain for **60 s**. Do not rinse, press, rub, shake or crush retained material. Any discrete thickener agglomerate retained fails. Photograph the residue with a millimetre reference; continuous liquid coating alone is not an agglomerate, and an obscured mesh is ungradable. The texture screen is final-state examination. It does not prove a lump never existed earlier; the model's irreversible lump rule is a disclosed approximation. Juice/powder spills remain whole-trial checks from video and collection.

Apply the common validity, quality and released/support/stillness gates to **every** positive tier, including partial-dose trials: any juice spill, at least 5% off-cup powder, a dry defect or a sieve-retained agglomerate gives zero. Then award the highest supported tier:

| Score | Delivery and completion |
|---|---|
| 0 | A common gate fails or D_low is below 50% |
| 0.25 | D_low at least 50% |
| 0.5 | D_low at least 95% |
| 0.75 | D_low at least 95% and either tool correctly put away, with the release/stillness hold below |
| 1.0 | D_low at least 95% and both tools correctly put away, with that hold |

For **every positive tier**, require **1 s** with the tumbler within 5 degrees upright, its centre within 15 mm horizontally and 3 mm vertically of its start; tumbler, whisk, scoop, holder and coaster below 10 mm/s and 0.5 rad/s; all gripper/fingertip faces more than 5 mm from those five. A correctly stowed whisk has its lowest collision point 0-3 mm above the holder floor, inside its 18 mm bore radius, and lean at most 15 degrees. A correctly stowed scoop has its complete footprint inside the 45 x 60 cm mat, underside within 3 mm above its top, and lean at most 10 degrees. The holder and coaster must themselves rest on the mat; moving a holder and whisk together into the air is invalid. The tumbler bottom must bear on its coaster. A non-stowed whisk/scoop may earn a material-progress tier only after release on a declared support: the whisk on the holder/tumbler floor or tabletop/mat, the scoop on tabletop/mat. No tool or support may be held or airborne. Support tests allow at most 0.25 mm numerical contact penetration (the first native trace showed approximately 0.1 mm at the cup/coaster); deeper embedding fails. Use side/overhead video and a ruler; a held or airborne tool is not put away. A/B/C are judged separately and averaged.

### Simulator correspondence

The simulator independently conserves scoop, spill, dry, dispersed and lump inventories; missing, nonfinite, negative or inconsistent accounting gives zero. Delivery is (dose-scoop-spill)/dose. Exact simulated values have a 1e-6 comparison allowance, not a claim of zero physical weighing uncertainty. A dry region is drawn as a 2 mm island once its inventory reaches the chosen 1 mm-deep circular-patch amount (0.00314159 mL); this visible marker is the scored dry defect. Any nonnegligible modeled lump inventory draws an agglomerate and fails the coarse sieve proxy. These marker/response rules are estimates, not calibrated particle hydration or a simulated sieve. The former 0.05 mL lump-total rule, 50%/95% wetted tests, dry-percentage test and concentration-uniformity test are not score predicates. Internal wetting/mixing dynamics still make dumping, no-whisk pouring and late whisking form defects.

Wet scoop/juice contact and powder leaving the recoverable liner invalidate the modeled weighing protocol. No drying correction is assumed. Both arms must supply all gripper and fingertip states with finite geometry. Unknown state fails closed. Stowage hold is 20 consecutive control steps, reset when its conditions fail. Powder history resets only on the real trial-reset callback. A hold certificate must match the current object states; it cannot be reused after movement or missing data.

### Optional flow diagnostic: zero effect on primary score

After the last robot stirring, leave the drink undisturbed for **60 s**. Before spoon/sieve examination, take original unsieved samples at mid-depth from the centre and from 5 mm inside the wall on the +x side. Record sample/draw/test times, temperatures and locations. Use clean dry 10 mL slip-tip syringes conforming to the IDDSI 61.5 mm zero-to-10 mL scale length, fill to 10 mL, and record the volume remaining after 10 s of gravity flow. Repeat each location with a fresh sample/syringe and retain every sample/effluent for the texture examination. Report individual numerical readings and any blocked/abrupt/no flow. Do not classify no flow as level 3 or infer an even drink from two broad bands.

**Every flow result, including a boundary, discordant, blocked or no-flow result, changes the primary score by zero.** It never upgrades a delivery tier, overrides a quality failure or establishes a wetted percentage. No primary benchmark score certifies a care-plan outcome or a drink for serving. A scored consistency claim would require a separately validated physical recipe/test relationship; none is claimed here.

## Demo

The hand on the utensil cup's side lifts the whisk out by its handle, leans it 20 degrees away from the scoop's side
and stirs in an ellipse 2.4 cm across and 0.8 cm deep, once a second, near the bottom of the juice and a little
toward where the thickener will land. The other hand pinches the scoop's handle 5 cm behind the bowl, lifts it,
carries it over the tumbler already tipped 18 degrees, and tips it on to 36 degrees over 34 s and to 42 degrees over
a further 24 s (the last layer on the floor slides off near 39 degrees), with its front edge held 2.8 cm from the
tumbler's centre and 1 cm above the rim, so the thickener runs in at about 1 to 2 mL/s. The whisk
goes on for about 20 s after the scoop is empty. The scoop goes back down on the mat where it started and the whisk
back in its cup, and both hands let go. The demo uses `tilt.py` for the leaning grips.

Run `python near_misses.py` for delivery boundaries, visible dry/lump defects, conservative accounting, every tool stowage combination, unknown/missing-arm data and the existing pour/whisk/replay controls. The revised native demo scored 1.0 / 1.0 / 1.0 in A/B/C, with success termination at 1,848 / 1,847 / 1,848 steps. The matching Dream run used `dreamlabs/dream-1-beta` and scored 0.0 / 0.0 / 0.0, reaching 2,400 steps in each lot. Both used 20 Hz, the published instruction, zero object jitter and identical task.py/helper hashes; there were zero run errors or abstentions. The media package shows lot A; the three per-lot records and camera streams were checked separately. These are simulation results, not measured hardware outcomes.

## Known limits

- The powder and juice are a model with stated rules, not a fluid or granular simulation. Thickening over time
  (viscosity rising) is not modelled; wetting/mixing are internal response dynamics, while the score reads delivered amount, visible defects and tool put-away.
- Only the whisk stirs; the scoop or a finger dipped in the juice does not.
- The tumbler and utensil cup are 20- and 16-sided; the whisk's head is a solid convex body, not wires.

## Closest tasks

- TK-AUSTZF (a lab polymer stock sprinkled into a stirred beaker) poses the same physical problem in a specialist
  setting and was closed for importance and scoring, not as a duplicate. This task is a core duty of a large care
  occupation, gives partial credit for measured delivery and the existing tool put-away goals. Its 2 mm observable-defect screen is a coarse physical counterpart to disclosed material proxies, and the flow test is diagnostic only.
- Comparison names in BEHAVIOR-1K are `make_instant_coffee`, `make_hot_cocoa`, `making_a_drink` and `mixing_drinks`. This benchmark grades delivered dose, the specified dry/lump examination and tool put-away; abrupt addition fails under its disclosed irreversible-lump model.


Background mixing guide: [Thick-It Original guide](https://www.vitalitymedical.com/pub/pdf/guide-thick-it-original.pdf). The 30 / 45 mL benchmark doses are chosen values and do not establish a prescribed consistency.

Sources for the controlled products and diagnostic method: [Mott's Original Apple Juice](https://www.motts.com/products/juice/motts-100-original-apple-juice), [Thick-It preparation instructions](https://thickit.com/education/how-to-use/), and [IDDSI testing methods](https://www.iddsi.org/images/Publications-Resources/DetailedDefnTestMethods/English/V2TestingMethodsEnglish31july2019.pdf). Benchmark thresholds and the room-temperature range are design choices, not clinical validation.

The tumbler shell mass is an estimate, not a measured product: pi*(43^2*65 - 40^2*60) = 75,979 mm^3, about 91 g at an assumed polycarbonate density of 1.20 g/cm^3. The 20-sided collision hull approximates that cylindrical shell. [SABIC LEXAN sheet properties](https://ff.sabic.eu/uploads/resources/SABIC-SFS-6211-EN%20LEXAN%20Glazing%20Guide%20-%20rebrand%202021.pdf) supplies the density reference; it does not identify or validate a specific tumbler. Record the actual vessel mass when reproducing physically.
