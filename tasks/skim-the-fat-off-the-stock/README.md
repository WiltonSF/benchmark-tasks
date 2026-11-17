# Skim the fat off stock

A room-temperature water/oil analogue tests selective floating-layer removal with a real-size ladle. No heat or raw food is involved. This is a rigid simulation with material proxies; no measured robot or physical calibration result is claimed.

## Run

Run both published starts. For Dream-1 use the identical command and instruction, replacing `--policy yam_scripted` with `--policy dream` and camera dimensions with `-E cam_width=320 -E cam_height=176`:

```sh
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 --policy yam_scripted --scorer task_score --max-steps 3300 --epochs 2 --instruction "Remove at least 70% of the fat from the stock into the waste cup, taking no more than one part broth per four parts fat. Spill no more than 2 mL. Keep both vessels upright in their starting places throughout. Finish with the ladle on its rest and both hands clear."
```

`TASK_LOT=A` or `TASK_LOT=B` pins one start; otherwise trialk uses layoutk modulo2. The screenshot and videos showA. Reported evidence must cover both starts. The demo reads only the published visible starting reference position; it does not read hidden joint, friction, score or fluid state. Its side grasp pinches the handle top/bottom, resisting pitch through geometry, and its leaned approach clears the pan rim. A straight approach struck the rim in development and did not skim.

## Physical setup and grading

Use the dimensions, materials and exact positions in task.json. The reference on the molded grip is a cross-section32.5mm from its bowlward end, not the grip's mass centre. Floor and wall thicknesses are conservative collision envelopes for thin metal. Nominal mass, COM and inertia are not calibrated; the simulated ladle remains100g and fluid inventory does not change body inertia.

For a room-temperature bench version use650mL water with food-safe brown colouring and160mL yellow vegetable oil. Let them separate before the trial. The protected physical bench, runoff lip and secured robot covers are specified in task.json. The mat and lip collect runoff; nothing is returned during grading. These safety accessories are omitted from simulation, and table-relative heights are measured from the protected working surface. Measure the phases in each vessel after the trial by letting them settle and transferring them to graduated cylinders with1mL divisions. Measure small runoff with a graduated2mL syringe. Liquid still inside the ladle is retained; liquid on its exterior or the rest counts as spill. The proxy assumes a3mm stream radius, conservatively shrinking each receiving aperture by3mm. If measurement uncertainty could change a threshold decision, give no full credit. The waste must contain at least112mL oil and at most one-quarter as much water as oil; spill at most2mL. These are benchmark design thresholds, not culinary standards.

Mark both vessels' starting outlines and use overhead/side video plus a ruler/inclinometer to check their5mm/2.56degree whole-trial limits. Use side/overhead views to confirm the ladle bears on both rest supports, the cup stands on the table, all hands are more than5mm away, and everything is still for2s. A pose with the same bottom height but no supporting contact fails. Simulation requires an upward SDK contact signal of at least 0.75 total, with at least 0.02 on a handle bridge and the bowl pad, and rejects support from other arm links. These are raw SDK signal thresholds, not calibrated newtons. A diagnostic found about 4.91 reported units under the measured 100 g simulated ladle; its absolute force scale is unverified. No division or invented conversion is applied. Actual collider-specific contacts, rest geometry, stillness and full arm clearance jointly certify support; zero or unknown data gives no credit. For an instrumented physical bench, support the bridge and bowl pad on independent force sensors, compensating thickness to preserve the published heights. Require positive load above measurement uncertainty on both supports, and total upward load consistent with the wet utensil's independently measured weight. Unknown support fails. This describes a physical verification protocol; no physical sensor result is claimed. The same safe completion gate applies to partial credit. task.json lists the exact simulator thresholds and tiers.

## Material proxy and checks

Fat occupies equal-area surface cells; conservative relaxation redistributes it in0.4s. Rim inflow integrates the fat and broth depth bands separately and subtracts the ladle's internal head. The finite curved cavity and its lowest rim bound retention; there is no unlimited pumping. The proxy assumes instantaneous separation inside the ladle and fat-first outflow; a controlled deep dip, partial decant and return can therefore also be valid. An unseparated deep scoop fails purity, but depth alone is not forbidden. Rotation makes excess leave at the lowest rim; gravity and vessel apertures route it to the pan, cup or spill inventory. The cup also loses liquid if tipped. Each phase is conserved independently; state or timing gaps invalidate the trial.

The liquid surfaces are visual meshes, not colliders or CFD. The ladle display intersects a horizontal free surface with the curved cavity; the pan field is continuous. No emulsification, viscosity calibration, slosh or heat is modeled.

Release samples every actual gripper/fingertip hull face at about3mm, caches that mesh, and merges samples in2mm voxels. A7mm conservative allowance covers both discretizations when certifying the5mm physical gap. Shape OBBs further err toward rejection. Regressions include a125mm face0.5mm from the target with no hull vertex close, unknown state, hovering/deep scoops, finite filling, conservation, dropped/airborne tools and false rest support.

```sh
python near_misses.py
```

Final Isaac Lab-Arena runs used20 Hz and the identical instruction on both starts. Scripted scores were1.0/1.0 (mean1.0), completing3006/3007 actions under the3300-action cap. Dream-1 scored0/0 (mean0.0), reaching the cap on both starts; no refusal occurred. LotA delivered119.341 mL fat and6.263 mL broth; B119.183 mL fat and6.837 mL broth. Both had zero modeled spill and completed the supported, still, released gate. These are simulation/proxy results, not measured robot results.

The waste cup renders its aggregate inventory dark in this build; displayed cup colour is not a purity indicator. Conserved independent phase volumes determine purity. A deliberate fixedA grasp on physical startB left the ladle on its rest and transferred nothing during a260-step grasp check. This is a short replay falsification, not a full-duration replay result.

The final task and helper hashes matched their executed copies; task.py predates both final runs. Independent reviews inspect the submitted setup, videos and per-lot results.

## References

[CIA Basic Culinary Training, PDF page26](https://www.ciaconsulting.org/wp-content/uploads/2025/06/Day-1-Safety-Mise-en-Place-Knife-Skills-Stocks-CP.pdf) teaches stock degreasing. [Winco LE-2](https://www.wincous.com/products/le-2) provides a70mm,60mL bowl reference; [Vollrath58620](https://www.vollrathfoodservice.com/products/smallwares/kitchen-essentials/ladles/stainless-steel-ladles/58620) provides a real ladle mass reference. This generic molded-grip utensil is not an exact copy of those products. All unsourced dimensions and coefficients are estimates.
