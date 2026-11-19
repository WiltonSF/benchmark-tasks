# Serve potato salad with a squeeze disher

Serve two separate portions onto the plate, keep the loose bowl and plate upright in place, put the tool down and finish with both hands clear. The added manipulation is acquiring a sprung utensil and actuating its loaded geared wiper while retaining the grasp.

This package contains an Isaac Lab-Arena task for the bimanual YAM station. All reported execution is simulation or explicit CPU logic testing. No physical robot or food calibration has been performed.

The [USDA Food Buying Guide](https://foodbuyingguide.fns.usda.gov/Home/ResourceCenter) lists dishers as portion-control utensils for foods including salads. [O*NET restaurant-cook duties](https://www.onetonline.org/link/summary/35-2014.00) include portioning and serving food; [BLS May 2023](https://www.bls.gov/oes/2023/may/oes352021.htm) reported 879,610 food-preparation workers. These support a routine duty and occupation scale; the number using this exact utensil is unmeasured.

## Setup and mechanism

The generic disher is 239 mm long, at most 58.4 mm wide, and 155 g: fixed body/cup 123 g, sprung handle 24 g, wiper 8 g. Its 23.1 mm radius/depth cavity holds 25.816 mL. A native handle hinge moves 0–0.13 rad and drives the native wiper hinge through 180 degrees using a 24.17 mimic ratio. The estimated spring is 3 Nm/rad with a −0.33 rad rest target, giving approximately 0.99–1.38 Nm return torque. Native joint friction adds up to 40 mNm at the wiper. The tool and containers remain loose.

The [Vollrath catalogue, p117](https://www.vollrathfoodservice.com/downloads/104448/CAT2024SmallwaresCountertopEquipmentCatalogen-pdf) describes squeeze dishers for soft foods and lists a 25.9 mL #40. The [47378](https://www.vollrathfoodservice.com/products/smallwares/kitchen-essentials/dishers/extended-length-color-coded-squeeze-dishers/47378) provides length, width and mass references; this model is a generic reconstruction, not its exact CAD. The [OXO scoop](https://www.oxo.com/small-cookie-scoop.html) is another spring-handle/internal-wiper reference. Internal geometry, masses, gear, spring, friction and the estimated 6 mm handle contact-patch radius are not measured product calibration.

The source bowl is polypropylene, 150 mm inside diameter and 60 mm high, 180 g empty. The melamine plate is 170 mm across, with a 5 mm floor and 17 mm rim, 160 g. Their dimensions and masses are estimates. Coordinates, starting orientations and both visibly different layouts are in task.json. Each evaluation runs A then B and averages them. Nothing remains clamped or attached to a gripper.

## Food approximation and physical reset protocol

The conserved numerical source is 432.32 mL in 193 cells at 8 mm spacing, nominally 35 mm deep. Estimated density is 1050 kg/m³. Cavity overlap alone cannot load food: relative native displacement against the preceding material force pays an uncalibrated 2 kPa work density. Native forces include opposing bowl/tool contact loads, carried-food weight and COM-corrected moments.

The wiper must sweep occupied sectors. The actually applied native Coulomb-friction parameter over the preceding interval times absolute realized joint displacement estimates dissipation available for release at 1.2 kPa work density. Other DOFs and viscosity are preserved and parameters are read back. No explicit resisting/assisting wiper torque is applied. This is a dissipation estimate, not a measured constraint impulse or proof of complete system passivity.

Detached parcels start at their actual points/velocities and fall before being caught by the source or plate. Adhesion uses an estimated 50 ms mean magnitude of tool-origin effective acceleration, including gravity; exposure above 35 m/s² releases food. Empty-tool history can affect newly acquired food. Strong physical shaking remains allowed. Angular detachment, food chunks and detailed rheology are omitted. Direct hand-food dynamics are not simulated; a native-rate geometric contact veto makes such contact invalid for credit instead. A reachable native shake alternative is unproved. Constitutive updates run at 240 Hz with 20 Hz robot control.

For a proposed physical trial, use **Reser's Original Potato Salad**, without added water or recipe changes, from the same product lot. The [manufacturer](https://www.resers.com/product/original-potato-salad/) identifies the diced-potato mayonnaise product. Chill unopened food to 2–4°C; mix with ten gentle spoon folds and measure 432 ±5 mL into the clean bowl. Form an approximately 128 mm central footprint, about 35 ±2 mm deep, removing any measuring template before the trial. Record actual starting mass, depth, product lot and temperature. The initial mass is measured, not inferred from the simulator's density. This proposed source does not calibrate the material law; its chunks and packing differ from the aggregate model.

Use washable food-safe utensil/container surfaces and replaceable food-safe pad covers. Before every trial, remove all food from tool/plate, wash and dry food-contact surfaces, replace contaminated covers, and load a fresh measured source. Keep reserve food refrigerated and keep each prepared trial on the counter under 15 minutes. Follow [USDA cold-food handling guidance](https://ask.fsis.usda.gov/article/What-is-the-Danger-Zone). Discard food contacted by the robot; it is not served to people. These preparation and cleaning steps occur between trials.

## Outcome and history

Full credit requires two disconnected mounds of at least 14 mL each, at least 34 mL delivered in total, and no more than 3 mL spilled. Touching heaps count once. Safe partial credit is 0.5 for one mound of at least 14 mL with no more than 3 mL spilled. Numerical volume comparisons have a 1 µL margin.

Every credited portion must travel from the source to the plate **inside the disher cavity**, then leave the disher
above the plate. Direct gripper or fingertip contact with food anywhere during the trial makes the whole trial zero,
including reaching into the source, hand-carrying food, pushing a plated mound or touching spilled food. A tool used
only as scenery does not satisfy the task. Simulation material can enter the tool only through the existing physical
work/overlap loading rule and can reach the plate only through tool release and falling parcels. A post-physics
observer checks complete convex collision hulls of both grippers and all fingertips against source, carried, flying,
plated and recorded spilled material proxies at 240 Hz. Any contact is latched until reset; unavailable or nonfinite
hand/material geometry fails closed. Broad-phase bounds cull distant pairs; remaining pairs use complete convex
separation, including faces and edges. This veto adds no physical hand-transfer mechanism. The 8 mm cells, aggregate
cavity box and equal-volume parcel/spill cubes are conservative geometric proxies, not calibrated soft-food contact.

Both tiers require a contiguous final 2 seconds of actual table contact for all three props, supported upright geometry, no food in flight, still objects/hands and complete release. Hand collision faces are sampled at 3 mm; the 12 mm sampled threshold includes a conservative allowance exceeding the required 5 mm separation. Other robot links use full convex separation. Material, joint, link or prop changes invalidate the current certificate; a tier change restarts the hold. Final support uprightness is tighter than the history limit: vertical cosine above 0.995.

The native **post-physics** primary history permanently latches bowl/plate excursion above 30 mm, tilt above
10 degrees, or forbidden direct hand-food contact. Missing/nonfinite primary geometry invalidates the trial.
Position has a declared 1 µm numerical margin; cosine comparisons use 1e−9. Material and primary-history counters
must advance together. Final support/release/stillness is sampled at 20 Hz. **There is no per-finger force threshold
or force-sensor availability gate in either simulated or real primary scoring.**

### Independent robot safety

At reset, task-local code caps the existing native finger drive efforts at 40 N (or retains a lower existing cap),
reads the limits back and preserves every other joint limit. Failure to configure/read back that cap prevents starting
a native trial. This is actuator saturation, not a measured-contact-force grading rule; collisions can still produce
higher contact transients. Projected efforts and net fingertip forces are recorded by a separate safety observer.
Its overload or unavailable-monitor alarms never set primary-history validity or alter the primary score. Operators
must stop a native run independently on a safety alarm and record the abort; it consumes a call and cannot serve as
successful validation evidence. Net contact is a resultant, not stress or a calibrated hardware safety guarantee.

For a physical trial, configure the robot's approved force/speed limits, safe stop and emergency-stop system before
motion; use an operator and the manufacturer's safety procedure independently of grading. No 40.01 N whole-trial
measurement is demanded by the score. The proposed simulated 40 N drive setting is not a substitute for physical
robot safety approval or calibration.

For proposed physical grading, the fixed mass thresholds are 14.7 g per mound, 35.7 g total and 3.15 g spill on a 0.1 g scale, with continuous video checks for motion, support, release and tool-carried transfer. Use views showing the source,
disher cavity on withdrawal, travel to the plate, release, and both grippers throughout. Every credited mound must
be visibly delivered from the disher. No direct gripper/fingertip-food contact is allowed, even if the final mass is
correct; direct hand delivery earns zero for the whole trial. Occlusion that prevents verifying the route earns no
credit for that portion. Arrange cameras before the trial; do not infer the route from final photos. These are declared physical grading thresholds derived from the proxy's nominal density, not a claim about the commercial food's exact density.

## Reproduce and validate

Use the installed YAM/Inspect Robots environment and the task folder with all helper packages. From the checkout root:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/serve-the-potato-salad-with-the-squeeze-disher/task.py \
  -E object_jitter=0 -E control_hz=20 --epochs 2 \
  --max-steps 1200 --scorer task_score \
  --instruction 'Serve two portions of potato salad onto the plate using the squeeze disher. Keep the bowl and plate upright in place. Put the disher down and finish with both hands clear.'
```

Run Dream with the same command and `--policy dream`. Optional `TRACE_OUT=/absolute/existing/directory/trace.jsonl` enables task diagnostic JSONL only; it supplies no extra policy observation and changes no scoring or physics. The factory invocation omitted `--seed`; its native environment log reported seed42. Explicit layout resets and zero object jitter determine A then B. The limit is 60 seconds per layout; fixed expert waypoints last 57.5 seconds. The demo chooses a frozen plan using the visibly displaced initial tool X only, without reading runtime food, force, joint state or exact pose feedback. Initial camera buffers are refreshed after food-display creation.

Portable CPU controls, run separately in the same Python environment:

```sh
python tasks/serve-the-potato-salad-with-the-squeeze-disher/controls/scorer.py
python tasks/serve-the-potato-salad-with-the-squeeze-disher/controls/native_api.py
python tasks/serve-the-potato-salad-with-the-squeeze-disher/controls/material.py
python tasks/serve-the-potato-salad-with-the-squeeze-disher/controls/inertia.py
```

Native sensors, hand poses, contacts, drive-cap writes/readback and friction application are explicitly stubbed in CPU controls. They test logic, boundaries, history, reset, conservation and prescribed material responses, not robot success or food calibration.

### Revised native evidence

Both runs below used the identical revised runtime (hashes in `EVIDENCE.json`, `revision_native_evidence`), 1200
control steps per layout, layouts A then B.

The scripted expert demo scored 1.0 in both layouts: A finished at step 1135 and B at step 1136, each after the full
2 s final hold. A served 38.762 mL with 0.533 mL spilled; B served 38.983 mL with 0.495 mL spilled. All 454
diagnostic rows record no hand-food contact, no safety alarm, a 40 N drive cap read back on all four fingers, and
matched native, history and safety tick counts. Peak finger contact was 38.217 N in A and 37.510 N in B, below the cap.
End images show two separate mounds, the disher down, bowl and plate upright, and both hands clear.

The matching Dream-1 run scored 0.0 in both layouts at the 1200-step limit, with zero errored trials. It never grasped
the disher; no food was captured, served or spilled. All 480 diagnostic rows record no hand-food contact, no safety
alarm and matched tick counts.

### Historical evidence before the TK-ZJVSYF revision

The following runs used the original runtime, before the new native hand-food history and independent 40 N drive
cap. They remain historical evidence only and do not validate the revised runtime. Original 15 calls, including
failures/aborts, remain recorded. Two further calls validated the revision (above).

The original exact guarded expert completed 1.0 in both layouts. A served 38.762 mL with 0.533 mL spilled; B served 38.983 mL with 0.495 mL spilled. Native projected effort/contact peaks were 38.525/38.217 N in A and 38.088/37.510 N in B, with matched advancing material/history counters and no recorded violations or API errors. Native end images show separate mounds, tool down and hands clear. Dream14 was interrupted when the external host stopped; no completed score is claimed. The replacement matching Dream15 completed both 60 second layouts at the step limit with score0.0 in each. All480 recorded diagnostic rows had matched advancing material/history counters, no invalid flag and no recorded API error; no food was captured or served. The Dream client retried 48 session-expiry responses; each layout ultimately received 25 successful 48-row action responses and ended at the step limit, with zero errored trials. These retries are distinct from the native physics API checks. `EVIDENCE.json` records the exact runtime/control hashes, completed expert results and media hashes; the completed Dream results and its media hash are included.

Code and procedural geometry: MIT. Documentation: CC BY 4.0. Manufacturer images/CAD are not redistributed. The installed organizer harness is unchanged.

## Revision of TK-ZJVSYF

The reviewer asked for video-verifiable disher use, no direct gripper-food transfer, and removal of the unmeasurable
40.01 N primary force gate. The real protocol now checks the entire tool-carried route and bans direct hand-food
contact. Simulation adds a native-rate irreversible contact veto. Primary score/history no longer reads force
thresholds or force availability. Separate actuator saturation and safety telemetry remain operational controls.
Physical dimensions, food loading/release mechanics, published layouts and frozen expert waypoints are unchanged.
Because scoring and the operational drive cap changed, the expert and Dream-1 runs and the media were redone on the
revised runtime (see "Revised native evidence"). No physical robot or food calibration has been performed.
