# Fit the server security bezel and check its latch

Independent claim of [PR2367](https://github.com/robocurve/benchmark-tasks/pull/2367), TK-DZ2TW2; credit to the original task author. The latest review explicitly permits fabrication-ready custom hardware with actual push-to-latch operation. This is that custom training assembly, not the Lenovo mechanism quoted by the original task. It is not vendor accurate and does not claim dense-rack access or key-lock security.

Fit the security bezel and leave its latch checked with both hands clear.

## Geometry and setting

Coordinates below are millimetres from the midpoint between arm bases, x away from the arms, y left. Heights are above the747.5mm table. Native scene geometry is in task.py; the fabrication drawing and assembly specification are in fixture/parts.scad and fixture/ASSEMBLY.md. All specified dimensions, forces, masses and friction values are design settings, not hardware measurements.

The bezel is430 x44.45 x25mm, with a perforated front,8mm upper/lower rails and no added handles. Its350g target mass includes the8g sliding latch. It starts upright on its25mm-deep lower edge at x220,y0. Both grippers pinch the actual25mm top edge,325mm apart. The fitted root datum is(370,-2,66.675); the front face is x357.5. A40mm recess puts the chassis front at x410 behind the ear datum x370. Four physical rear stops limit inward motion to0.4mm. They are part of the specified counterpart, not a pose constraint.

The base has a200 x430mm through-opening under the lower chassis, so its floor remains at the table datum without overlapping a base plate. The12U open bench frame has front and rear channel columns at x370/610,y+/-232.55 and a560mm top crossbar. It contains one complete lower dummy chassis and the target second1U at44.45–88.9mm; tenU above are empty. Roofs, floors, side walls, fronts, rears, ears, posts and the top beam remain in the collision model and drawing. This is a partly populated service/training setting. A short7U front beam obstructed the native forearms, so that setting is not claimed feasible.

The specified materials are PETG for the frame, aluminium splice plates if split, PETG/aluminium receiver carriers with steel contact strips/keeper, steel rack and both dummy-chassis sheets, steel tabs/guides/stem/fasteners, spring-steel compression spring, POM nose and plastic yellow indicator.

At the right end, two5 x10 x5mm tabs lie at local(5,-220,+/-12). The receiver's fore/aft boundaries are x371.25 and380, vertical openings are8mm high at target z+/-12, and the side-opening channel runs from y=-218.5 to-233.5. The front and back walls prevent a straight outward escape. Approach with the bezel8degrees open, slide the tabs sideways into those real channels, then pivot the other end in. The slot dimensions allow the finite rotated tab profile; tab capture is physical contact, not a state-based lock.

At the opposite end, the POM nose has an actual45degree lead-in and a square retaining face. Pushing the bezel retracts the slider against its compression spring; after crossing the steel keeper, the spring returns it into the behind-catch window. No release is held during installation because this custom hardware is designed for push-to-latch. Pull the flag inward for removal/reset. The guide bore has0.15mm nominal clearance on each side; a stem cross-pin and two fixed side lugs physically define the0–8mm stroke. Their geometry is included in the CAD. The native prismatic joint idealizes this linear bearing, without commanded following, teleporting or a scripted lock.

Spring: spring-steel0.5mm wire,6mm outside diameter,8 active coils plus two closed/ground end coils,22mm free length. With mean diameter5.5mm and estimated shear modulus79GPa, k=G*d^4/(8*n*D^3)=0.464N/mm. The installed20.92mm length gives0.5N specified preload;8mm retraction reaches about4.21N. These are estimates, not a spring calibration. Verify free return and the retention check on the built assembly; adjust guide finish or reject the assembly if it does not return. Use the exact geometry and spring in every trial.

Frame/receiver friction0.35 static/0.25 dynamic represents dry plastic/metal contacts and is unmeasured. Stock rubber fingertip material supplies the other contact surface. The pose preflight used actual25mm jaw openings; the demo commands a23mm unloaded gap (-0.0115 per finger), with loaded openings and native efforts to be verified in an ordinary trace. No robot actuator changes are made.

## Observable engagement and retention

The yellow flag is visible through its front window. An extension reading within1mm of the outward mark is necessary but not sufficient. After the robot releases, attach a spring balance through an ordinary front vent at the centre and apply10N outward for1second. Observe both ends and measure the front at both ends/middle with a ruler or a2mm feeler gauge; no point may move more than2mm and neither end may escape. The evaluator performs this check; no special grasp handle is added. The balance and hook are evaluation tools, not part of the robot's task.

The simulation applies the same outward load to the actual free bezel using Isaac Lab's permanent wrench composer, ramping1N per50ms up to10N. The native physics-step observer reads the composer's force buffer in body coordinates, rotates it to world, and integrates the queried physics dt only while it contains10N outward (1mN numerical comparison allowance). It logs the applied vector, physics dt, native sample count and full 3D Euclidean face displacement. Each control observation also requires fresh native tick progression; an observer that stops updating fails closed. A contact-reaction view is diagnostic evidence only; no extra reaction-force tolerance is part of success. A missing force buffer, pose, callback or hand state fails closed. The ordinary trace must establish that this works; code alone is not load-test evidence.

The owning API states that permanent wrenches persist and are written each simulation step: https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.assets.html . The exact host API and recorded values must agree before a validated result is claimed.

## Scoring and limits

Every credited state is released and still for1second. All stock arm collider hulls, including forearms and elbows, must also stay over5mm clear of the bezel so the pull cannot be braced by an arm. Grippers and fingertips must additionally stay over5mm clear of the rack. All native collider hull faces are sampled at3mm, deduplicated and culled beyond50mm; conservative whole-part envelopes cannot hide a touching hand. Unknown transforms/data receive no credit. Native finger actuator and fingertip loads are freshly observed at every control step, with a40.01N numerical limit. There is no100N all-link measurement requirement.

0.25 is actual capture of both tabs. Capture checks every tab corner against x371.25..380mm and its z slot centred at66.675+/-12mm with4mm half-height, allowing100micrometres at these x/z boundaries. The most negative y corner must be at least-233.6mm (100micrometres at the outer back wall) and at most-223.5mm (at least5mm insertion past the-218.5mm mouth, no added allowance). This tab-contact tolerance is separate from the pose allowance below.0.5 additionally requires all six face samples, local x=-12.5mm,y=-200/0/+200mm,z=+/-18mm, within2mm fore/aft of x357.5, lateral root y=-2+/-3mm, root height66.675+/-2mm, and tilt from vertical at most3degrees. These limits are inclusive, with 1micrometre positional and 1microradian angular numerical allowance; the finite tab-slot clearance must independently pass and can be stricter.1.0 additionally requires the flag returned and the actual native1second pull passed with both ends captured and no more than2mm movement. A failed pull or overload scores0. Full seating is not inferred from proximity or latch appearance alone.

Rigid bodies omit printed-frame bending, bearing play and wear. Geometry and spring values define a prototype to build and check, not measured hardware or commercial-server performance. The proof load is intentionally small; it does not certify shipping, tamper resistance or vibration. Native endpoint/path IK is not a loaded simulation result. The demo uses only the visible fixed layout, reads no hidden state, and does not choose among interchangeable parts, so no layout lots are claimed.

## Run and reset

```
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 --instruction "Fit the security bezel and leave its latch checked with both hands clear." --scorer task_score --max-steps 1000
```

Use policy dream for one Dream-1 run with the same instruction and1000-step limit. The scripted path uses mirrored jaw orientations, controlled partial opening before lifting clear, then full opening. The required scripted demo scored1.0 and the ordinary Dream-1 baseline scored0 after1000steps (normal step-limit completion). Both used the identical runtime/CAD and fixed layout with object_jitter0. The logs report Isaac Lab environment seed42; no separate Inspect trial-seed claim is made.

The final demo's terminal native certificate at31.45s records0.9999999999999977s of full load,240 actual physics samples at queried dt0.004166666666666667s, and world force(-10.000001582,~0,~0)N. Maximum full3D face displacement was0.519964mm; maximum front-datum error1.285382mm; peak native finger/tip load8.226993N. These are simulation results, not measured hardware or force calibration. The demo finished2026-11-18T13:27:57Z; Dream finished13:32:04Z.

Development traces scored0,0,1.0. The final script uses a fixed4.5mm carry-height correction and a slow15mm inward command against the real rear stops, holds, then relieves push before opening. These waypoints accommodate observed native arm compliance; the specified fixture, mass, slots, cam and scoring stayed unchanged.

To reset, retract the latch by its visible flag, swing the left end out, slide both right tabs out, and return the bezel upright to its marked starting pose. Inspect guide clearance, flag return, tabs, keeper and stops before each trial. Keep all parts dry; use no adhesive, magnets or added handles. Isaac Lab environment seed42, fixed layout and object_jitter0.

Original task code MIT; documentation and generated custom CAD CC BY4.0. Platform robot assets remain under their supplied licences. No vendor scene assets or competitor code are included.
