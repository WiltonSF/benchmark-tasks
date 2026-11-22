# Lower the Toyota Yaris parking brake using its button and an extension grip

Status: NEEDS-MORE. Demo-only revision0.10.2 reached native normal A=1.0, B=1.0 after lot B trace1.0. Dream-1 completed A=0, B=0 on the identical frozen runtime and instruction, with both trials reaching2400 steps and no abstentions. A button-return limit projection remains unresolved. No measured robot result or physical qualification is claimed; constructed checks are separate from native demonstrations.

Drivers release a mechanical parking brake before driving. A full-size Toyota Yaris46201-52080-B0 lever and46235-52020 bracket sit between installed seat and console access envelopes. One complete gripper supports the loaded lever through an extension while the other depresses its original axial button. The extension provides leverage; it has no button linkage, assisting spring or powered motion.

## Credit and original feedback

Idea credit: [PR3163](https://github.com/robocurve/benchmark-tasks/pull/3163), TK-4JYPSE. The mechanism, judge, fixtures and demo are independently written using the task kit. No original task code is copied.

Last verdict, November 20:
> Construct validity (not met): The open bench fixture omits the seats and console that constrain access, while the specified spring deliberately applies only a fraction of real cable load.

> What to change: Recreate the handbrake's installed surroundings, including representative seat and console constraints, rather than an open bench fixture. Specify and substantiate a representative cable load for the applied brake, reproduce approximately that load in the real fixture and simulation, and demonstrate release within the run limit with the specified arms. Verify that the full-score demo ends with both grippers at least 3 cm clear for one second.

No separate Flags section was present. Importance was accepted; construct validity and feasibility were revisable.

| Feedback or pre-review objection | Response and remaining limitation |
|---|---|
| Include installed surroundings | Full rigid inner seat envelopes90x560x150 mm,220 mm clear gap,80 mm console slot, and an anchored floor board are present. They retain their geometry under a rigid fixture yaw/translation. Dimensions are explicit estimates rather than measured OEM cabin surfaces. |
| Substantiate representative cable load | Toyota's200 N application-force check at an estimated250 mm handle radius motivates the50 N m retained-load estimate. Application force is not a release measurement. The distinction remains explicit; the curve requires real enrollment. |
| Reproduce load physically and in simulation | The unchanged curve3+112.2 theta(rad) N m acts continuously as opposing joint effort, with bearing reaction. A five-spring98 N/mm cartridge and33.8363 mm cable radius reproduce it on paper. Spring travel and cable margin defects are corrected; physical qualification remains required. |
| Demonstrate release using the specified arms | The earlier36.28 N contact at220 mm could supply at most7.98 N m versus50 N m. The revised kit adds a named200 mm extension and keyed adapter without reducing cable load or increasing YAM effort limits. The finishing-target revision now reaches1.0 in both normal lots; the button-return proxy and physical qualification remain open. |
| Both hands clear for one second | The full housing/finger surfaces are checked with a35 mm sampled clearance guard for a30 mm physical minimum, continuously for1 second on a supported ratchet or bottom stop. No held or falling state earns credit. |
| Scorer role and bypass gaps | Roles are symmetric; actual contact moments,5 N axial button force, cable effort, pose/travel integrity and non-hand robot contact are observed at240 Hz. Missing samples permanently invalidate. A released button may settle to its next tooth. Material fracture is not modeled. |

## Products, dimensions and cost

The [Toyota dealer listing](https://wholesaledirect.moderntoyota.com/oem-parts/toyota-parking-brake-control-4620152080b0) identifies the genuine Yaris lever. [Toyota's owner manual](https://assets.sia.toyota.com/publications/en/om-s/OM52736U/pdf/OM52736U.pdf) gives the release procedure; the publisher link was verified but the browser returned403 for its PDF. The [manufacturer-authored service-data mirror](https://toyotamanuals.gitlab.io/RM00S5E/xmlweb/rm00s5e/sds/sds.pdf) specifies200 N application force and6-9 clicks for the sedan. Six estimated4-degree teeth give the24-degree start. Dimensions, masses, tooth pitch, button force and retained load remain stated estimates.

The [RRS DTMVHHBE extension](https://www.demon-tweeks.com/eu/rrs-extension-handle-for-hydraulic-handbrake-dtmvhhbe/) is published as200 mm long, sold for an RRS hydraulic handbrake. Its use on the Yaris requires the custom adapter; OEM compatibility is not claimed. Diameter30 mm is an estimate from the cylindrical product image, to be enrolled on the purchased part. Split blocks key onto the exposed12x26 mm steel bar at100/150 mm radii. A20x20 mm spine supports the extension90 mm to the side of the lever, with a512 mm custom spine and the full200 mm grip from592 to792 mm radius. The stock button and its full8 mm stroke remain clear. The adapter has no moving release mechanism. See fixture/dimensions.json and fixture/extension.svg.

Allow about$150 for the lever,25 for the bracket,45 for stock plywood/access panels and fasteners,60 for five springs, guide rods/endplates and cable,30 for the extension and40-80 for stock adapter material/one-time machining. These are allowances, not complete supplier quotes; total roughly$425-485 including a whole3M trial pack and calibration scale. No parts have been purchased. Fabrication and assembled load qualification are one-time preparation. A qualified reusable kit resets in1-3 minutes with no consumed specimen.

The whole installed assembly is yawed68.60717 degrees about world z, pivot at(670,-100,172.5) mm above the table. The18 mm floor board rests on104.5 mm anchored risers; the pivot is50 mm above its top. Local dimensions and world placement are versioned in fixture/dimensions.json. The access envelopes are not load-bearing helpers for lever motion. A/B differ only in the loose yellow card; this does not establish resistance to memorized replay.

## Physical mechanism and independent judge

Lever plus adapter mass is estimated1.57 kg. Its revolute joint has opposing cable torque3+112.2 theta N m throughout lifting and lowering, in addition to gravity. Actual signed actuation effort is read from PhysX. The20 g button is a slider parented to the lever with1000 N/m spring,5 N at5 mm release travel and8 mm maximum stroke. Ratchet and loaded-button jam are declared joint-limit proxies. Limits provide the tooth/bottom stop; they never lift or lower the lever.

A physical lift at least1 degree above the current tooth must coincide with contact moment at least the full declared cable torque. A different hand must then press the stock button inward at>=5 N and>=5 mm. Both normal and friction contact impulses contribute to the measured holding moment about the actual pivot. Either hand role is accepted. During button-held descent, another hand must contact the lever. Releasing the button catches the next4-degree tooth, with passive settling allowed. A locked-tooth descent is permanently invalid.

The native monitor reads live link transforms, joint positions and contact patches after every240 Hz physics step. Missing/nonfinite data, a sample gap, absent cable effort, out-of-range travel, inconsistent mounted/link poses or non-hand robot contact permanently give0. Non-hand contact uses summed force magnitudes, preventing a cancelling force couple from escaping detection. Rigid links do not simulate material fracture; inspect and reject physical damage during enrollment.

Final-state scoring requires ratchet/bottom-stop support, lever speed<0.5 degree/s, button speed<2 mm/s and both complete hand surfaces>=30 mm physically clear for1 second:0.25 at<=20 degrees,0.5 at<=16,0.75 at<=8,1.0 on the bottom stop within0.5 degree. Above20 degrees, held, moving, unsupported or still clearing earns0. Scores average lots A/B. The printed protractor makes angle tiers visible. Surface triangles are sampled at<=3 mm edge spacing with1 mm voxel thinning; a conservative5 mm sampling-error bound is subtracted before testing the30 mm physical minimum.

| Attempt or near miss | Outcome |
|---|---|
| Push down with no button contact |0; locked ratchet |
| Press button without unloading pawl |0;1 mm jam |
| Lift by a force-free state edit |0; actual contact moment and integrity required |
| Same-hand lift/button shortcut |0; opposite roles required |
| Either arm supporting, other operating |Same scoring |
| Use forearm or other robot link |0 permanently above2 N contact-noise guard |
| Release button mid-descent |Catch next tooth; only a supported final tier can score |
| Hold at bottom, end near the lever, or clear for0.99 s |0 |
| Mount/link pose corruption then restore |0; history persists until environment reset; material fracture is not modeled |
| Missing contacts, cable effort or physics samples |0 |

near_misses.py constructs full observation histories for the independent judge; it does not edit hidden completion flags. The lane's extra pure sequence controls exercise force-free motion, insufficient36 N support, symmetric roles, supported stops, clear/hold boundaries and permanent invalidation. Passing these tests does not demonstrate physical completion.

## Reproduction and evidence

Isaac Lab-Arena with yam_arena and Inspect Robots. No simulator port or upstream changes. Copied task-kit files are MIT; task code is MIT, documentation/drawings CC BY4.0. No OEM CAD or commercial meshes are redistributed. Local kit additions support rotated primitives, a nested button slider and task-owned per-physics-step observation. Native YAM shoulders remain27 N m, wrists7 N m; finger limits40 N each. No effort limit is raised to obtain success.

Run at20 Hz, object_jitter=0, deterministic seeded resets and two epochs. Use the exact instruction in task.json for the scripted and Dream runs:

    Raise the lever slightly, hold its end button in while lowering it fully, then move both hands clear.

The demo is a fixed sequence of Cartesian FK waypoints derived offline from visible geometry and stock joint limits. It never reads simulator observations, lot IDs or scorer state. It uses a physical ratchet catch before regripping for the lower-load finish. The prior native6 source scored A=0.5 and B=0.5. No Dream run occurred on that earlier source because its full-score A/B gate was unmet. The current finishing-target revision is described below.

The prior eight invocations comprised two initialization failures and six completed zero-score diagnostics. The final old trace measured36.28 N support,31.60 N axial button contact,50 N m cable effort and no lever lift. It is failure evidence on an older source. The fresh six native invocations are exhausted: one initialization error, four completed zero-score traces, and the final normal A/B demo at0.5/0.5. That earlier six-run allocation used no Dream. The current four-run continuation used two native invocations and one Dream. No measured robot result or physical fixture qualification is claimed.

The direct predecessor is TK-4JYPSE; this is its same-core lapse claim. The BEHAVIOR-1K name screen found no parking-brake release entry. Live ownership must be rechecked before coordinator action. A fresh independent review is required after the new demonstrations.

The fixed scripted targets compensate estimated contact loads through the stock80/10 N m/rad position gains. The standard YAM controller already supplies robot gravity compensation; it is not duplicated in the position targets. This is open-loop position-target compensation, not increased motor effort, a pose write or scorer feedback. Actual actuator/contact work still occurs in physics and is independently measured.

The current custom adapter is unusually long: overall radial reach792 mm, grasp780 mm, compared with the original297 mm button radius. It changes operator access and must be assessed as a tool-assisted task. No claim is made that this represents an unmodified passenger cabin. The solid6061 spine is20x20x512 mm, on the -90 mm side; full dimensions and proof requirements are in the fixture package. The floor board is440x580x18 mm.

The terminal100 mm of the grip has a1.07 mm [3M GM614/TB614 sleeve](https://multimedia.3m.com/mws/media/1001810O/3m-gripping-material-trial-bags.pdf?fn=Gripping-Material-Trial-Bags-TDS.pdf). The cited3M trial-bag sheet reports dry kinetic2.8 for the matedGM613/TB614 pair, with wet2.6; it does not report synthetic leather or YAM rubber for that test. Model pair static1.6/dynamic1.4 remain unmeasured dry-contact estimates; the mated-pair data motivate material selection but do not establish a YAM coefficient or lower bound; the native material authors the stated pair1.6/1.4 with max combination, leaving robot pad materials unchanged. Enroll the actual pair for112 N tangential traction at80 N total pinch. Adhesive/sleeve fabrication is one-time preparation, reused between1-3 minute trials. No friction measurement or hardware purchase has occurred.

The current source corrects rotated-box USD authoring to translate, rotate, then scale. A real OpenUSD CPU regression reduced maximum corner error on a512 mm rotated test bar from141.50 mm to0.000066 mm. The native5 straight-bar contact image was diagnostic failure evidence, not a successful release. The earlier object-material/gripper-pad confusion is corrected by explicitly authoring the stated coated-pair estimate1.6/1.4 with max combination. Loaded demo grasps now keep a fixed orientation relative to the lever; they do not roll the gripping pads around the cylinder while holding the load.

Cable enrollment measures tension directly with a calibrated [VEVOR OCS-L300 kg scale](https://www.vevor.com/crane-scale-c_10459/vevor-digital-crane-scale-660-lbs-300-kg-max-capacity-handheld-hanging-scale-with-cast-aluminum-case-lcd-display-0-1-lbs-division-3-unit-switch-portable-design-for-farm-hunting-fishing-orange-p_010480870666), published50 g division. Allow$35 for the whole instrument and$40-60 for the whole3M trial pack, giving roughly$425-485 including the earlier kit allowances. Fit the scale inline in the horizontal anchored cable route with its housing supported but no axial load shunt; include its compliance when enrolling the assembled curve. At0/8/16/24 degrees cable tensions are88.7/551.7/1014.7/1477.6 N. This avoids mistaking tool gravity for cable load. The old200 N hand-force equivalent at250 mm describes cable torque alone; actual handle force includes gravity. Physical fabrication and calibration remain unperformed.

The explicit cable effort is verified against the preceding240 Hz pose where that effort was written. Contact gates require the larger of the current curve and actual applied effort. Fast passive catches therefore retain full physical cable load without a false timing rejection. A forged reference pose or missing effort permanently invalidates the trial. Nonhand contact magnitudes are summed across all links and both targets before the2 N guard.

Final native6 (demo-20261122T192103) physically lifted to26.136/26.173 degrees with peak holding moment65.032/64.605 N m and left-hand support114.059/112.215 N in A/B. With the other hand pressing the button, both descended and caught12 degrees, then cleared for9.704 s. No scorer invalidation occurred. The second right-hand grasp briefly lifted but slipped before the left button press; full release was not demonstrated. Runtime source remains frozen; only evidence text, task.json scores and the manifest status were updated afterward. Source task.py SHA256 ecb750d988627ed4fba0062416f284dc1a77b7e46caecaf05a5655797560c10f. Recorded media, official logs and force traces accompany that earlier diagnostic.

## Exact reproduction commands

These commands document reproduction for a separately authorized allocation. Do not repeat a completed Dream allocation. Set GPU_HOST to the authorized simulator host, with its existing task stack installed. They do not install or port a simulator. Use the unchanged instruction shown below. Keep the seven runtime files fixed between the final normal demo and Dream.

```bash
TASK_DIR=work/benchmark-tasks/tasks/lower-the-yaris-parking-brake-between-the-seats
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 work/yam-isaac-lab-arena/.venv/bin/python factory/check.py "$TASK_DIR"
rsync -a --exclude __pycache__ "$TASK_DIR/" "$GPU_HOST:tasks/lower-the-yaris-parking-brake-between-the-seats/"
```

Trace lot B before a full demo, then inspect actual lift, cable effort, contact moment, button force and clearance. A trace consumes a native invocation.

```bash
ssh "$GPU_HOST" 'TASK_LOT=B bash ~/factory/gpu_run.sh ~/tasks/lower-the-yaris-parking-brake-between-the-seats trace 1600 "Raise the lever slightly, hold its end button in while lowering it fully, then move both hands clear."'
```

Normal A/B demo, matching the final native6 mode and limit:

```bash
ssh "$GPU_HOST" 'bash ~/factory/gpu_run.sh ~/tasks/lower-the-yaris-parking-brake-between-the-seats demo 2400 "Raise the lever slightly, hold its end button in while lowering it fully, then move both hands clear." --epochs 2'
```

Only after both official normal-demo scores are1.0, run the authorized single Dream-1 on exactly the same runtime and instruction:

```bash
ssh "$GPU_HOST" 'bash ~/factory/gpu_run.sh ~/tasks/lower-the-yaris-parking-brake-between-the-seats dream 2400 "Raise the lever slightly, hold its end button in while lowering it fully, then move both hands clear." --epochs 2'
```

The normal/Dream pair is fetched for this revision: normal1.0/1.0, Dream0/0. Local media is retained; no submission or public media ID is claimed. Known model and physical-qualification limitations remain open.

## Scope and remaining physical qualification

The familiar operation is releasing a loaded parking-brake pawl with its button. This implementation is a tool-assisted reproduction: the original button remains inside the seat/console slot, but the holding point is713-780 mm forward of the pivot while the seat envelopes end at430 mm. It tests the constrained button hand and coordination with an external leverage tool. It does not establish full passenger-cabin reach or unmodified driver access. The unusually long tool remains a construct limitation for independent review.

The physical fixture is a fabrication proposal. Use the purchased OEM bracket itself as the drilling template; preserve its ratchet and button linkage. The cable cam and purchased RRS interface require transfer measurements before machining. The proof loads, spring-bank enrollment, complete load path and reset procedure in fixture/README.md are requirements, not completed qualification. Cost allowances include the complete adapter material, five springs, terminated cable, anchor hardware, full grip trial pack and inline scale; actual machining and instrument quotes remain to be confirmed. Do not claim an assembled or fabrication-ready kit from these drawings alone.

The simulation samples contact forces and integrity at240 Hz. Its force trace currently saves20 Hz diagnostics; it is not a complete substep record for independent rescoring. The proposed real trial first enrolls the cable curve, button force/travel and grip traction, then records angle, separate hand roles, button motion and complete hand clearance. The cable scale measures cable load, not signed hand moment. Video alone does not establish the continuous5 N button force or50 ms signed holding-moment rule to a stated error bound. A physical trial needs a calibrated contact-force measurement procedure, synchronized logging at least100 Hz, sensor placement that does not change the grasp or button geometry, uncertainty margins and an included instrument cost. That procedure is still open; no physical force-gate validation is claimed. Angle-only inspection must not be presented as equivalent to those dynamic force rules.

Closing-target continuation: the first tested candidate changes only the finishing right-hand target from0 to-10.8 mm. The stock5000 N/m finger drive then has approximately26.35 N nominal preload per jaw at the32.14 mm coated diameter, below the unchanged40 N cap. This leaves room for unequal jaw forces under the oblique load. The52.7 N total preload and73.78 N nominal dynamic traction are model estimates, not measured traction or success evidence. Arm positions, orientations, durations, the working first stage and all physical/scoring parameters are unchanged.

The proposed physical enrollment and manual outcome protocol is in [fixture/measurement.md](fixture/measurement.md). It gives the gauge, scale and video steps and explicitly separates those observations from uninstrumented native force/history equivalence. It is not physical validation.

## Current full-score native evidence and return-proxy limitation

Normal demo-20261122T201638 scored A=1.0 and B=1.0 after lot B trace-20261122T200603 scored1.0. The sole runtime change from the prior0.5/0.5 source was the finishing right-hand finger target,0 to-10.8 mm. All arm poses, durations, left-hand commands and physical/scoring parameters are identical. Actual finishing support peaks45.50/46.69 N and moments35.38/36.28 N m sustained release of the12-degree tooth; both trials reached the bottom stop and conservative81.07 mm hand clearance for1.029 seconds.

The240 Hz monitor also retained button-return contact peaks355.65/356.21 N. An isolated regression shows the existing jam proxy can reduce the slider's upper limit from its current4.4 mm position to1 mm, imposing3.4 mm of projection. This is a numerical return mechanism, not a measured OEM spring force. It does not supply the demonstrated initial lever lift, but its artificial return work remains a fidelity defect for review. A non-intruding-limit candidate exists separately and is not part of this frozen runtime. Native success is not proof of physical button strength or a completed hardware procedure.

Dream-20261122T203658 ran the exact unchanged instruction and seven runtime files used by the normal demo. Both trials hit2400 steps with score0, no errored trials and no recorded abstentions. Neither physically lifted or pressed the button; lot B contacted the lever but its peak signed holding moment12.23 N m was below the50 N m start load. This is a failed attempt, not a recorded refusal. The instruction and runtime were not reworded or changed afterward.
