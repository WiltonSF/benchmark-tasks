# Square the racked gantry against both end stops

The scripted expert scored **1.0 in both layouts** on the current native Isaac Lab-Arena scene, reaching full score in 15.60 and 15.65 seconds. Dream-1 scored **0 in both layouts** and hit the 1,200-step limit. These are simulation results, not hardware measurements. Expert and Dream runs used the same source snapshot and instruction.

This independently implemented claim credits [PR 2013, TK-Y97RFJ](https://github.com/robocurve/benchmark-tasks/pull/2013), preceded by TK-WYFJHJ and TK-H5595Z, and the earlier gantry proposal [PR 419, TK-RWRT5A](https://github.com/robocurve/benchmark-tasks/pull/419). The captured profile prevents a lift-and-carry shortcut. Current rail engagement, bounded stop gaps, both-sided height checks and executed counterexamples address the original scoring feedback. Fresh expert/Dream videos and a setup screenshot belong to this captured scene. Publication must wait until the original hold lapses and a fresh lapse check permits the claim.

Squaring the ends of a small Cartesian gantry is a real printer adjustment. [Prusa's procedure](https://help.prusa3d.com/article/adjusting-belt-tension-core-one-core-one-indx-core-one-l_845048) describes disabled motors, loosened belts and counter-force at the gantry ends. This scene isolates captured sliding and end-stop alignment. It does not represent a complete printer calibration, measured bearing dynamics or proof that every one-handed strategy fails.

Two 250 mm rails, 340 mm apart, carry a 20 × 20 × 400 mm beam and a toolhead. The estimated moving mass is 0.8 kg. The [HIWIN MGN12H table](https://www.hiwin.de/en/Products/Linear-guideways/Blocks/Miniature-guides/MGN-HIRES-series/c/4500) supplies the 45.4 mm length, 27 mm width and 13 mm overall mounted height. With the block bottom 3 mm above the mounting surface, its body is 10 mm high here. The rail is 12 mm wide and 8 mm high.

Rectangular rail grooves and fixed retaining lips are an explicit sliding-contact proxy for bearing capture. They are generated geometry, not manufacturer CAD. The 0.25 mm running gaps retain the blocks while allowing limited yaw and contact binding. Recirculating balls, preload, structural elasticity, motors and belts are absent. Friction 0.2, mass, groove details and contact response are estimates. Compound inertia comes from the collision volumes, not a calibrated steel/aluminium/acetal mass distribution. No rule changes a joint, welds the beam, teleports a part or declares it square after a command.

The expert uses published visible initial poses and fixed Cartesian waypoints. One fixed left-hand load calibration applies to both starts. It reads no lot label, runtime object pose, contact state or hidden parameter. `tilt.py` supplies waypoint orientations to the stock scripted planner; it does not change Dream-1, the scorer or installed harness files.

Only the final state earns credit. Both ends need at least 25 mm net forward travel made while both carriages are engaged, valid final engagement and no more than 2 mm accumulated travel observed off the rails. Net displacement prevents positive jitter from accumulating credit. For every positive tier, engagement requires:

- Both block centres within 0.6 mm of their rail centre-lines.
- Every top corner within 1.0 mm of its seated top height; yaw and upright tilt each at most 0.7°.
- Rear edges no more than 0.5 mm behind the rail starts; no front corner more than 0.10 mm past its stop plane.

The end-state ladder is 0.25 for that engaged slide; 0.5 when all front corners are within 5 mm of the stop planes; 0.75 when every front corner is within 0.05 mm, none more than 0.10 mm past a stop, and every top corner is within 0.5 mm of seated height. Full score also needs the beam below 3 mm/s and 0.03 rad/s for one second while every gripper and fingertip stays more than 5 mm clear of every task surface. The robot may withdraw its hands during this interval. Fixed stop faces physically oppose further travel. Signed bounds reject sunk, lifted, sideways and beyond-stop states. Unknown physical observations permanently invalidate the trial.

Release uses full collision-face samples at 3 mm spacing with 2 mm voxel thinning. A 12 mm sampled gap includes a conservative 7 mm sampling allowance. The separate `sampled_sim_feasible` diagnostic uses 20 Hz native net fingertip and projected joint loads; it is not a whole-trial force certificate or a primary score condition. Expert peaks were 13.624 and 13.285 N, with no sampled feasibility violation. A physical trial must independently verify the approximately 40 N finger limit using calibrated sensors or a validated force limiter.

The native expert kept both carriages engaged at every recorded step, with zero off-rail travel, and ended with all stop gaps within about ±0.6 micrometres and top-height errors below 0.253 mm. The 17 CPU scorer controls plus the reset case reject off-rail, below-rail, carried, one-end-home, beyond-stop, stale and nonfinite observations. An independent flat-face hover probe verifies that face sampling rejects contact missed by vertices alone. These checks do not measure hardware fidelity.

The physical counterpart uses the dimensioned captured sliding fixture below. Measure progress and lateral position with a ruler/caliper, seated height with a dial indicator, stop contact using a 0.05 mm feeler at both face corners, and engagement/release/stillness on video. The build has not been fabricated or measured.

| Part | Material and dimensions | Assembly coordinates |
|---|---|---|
| Two profile tracks | Steel, 250 × 12 × 8 mm. Foot width 12 mm at z = 0–3 mm; neck width 8 mm at z = 3–5 mm; head width 12 mm at z = 5–8 mm. | Along x = 0.030–0.280 m, at y = ±0.170 m, bottom on the bench. |
| Two captured blocks | Acetal, 45.4 × 27 × 10 mm envelope; profile below. | Centres at beam-local y = ±170 mm. Fit from the rear rail ends. |
| Beam | Aluminium, 20 × 20 × 400 mm; a 2 mm wall extrusion is one physical option. | Along y = ±200 mm; underside 13 mm and top 33 mm above the bench. Bolt rigidly to both blocks. Simulation omits the hollow and bolts from collision geometry. |
| Toolhead ballast | Steel, 40 × 50 × 40 mm, approximately 0.624 kg at ordinary steel density. | Centre at beam-local (30, 0, 10) mm. Measure the assembled mass; 0.8 kg total is an estimate. |
| Two stops | Steel, 10 × 30 × 20 mm along x/y/z. | Centres (0.285, ±0.170, 0.010) m above the bench; rear planes x = 0.280 m. Fix rigidly. |

Each block spans x = ±22.7 mm about its centre. Relative to the rail centre, its sidewalls occupy y = 6.25–13.5 mm and −13.5 to −6.25 mm, z = 3–13 mm. Its top plate occupies y = ±13.5 mm, z = 8.25–13 mm. Retaining lips occupy y = 4.25–6.25 mm and −6.25 to −4.25 mm, z = 3.25–4.75 mm. These coordinates define the complete contact profile. The lips sit below the rail heads. Use flush countersunk M3 track fixings at x = 0.055, 0.155 and 0.255 m; fixing holes and fasteners are not collision geometry.

Use clean, dry steel/acetal contact surfaces. Measure straight breakaway/sliding and off-centre binding with a spring scale before claiming comparable physical difficulty. The coefficient 0.2 is uncalibrated; do not infer a measured force from a material name. This fixture approximates bearing capture with sliding lips rather than claiming exact HIWIN behavior.

Reset by sliding the captured assembly, keeping both lips in their grooves. The beam origin is its centre, not a block or the toolhead. Lot A starts at (x = 0.216, y = 0, z = 0.023) m above the bench, yaw −0.45°. Lot B starts at (0.205, 0, 0.023) m, yaw +0.45°. Allow the assembly to settle downward by its 0.25 mm top running clearance. The first observed settled carriage positions establish the travel baseline. Rails and stops retain their coordinates.

Trial 1 uses A and trial 2 uses B; the task score is their mean. The setup screenshot and composite videos show the first layout, while the run records include both. Run `--epochs 2`; `TASK_LOT=A` or `TASK_LOT=B` is only for single-layout debugging. From the task folder:

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 --instruction "Move the gantry's two carriages along their rails until both rest square against their end stops, then let go." --scorer task_score --max-steps 1200 --epochs 2
```

Use the identical instruction and settings with `--policy dream`. Recorded expert runs used a 520-step cap and finished well within it; Dream used 1,200. Both snapshots have task.py SHA-256 `78564a52b096fdd2617cd88251c190da497f21a9af9d2f7ac05666120cdfa5bc`. README and score-reporting text were finalized after those runs; runtime files are unchanged.

Code MIT; documentation CC BY 4.0. All task geometry is generated. Installed harness files are unchanged.
