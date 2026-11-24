# Release 60 Torpac-size tablet replicas within the pusher force limit

**Instruction:** Hold the plate by its tab, push all 60 replicas onto parchment with the pin at no more than 4 N each, then set both down and clear both hands.

Category Other: laboratory tablet formulation. Two deterministic lots; 4,000 control steps (200 s at 20 Hz) per trial. The current reusable prototype has no final full-score evidence yet. No medication, active ingredient or patient use is involved.

## Procedure and adaptation

[Torpac brochure 2509](https://www.torpac.com/Reference/prordtm-2g.pdf) supplies the X-MRDT60x075 mold footprint (120 x 152 mm), openings (5.5/6.5 mm), cavity depth (3.7 mm) and nominal 75 mg tablets at density 0.75 g/ml. Separate plate thickness and pitch are not specified. The model uses 3.7 mm plate thickness and estimated 12.5 x 14 mm pitch.

[Maker instructions 2001](https://www.torpac.com/Reference/prordtm-2g-instructions.pdf) lift the cavity plate off its base, invert it, support it by hand and sweep with a plastic spreader; remaining tablets are pushed with a finger, without metal tools. About 0.1 mm dried top-face shrinkage,0.4 N release,4 N crush and 0.04 mm radial clearance were obtained by phone from a technician who works with this product (Nov 2026). The maker fills the wide face before inversion, so the reported fill face points down during demolding. This replica instead puts a declared 0.1 mm recess on the pushed narrow face; that placement is a design choice, not a reported real pushed-face reading. A flat card sharing load with the plate lands did not deliver the required individual tablet forces in corrected pilots. The proposed adapted spreader therefore has a central 1.8 mm polymer pin projecting 12 mm below the card, for sixty separate pushes. The instrumented pin stands in for the maker's remaining-tablet finger push, one force-logged push per cavity. It is not an unmodified maker tool.

The primary low-cost trial route under development is RDT60-R1: a reusable split resin plate, sixty printed pucks and commercial silicone rings, plus an upright grip tab for the supporting hand. The tab and extension adapt the supporting hand's hold on the plate edge; they are not parts of the maker plate. The footprint and cavities remain true size. This substitutes a declared force window for pharmaceutical fragility. It is not evidence that rigid printed parts reproduce an RDT tablet's fracture behavior. Physical force qualification is mandatory and has not been performed.

## Scene and fixtures

Millimetres; x forward, y left from the midpoint between arm bases, heights above counter. Proposed RDT60-R1 reusable resin replica of the Torpac X-MRDT60x075 cavity plate: 120 x 152 x 3.7, centre (275,0), long side along y, back face 16.6 high; 60 tapered holes, six x ten at 12.5 x 14 pitch, narrow 5.5 ends up, wide 6.5 ends down. Sixty white tapered resin pucks, 120 mg each estimated, 3.6 high, pushed faces deliberately recessed 0.1 mm with 0.04 mm radial clearance; 5 x 1 silicone retention rings in the physical replica. Plate has a 30 x 177 x 10 rim extension at local (y 164.5,z 3.15) and a 30 x 30 x 55 hollow grip tube with 2 mm walls, central 2 mm compression web and open x ends, axis canted 2 degrees about each local x and y axis (first x, then y), centred at y 250, rising from the back face; assembly mass 160 g estimated. Initially rests on two 12.7 square x 152.4 aluminium bars at x 221.35/328.65 on a 260 x 300 x 0.2 parchment sheet centred (275,0). Adapted polymer spreader: 90 x 70 x 3 card, central 1.8 diameter pin projecting 12 below, 20 x 12 x 25 upper grip boss, and a 58 x 16 x 19 load-cell cover at local (-20,0,-19) relative to card centre; fixed 12 x 14 x 25 sensor frame at local (-40,0,0.5), lower M5 head 8.72 diameter x 5 at (0,0,-31.85); total mass 55 g estimated. Stands upright, pin tip 6 high, in a 100 x 40 x 20 resin base block with 4.5-wide slot/floor 6 and central 12 x 10.5 screw-head clearance and 12-high shoulders supporting the card ends, at (220,-200). Lot B moves only block/spreader to (410,-190). All other starts identical; jitter disabled.

The tool block has two upper end guides,30 x10 x19 each, centred at local(30,+/-7.5,28.5), top38 high and finished slot5.0 wide. They engage the card's plain right end, clear of the sensor housing; block mass106g estimated. With a12mm pin, the20mm base alone leaves only2mm of card engagement and permits large lean. The upper guides provide20mm engagement; verify upright support and free withdrawal after one-time fitting.

A 30 x 25 x 12.9 resin parking block at (275,220) supports the long connector before pickup and after set-down. The left hand physically pinches the tab and supports the plate during pushing; bars are parking supports. There is no hand attachment or plate pose hold. The demo commands a 10 mm hand lift to compensate observed compliance; the actual plate must stay within the unchanged 10 mm displacement limit. Whether the final prototype actually clears the bars remains to be verified from its trace. A 12 x 44 x 3 stop ledge is centred at local x2/y250, underside57.3 mm above the plate back. A 12 x 12 x 3.5 neck begins54 mm above the back and overlaps tube and ledge. The ledge limits downward slip against the inner finger pads; delivered clearance and stiffness remain unqualified. No attachment joins hand and plate. Both parts are set down before the final hands-clear period.

See [fixture/README.md](fixture/README.md). Custom geometry is original; no third-party meshes are used. Original watertight CAD, dimensions and a priced proposed package are in [fixture/BOM.md](fixture/BOM.md) and [fixture/ASSEMBLY.md](fixture/ASSEMBLY.md). They are unbuilt and physically unqualified, not an obtained kit.

## Scoring and inspection

Whole-trial zero if spreader normal compression exceeds 4 N on any tablet; aggregate normal compression from spreader/gripper contacts on plate/tablets exceeds 6 N; a gripper touches a tablet (>0.02 N contact tolerance); plate moves >10 mm from start or tilts >5 degrees; any tablet falls off counter; or required state is unknown. Physical chips/cracks >=0.5 mm also zero; rigid simulation proxies damage with force caps. Moving any bar, parking block or tool block over 5 mm permanently zeros the trial. Non-pin spreader contact above 0.02 N on plate/tablets also zeros; all spreader contacts still count toward the damage caps. A loaded force/stroke push with less than 0.5 mm clearance over parking supports also zeros it. A release requires contact on the spreader's pin itself at >=0.40 N on that tablet and actual downward tablet travel >=2.2 mm against retention; plate/neighbor force cannot release it. Height and off-counter checks use the actual transformed frustum vertices; a rotated enclosing box can extend below a resting puck. The 1 mm and 5 mm limits are unchanged. Count free pucks resting on parchment, lowest point within 1 mm of sheet, top <=5 mm up, speed <=10 mm/s, centre inside 260 x 300 sheet. All scores require hands >=30 mm from plate, grip tab, bars, space under plate and tool for 1 s. 0-14 count: 0; 15-29: 0.25; 30-44: 0.5; 45-59 or all 60 before stillness: 0.75; all 60 with plate/tool/pucks moving <1 mm for 1 s: 1.0. Two lots averaged.

Use overhead and side video, a final parchment photograph with numbered grid, ruler and 10x loupe. A held puck, puck in its hole, off-sheet puck or puck standing on edge does not count. Gripper clearance includes the new grip tab. Force at each tablet is measured independently in simulation; contact with the plate or a neighbor contributes zero to that tablet's release. A whole-plate scale cannot resolve individual tablet forces. Physical enrollment uses an instrumented pin/load cell; the drafted load path and calibration checks are in fixture/ASSEMBLY.md. No assembled force accuracy has been measured.

## Exact simulation rules and why the thresholds

- `F_REL=0.40 N`: the pin's own compressive normal force on tablet k must reach this threshold. About 0.4 N release,4 N crush,0.1 mm fill-face shrinkage and 0.04 mm radial gap were obtained by phone from a technician who works with this product (Nov 2026). Pushed-face recess is the replica design choice described above. Proposed physical enrollment is 0.40-0.45 N peak with the same pin; reject assemblies outside the window.
- `STROKE=2.2 mm`: that tablet must also have moved down by this amount relative to the plate, representing exit from the proposed ring engagement. Contact force and travel must occur together; no elapsed time, edge position, neighbor state or plate press releases it.
- A temporary native prismatic retention joint resists motion along the bore. Its implicit drive is capped at `RET_MAX=0.45 N`, stiffness `RET_K=RET_MAX/STROKE=204.545 N/m`, damping `RET_C=2*sqrt(RET_K*TAB_M)=0.31334 N s/m`. These are unmeasured model choices. The drive cap is 0.45 N; release requires >=0.40 N own force. Earlier development releases at 0.400-0.405 N did not reach that drive cap. Replica enrollment is 0.40-0.45 N, not a claim that 0.45 N was overcome in those logs. It applies equal/opposite force and moment to plate and puck through the native solver. It does not push the puck out or preload the taper. The joint is disabled immediately when that puck completes its own loaded stroke; thereafter it is a fully free 6-DoF rigid body, with no guide, delayed hold or pose correction. Before release the guide approximates ring/bore alignment and replaces redundant puck/plate wall contacts; pin/puck contacts remain active. Full puck/plate collisions are restored in the same release step that disables the guide. After release contacts and gravity determine motion.
- `F_TAB=4 N` is the supplied dried-tablet crush figure; `F_HI=6 N` aggregate is a protocol plate-load limit, not a measured crush force. Reusable parts must separately pass proof and inspection checks. `TOUCH_N=0.02 N` is the gripper no-touch detection tolerance. Rigid bodies do not fracture; physical chips/cracks remain a separate inspection. The force caps do not prove pharmaceutical fracture equivalence.
- The contact adapter sums compressive contributions per actual contact point before adding contacts, so opposite normals cannot cancel. Tablet forces are isolated from plate force. Actual contact points must lie on the colliding pin (0.4 mm numerical allowance from two 0.2 mm contact offsets); upper-frame/card contact cannot release a puck. This point check classifies an existing force contact and never creates force or release by proximity. PhysX PGS uses 128 position and 128 velocity iterations. Contact and travel monitor at native 240 Hz; pose/hand/stillness checks run at 20 Hz. The plate, connector/grip and tool are rigid; real bending, cable motion and sensor flex require physical qualification. Unknown or nonfinite state fails closed. Numeric contact offsets are 0.2 mm, rest offset zero; tablet/hole cross-sections are eight-sided approximations. Pose writes occur at reset only.

At trial initialization only, the SDK scene-reset API places the spreader and its block 10 mm above their declared counter location, with zero initial velocity, and lets them settle under gravity for 0.5 s. Both layouts passed a two-epoch initialization check within 0.001 mm of their declared positions. This avoids the observed lateral drift after a direct in-contact relocation. No fixture pose is corrected during a trial; the 5 mm fixture-displacement limit remains active.

`TABLET_LOG` records source SHA256, native dt, lot, score, per-tablet release force, travel and physics-step index. Synthetic boundary cases in `near_misses.py` exercise the actual rule/scorer; they are not GPU or physical trials. The dedicated plate-only GPU control did not run on the final geometry. An incidental plate-only interval is documented in validation/rim-press21.json, not presented as that dedicated control. The final A/B DEMO completed with0.0/0.0; Dream-1 was operator-interrupted. No earlier run is presented as final-source evidence.

## Unintended strategies

| Attempt | Rule |
|---|---|
| Press only plate, or only neighbor | Target cannot release |
| 0.39 N on target | No release |
| 0.40 N without target's 2.2 mm stroke | No release |
| Force and stroke on different tablets | No substituted release |
| >4 N on one puck or >6 N aggregate | Permanent zero |
| Gripper contacts a puck | Permanent zero |
| Plate displacement >10 mm, tilt >5 degrees, off-counter puck | Permanent zero |
| Hold tool or grip tab at the end | Hands not clear; zero |
| Do nothing | Zero |

## Real kit, cost and reset

Proposed reusable kit, physical qualification pending: print plate halves, pucks and fixtures once, install 5x 1 silicone rings, finish/gauge fits once, then reseat the same 60 pucks at the counter edge and use the 0.1 mm stop gauge on each face after returning the plate upright (fixture/ASSEMBLY.md). No per-trial baking/curing. Qualify every puck with the actual polymer pin: release peak 0.40-0.45 N through the 2.2 mm engagement, no visible chip/crack, and repeatability over three cycles. Reject or refit assemblies outside that range; this is a required enrollment check, not a measurement already obtained. Calibrate force logging with known masses and record each push. Set plate on bars and parchment at (275,0), verify all 60 seated 0.1 mm recessed, place tool/block at lot position, tare sensors, photograph setup. Reset time estimated 3-5 minutes, unmeasured. Primary pharmaceutical mold/preparation is a separate optional reference route, not the reusable reset claim.

The maker mold is listed at US$470 in brochure 2509 (effective Sep 15, 2025); its 30-minute bake plus 5-10-minute cooling is not a reset in minutes. That route is therefore not the claimed repeatable low-cost kit. Printed accuracy does not establish the 0.04 mm gap or 0.1 mm recess. Matched parts are finished once, rings set retention and the verified stop gauge sets seating; nominal SLA tolerance is too coarse to supply these fits directly. The proposed reusable CAD and rate-card estimate are supplied; a vendor-accepted quotation, ring force qualification and a measured reset remain incomplete. No invented kit quotation or physical measurement is claimed.

## Distinctness and limits

Unlike flexing an ice tray (TK-YTH6M4) or a printer build plate (TK-A8X2KJ), the cavity plate stays rigid and each small part needs its own loaded push. Unlike tablet counting (TK-QQSJZE), parts begin retained in a mold. Unlike a single blister release (TK-GJ7LSU), sixty positions and a force window govern a complete batch. These are prior screened comparisons, not a fresh duplicate-board audit. Fixed lots allow memorized scripted replay. No physical robot results are claimed.

## Earlier verdict, quoted in full

> **Not accepted** (TK-UB4LFU).
> 
> Construct validity (not met): The rigid spreader releases tablets from plate contact alone even though their faces are recessed 0.1 mm and receive no simulated push.
> 
> Feasibility (not met): The demo's simulated release rule substitutes for pushing the tablets, so its full score does not demonstrate the specified physical action.
> 
> What to change: The simulated release currently fires when the spreader presses the plate over a hole, without pushing the tablet itself; the modeled tablet face is recessed below the plate. Adjust the tablet seating, spreader or release method so the real task can physically push each tablet out, and implement the simulation so release requires that same physical work. Update the fidelity note and provide a full-score demo using the corrected mechanism.
> 
> To submit a revision, push your changes to this pull request (no need to open a new one). Your idea stays reserved for 48 hours from this review; if no revision arrives by then, the hold lapses and another team can claim the idea.
> 

## Point-by-point response

1. **Plate-only release:** removed. `contact_loads` isolates every tablet's actual spreader force; plate and neighbor tests release nothing.
2. **Recess/physical access:** the technician reports fill-face shrinkage; the replica deliberately recesses the pushed narrow face 0.1 mm. A colliding central 1.8 mm pin reaches it. Every hole is physically reachable by the pin, rather than relying on a card bridging the lands.
3. **Same physical work:** the new reusable model requires own force >=0.40 N and actual 2.2 mm resisted travel, with native reaction on the plate. No hand weld, live pose writes or post-release guide.
4. **Fidelity:** task.json, this note and fixture dimensions disclose replica, estimates, temporary retention and fracture limitations.
5. **Full-score feasibility:** not met. A retained tablet21 outside its retry column and later latched a zero after plate displacement reached10.011 mm against the unchanged10 mm limit. B also latched a zero after10.041 mm plate displacement. The completed official DEMO grades are A0.0/B0.0, mean0.0 ([receipt](validation/final-demo.json), [44 actual release events](validation/final-demo-releases.csv)). Dream-1 was operator-interrupted, with no completed model grade ([record](validation/model-attempt.json)). Small development pilots do not establish full-batch success.
6. **Flags:** none in the sole verdict. Whole-task kit cost/reset and supporting hand are being addressed, not assumed settled.

## Running and results

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 --max-steps 4000 --epochs 2 \
  --instruction "Hold the plate by its tab, push all 60 replicas onto parchment with the pin at no more than 4 N each, then set both down and clear both hands." --scorer task_score
```

The scripted waypoints last195.3 s; the configured policy then holds its final pose for3 s, staying below the200 s simulated trial limit. After its first pass it physically retries the first column on eight rows; those pushes use the same force, travel and damage gates. Aim offsets were calibrated from native development traces; they are fixed motion commands, with no state feedback or scoring exception. This demonstrates a task-specific script, not general policy competence.

`TASK_LOT=A/B` selects a deterministic lot. The frozen-source two-epoch DEMO completed: official A0.0/B0.0, mean0.0,7932 control steps and policy_stop in both trials. A released43/60 and B1/60; both reported hands clear at the final logged198.25s sample. A latched a zero after10.011 mm plate displacement, and B after10.041 mm(limit10 mm). [Final native/official summary](validation/final-demo.json) and [44 release events](validation/final-demo-releases.csv) record this failed demonstration. The native release force ranges were A0.400123–0.415843N and B0.403882N, with actual resisted travel A2.208905–2.483616mm/B2.423607mm; these are simulation values, not physical measurements. An additional pinned-B attempt was operator-interrupted with7/60 released at its last36 s sample, not a completed grade. Dream-1 was interrupted by the operator in A after1,493 control steps; its last logged sample was74.5 s with0/60 released. B was not run and there is no completed model grade. The partial recording documents the interrupted attempt; no completed performance grade is available. An earlier full attempt on an older source scored0 in A after a support-clearance violation at27 releases; B was not run. Development pilots are diagnostics, not full-score evidence. Dried-tablet force/gap and fill-face shrinkage figures come from the technician phone report; pushed-face placement is a replica choice. No replica force, reset time or hardware score has been measured.
