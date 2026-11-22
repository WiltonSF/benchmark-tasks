# Restore the trace overcoat without bridging its contact pads

**Instruction:** Restore the protective overcoat on the repaired trace with the brush, keep coating off the pads, and put the brush back on a rack.

**Category:** Impact on jobs

Electronics assemblers rework boards that fail inspection. After a damaged copper trace is repaired, the copper that
lost its solder mask has to be recoated.
The pads beside it were bare on the original board and must stay bare, so they can still be soldered and probed:
a local touch-up restores what the mask covered and nothing else. The assembler dips a brush in a small cup of
coating, tips it so only the leading bristles touch, draws a light stroke along the repair, reloads when the brush
runs dry, and keeps the stroke short of the pads. Pressing harder splays the bristles sideways onto the pads; a dry
brush leaves bare copper. The task asks for that skill on a complete original
controller board, chosen because its full CAD is public, so trace, pads and parts are exact.

- O*NET 51-2022.00 Electrical and Electronic Equipment Assemblers (246,300 workers in 2025, BLS projections as
  listed on O*NET OnLine), core task: "Adjust, repair, or replace electrical or electronic components to correct
  defects and to ensure conformance to specifications."
- IPC-7711/7721 (Rework, Modification and Repair of Electronic Assemblies) includes coating replacement after a
  repair as its own procedure.
- The coating maker's data sheet (MG Chemicals 422B): "This product can be applied by brush for rework or
  touch-ups." The same sheet specifies UV-A fluorescence for inspection.

No electrical, cure or insulation result is scored. The task measures visible application only.

## Scene (true size, named parts, two fixed layouts)

| Part | What it is | Size and source |
|---|---|---|
| Controller | Complete unpowered Adafruit CRICKIT for Circuit Playground Express #3093, underside up | 85 x 81 x 14.7 mm, 30.7 g, maker page; pads, trace and components from the maker's Eagle CAD |
| Repair | Interior of the native ANALOG8 bottom trace, re-exposed for the repair | native x -29 to -15 mm, 14 x 0.254 mm, unchanged width |
| Nearest pads | GPIO1:24 annulus 1.5 mm from the trace start; TP1 6 mm past its end | native positions and radii |
| Service tray | 120 x 120 x 6 mm floor, two 18 mm side walls, printed TRACE label | custom; drawing in fixture/setup.svg |
| Standoffs | Four M3 x 16 mm hex standoffs (5 mm across flats) fixed in the tray, under the board's own corner mounting holes U$43, U$44, U$49 and U$50 | commodity hardware; the laminate rests on them level (underside 22 mm, top 23.6 mm above the counter), components hanging free |
| Brush | MG Chemicals 859 horsehair brush, whole plywood handle | 196.85 x 11.1125 mm handle, 34.925 x 6.35 mm bristle face, 11.1125 mm trim (maker chart, July 2024); 8 mm thickness and 20 g are estimates |
| Coating | MG Chemicals 422B, 5 mL in a fabricated 44 x 42 x 22 mm metal service cup (36 x 34 mm inside, 4 mm floor) on a 50 x 70 x 10 mm caddy; the 422B-55ML bottle capped in its spill tray | maker data sheet |
| Racks | Two brush racks (130 x 60 mm floor, two 16 x 40 x 30 mm bars) at (160, +/-140) mm, yaw +/-30 degrees | custom |

Counter coordinates in mm, z up. Layout A: board centre (320, 30), brush on the left rack at +30 degrees yaw, cup
(220, 10). Layout B: board and tray moved to y = -30, brush on the right rack at -30 degrees yaw, cup (220, -10).
Trials alternate A, B; the score is their mean. No seed changes the geometry (object_jitter=0).

## Scoring (simple tools: UV-A lamp, macro camera, 0.5 mm rule, side view, video)

**Score 0 for the whole trial, even if later wiped or put back, if at any moment:**

1. coating covers 0.5 mm2 or more of the original pads in total (calibrated pad masks on the UV-A macro photo);
2. one connected film touches two separate pads (a bridge in the photo);
3. a new copper scratch 1 mm or longer appears (macro photo against the baseline; in simulation, bristle tips
   pressed more than 3 mm into bare copper along a connected path of 1 mm or more);
4. the bristles are pressed more than 5 mm past first touch (side video against the rule; in simulation, a bristle
   tip or the wooden backing more than 5 mm below the board surface);
5. the board moves more than 5 mm or tilts more than 3 degrees, the tray more than 8 mm or 3 degrees, or the cup
   more than 8 mm, or tips past 5 degrees and spills (video, rule): a knocked cup or tray on a bench with a wet,
   unpowered board is treated as a spill;
6. at the end, a pool stands 1 mm or more proud (side view against the rule), or a bare gap of 0.5 mm or more lies
   between the first and last coated points of the trace (macro photo, 0.5 mm grid);
7. the record is incomplete (missing observation).

**Otherwise, coated length of the repaired copper, counted in 0.5 mm steps on the registered UV-A photo:**

| Score | Coated length of the 14 mm repair | Read with |
|---|---|---|
| 0 | under 25% | photo |
| 0.25 | 25% or more | photo, 0.5 mm grid |
| 0.5 | 50% or more | photo, 0.5 mm grid |
| 0.75 | 80% or more, or 95% or more without the final gate | photo, 0.5 mm grid |
| 1.0 | 95% or more (27 of 28 steps) and the final gate | photo, video |

Bare ends count against coverage. An internal bare gap scores 0 (rule 6) because it hides exposed copper inside
a coat that looks finished; a short coat at an end is visible and is graded as partial coverage. The same pad
keep-out applies at every tier.

**Final gate for 1.0:** the whole brush is released on either rack (wood resting on both bars), the board rests on
all four standoffs inside the tray, the cup stands on its caddy and the tray on the counter, the board, brush, cup and tray are all still (under 3 mm/s and 0.03 rad/s), and
both arms are at least 30 mm from all four for 2 seconds (video).

**In simulation** the same rules are computed every step from object poses and the declared coating model below;
the film is sampled in 0.5 mm cells and counts as visible from 25 micrometres wet thickness. The arm clearance uses
the arm links' surface points against each scored part's full bounding box. Any missing or invalid state scores 0.

## Declared coating model (the simulator does not do liquids)

Each of the brush's 21 bristle bundles carries a finite load, dry at the start. A bundle loads only while its tip is
inside the cup's liquid (1 microlitre/s, up to 3 microlitres), debited from the 5 mL in the cup; tipping the cup more
than 5 degrees spills it. A wet bundle whose tip is pressed into the board surface deposits into the 0.5 mm cells
within 1.1 mm of its tip, at a rate that grows with how far it is pressed (an estimated 100 N/m per bundle, used as
a force proxy, not applied back to the tool). More than 2 mm of compression splays the footprint by 2 mm per extra
millimetre, up to 4 mm. Deposited volume equals the bundle's loss: an empty brush lays nothing, a dry or hovering
brush lays nothing, and film never disappears. The wooden backing is checked separately for intrusion. These rates
are estimates, not a calibrated liquid; fixture/SURROGATE.md gives a repeatable fluorescent stand-in and the
checks that would relate it to the model on hardware. No such hardware check has been run.

## Fidelity

Board, pads and trace come from the maker's CAD; brush and bottle sizes from maker charts. Brush thickness and mass,
cup mass and component heights are estimates; header pin tails and solder fillets on the underside are not
modelled, so pads are flat annuli. Coating is the declared finite-load transfer rule above, not fluid flow: no
capillary spreading, levelling, cure or insulation. Bristles do not collide. Their estimated 100 N/m per bundle
(about 0.3 N for three bundles at 1 mm) is used only as the transfer and damage rule and is not fed back to the
tool; real horsehair at that load bends rather than pushes the brush away. The wooden backing is a solid collider.
The brush is held by friction in the gripper. The board rests unscrewed on its standoffs; the cup is a fabricated
metal cup drawn in fixture/setup.svg.

## Bypass table

| Shortcut | Result |
|---|---|
| Do nothing; move the board; hover a wet brush | no film on the trace: 0 |
| Sweep with a dry brush | no transfer: 0 |
| Broad sweep across the area | pad fouling or a bridge latches 0 |
| Press hard to lay a thick coat | splay onto GPIO1:24 or TP1 latches 0; over 5 mm latches 0 |
| Foul a pad, then wipe or move away | the zero is latched for the trial |
| Keep stroking with an empty brush | the load is finite; no new film |
| Lift the board off its standoffs, or move it | support or displacement rule: at most 0.75, or 0 |
| Finish but keep holding the brush | capped at 0.75 until released and hands clear for 2 s |

**One arm:** the job is a single-arm task. The arm on the brush's side does it; the other arm stays parked. Either
arm may be used, and nothing in the scoring needs two.

## How this differs from close tasks

- **TK-7P97ME** (accepted, finish painting a panel with a roller): an open panel and a roller with continuous
  coverage. Here a finite brush load must be picked up from a cup and reloaded, and only the leading bristles of a
  raked brush may touch, between native pad annuli 1.5 mm from the target; the roller task has no splay or reload.
- **TK-RB2759** (pending, paint fingernails) and **TK-QV6CAT** (pending, blush inside an oval): brush deposition
  inside an outline on a practice body. Here the outline is a 0.254 mm copper trace on real controller hardware,
  the keep-outs are solderable contacts, and the tool must be loaded from a cup and returned to a rack.
- **TK-SANNN9** (accepted, scratch a gift card): removes a coating with a coin. **TK-4T97N9** (accepted, spackle
  nail holes): fills and strikes flush. Neither loads a finite liquid or controls a bristle footprint.
- **PR 2563** (PCB stencil printing): registers a stencil and fills apertures. Here the coating is brushed directly
  onto exposed copper.

## Design notes and limits

1. *Is a full score feasible?* A scripted, open-loop demo earns 1.0 in both layouts (Results).
2. *Why standoffs?* Resting underside-up on its own component bodies the board rocks; a light touch tipped it past
   3 degrees. Boards are held level for bench rework, here on four standoffs under its own mounting holes. The
   3-degree whole-trial rule is unchanged.
3. *What liquid for shared-rig trials?* fixture/SURROGATE.md gives a water-washable fluorescent stand-in (60.00 g
   Sigma-Aldrich G5516 glycerol, 39.99 g deionized water, 0.010 g Kingscote Bright Dyes FLT Yellow/Green 106001),
   its preparation, and the dry, hover, light, overcompressed and depletion checks that relate it to the declared
   model. It has not been tried on hardware.
4. *Who does this work?* Electronics assemblers repairing defects on boards (O*NET core task above); IPC-7711/7721
   treats coating replacement after a repair as its own step.
5. *How does it differ from painting a panel (TK-7P97ME)?* The differences are measurable in the scorer: one load
   stretched over a slow 14 s stroke runs dry at 36% coverage, so the brush must be reloaded; a mirrored brush
   heading started at the trace end fouls GPIO1:24 with its trailing bundle; pressing 3 mm splays the footprint onto
   the same pad (fixture/check_material.py). A roller on an open panel has none of these.
6. *Is the target real?* It is the board's own ANALOG8 route from the maker's CAD, 0.254 mm wide; no copper is
   added, widened or moved.

## Results

**Scripted demo (open loop, one arm): A 1.0, B 1.0, mean 1.0.** The plan uses only the published starting layout
and fixed per-arm landing offsets measured from earlier scripted runs of the visible brush (task.py `CAL`); it never
reads live poses, forces, the film or the score. The arm on the brush's side picks the brush up by its handle,
tips it 40 degrees and dips the leading bristles 1.5 mm into the cup for 3.5 s, draws a 4 s stroke along the trace
(1 mm nominal compression; 0.1 to 1.9 mm measured), returns to the cup to reload, draws a second 4 s stroke in the opposite direction, puts
the brush back on its rack and withdraws.

| Measured in the run | Layout A | Layout B |
|---|---|---|
| Coated 0.5 mm steps (visible film 25 um or more) | 28 of 28 | 27 of 28 (first step 18 um) |
| Internal gap | none | none |
| Film on any pad | 0 mm2 | 0 mm2 |
| Thickest film | 0.37 mm | 0.33 mm |
| Deepest bristle compression | 1.9 mm (no splay) | 1.8 mm (no splay) |
| Board tilt and movement | 0.0 degrees, 0.0 mm | 0.0 degrees, 0.0 mm |
| Final gate | brush on rack, all still, arms clear 2.05 s | same |
| Peak finger effort | 8.5 N | 8.7 N |

In layout B the right arm's reachable wrist lean turns the brush so that its leading bundle nearest GPIO1:24
trails the middle one; the stroke therefore starts 0.95 mm further along the trace and leaves the first 0.5 mm step
thin, which still reaches 95% (27 of 28). Each stroke runs its 3 microlitre load nearly dry; the second, reloaded
stroke in the opposite direction covers what the first one thinned.

**Dream-1: A 0.0, B 0.0, mean 0.0.** Same source, instruction and `--epochs 2`. Both trials ran to the
2,400-step limit without a refusal; the arms moved but never picked up the brush (it stayed on its rack, 0 mm
moved), so no coating was loaded or laid and no fault was recorded.

No hardware trial has been run.

## Reproduce

Embodiment yam-isaac-lab-arena with Inspect Robots. 2,400 steps (120 s at 20 Hz) per layout; `--epochs 2` runs
A then B.

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 --instruction "Restore the protective overcoat on the repaired trace with the brush, keep coating off the pads, and put the brush back on a rack." --scorer task_score --max-steps 2400 --epochs 2 --store-frames
inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 -E control_hz=20 --instruction "Restore the protective overcoat on the repaired trace with the brush, keep coating off the pads, and put the brush back on a rack." --scorer task_score --max-steps 2400 --epochs 2 --store-frames
```

Offline checks: `fixture/check_material.py` (dry, hover, finite supply, pad fouling, gap, pool, held tool) and
`fixture/check_protocol.py` (hold timer, support, stillness, cup spill, backing intrusion, scars, missing state).
These test the scorer with direct inputs; they are not robot runs.

**Preparation, once per test board:** scrape the solder mask off the 14 mm interior of ANALOG8 with a fibreglass
pen, keeping the copper width, and clean with isopropyl alcohol.

**Reset between trials:** unpowered board, every pad clean and dry, repaired copper bare; board resting on the four
standoffs in the tray at the layout's position; dry brush on its rack; cup refilled to 5 mL, gently mixed, no
thinner; bottle capped; baseline UV-A photo taken. For repeated trials use the water-washable stand-in and wash the
board with deionized water, then isopropyl alcohol, and dry it; keep 422B for a final check on a fresh prepared
board. Film and violation records reset only with the environment.

Licences: authored code MIT, authored text and drawings CC BY 4.0; the board geometry derived from the maker's CAD
keeps its CC BY-SA terms (fixture/source/).
