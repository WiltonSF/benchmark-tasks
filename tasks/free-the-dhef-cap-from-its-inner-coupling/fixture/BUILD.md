# DHEF-20 front-end replica for the real trial

A printed reproduction of the parts of the Festo DHEF-20 adaptive shape gripper that this task's removal touches: the
nose with its three-lug bayonet, the knurled catch ring and the cap's bush, the piston rod, the coupling on the rod's
end, and the blue releasing ring with the latch it works. A genuine Festo DHAS-GA-B22-S inverting cap (part 8097634,
bought as a spare part) goes on it, so the specimen is the maker's own silicone cap. The replica design stands on a printed
bracket screwed to a board clamped to the table, with the gripper's axis at the pose in `task.json`. No gripper,
robot arm or compressed air is needed.

**Status: designed and checked in CAD only.** Nothing here has been printed, assembled, fitted to a cap or measured.
Every force below is a design value or an estimate, and the checks in `ENROLMENT.md` are what a lab runs before the
first trial.

This reproduces nominal front-end geometry from `task.py`, not a measured Festo coupling. The supplied 13 mm
head / 6 mm neck / 10 mm head length and 37 mm rim seat are design inputs. Caliper measurements of an acquired
genuine cap set the fitted pocket, keeper clearance, cam slope and rim seat; a usable undercut must first be
confirmed. Keep fitted outputs and their manifest separate from the nominal STLs. A fit failure stops the genuine-cap
route. The mould fallback is an unqualified alternative, not evidence that the genuine cap fits.

## Rebuild and cost

From this folder, verify the downloaded files before regenerating anything:

```bash
shasum -a 256 -c SHA256SUMS
```


If regeneration is desired, use a separate copy of this fixture folder, with Python 3.9:

```bash
python3 -m venv /tmp/dhef-fixture-venv
/tmp/dhef-fixture-venv/bin/python -m pip install -r requirements.txt
/tmp/dhef-fixture-venv/bin/python make_replica.py
shasum -a 256 -c SHA256SUMS
```

The source uses parametric Python/manifold solids rather than OpenSCAD. `P` holds replica design choices;
simulation stations and travels are read directly from `../task.py`. STL coordinates are millimetres.
The generator regenerates nominal geometry, drawings, checks and hashes; verification afterward checks regenerated consistency, not original downloaded-file integrity. Keep measured variants in a separate copy.

Primary kit: **US$302.48 listed purchases + US$86.40 estimated print services = US$388.88**, before other
shipping and tax. Every full-pack cost is in `BOM.csv`; quoted rates and estimated weights/hours are separated.
No print-service order, quote or tolerance acceptance has been obtained. No printer purchase is required.
Existing table, test arms, cameras and inspection lamp are excluded as trial infrastructure. One-time assembly
uses the listed iron, keys, screwdriver and knife. The primary route uses exported print fits and soft-pine
self-tapping screws; if tapping/drilling/postprocessing proves necessary, obtain and record that extra cost.
A few hundred dollars is a budget estimate, not a guaranteed delivered price. The old unquoted JLC minimum and
unavailable hardware are not counted. The fallback cast-cap route adds separate mould/material costs in BOM F1.

## Files

| File | What it is |
|---|---|
| `make_replica.py` | Generates every STL and drawing from the dimensions in `../task.py` (read from its source) and the design values at its top; runs the checks; exits non-zero if any fails. Python 3.9+; pinned packages in `requirements.txt`. |
| `stl/` | One STL per part, each in its assembled place (frame below). |
| `CHECKS.txt` | The generator's checks on the meshes and design values, and their results (80 passed at the last run). |
| `drawing.pdf` | Three sheets: section through the axis, the latch at 0, 1.5 and 3 mm of releasing-ring travel, the bayonet seen from the cap's tip. `section.png`, `latch.png`, `bayonet.png` are the same sheets; `assembly.png` is an exploded render. All are CAD images, not photos. |
| `BOM.csv` | Parts and tools, each with the listing, price as shown, URL and date checked. |
| `ENROLMENT.md` | Sizing the replica to the cap in hand, acceptance checks with numbers, the check before each trial. |
| `SHA256SUMS` | Hashes of every file here, written by the generator after all edits. |

STL frame: z runs along the gripper's axis from its mounting face toward the cap's tip (task.py's `u`, mm); y is up;
x is level across the axis (world +x in the task's setup). Slicers expect the bed's normal along z, so most parts
print as exported (axis vertical); the bracket is turned so its base lies on the bed.

## Nominal shared dimensions and design targets

Nominal simulation dimensions are read from `task.py` and checked on CAD meshes (`CHECKS.txt`). Force targets are not physical measurements. Adapted cap-fit inputs must be logged separately.

| Value | task.py | Replica |
|---|---|---|
| Catch ring 57.5 mm across, u 117 to 141.5; lugs u 117 to 121, 18 deg wide, at 90, 210, 330 deg | `R_RING_OUT`, `U_RING0/1`, `U_LUG0/1`, `LUG_W`, `LOCK_TH` | same |
| Nose 38 mm, front at u 128; lip u 121.5 to 125.5, 48 mm across, three 50 deg gaps at the lugs' open places; stops 8 deg wide past locked and past open | `R_NOSE`, `U_NOSE`, `U_LIP0/1`, `R_LIP`, `GAP_W`, `STOP_W` | same |
| Catch turns 45 deg, counterclockwise seen from the tip, from locked to open; locked, the lugs are behind the lip (a pull is blocked); open, they clear it | `TURN`, `OPEN_TH` | same (checked by projecting the lugs on the lip) |
| Bush bore 30 mm; cap rim seat 37 mm, its front at u 146.5 | `U_BUSH1`, carrier shapes | same |
| Rod 10 mm; pushed back up to 55 mm; about 6 N to move | `R_ROD`, `ROD_TRAVEL`, `F_ROD` | 10 mm hardened shaft; collar meets the bulkhead after 55 mm; a nylon-tip brake screw to be set to 6 N (enrolment) |
| Coupling 19 mm across, u 188 to 216; knob pocket u 201 to 212, 14 mm | `R_CPL`, `U_CPL0/1`, `U_POCKET0/1`, `POCKET_HALF` | same; the pocket is round (the simulation draws it square) |
| Keepers u 212 to 216, inner edge 3.5 mm off the axis, open 4 mm | `_keeper_shapes`, `KEEP_OPEN` | same |
| Releasing ring 22 mm across, u 187 to 200.5, 3 mm travel toward the rod | `R_SLEEVE`, `U_SLEEVE0/1`, `SLEEVE_TRAVEL` | same; stops at both ends |
| Return force on the releasing ring 1,000 N/m | `SLEEVE.spring` | Lee Spring LC 036G 06 M, published rate 1.00 N/mm, plus 0.3 N preload to hold the ring on its stop |
| The latch lets the knob go from 1.5 mm of ring travel | `SLEEVE_OPEN` | the keepers reach the 13 mm head's edge at 1.5 mm (cam 2 mm of lift per mm of travel) and are fully open (4 mm) at 2 mm |
| Knob head 13 mm, 10 mm long, neck 6 mm | `R_HEAD`, `U_HEAD0/1`, `R_NECK` | task.py's estimates by default; replaced by caliper values of the cap in hand (`ENROLMENT.md`) |
| Axis level, 80 mm above the table; mounting face at y 165, axis at x 270, cap toward -y | `H_AX`, `X_AX`, `Y_F` | the bracket and its 19 mm board give the 80 mm |

## What is the replica's own design

Festo has not published the inside of the DHEF's coupling or its bayonet (`task.py` calls them estimates), so these
parts are a design that does the same job in the same space:

- **Latch.** Each of the two keepers is a pair of cheeks either side of a flat drawbar. The two drawbars run forward
  from the releasing ring in channels in the coupling's top and bottom; a 1 mm dowel through each drawbar rides an
  inclined slot in its cheeks. Pushing the ring toward the rod draws the dowels back and the slots lift the cheeks
  outward, nominally 2 mm per mm of travel, to 4 mm at 2 mm; from 2 to 3 mm the slot runs straight and the keepers stay open.
  The return spring pushes the ring forward again and the same slots close the keepers, so no keeper spring is used.
  Nothing powers the latch: only the hand on the ring opens it.
- **Difference from the simulation's latch rule:** `task.py` opens the latch at 1.5 mm and keeps it open until the
  ring is back under 1.0 mm. The replica has no such band: below 1.5 mm the keepers overlap the head (by 0.5 mm at
  1.25 mm, 2 mm at 0.5 mm), so it holds the knob again as soon as the ring returns under 1.5 mm.
- **Sizing.** Measured cap inputs change the first cam slope to preserve release at 1.5 mm; the second segment still reaches 4 mm opening at 2 mm travel. The nominal supplied STLs use the simulation's 13/6/10 mm estimates. Fitted values and manifests are logged separately. Out-of-envelope caps are rejected.
- **Pull on the knob.** The head bears on the cheeks' back faces, flat across the axis, and side tongues carry the
  load into the coupling's jaws; a pull does not open the keepers. A pull does press the cheeks against their guides,
  so the ring is harder to push while the cap is pulled; Festo's manual also has the tip pressed into the coupling
  while the ring is pushed. The physical operator can unload the tip before release. The simulation does not enforce or measure a separate tip press.
- **Return spring and rod.** The spring sits on the rod between the releasing ring and a loose printed spacer (14 mm)
  that rests on the rod collar inside the nose. In front of the nose, the replica's rod therefore carries the spacer
  (14 mm across, 33.5 mm long) and the spring (12 mm across), where the simulation draws a bare 10 mm rod; both stay inside the 22 mm releasing ring's
  outline and inside the cap at the start.
- **Fixing the cap.** The cap's rim is stretched over the bush's 37 mm seat, which has a groove for its bead; the
  seat's diameter is a parameter set from the cap's measured rim. The catch ring, its front wall and the bush are one
  print (the simulation's rigid carrier); This integrated print eliminates the separate knurled nut; M3 screws join the body sections, not the cap rim.
- **Bayonet feel.** No detent: the catch turns against fit friction only, as in the simulation.
- **Refitting.** On the real gripper the knob snaps in past the coupling. On the replica the keepers cannot be pushed
  open by the knob, so the releasing ring is held pushed while the knob is pressed in, then let go.

## Printing

- **PETG** (0.2 mm layers, 0.4 mm nozzle, 4 walls, 40 % infill): `bracket` (base on the bed), `body_rear` (on its
  flange), `body_front` (on its back face), `carrier` (on the ring's back face; the 45 degree cone under the front wall
  needs no supports), `spacer`. Budget 500 g including supports; not a sliced or measured weight.
- **Resin** (SLA, or a 0.25 mm FDM nozzle if no resin printer is at hand; resin is advised for the 1 mm slots):
  `coupling`, `sleeve` (the releasing ring with its drawbars; blue resin or painted blue), the four `keeper_*` cheeks,
  `knob` (a stand-in used for the latch checks; also the insert of the fallback cap).
- Clearances in the STLs are 0.15 mm radial on sliding fits and 0.05 mm on the slots. A tight print must be rejected or reprinted; acceptance travel checks are not relaxed by enlarging working slots.

## Assembly (once; about 2 hours after printing, plus the epoxy's cure)

1. Melt the M3 inserts into the bracket's upright plate (4) and the rear body's front face (3).
2. Screw the bracket to the board (4 self-tapping wood screws; seat carefully in soft pine), the rear body's flange to the bracket (4 x M3 x 12).
3. Slide the shaft collar onto the rod's front end, then pass the rod's back end through the front body from the
   front, through the rear body's bulkhead and flange and the bracket. Screw the front body on (3 x M3 x 25 from the
   nose's face). Pull the rod forward until the collar sits on the nose's step and tighten the collar's set screw with
   the rod's front end at u 199 (the rule: 199 mm in front of the mounting face).
4. Through the 5 mm access hole under the rear body, drive the M4 nylon-tip set screw into the 3.3 mm pilot next to
   the rod (3.6 mm of PETG; the metal screw is intended to form its thread in the PETG; verify on a spare print first. If it does not start cleanly, stop and obtain a tapped part rather than forcing it) until the
   rod needs 6 N to move (`ENROLMENT.md`).
5. Spray the coupling's channels, the cheeks and the releasing ring's bore with dry PTFE lubricant and let it dry.
6. Slide the releasing ring onto the coupling from the front, drawbars in the channels, until its back wall meets the
   coupling. Drop each cheek into its window from outside, its tongue in the jaw's groove, either side of the drawbar;
   line the slots up with the drawbar's hole and push a 1 x 6 mm dowel in from the side (at the dowel's height the
   coupling's round outline leaves the slot ends open).
7. Thread the spacer and the spring onto the rod, then the coupling (with its ring and keepers) onto the rod's end,
   and bond it with the epoxy, the keeper windows straight up and down. Let it cure.
8. Paint the index line on the catch ring and the 'locked' and 'open' marks on the locking cover into their grooves.
9. Fit the cap (`ENROLMENT.md`, Reset), set the catch ring over the nose with the lugs in the gaps, push it home and
   turn clockwise (seen from the tip) to the locked mark, the index line at the top; check the available stop play before trials.

## Placing it at the task's pose

The full 4 ft board can be used extending away from the test workspace; cutting it to 600 mm is optional.
Clamp the board on the table with two C-clamps where it overhangs the table's left edge (+y), running along y, so
that the gripper's axis lies over x 270 at 80 mm above the table, the mounting face at y 165 and the cap pointing to
-y (toward the right-hand arm). The board and bracket sit where the simulation has the UR5e's wrist and forearm; the
arms are intended to approach the cap and the gripper's front at the same nominal stations. The bracket does not
reproduce the UR5e's surrounding collision geometry, so identical arm access has not been demonstrated.

## Fallback: casting a cap

If no genuine cap can be bought, or its knob cannot be held by a keeper insert, `mould_half_xpos`, `mould_half_xneg`
and `mould_core` cast a cap to `task.py`'s outline (41 mm at the rim, 25 mm at the tip, 82.5 mm long) with a 1.5 mm
wall, around the printed `knob` (its holed disc anchors it in the tip). Platinum silicone (Shore about 20A); fill the
bolted outer halves part-way, lower the core slowly so the silicone rises up the wall and out of the lid's vents; cure
per the maker (4 h for the one listed in `BOM.csv`). This is a one-time casting, reused for every trial, not a
consumable. It is untested; its rolling force and wear differ from the genuine VMQ cap's, and a trial with a cast cap
is reported as such.

## Licence

Written for this task: `make_replica.py` under the MIT licence; the STLs, drawings and these notes under CC BY 4.0.
No Festo CAD data or drawings are used or redistributed; the outer sizes come from Festo's published catalogue as
quoted in `../README.md`.
