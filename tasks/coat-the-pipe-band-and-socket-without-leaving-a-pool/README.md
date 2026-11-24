# Coat a PVC pipe band and fitting socket, leaving no bare window or cement on the stop or in either bore

Plumbers coat the pipe end and fitting socket before joining PVC drain pipe. Here two bench vises hold a
1-1/2 in Charlotte Pipe joint ready for the coating step. The robot takes the Oatey Rain-R-Shine lid dauber,
coats the pipe down to its socket-depth line, coats the socket wall, dips again, gives the pipe a second full
pass, then puts the dauber back. Joining and bond testing are separate operations.

The folder name retains “pool”; pooling is no longer scored. The scene has two arms; the scripted demo uses
one arm against the two vises.

## Revision response

The substantive verdict, verbatim:

> Construct validity (not met): The simulated swab's non-colliding soft layer coats on geometric overlap without producing the stated 0.2–1 N contact force.
>
> Scoring (not met): The specified overhead and wrist videos cannot reliably measure hidden swab compression or identify a quarter of the socket being recoated.
>
> What to change: Make the simulated swab exert approximately the real contact force during coating rather than allowing its soft layer to pass through surfaces without resistance.

1. **Non-colliding layer and force.** The entire 19 x 25 mm wool cylinder now collides through a compliant
   material. The wire collides too. `read_contacts` reads PhysX contact points and normal forces, and
   `wool_contact` excludes contacts attributable to the lid or wire. `apply_contacts` credits paint only with
   at least 0.20 N of reported total wool normal force on that surface, under the posed wool and within 6 mm
   around the circumference of a reported contact point. No reported contact and 0.19 N both leave no paint.
   Contact forces act on the held tool and the fixed vise-held parts through the physics engine.
2. **Hidden swab compression.** No score depends on squeeze depth, a hidden core, or maximum indentation.
   The old 1–5 mm squeeze rule is removed from code, text and fixture data. Real scoring checks visible blue
   in the photographs. The simulated 0.20 N minimum is a proxy for film transfer, not a force reconstructed
   from video; force above 1 N carries no separate penalty.
3. **A quarter of the socket recoated.** The socket repeat-coat penalty is deleted. No score counts socket
   recoats or infers a pool from hidden history. The remaining contamination outcomes are visible blue on
   the stop, in either bore or on the bench, checked after the trial. In simulation those require actual
   reported wet-wool contact, not merely entering empty bore air.
4. **Approximately real contact force.** The dry new 19 mm Oatey wool dauber has about 40 g-force/mm
   radial resistance, obtained by phone from a technician who works with this product (Nov 2026).
   That is 0.392266 N/mm. The compliant material uses 392.266 N/m at each contact point.
   An earlier simulation static press, with the material set to 200 N/m, gave 0.3-0.45 N at 0.9 mm
   indentation (about 330-500 N/m, 0.85-1.3 times the technician figure). It is a historical diagnostic,
   not a stiffness calibration of the final 392.266 N/m source. The final demo's coating forces of 0.2-1.9 N
   correspond to about 0.5-4.9 mm of squash using the technician's dry figure. The upper end lies within
   the tested 1-5 mm range; the lower end extrapolates below its first point. These are calculated dry-curve
   equivalents, not measured wet deflections. Wet friction and minimum transfer force remain estimates.
   The demo's simulated coating forces are listed under Run and results.
5. **Coarse visible order.** Full score needs a whole pipe orbit before first socket entry and another after
   final exit and a fresh dip, plus final photographs showing complete coverage. Simulator before/after
   coverage masks and accumulated angular travel implement those passes. Real graders need not identify
   which individual window was repainted when. Reset uses a new coupling pushed dry onto a reusable stub;
   no cemented support must cure between trials.

## Run and results

```sh
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/coat-the-pipe-band-and-socket-without-leaving-a-pool/task.py \
  -E object_jitter=0 -E control_hz=20 \
  --instruction "Coat the pipe all round down to the line, then the socket wall, then the pipe all round again after a fresh dip. Keep cement off the stop and out of both bores. Put the dauber back and let go." \
  --scorer task_score --max-steps 2400 --epochs 2
```

Use `--policy yam_scripted` for the open-loop demo. Trial k uses lot k mod 2; `TASK_LOT=A` or `TASK_LOT=B`
pins a lot for debugging. One trial allows 120 s at 20 Hz. The planned demo lasts 87 s in A and 89 s in B.
`python near_misses.py` runs synthetic contact-record scoring controls; those are not physical trials.

**Revised-source evidence (Nov 24 UTC).** Scripted demo A/B 1.0/1.0; Dream-1 A/B 0.0/0.0.
Both runs used identical Python source; task.py SHA256 `20c0489924152daca5e38c4e4bf0f8a25cae9389bc4ea1b4d71bfa3784d17c32`.
Demo `demo-20261124T034740`; Dream-1 `dream-20261124T035644`; a 2,400-step limit per layout, `--epochs 2`.
Dream termination reasons: ['max_steps', 'max_steps']. No refusal or abstention is recorded in the completed Inspect JSON (abstentions {}, errored trials 0).

Force measurements from the scripted demo:

| Lot | Surface | Min / median / max N while crediting paint | Steps |
|---|---|---|---|
| A | pipe | 0.228 / 0.640 / 1.125 | 172 |
| A | socket | 0.229 / 0.704 / 1.502 | 66 |
| B | pipe | 0.230 / 0.713 / 1.198 | 178 |
| B | socket | 0.205 / 0.626 / 1.805 | 78 |

Peak total wool contact forces across all steps, including steps earning no new paint: A pipe/socket 1.125/1.502 N; B pipe/socket 1.198/1.908 N.

These are simulation force measurements. Contact multiplicity can raise force above 1 N; no upper-force
score is claimed. Dry technician resistance, estimated wet-film transfer and simulated forces remain distinct.
In the scripted demo, both full pipe passes and the socket were complete in both layouts, with no contamination/state fault.
The 27 synthetic scorer controls pass.

The videos for this revision are new; the earlier videos showed the rejected overlap model.

## Scene (millimetres; x away from the arms, y to the robot's left, heights above the bench)

| Object | What it is | Size and place |
|---|---|---|
| Pipe | 1-1/2 in Schedule 40 PVC (Charlotte Pipe, ASTM D1785/D2665): OD 48.26 (1.900 in), minimum wall 3.68 (0.145 in) | 100 long, standing on the vise base, top end 115 above the bench, axis at (290, -220). Black line round it 18.3 below the top (the socket depth). |
| Pipe vise | small cast-iron bench vise, jaws closed on the pipe (sizes estimated) | base 120 x 100 x 15; jaws 70 x 18, tops 68 above the bench. Fixed. |
| Coupling | Charlotte Pipe PVC DWV coupling, part no. 100, 1-1/2 in, hub x hub: Socket mouth 48.6, depth 18.3, outside 56: figures obtained by phone from a technician who works with this product (Nov 2026). Straight socket model; 3.175 centre stop from the catalogue; model overall length 39.775. | Lower hub pushed dry onto a 1-1/2 in pipe stub held in the second vise. Mouth 105 above the bench, stop face 86.7, axis at (200, -100). |
| Coupling vise | same vise, jaws 58 high | Fixed. |
| Cement | Oatey Rain-R-Shine medium blue PVC cement, 8 fl oz (237 mL), item 30891 | Round can 70 across, 87 to the shoulder, neck 36 across up to 95 (sizes estimated); chosen cement surface 40 above the can bottom; 0.25 kg. Centre (180, -270). |
| Dauber | the can's own lid dauber: blue lid 40 across x 16 tall, 2.5 mm wire 45 long, wool swab 19 across (obtained by phone from a technician who works with this product, Nov 2026) x 25 long (length estimated); 12 g | Lid resting loose on the neck (not screwed down), swab hanging in the cement. |


Lot B moves the can, with its dauber, to (245, -345). No other object moves between layouts. Results average A and B.
`parts.py` builds primitives and compliant contact materials; fixed workholding and the existing bench collider
are authored as kinematic bodies so contact filters can name them. Their poses are fixed. `lots.py` declares
these two layouts. The line, label and cement level are drawn without collision; the wool and wire collide.
All assets are task-authored primitives. No third-party mesh or texture is distributed.

## Scoring

Use overhead and wrist video for gross order and can movement. Within 30 s of finishing, photograph the pipe
from four sides through a transparent sleeve with 6 x 48 windows held 2 mm off the wet surface. Photograph the
socket wall through its mouth under a ring light, comparing its sectors against a dry reference socket bearing
the same grid. Tilt the inspection mirror as needed to see every sector; an occluded sector is unscored until
photographed. Inspect both bores and the stop with torch and mirror. A window is coated when blue is visible in
it; the scoring resolution is about 3.0 x 3.2 mm, not microscopic film continuity.
Print [the pipe sleeve grid](fixture/pipe-sleeve-grid.svg) and [dry socket reference grid](fixture/socket-reference-grid.svg)
at 100% scale and verify the 50 mm bar. The sleeve print allows for its 2 mm radial stand-off; its printed
sector width is consequently larger than the corresponding pipe-surface window.

**Whole-trial zero:** wet wool deposits blue on the stop face, inside either bore, or on the bench outside
the can; the can tips over 15 degrees or slides over 20 mm. The simulator records actual positive-force wool
contact with these surfaces. Stop-face contacts also require a vertical contact normal, distinguishing
the stop from the adjoining low socket wall. Simply being in empty air inside a bore does not deposit paint. Blue markers render the actual
contamination contact locations, as well as the coated target windows.

**Coating proxy:** each surface has 6 rows x 48 columns, sampled at 3 x 3 points per window. A loaded wool
contact must exert at least 0.20 N total normal force on the scored surface. Only windows under the wool and
within 6 mm of a contact around the circumference count. No interpolated path paints between reports. One dip
loads 3,200 mm2, estimated from 0.5 mL divided by a 0.15 mm film; each surface is about 2,770 mm2. The swab tip
8 mm below the cement level reloads it. A repeated window on the same dip consumes no further budget and adds
no coat. A fresh dip is needed for the second pipe coat. Neither squeeze depth nor socket recoat share is scored.

**Every nonzero score:** dauber and can at rest (linear speed <=3 mm/s, angular speed <=0.03 rad/s), with both
hands at least 30 mm from both objects for the last 1 s. Hand clearance samples all faces of both grippers.

| Score | Condition |
|---|---|
| 1.0 | All windows of both surfaces coated; a complete pipe orbit before first socket entry and a second after final exit and a fresh dip; dauber returned to can, lid within 3 mm vertically and 6 mm horizontally of the neck, hanging within 10 degrees of upright |
| 0.75 | All windows of both surfaces coated, without meeting all full-score order/stow conditions |
| 0.5 | One surface complete and at least half of the other |
| 0.25 | One surface complete |
| 0 | All other outcomes, including any whole-trial fault or missing hands-clear hold |

The simulator requires complete pipe coverage masks before/after the socket phase and at least 360 degrees
of actual pipe-orbit travel in each phase. Re-entering the socket clears the after mask and orbit.
Photographs provide final coverage in a real trial. The overhead video checks, as countable events, that the wool
stays in contact with the band below the pipe top through a full turn before the first socket entry, dips again
after the last socket exit, and then makes another full turn. A second coat of blue on blue is not photographed;
the simulator's per-window after-pass mask is its stricter stand-in for that full turn.
`classify`, `wool_contact`, `apply_contacts`, `in_order`, `tier` and `score` contain the corresponding rules; non-finite contact data fails closed.

### Negative controls

The 27 synthetic scorer controls cover no-op, no report despite overlap, 0.19/0.20 N, force above 1 N,
wrong-surface and wire-only force, held dauber, a short release hold, missing second pass, wrong order,
additional socket coating, return to socket after the last pipe pass, single-surface partial credit, bore/stop/
bench contamination, empty bore air, non-finite force, half an orbit, and normal-based separation of the low wall and stop face. They test the rule with explicitly
provided contact records; GPU contact generation and demo feasibility are checked separately.

## Fidelity and estimates

Pipe outside diameter/wall and coupling stop are maker dimensions at true size. Figures obtained by phone
from a technician who works with this product (Nov 2026) are: a dry new Oatey 3/4 in dauber ball 19 mm across,
about 40 g-force per mm radial squash against 1-1/2 in PVC (kitchen scale, 1/3/5 mm, three runs), socket mouth
48.6 mm, socket depth 18.3 mm and coupling outside diameter 56 mm. The simulation uses those figures.
Pipe and coupling walls are modelled as overlapping vertical cylinders.
The socket is represented as straight rather than modeling an unmeasured taper; overall model length is
2 x 18.3 + 3.175 = 39.775 mm. Vises, can, lid, wire and wool length remain estimated. Cement level is a scene choice.

The whole wool is a rigid geometry with a compliant contact material, not a deformable mesh. Its material
stiffness is 392.266 N/m per contact (40 x 0.00980665 N/mm), damping 0.5 N s/m, wet friction 0.20/0.15 (estimate).
PhysX contact multiplicity can increase aggregate resistance above the dry technician figure. This is not a
calibrated wet-wool force curve. The 0.20 N coating minimum is about 0.5 mm of squash at the technician's 40 g/mm, a light wiping touch;
no wet-film transfer measurement has been supplied, so it remains an estimate. Damping contributes
0.005 N at 10 mm/s. Dry resistance, wet film transfer and simulated forces are separate quantities.

The 6 mm circumferential footprint and wool-height mask approximate film transfer around each reported contact;
paint is a declared state, not fluid. There is no solvent softening, cure, bond, hydrodynamics or pooled-volume
measurement. The workpieces/vises are fixed, can loose, and dauber held only by frictional grip. No grasp weld,
scoring force or pose correction carries the tool. Two published layouts test repositioning, not generalization
to unseen layouts. Forces above 1 N are not scored as failures because hidden compression cannot be read from video.

### Measuring the open estimates

- **Wet-wool transfer and force:** hold a spare pipe horizontally in a jig on a tared scale and press the dauber
  vertically downward, or use a force gauge aligned with the push. Record first continuous blue transfer in N,
  then force at 1/2/3 mm indentation. Convert scale grams to N with 0.00980665 N/g. A sideways force on a
  horizontal kitchen scale is not a valid reading. Use the transfer reading to replace `F_COAT`; compare the wet curve with the dry technician figure before revising `SWAB_K`.
- **Load:** weigh the dauber dry, after one dip, and after coating until dry; photograph/ruler the coated area
  and replace `LOAD_AREA`. The 3,200 mm2 figure remains an estimate, not a measured consumption rate.
- **Geometry:** caliper the neck/lid/wire/wool; compare pipe/coupling with the table. The real lid must be
  graspable by the parallel fingers and the wet wool must fit through the can neck without snagging.

## Real kit, sources and enrollment

Prices reopened Nov 24, 2026, USD before tax/delivery; no purchase was made:

| Item | Opened price and source |
|---|---|
| Two Central Machinery 59113 2-1/2 in table vises, smooth jaws | 2 x $22.99 = $45.98; [Harbor Freight](https://www.harborfreight.com/2-12-in-table-swivel-vise-59113.html) |
| Charlotte PVC071120200HA 1-1/2 in x 2 ft Schedule 40 pipe, cut into 100 mm offcuts beforehand | $5.93; [Home Depot](https://www.homedepot.com/p/Charlotte-Pipe-1-1-2-in-x-2-ft-PVC-DWV-Schedule-40-Pipe-PVC071120200HA/100581718) |
| Charlotte PVC 00100 0800HA coupling, UPC 611942031894 | $0.79 each; [Family Hardware](https://www.familyhardware.com/product/charlotte-pipe-1-1-2-in-hub-schedule-40-dwv-pvc-coupling/442505) |
| Oatey Rain-R-Shine 30891 8 oz can with lid dauber | $9.29 each, seller minimum two ($18.58); [WebstaurantStore](https://www.webstaurantstore.com/oatey-rain-r-shine-30891-8-oz-pvc-medium-blue-cement/83530891.html) |

These core items total $71.28. A bench, PPE, solvent-proof catch tray, torch, inspection mirror, printed grid,
ruler and cameras are also needed; their prices are not included in that subtotal. The simulation vises are
simplified dimensions, not a scan of model 59113. Set the real jaw height/bench shims to the specified pipe and
socket heights; use the smooth plastic/wood jaws. No custom machined or curing specimen is required.

Before enrolling a specimen: pipe cut square and deburred, white surfaces clean and bare, line 18.3 mm below
the top; coupling dry and bare; bores and stop clean under torch/mirror; can cement fluid after stirring, not
gel-like; wool intact and neck unobstructed. Check the coupling and pipe do not shift visibly (>1 mm against a
ruler) under about 3 N lateral force (300 g hanging mass via a horizontally guided string); clamp/reset again if they do. Check every
photographic sector is visible in the reference views before starting. Wet-film transfer/load estimates still need the
measurements above; a model matching them numerically is not real validation.

Sources for dimensions and procedure:

- [Charlotte DWV dimensional catalogue](https://www.charlottepipe.com/uploads/documents/technical/Plastic_Pipe_Fittings_DC-DWV609.pdf),
  printed p7: 1.900 in OD and 0.145 in minimum wall; printed p11: no.100 PVC coupling 1-9/16 in overall,
  1/8 in stop, giving (39.6875 - 3.175)/2 = 18.256 mm per side. The supplied technician socket depth replaces that derived depth in the model; its mouth/outside
  diameters are technician figures, not dimensions inferred from catalogue length.
- [Rain-R-Shine technical sheet (30893, same cement)](https://blobanarus.blob.core.windows.net/oatey-oateyproducts/30893.pdf), p2,
  direction 6: “leave no uncoated surface”; pipe/socket/second-pipe sequence. It gives 1–2 minutes set time at
  70–90 F and warns “HIGHLY FLAMMABLE LIQUID AND VAPOR”. It provides no wool force figure.
- [O*NET 47-2152.00](https://www.onetonline.org/link/summary/47-2152.00) includes assembling pipe/fittings using
  cement and plastic solvent; [BLS](https://www.bls.gov/ooh/construction-and-extraction/plumbers-pipefitters-and-steamfitters.htm)
  reports 510,600 plumbers, pipefitters and steamfitters in 2025. The coating step is part of that work, not a
  claim that a robot performs the occupation or that this task predicts joint strength.

## Reset and real operation

1. Clamp a fresh prepared 100 mm offcut, top 115 mm above bench, axis (290, -220), line 18.3 mm below top.
2. Push a fresh dry no.100 coupling onto the reusable lower stub, mouth 105 mm above bench, axis (200, -100).
   Check the holding test above. Nothing is cemented or cured for the support.
3. Stir cement and set its surface 40 mm above the can bottom (a scene choice). Can at (180, -270), or
   (245, -345) for B. Lid rests loose on neck, wool in cement. Arms ready, fingers open; start within 15 s.
4. Use a new bare offcut/coupling for each scored trial; reuse the dauber and stub. Prepared replacements make
   reset about a minute. Photograph within 30 s after the trial.

Follow the product label: use ventilation suitable for solvent vapor, keep ignition sources away, use suitable
protective gloves and eye/face protection, and a solvent-proof catch tray. Follow the label's respiratory protection
requirements. Protect gripper pads with disposable finger coverings and replace any wetted coverings. Rehearsal
with blue water-based gel is a different medium and does not establish the solvent-cement force. Only the named
cement defines this task's real trial; neither the coating specimen nor the dry support joint is put into service.

## Distinct skill and remaining scope

The robot must manage a finite dauber load while wiping an outside cylinder and an inside socket, avoiding a
stop/bore and completing visible pipe/socket/pipe order. This differs from flat roller finishing (TK-7P97ME),
roller loading (TK-F6JFF7), putty-knife filling (TK-4T97N9), and nail-polish application (TK-RB2759).
Thread tape winds a tensioned strip; dry PVC assembly
aligns/inserts parts. Neither is this contact coating step. No fluid bond or human baseline result is claimed.
