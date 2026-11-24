# Prepared gel sheets for repeated trials

Each trial uses one cured GelSlim 4.0 gel mold: the printed outer ring, the 1/4 in acrylic hexagon pressed into it and a
3 mm XP-565 silicone sheet cast over the acrylic. A sheet needs 16-18 h to cure, so sheets are never cast between
trials. They are cast in batches the day before, logged, and kept assembled in their molds until used. Swapping in a
prepared mold at the start of a trial takes a few minutes (an estimate, not timed).

Sources opened for this procedure (checked Nov 23, 2026; prices USD before tax/shipping):
- Maker's gel-pad steps: https://sites.google.com/view/andreasipos/projects/gelslim-4-0/gel-pad-fabrication
  (70 g base, activator 10% of the base weight, 1.5 h working time, centrifuge, vacuum, "Wait 16-18 hours for the
  silicone to fully cure", press the acrylic out, peel corner first).
- Maker's mold steps: https://sites.google.com/view/andreasipos/projects/gelslim-4-0/gel-mold-fabrication
  (laser-cut 1/4 in cast acrylic base, printed outer ring; paper backing left on; the acrylic can be flipped for a
  fresh casting surface).
- Acrylic: McMaster-Carr 8560K354, clear cast acrylic 12 x 12 x 1/4 in, $25.36 each, thickness tolerance -0.04 to
  +0.02 in: https://www.mcmaster.com/product/8560K354/

## Molds (reusable)

| Item | Per mold | Notes |
|---|---|---|
| Acrylic hexagon, 170.1 x 147.3 x 6.35, from hex_acrylic_base.DXF | half of one $25.36 sheet (two hexagons fit a 12 x 12 in sheet) | laser cut; paper backing kept on the back; measure thickness, reject outside 6.0-6.6 mm |
| Outer ring, black PLA, from outer_ring.STL | about $2 of filament (estimate) | reject if the acrylic does not seat flat on the ledge |
| Mold ID label | - | written on the ring's outer wall |

The minimum A/B inventory is **four molds/sheets in one independently logged batch**: two scored trial sheets, one strip witness and one intact-sheet/start witness. Two acrylic blanks ($50.72) plus four rings ($8 estimated filament) give **$58.72** reusable mold components, excluding cutting, printing service and consumables. Ten scored trials require five separate four-sheet batches: **20 molds/sheets**, ten acrylic blanks ($253.60) plus twenty rings ($40 estimated filament), **$293.60** mold components. A material lot or preparation day does not merge separate batches into one qualification unit.

## Casting a batch (the day before)

1. Material: one lot of XP-565 base and activator (Silicones, Inc.; current price on request). Record
   the lot numbers.
2. Room 20-24 C; record it. Molds assembled, acrylic seated paper side down, on a rack levelled with a spirit level.
3. Per mold, its own mix, as the maker does: base 70.0 +/- 0.5 g, activator 7.0 +/- 0.1 g (10% of the base weight)
   on a 0.1 g balance; mix 3-4 min; centrifuge and degas as the maker describes; pour within the 1.5 h working time.
   Batches contain 1-4 sheets, as obtained by phone from a technician who works with this product (Nov 2026). Each batch has its own process and witness records; at least four sheets are needed for two trials plus both witnesses. Do not combine sessions into a single qualified batch merely because the material lot matches.
4. Write on the batch sheet for each mold ID: weights, lot, room temperature, pour time, earliest use time (pour +
   18 h). Cover the rack against dust.
5. Two molds per four-sheet batch are witnesses (strip and intact sheet), leaving two scored specimens. Record each individual pour time; trial sheets must be 18-24 h old when used.

## Acceptance checks (before a mold is used)

- 18-24 h since pour; the sheet's top face is tack-free to a gloved fingertip at a spot outside the 24 mm
  corner bands.
- Photograph the top face with a 1 mm rule: reject any bubble, void or defect 1 mm or larger, and any sheet that is not
  level with the ring's relief step all round.
- **Strip witness, each batch:** cut one sacrificial sheet to a 24 mm WIDTH strip. Pull straight off unprimed acrylic at a recorded 90-degree angle/slow speed with a calibrated low-range instrument (e.g. PASCO ME-9511, 5 N; $28 listed Nov 23; [source](https://www.pasco.com/products/lab-supplies/measurement/metric-spring-scale-5-n-range)). Record first-lift and steady force, age and temperature, scale resolution and uncertainty. The phone figure is about 0.5 N per 24 mm strip width. The simulation uses 1.2 times this line load, not a uniform 0.6 N per full sheet.
- **Intact witness, each batch:** use the second sacrificial sheet with the named pick/handle. Record first-lift/steady/peak force versus exposed front position and before/after macro photographs. Width-scaled simulation force is 2.105 N at the two end bands and 3.683 N at broad bands. The complete measured envelope plus instrument uncertainty must be covered by these forces, and a strip witness alone cannot qualify it. Also record pick entry depth <=8 mm, handle slip, tearing/puncture and sheet bending/laying behavior. No intact-sheet force or damage/bending qualification exists here; the 1.2 multiplier is a proposed engineering bound requiring that test.
- **Pad grip record on the intact witness after demolding:** record actual pad material, normal pinch force and tangential load at slip or damage, at the same age/temperature. Use a suitable calibrated peel/pull instrument and report any over-range. Photograph both faces before/after. The simulated gel friction0.7/0.55 and damage/bending proxies are unqualified; a successful simulated grip does not establish hardware grip capacity. This reuses the intact witness, not a scored trial sheet. No completed grip record exists.
- **Ring push-out, every mold or documented matched cohort:** use a calibrated instrument covering **at least 15 N**, separate from the peel scale. [PASCO ME-9513](https://www.pasco.com/products/lab-supplies/measurement/metric-spring-scale-20-n-range) covers 20 N; the manufacturer product and measurement-category pages opened Nov24 list **$28**, with a possible5% tariff surcharge (**$29.40** budgeted). [Category price corroboration](https://www.pasco.com/products/lab-supplies/measurement). The initial page variant showed quote-only; the subsequently opened US listing explicitly displays the price. Phone figures obtained by phone from a technician who works with this product (Nov 2026): nominal7 N, range5-15 N, worn loose rings about3 N. Only standard rings whose measured peak plus uncertainty lies in **5-7 N** enroll in the current7 N simulation. Higher-force or worn3 N rings are separately labeled cohorts requiring changed constants and fresh native evidence. Do not select a loose worn ring to make the demonstration easy.
- Use the oldest accepted mold first. A mold older than 24 h is held for a separate age cohort, not pooled with this batch. The 18-24 h interval and the dimensional/weight tolerances are proposed enrollment limits, not measured process capability.

## Trial reset (per trial)

1. Take the next accepted mold, check its ID against the batch sheet, and place it at its lot position
   (fixture/dimensions.json); place the stand.
2. After the trial, photograph both faces of the sheet with the 1 mm rule.
3. Clean the acrylic and ring (soap and water, dry) and return them for the next batch.

## Equipment and price limits

Prepared-mold swaps require no curing operation between trials. The advance-preparation lab still needs the maker's
balance, mixing cups/sticks, syringes, centrifuge and vacuum chamber, plus PPE and a level rack. These are not included
in the mold subtotal. XP-565 is supplied by Silicones, Inc. / its distributor [Sterling Supply](https://www.sterlingsupplyinc.com/xp565);
a current quantity/price quote is needed. The maker's [public bill of materials](https://docs.google.com/spreadsheets/d/1LvoSZAjfaGMRUC3wUko6qwsdmanVDKur6xZltkolHr8/edit) lists XP-565 at $34.88 per purchase unit, with 946.35 mL in its allocation formula (CSV export opened Nov 23, 2026). This is a published planning price, not a verified current offer or a promise that the needed quantity is available. No complete under-$300 preparation-kit price is established here.
The PYREX 3140-90 stand has a single-unit listing at [Zoro G3248604](https://www.zoro.com/corning-pyrex-crystallizing-dish-90-x-50-mm-3140-90/i/G3248604/), $22.59/ea (opened Nov 23, 2026). The manufacturer separately lists $372.63 per case of 18. Four molds, one stand and one $28 PASCO peel scale total $109.31 before tax/shipping, acrylic cutting, silicone and advance-preparation equipment. This is a partial kit subtotal, not a complete quoted trial cost.

The XP-565 [manufacturer information sheet](https://static1.squarespace.com/static/5b1ecdd4372b96e84200cf1d/t/5b608e1a758d46a989c0bf93/1533054492488/XP-565%20%281%29.pdf) specifies 100:10 by weight, 1.5 h work time, 16-18 h cure and specific gravity 1.02. Its hardness changes with age, hence recorded trial age. It supplies no acrylic peel force. The estimated few-minute swap has not been timed on hardware; per-batch force measurements and a complete purchasing quote remain unverified.

## Blank batch record

Use one row per mold; retain rejected molds in the record. Times are local timestamps with a UTC offset. Record the witness peak and steady force separately, in N, with its age and strip width; an empty reading means unmeasured.

| Batch / mold ID | Base / activator lot | Base / activator g | Room C | Pour time | Earliest use (+18 h) | Actual use / age h | Defect check | Witness peak / steady N | Accepted / reason |
|---|---|---|---|---|---|---|---|---|---|
| | | | | | | | | | |

For each trial, also record lot A/B, mold ID, sheet before/after photo IDs, swap start/end timestamps, and any ring or acrylic damage. The swap duration is calculated from those timestamps; no completed swap record exists yet.

## Lower-equipment preparation option (proposed; not hardware-qualified)

The XP-565 manufacturer sheet also gives a cup-degassing route: mix 100:10 by weight in a container holding 4-5 times the mixed volume, evacuate at 28-29 inHg, wait for the expansion to collapse, then hold another 2-3 minutes. One 77 g mix occupies about 75.5 mL at the listed mixed specific gravity 1.02; a 1 qt cup comfortably contains its roughly fourfold expansion. Slowly pour the degassed mix into the same level assembled mold. Inspect the cured mold and use the same batch witnesses and age limits above. This proposed adaptation avoids the syringe centrifuge and compressed-air dispenser; bubble-free casting and force consistency still require qualification. The maker's original route remains available in an equipped lab.

Additional priced planning components (USD; opened Nov 23, 2026):

| Item | Price basis | Source |
|---|---|---|
| VEVOR TW-1K, 3 gallon chamber + 3.5 CFM pump, glass lid, hose/gauge/oil | $128.90 listed | [VEVOR SKU ZKBTTZ35CFM15SWOEV1](https://www.vevor.com/bomba-de-vacio-c_11109/vevor-3-gallon-vacuum-chamber-and-3-5-cfm-pump-kit-tempered-glass-lid-vacuum-degassing-chamber-kit-single-stage-vacuum-pump-with-250-ml-oil-bottle-for-stabilizing-wood-degassing-silicones-epoxies-p_010201920297) |
| Digital balance, B0897BW3B9 | $31.55 in maker BOM | [Maker BOM](https://docs.google.com/spreadsheets/d/1LvoSZAjfaGMRUC3wUko6qwsdmanVDKur6xZltkolHr8/edit); confirm at least 0.1 g readability and check with reference masses before use |
| Two HDX 1 qt mixing cups, model 1057842 | $2.28 each, $4.56 total | [Home Depot HDX listing](https://www.homedepot.com/b/Paint/HDX/Bucket/N-5yc1vZar2dZ9tkZ1z0n3ir); [product](https://www.homedepot.com/p/HDX-1-qt-Natural-HDX-Paint-Bucket-1057842/337378666) |
| Mixing sticks B09325532F, safety glasses B07GB8Y11G, nitrile gloves B00GS8W3T4 | $4.98 + $16.99 + $9.99 per listed pack | Maker BOM above |
| XP-565 purchase unit | $34.88 published planning price | Maker BOM above; current pack contents, price and availability require a supplier quote |

These components plus the $109.31 four-mold/stand/peel-scale subtotal give **$341.16** in priced planning components for one A/B pair and its two witnesses. Add the verified $4.95 iFixit pick set and about $5 estimated handle/bolts: **$351.11** planning components, adding the20N instrument at$29.40 and$1.40 surcharge on the existing5N scale gives **$381.91**, before the omissions below. They do not establish a complete current purchase quote: cutting/printing services or workshop access, level rack, inspection/ruler/caliper, clamps, reference masses, cleaning supplies,  tax and shipping remain outside that subtotal. No complete cost ceiling or successful preparation run is claimed. The four-sheet minimum already includes both required witnesses. A ten-trial inventory changes only mold components from$58.72 to$293.60 and still needs five independently qualified batches. The published silicone-unit quantity/price is unverified and cannot establish how many kits buy that inventory.

## Explicit unresolved physical gates

This native supply follows the board's invitation to describe advance casting, but it still consumes a hand-cast multi-hour-cure sheet per trial and therefore does **not** pass the stricter local ready-made/minutes-reset specimen gate. The phone cure figures are about16 h at room temperature or1-2 h near60 C; this specification retains the maker room-temperature route and does not claim an oven cure was qualified. No ready-made replica has been qualified to reproduce the bonded-sheet forces and fragility. Supply records, yield, timed swap, complete quote and per-trial cost remain empty.

The dish is a proposed loose support, not documented maker workholding. Its real stability and seam access under7 N push/start/full-width peel loads have not been tested. Simulation support geometry and a successful native demo cannot by themselves validate this physical adaptation. Before a hardware trial, record loaded stability and actual support/hand roles; do not substitute an arbitrary clamp and label it the maker fixture.

Required batch record additions: independent batch ID; two witness IDs; measured strip peak/steadyN; intact-sheet start/peakN versus front; ring peakN plus uncertainty; actual accepted/rejected sheet counts; tool and dish observations; actual trial age; swap timestamps. Empty fields mean unmeasured, never zero. Cost records must separate reusable capital, each batch's four sheets/two witnesses and two usable trials, measured yield, and labour. No defensible marginal trial cost is established from unmeasured yield or the maker's planning-unit price.

## Cost allocation for the minimum pair (planning, not a completed purchase)

Reusable mold/tool/inspection/preparation components subtotal **$332.06**: four molds$58.72, dish$22.59, pick set$4.95, proposed handle/bolts$5 estimate, two PASCO scales$58.80 including budgeted5% surcharge, chamber/pump$128.90, balance$31.55, glasses$16.99 and two mixing cups$4.56. Purchased material/PPE/stick packs in the maker BOM add$49.85 (XP-565$34.88, sticks$4.98, gloves$9.99), for the stated$381.91 planning components. Pack quantities, current silicone quote and actual allocation remain unverified; this is not a per-batch purchase quote.

One four-sheet batch nominally mixes **280g base+28g activator=308g**, including two destructive witnesses. At exactly two accepted trial sheets and no rejects, nominal material use is154g mixed material per usable trial including witness allocation. Actual yield, batch labour, cleaning, material pack count and cost per usable trial are unknown until the batch record is completed. No price is calculated from an assumed pack size. Cutting/printing service or lab access, level rack, clamps, ruler/caliper, reference masses, cleaning, tax/shipping remain excluded from the component subtotal and must be priced or declared available infrastructure before calling it a complete kit.

## Area-work model enrollment

The declared width-aware envelope is2.105/3.683N; the actual opposing load ramps over0.4mm of that front's own opening to a further1% margin (2.126/3.720N) at the robot-contacted material point, with paired acrylic reaction. Positive resisted work requires at least the original envelope. The inference25J/m² =1.2*(0.5N/0.024m) uses the90-degree, inextensible peel-energy approximation ([Kendall1975](https://bdml.stanford.edu/twiki/pub/Rise/AdhesionModels/Kendall75peeling.pdf)). It is not another technician measurement. It gives25.571/75.113/89.497mJ per native band area, in symmetric order. Record the intact witness's force-displacement curve and angle as well as peak/steady force; work must be covered too. Stretch, other angles, rate, bending and damage are still unqualified. Do not label this inference a production-part calibration.
