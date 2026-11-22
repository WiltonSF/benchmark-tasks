# Cut in a window sash rail with a sash brush: paint the putty up to the glass without flooding the pane

Validation status: the scripted reference scores 1.0 in both layouts on this source, each trial ending at 48.5 s of the 120 s limit. Dream-1: A 0 and B 0, both at the 120 s limit. No physical trial has been run.

A four-light pine window sash lies flat, putty side up, on a decorating bench. Its near (bottom) rail has already
been painted dark green along one of the two near panes; along the other pane it is still bare grey primer. The
robot has to finish that strip: paint the putty bevel and the rail beside it, right up to the glass, with a 2 in
angled sash brush loaded from an open quart can, without flooding the glass, and then lay the brush back across its
rest. A can, a rest and a brush stand on each side of the sash, mirror images of each other; which pane is bare
changes between the two layouts, and nothing else does, so the bare strip is the only cue.

The difficulty is the edge. The paint has to reach the line where the putty meets the glass (the sight line), may
lap onto the glass by up to 2 mm (painters often lap about 1/16 in to seal the putty), and must not go further. The
brush is compliant: the harder it is pressed, the further its bristles splay and the further the paint edge moves
toward the glass. With the reference's 45 degree hold and its planned 2 mm of compression, a hand held about 1 mm too
high lays no paint and about 1 mm too low splays the bristles onto the glass. The sash is fixed on the bench, so the
hand's height sets the pressure.

## Products and real setting

| Item | Product and size | Source / status |
|---|---|---|
| Sash | Four-light pine sash, 20 x 25 in (508 x 635 mm), 1-1/8 in (28.6 mm) thick, putty glazed with four 8 x 10 in single-strength (2.4 mm) lights; stiles and top rail 2-3/8 in (60.3 mm), bottom rail 3-3/8 in (85.7 mm), muntins 7/8 in (22.2 mm, estimate) | Sized as the standard 20 x 25 in 4-lite barn sash with 8 x 10 in glass ([Northview listing](https://www.doitbest.com/product/163815/northview-window-20-in-x-25-in-pvc-4-lite-barn-sash-163815/)); ordered putty glazed in pine from [B&B Wood Products](https://store.bbwood.com/barnsash.aspx), whose page offers custom sash "1-1/8, 1-3/8" and 1-3/4" thick putty glazed in Pine". Exact custom price not listed. |
| Putty bevels | Cured oil-based glazing putty, 3/8 in (9.5 mm) wide along rails and stiles and 1/4 in (6.35 mm) along muntins, sloping from the glass up to the wood face; glass face 3.95 mm below the wood face | Estimates for a 1/4 in deep rabbet with single-strength glass |
| Brush | Purdy XL Glide angular trim brush, 2 in, item 144152320: nylon/polyester filaments, hardwood handle | [Maker page](https://www.purdy.com/en/products/brushes/xl-glide): "Ideal for walls, ceilings and trim." A retailer describes "great cut-in ability" ([Sherwin-Williams](https://www.sherwin-williams.com/homeowners/products/purdy-xl-glide)). Modelled as a 140 x 22 x 15 mm handle, 25 x 54 x 18 mm ferrule and 68 mm of bristles, 50.8 mm wide and 12 mm thick at the tips; 85 g. The maker lists length-out from 2-7/16 to 3-3/8 in across sizes, not for the 2 in alone, so 68 mm and the mass are estimates. |
| Paint | Water-based acrylic satin enamel, dark green, about 0.7 L in a standard 1-quart paint can (4-1/4 in, 108 mm, diameter; 124 mm tall), lid off; paint surface 90 mm above the can floor | Standard can size; colour and fill are fixed in the reset sheet |
| Brush rest | Pine block 165 x 60 mm, 38 mm tall at the ferrule pillar and 39.5 mm at the handle-end pillar, with a 110 x 20 mm notch, so the brush lies level across it with its bristles in the air | Drawing: `fixture/brush-rest.svg` |
| Bench | The station table top, used as the decorating bench; the sash lies flat on it | — |

Painting window sash is ordinary home and maintenance work. The part a brush cannot skip is cutting in at the glass:
the putty has to be covered and sealed to the glass, which is why painters lap the paint just onto it, and anything
more is cleaned off the pane afterwards with a razor scraper.

## Starting layout

World metres, station frame: x away from the arms, y to their left, z above the bench top (0.7475 m).

| Object | Layout A | Layout B |
|---|---|---|
| Sash | Fixed, flat; outer edge of the near rail at x = 0.120, centred on y = 0; near-rail sight line at x = 0.2057; near-left light y 0.0111 to 0.1937, near-right light y -0.1937 to -0.0111 | Same |
| Already painted | Near rail beside the near-right light: 182.6 x 25 mm from the sight line | Beside the near-left light |
| Bare strip to paint | Near rail beside the near-left light | Beside the near-right light |
| Paint cans (two) | Centres (0.170, 0.360) and (0.170, -0.360), on the bench | Same |
| Brush rests (two) | Centres (0.4175, 0.350) and (0.4175, -0.350), long axis along x, ferrule pillar toward the arms | Same |
| Brushes (two) | Across each rest, broad face down, handle end at x = 0.500, bristles pointing toward the arms and overhanging the rest, axis 47 mm above the bench | Same |

Trials alternate A and B and are averaged; the screenshot shows A.

## Score

Score 0 if any of these happens during the trial (whole video, then the measurements named):

- paint lies more than 2 mm onto any glass from its putty line, anywhere on the sash (steel rule square to the sight line);
- more than 2 cm2 of paint lies anywhere other than the near rail and its putty: stiles, muntins, their putty, the bench (1 mm grid photo);
- the loaded brush is held pressed on the sash in one place for 1 s or more, which leaves a blob (video, stepped frame by frame);
- the bristles are pushed in to the metal ferrule (9 mm of the 10 mm the tips can give; video);
- a brush falls off the bench, or a paint can is tipped more than 15 degrees or shoved more than 20 mm (video).

Otherwise the score is read at the end from a top photograph of the bare strip, 182.6 x 25 mm measured from the sight line
along the light, against a 1 mm grid; a 1 mm square counts as painted when more than half of it shows paint at 10 pixels
per mm or finer. Paint elsewhere on the near rail (its outer 60 mm) is not penalised, and a lap onto the glass is allowed
up to 2 mm but not required:

| Score | Result | Measured with |
|---|---|---|
| 0.25 | at least 25% of the strip's 1 mm squares painted | top photo on a 1 mm grid |
| 0.5 | at least 50% | top photo on a 1 mm grid |
| 0.75 | at least 80%, and at least 80% of the 1 mm line next to the glass painted | top photo, 0.5 mm scale at the sight line |
| 1.0 | at least 95%, and at least 95% of that line painted | top photo, 0.5 mm scale at the sight line |

Every nonzero score also needs both brushes lying across their own rests (handle end on the end pillar, ferrule on the
ferrule pillar), both hands at least 30 mm from both brushes and the brushes still for 1 s (video). Either brush may be
used. The two layouts are averaged.
The strip that was already painted does not count; repainting it earns nothing.

In the simulation, painted means a declared film of at least 0.05 mm on a 1 mm cell; the grid covers the whole sash and
30 mm of bench around it, and any patch area beyond it counts as stray. "Still" means under 10 mm/s and 0.5 rad/s;
hands are sampled over every gripper and fingertip hull face at 3 mm spacing. The whole-trial zeros are latched: paint
does not come off once laid, so flooding and stray paint stay in the record.

## Declared paint and bristle model

The brush handle and ferrule are one rigid body (75 g). The bristle tips are a separate 10 g pad, 50.8 x 12 x 28 mm, on
a 10 mm prismatic joint along the brush axis with a 300 N/m spring and 2 N s/m damping (estimates): pressing the tips on a
surface compresses it through native contact. The tip is cut square; the angled cut of the real brush is not modelled.
Paint is a declared state on 1 mm cells over the whole sash and 30 mm of bench around it:

- Dipping: while the tip centre is at least 5 mm under the paint surface the brush loads at 0.4 mL/s, up to 0.4 mL per
  dip (estimate of what a cutting-in stroke draws from a loaded 2 in brush). Paint leaves the brush only by the rule below.
- Laying: once per 20 Hz control step, if the brush is loaded, the lowest tip corner is on the sash (within 1.5 mm of its
  surface there), the tips are compressed 0.7 mm or more (0.21 N of spring force, so they are pressed on the sash) and
  they moved at least 0.25 mm since the last step (5 mm/s), every cell under the patch is brought up to a 0.1 mm wet
  film, drawing 0.1 mm3 per cell (a 0.1 mm film over 1 mm2) from the load. A step longer than 25 mm (0.5 m/s) is
  treated as a glitch and lays nothing. The patch is the tip face projected straight down, deepened to
  14 mm from its lowest edge toward the bristle tips; it paints surfaces from 6 mm below to 4.5 mm above the lowest tip
  corner (bristles flex down the putty bevel onto the glass).
- Splay: above 3 mm of compression the patch grows by 1.5 mm on every side per further mm (at most 6 mm). Compression
  also moves the tip face back along the brush axis, which at a 45 degree hold moves its lowest edge 0.7 mm toward the
  heel per mm. Both move the paint edge toward the glass when the brush is pressed harder.
- Ferrule: 9 mm of compression means the bristles are pushed in to the ferrule: score 0.
- Blob: loaded, on the sash, compressed 0.7 mm or more and moving under 2 mm/s for 1 s: score 0.
- Can: tilted more than 15 degrees or moved more than 20 mm from its start: score 0.

No liquid flow, dripping, levelling or drying is simulated, and the sash, putty and glass are fixed rigid shapes. All
paint and bristle numbers are planning estimates, not calibrated properties.

## Real-trial procedure (proposed; not run)

1. Sashes: two identical putty-glazed sashes as above, one per layout, putty cured at least 14 days, base-coated grey. Mark
   the sight line ends of each near light with a pencil tick on the rail. Cost (checked Nov 21): B&B lists its stock
   four-light 8 x 10 pine sash (20-1/8 x 25 in, 1-1/16 in, groove glazed) at $88-100
   ([B&B](https://store.bbwood.com/BarnSash4.aspx)) and quotes the putty-glazed 1-1/8 in version on request. Three
   Purdy brushes (about $20 each), two quarts of enamel and a quart of grey base coat (about $20-25 each), the pine
   rests, a razor scraper, a steel rule and a drop cloth add about $160.
2. On the layout A sash pre-paint the strip beside the near-right light by hand, 25 mm wide; on the layout B sash, the
   strip beside the near-left light. Let them dry. These strips stay; only the bare strip is cleaned between trials.
3. Fill both quart cans to 90 mm with the paint batch; record the batch. A clean, dry brush on each rest as listed.
4. Place the sash, cans and rests at the listed positions with a tape measure and the pencil ticks.
5. After the trial: photograph the bare strip from above with a phone on a stand, the 1 mm grid card and a 0.5 mm scale
   laid along the sight line, two lamps at 45 degrees. Measure the furthest paint onto the glass with a steel rule.
   Check any mound with a 2 mm straightedge gap. Measure stray paint on the grid photo.
6. Reset: give each sash a grey semi-gloss base coat (wipeable) instead of flat primer. While the paint is wet, wipe
   the strip and the glass with a damp cloth and a razor scraper (water-based paint lifts off a semi-gloss base and
   glass within about 10 minutes) and dry 30 minutes; recoat the base when wiping leaves marks, and record the number
   of trials each sash has had. Rotate at least three dry brushes so a washed brush has dried before reuse. A drop
   cloth covers the floor around the bench. About 45 minutes per trial per sash; the two sashes alternate.

## Reference demo

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 --instruction "Paint the bare part of the near sash rail to match the painted part, right up to the glass, then lay the brush back on its rest." --scorer task_score --max-steps 2400 --epochs 2 --store-frames
```

Use `--policy dream` for Dream-1. Trial k uses layout k mod 2 (A first); `TASK_LOT=A` or `B` pins one. The layouts are
fixed; no jitter or seed changes the scene. `parts.py` builds the jointed brush and fixed sash, `lots.py` sets the
layout and `tilt.py` lets the scripted grip lean.

The reference uses the arm and the tool set on the bare strip's side. It grips the handle 95 mm from its end, lifts the brush, leans the
grip 45 degrees so the bristle tips point down toward the arms, dips the tips 12 mm into the paint, and makes two
strokes along the strip, moving the brush along its own width: the first with the lowest edge of the tips 1 mm onto
the glass, the second 12.5 mm back on the rail after a second dip. Each stroke plans 1.5 mm of bristle compression.
It then lays the brush back on the rest and moves clear. The plan is open loop from the published layout; it reads no
simulator state. Two fixed offsets, measured once in a development trace of layout A, move the first stroke 0.4 mm and
the second 1.5 mm toward the glass; the same offsets are used in layout B.

## How this differs from close tasks

- **TK-7P97ME** (finish painting a panel with a roller, accepted): broad coverage with a roller and an overlap limit
  against the earlier paint. Here a compliant brush must keep its paint edge between the putty line and 2 mm onto the glass, and
  the edge moves with bristle pressure.
- **TK-RB2759** (paint five fingernails, pending): small curved patches on a practice hand. Here one long straight
  boundary on a real window part, with a measured lap limit, a finite paint load and a reload.
- **TK-R7NTSZ** (strip paint off a sash with a heat gun and scraper, pending): removal, two tools. Here application.
- An earlier closed task cut in a wall with a paint edger whose guide wheels ride the skirting board: the tool sets the
  edge. Here nothing guides the brush.
- Household benchmarks with painting activities (for example BEHAVIOR-1K) judge whether an object ends up painted; as
  far as their published definitions show, none scores a protected boundary beside the painted area.

| Bypass | Scorer outcome |
|---|---|
| Do nothing, or paint without dipping | 0 |
| Hover the loaded brush over the strip without pressing | 0 |
| Press too hard on the glass-side stroke | 0 (paint more than 2 mm onto the glass) |
| Hold the brush 3 mm onto the glass | 0 |
| Stamp the brush in place | 0 (blob) |
| Run the stroke on over a stile or the bench | 0 once stray paint passes 2 cm2 |
| Repaint the strip that was already painted | 0 |
| Hold back 2 mm from the glass | at most 0.5 (line next to the glass bare) |
| Knock over or shove the paint can | 0 |
| One dip for the whole strip | at most 0.5 in the near-miss check (the load runs out) |
| Leave a brush in the hand, on the bench or across a can | 0 |
| Replay the fixed layouts from memory | not ruled out; no generalisation is claimed |

**One hand.** The reference uses one arm; the sash lies on the bench as it does in a shop (a 2-5 kg sash is not lifted
or held), and the other arm stays parked. Two-arm strategies are allowed and score the same way.

`near_misses.py` drives the declared rule with brush poses built from the published geometry and checks every row
above, the 2 mm lap boundary (1.9 mm passes, 3 mm fails), pressing to 3.0, 4.0 and 6.0 mm on the glass-side stroke
(only 3.0 mm stays clear of flooding), tips compressed but 5 mm off the sash, paint on a far pane, the ferrule limit,
the release gate and both layouts. These are scorer controls, not task runs.

## Evidence

Scripted reference on this source, two trials (A then B): A 1.0 and B 1.0, mean 1.0. Each trial ended on success at
48.5 s of the 120 s limit, with both brushes on their rests, both hands clear and the brushes still for more than 1 s.

| | Layout A | Layout B |
|---|---|---|
| Bare strip painted | 99.8% | 98.0% |
| 1 mm line next to the glass painted | 100% | 98.4% |
| Furthest painted cell onto the glass | 0-1 mm cell | 1-2 mm cell (at the limit) |
| Stray paint | 0 | 2 mm2 |
| Peak bristle compression | 3.0 mm | 3.0 mm |

Sampled every 0.25 s, the bristle compression had a median of 2.1-2.2 mm on the glass-side stroke and 2.0-2.1 mm on
the second (range 1.3-2.6 mm, about 0.4-0.8 N of spring force); the 3.0 mm peaks came at touch-down, at the onset of
splay. `setup.png` and `demo.mp4` show layout A.

Dream-1 on the same source, unchanged instruction, two trials: A 0 and B 0, mean 0. Both trials ran to the 2,400-step
limit with no refusal and no scorer error. Neither brush left its rest, no bristle was ever compressed and no paint was
laid; there was no flooding, stray paint, knocked can or dropped brush. `dream.mp4` shows layout A.

These are simulated results, not measured robot results.

## Licences

All geometry is procedural; no maker meshes are used. Code: MIT. Documentation and drawing: CC BY 4.0.
