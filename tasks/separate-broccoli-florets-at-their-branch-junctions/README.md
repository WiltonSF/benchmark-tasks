# Cut six whole florets off a broccoli head with a paring knife while the other hand steadies the stem

**Instruction:** Cut the broccoli head into six whole florets with short stalks, and leave the florets and the knife on the board.

One arm holds the main stem of a trimmed broccoli head that lies on its side on a cutting board. The other arm takes a paring knife and cuts through each of the six floret shoots 10-30 mm below its first side branch, so each floret keeps one whole crown and a short stalk. The difficulty is the branched, loose head: each stalk has to be cut through completely at the right place, the blade must miss the neighbouring crowns and stay clear of the holding hand, and the cut pieces must not be knocked off the board or cut again while the rest of the head is worked.

## Why it matters

- O*NET 35-2021.00, Food Preparation Workers, core task: "Wash, peel, and cut various foods, such as fruits and vegetables." ([O*NET](https://www.onetonline.org/link/details/35-2021.00); 908,500 employed in the US, 2025.) This is occupation-level evidence, not a count of workers who cut broccoli.
- Purdue Extension's broccoli guide: "Cut each head into small florets." ([Purdue FoodLink](https://extension.purdue.edu/foodlink/food.php?food=broccoli))
- The knife maker states the knife has a "Protective finger guard for added safety" ([Mercer Culinary](https://www.mercerculinary.com/product/millennia-slim-paring-knife-3-7-6-cm/)); the 30 mm hand-clearance rule below enforces the same concern for the holding gripper.

## Scene (every object, true size)

| Object | Product and source | Size and material | Simulated as |
|---|---|---|---|
| Cutting board | OXO Good Grips 11272900 carving and cutting board, $37.99 ([OXO](https://www.oxo.com/shop/kitchenware/cooking-baking/cutting-boards/carving-cutting-board.html)) | 535.9 x 368.3 mm (maker), polypropylene, 16 mm thick (estimate), non-slip feet | fixed box, friction 0.7 |
| Paring knife | Mercer Millennia M23900P, $6.19 ([maker](https://www.mercerculinary.com/product/millennia-slim-paring-knife-3-7-6-cm/), [WebstaurantStore](https://www.webstaurantstore.com/mercer-culinary-m23900-millennia-3-paring-knife-with-black-handle/470M23900.html)) | 76 mm high-carbon steel blade (maker), Santoprene and polypropylene handle; handle 100 x 22 x 24 mm, finger guard 10 x 30 x 26 mm, blade 18 mm tall (ruler estimates); 50 g (retailer's 0.11 lb shipping weight, an upper bound) | loose body; handle and guard collide, blade is a visible non-colliding plate governed by the cutting rule |
| Broccoli head | fresh green broccoli, sold by weight ([Walmart](https://www.walmart.com/ip/51259378)), trimmed as below | 250 g; main stem 34 mm across, 100 mm long; six 16 mm shoots 93-101 mm from the stem end to their first side branch (scored stalk 10-30 mm); crowns 34-38 mm across; rests on its stem and crowns, tipped 3.9 degrees | stem with the six shoot bases (round 16 mm cylinders) as one 100 g body; six 25 g floret bodies joined to it by tissue joints; drawing in `fixture/head-layout.svg` |
| Ruler | stainless steel rule | 300 x 20 x 2 mm, 10 mm ticks | fixed, beside the board at (490, 0) mm |

The station table is the counter; nothing else is on it. Coordinates are mm from the station origin, x away from the arm bases, y toward the left arm, heights above the board top.

| | Layout A (screenshot) | Layout B |
|---|---|---|
| Holding arm / cutting arm | left / right | right / left |
| Main stem axis | through (270, -30), 17.5 mm up, butt end toward the left arm at (247, -3) | mirror: through (270, 30), butt at (247, 3) |
| Floret directions from the stem end at (312, -80) | 18, -9, -36, -64, -91, -118 degrees | mirror: -18 ... 118 degrees, from (312, 80) |
| Knife | handle centre (200, -225), standing on its handle underside, blade pointing +x | (200, 225), blade +x |

Trials alternate A, B, A, B ... with zero jitter: these are repeats of two fixed layouts, not different heads.

## Scoring (real trial; the simulation's proxy is given where it differs)

**Score 0 if at any time:** any part of the blade comes within 30 mm of the gripper holding the stem or touches it; a gripper crushes a crown (real trial: crushed, split or broken-off buds in the post-trial photos; simulation: a gripper surface more than 2 mm inside a bud); a floret comes off without being cut (torn); pre-cut or substituted pieces are used. A violation is never undone by what happens later.

A **qualifying floret** was cut off by the knife while the other gripper held the main stem between opposed fingers, keeps its original crown (30-50 mm across), keeps 10-30 mm of stalk from every point of the cut face to its first side branch, has no knife cut more than 3 mm into the buds (real trial: a toothpick marked 3 mm from its tip does not go deeper into any cut in the buds), and lies wholly on the board.

| Score | Result | Measured with |
|---|---|---|
| 0 | no qualifying floret, or any Score-0 event | whole-trial video |
| 0.33 | 1-2 qualifying florets | ruler on overhead and side photos of every piece |
| 0.67 | 3-5 qualifying florets | same |
| 1.0 | all 6 florets qualify | same, plus final 1 s of video |

Caps: one or two damaged original crowns cap the score at 0.67, three or more at 0.33 (damage on any piece counts, kept or discarded). Every nonzero score also needs, at the end: the knife resting wholly on the board, the knife and all produce released, both hands more than 5 mm from them, and every loose piece still for 1 s (simulation: within 2 mm and 2 degrees; real trial: no visible movement of any piece in the overhead video for 1 s). A cut through the main stem is not penalised by itself; its resulting crowns and stalks are judged by the same rules.

## How the simulation cuts (declared rule with cutting resistance)

The head, florets and knife are rigid, and the blade has no collider. In its place, the edge meets a stated cutting resistance whenever it is inside a stalk:

| Force on the knife | Size | Direction |
|---|---|---|
| Feed resistance | 0.4 N per mm of edge inside the 16 mm section (6.4 N across the full width) | against the edge-first direction, while the edge is in the section, unless the knife is being drawn back out faster than 5 mm/s |
| Drag | 0.15 N per mm of edge inside the section (2.4 N across the full width) | against the blade's motion relative to the stalk over the last 0.15 s, in any direction (including sideways out of its own plane), when faster than 5 mm/s |

The simulator recomputes the force every 50 ms control step from where the edge is and applies it for the next step. The feed therefore depends on the edge being in the stalk, not on its speed in a single step (switching it on speed alone made the knife chatter in and out of the cut). Three consequences, all declared: a knife pressed into a stalk with less than the resistance is pushed back out rather than resting in the cut; the resistance first acts in the step after the edge enters a stalk; and with the arm's compliance the knife still bounces 2-5 mm out of and back into the stalk a few times per stroke as the resistance switches on (each cut in the traced demo still completed in about 1 s).

Basis (estimate, not a measurement): blade tests on parsley stems report a specific cutting energy of 2.25 mJ/mm² at a 0-degree blade angle ([Hosseinpour-Zarnaq, Minaei and Bazyar, Chem. Proc. 2022](https://mdpi.com/2673-4583/10/1/16)), about 2.25 N per mm of edge; a tender broccoli shoot under a sharp, thin paring blade is taken as about a fifth of that, plus face friction for the drag. The reduction to a fifth is an unsourced estimate: no broccoli-specific measurement was found, and a straight press cut with a blunter blade could need several times more. The resistance is applied to the knife at its centre of mass, which is where the demo grips the handle, so the arm holding the knife must push through it (the demo commands the edge 30 mm below the board to do so). The equal reaction acts on the stalk where the edge is, so it pushes the head into the board and against the holding hand. The moment the resistance has about the grip, because it acts at the blade about 100 mm from the handle's centre (about 0.5 N m as the edge enters, 0.85 N m across a full square cut), is **not** applied. In a trace with it applied, the first stalk contact turned the knife about 33 degrees tip-up within the two-finger pinch in one 50 ms step: a pinch on the handle's sides resists that twist only through pad friction. The simulated grip is therefore easier than a real one in this respect. Whether real fingertip pads hold the twist is not measured; the dulled-knife hardware check below tests it before any sharp trial,. A grasp pressing on the handle's spine and belly would resist the twist by finger pressure, but this scorer credits only a pinch on both handle sides, and such a grasp was not built. The resistance acts whenever the edge is in a stalk, whether or not anything is held; a cut is credited only while one gripper holds the stem and the other holds the knife by its handle. It acts on the outer part of each shoot's 16 mm stalk, from 56 mm below its side branch to the stalk's end 9 mm above it at the base of the crown (a cut is credited only from 2 to 56 mm below the side branch); the main stem and the inner part of each shoot have no resistance, and the blade in the buds is scored as damage instead.

A floret separates only when all of these hold, and then its tissue joint releases and it falls and collides on its own:

- the knife edge has swept every point of a 29-point grid over the full 16 mm stalk section, anywhere from 2 to 56 mm below the first side branch, moving edge-first **relative to the stalk** (positions are compared in the stalk's own frame, so knife and head moving together sweep nothing, and a spine-first or sideways pass does nothing); a point is crossed in a control step when it goes from ahead of the edge line to on or behind it, within the edge's length at both ends of the step;
- each grid point counts only if it was crossed during a control step in which the cutting resistance was acting on that stalk (an unloaded stroke cuts nothing); points crossed in the step in which the edge enters a stalk count once the resistance acts on it in the next step;
- one gripper holds the native stem with opposed fingertips and the other holds the knife by both handle sides;
- the resistance has acted on that stalk for at least 0.1 s while both grips held, counted since the grips were last both in place (a lapse in either grip, or a 30 mm withdrawal, restarts the count).

A partial incision is forgotten if the edge is withdrawn more than 30 mm from that stalk before the cut is complete; a real nick would remain. Nicks in a neighbouring stalk are not scored: in the lot B trace the knife tip met the next stalk for 0.05-0.1 s during the first two cuts without cutting it through. As in a real cut, a complete cut in the wrong place still takes the floret off: its stalk length is recorded from where each grid point was crossed, and only 10-30 mm qualifies. The released floret keeps the model's fixed shape (its body starts at the reference plane 26 mm below the side branch) whatever the recorded length, and its stalk is not shortened by any later cut (re-trimming a separated floret is not modelled). In a real trial a floret that is partly cut and then pulled off counts as torn (Score 0). The handle and finger guard do collide, so the knife comes to rest on the board when its flat underside meets it.

Damage: the whole blade plate is checked against the bud cylinders, inset 3 mm, at 1 mm steps of relative motion; deeper overlap marks that crown damaged for the rest of the trial. Gripper surfaces are sampled every 2 mm against the buds inset 2 mm. Friction (food 0.6/0.5, knife 0.7/0.6), masses and the cutting resistance are estimates; tissue compliance, sap and bud fracture are not modelled. The fingers are capped at 40 N.

Reset in simulation: the task disables the six tissue joints, writes every loose body at its exact declared pose with zero velocity, re-enables the joints and settles for 0.5 s. The first scored step compares every pose with the layout (4 mm, 3 degrees); a failed start scores 0 and is reported as invalid rather than as a task failure.

## Shortcuts and how they score

| Attempt | Result |
|---|---|
| Push the head or a crown to a new place, or knock a floret off | 0: no cut; a floret pulled more than 18 mm from its place on the head uncut is a whole-trial 0 |
| Hold the head against a still blade, or move knife and head down together | 0: no relative edge sweep |
| Push the blade through a stalk with the stem or the knife handle not held | 0: that path is not credited (the resistance still acts) |
| Cut with one hand while nothing holds the stem | 0: separation needs the stem held by the other gripper |
| Push the blade with fingers instead of holding the handle | 0: separation needs the handle held on both sides |
| Cut through the crowns, or 38 mm / 8 mm from the side branch | the floret comes off but is damaged or does not qualify |
| Drop the knife off the board or leave it on the ruler | 0 until the knife rests on the board |
| Saw past the holding hand | 0 for the trial (30 mm rule) |
| Leave pieces in the hand or still moving | 0 until released and still for 1 s |

One hand is not enough: the scorer gives no cut unless one gripper holds the stem while the other holds the knife.

## How this differs from close tasks

- TK-N9C8DA (saw a 1-2 cm slice off a loaf): one straight cut across a block. Here six separate stalks fan out at different angles, the cut must be placed in a 20 mm band below each crown's first side branch, and finished pieces must not be cut again.
- TK-VT2J8N (peel a cucumber): surface removal, not separating a branched item into parts.
- TK-53WVPU and TK-W7HHVG (cut a taut cord; cut a cable tie without nicking cables): single cuts of a held strand; no repositioning of a loose branched object between cuts.
- TK-CXPYGC (trim three hedge shoots, sparing the flowers): selective cuts on a fixed plant with shears; here the produce is loose and the holding hand must keep and move it.
- TK-XTCCB5 (remove tree supports from a printed part): the supports are scrap; here each piece is the product and its crown and stalk length are judged.
- Pending TK-JK7DHG (halve eight tomatoes in one pass) and TK-6JQPX3 (slice a carrot on a mandoline): parallel cuts of identical items, not stalk-by-stalk cuts of one branched head.

The board was searched for broccoli, floret and cauliflower titles on November 21: none.

## Real trial procedure

**Produce.** A whole fresh green broccoli head with its stalk (not pre-cut crowns), firm and dark green with closed buds (no yellowing), 150-200 mm across the head and 350-550 g as bought; keep at 2-5 C and use within 2 days. Reject frozen, limp, hollow-stemmed or woody heads.

**Trim to the test head** (on a separate board, by a person, before the trial):
1. Pull off all leaves. Cut the main stem square so 100 mm of stem remains below the lowest floret branch.
2. Keep six florets whose shoots leave the top 30 mm of the main stem, each shoot 14-18 mm thick and 90-100 mm long from the main stem to its first bud-bearing side branch, ending in a crown 30-50 mm across. Cut off all other florets at the main stem.
3. Lay the head on its side with the six shoots fanned to one side, about 27 degrees apart, as in the drawing. Weigh it: 200-300 g.

**Count and size check** (ruler and kitchen scale, recorded with an overhead photo beside the ruler): exactly six crowns; each crown 30-50 mm across its widest point; each shoot 14-18 mm thick and 90-100 mm to its first side branch; main stem 30-38 mm thick and 100 mm long. A head that fails is replaced, and the reason recorded.

**Setup.** Print `fixture/head-layout.svg` at 1:1 and tape it beside the board, or tape marks on the board, for the stem axis, the six shoot directions and the knife spot. Lay the head on its side and press the six shoots gently into one plane along the marks; a real head's shoots are not perfectly flat, so any shoot whose crown stays more than 10 mm above the board after this is trimmed off and the head replaced if fewer than six remain. Place the stem butt toward the holding arm. Stand the knife upright on the underside of its handle and guard at the marked spot, edge down, blade pointing away from the arms. The real handle is rounded, so a pea-sized dab (about 1 g) of reusable adhesive putty under the handle keeps it upright; it lifts off with the knife and holds far less than the gripper's lift. The simulated handle is flat-bottomed and stands without it; the putty is the only real-trial addition. Hardware trials first use the same knife with its edge dulled on a stone, to check reach, clearance, the 40 N finger limit and whether the pinch holds the knife's twist when the blade is pressed into a stalk. Both grippers' fingertips get food-safe, cut-resistant (ANSI A4 or higher) sleeves, the holding gripper's above all, since the blade passes near it; a tape mark 30 mm out from the holding fingertips lets the 30 mm clearance rule be judged from video. The sharp knife is used only behind an exclusion zone with an emergency stop.

**Measure after the trial.** Keep every piece. Photograph each piece from above and from the side beside the ruler: crown width, stalk length from the cut end to the first side branch, and any knife cut in the buds; probe each such cut with a toothpick marked 3 mm from its tip. Use the whole-trial video (overhead and both wrist views) for cutting, holding, the 30 mm clearance, release and the final 1 s still check.

**Reset.** Collect all pieces, wash and dry the board and knife, trim and check a fresh head, place everything at the layout marks.

## Reproduce

Stack: Python 3.12, Isaac Sim 6.1.0.0, Isaac Lab-Arena 0.3.0, yam-arena 0.1.0, inspect-robots 1.0.0. Task-local helpers: `parts.py` builds the scene parts and runs the reset and step callbacks; `lots.py` alternates the A/B layouts and writes the exact start poses; `tilt.py` gives the scripted holding grip its 45-degree lean.

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Cut the broccoli head into six whole florets with short stalks, and leave the florets and the knife on the board." \
  --scorer task_score --max-steps 2400 --epochs 2 --store-frames
```

Use `--policy dream` for the model run. `TASK_LOT=A` or `B` pins one layout. At most 120 s per trial (2400 steps); the scripted demo plan lasts 100 s and reached success at 95 s in both layouts. `python near_misses.py` runs the rule regressions (all six demo strokes cut and, in the nominal geometry, touch no other stalk; no-load, unheld stem, unheld knife, knife-and-head-together, spine-first and beside-stalk strokes cut nothing; 8, 38 and 50 mm cuts separate the floret but never qualify; a knife off the board or lifted scores 0; a stroke made while the resistance is not acting is not credited, while the points crossed as the edge enters a stalk are credited when the resistance acts in the next step; the resistance equals 0.55 N per mm of edge in the section going down and 0.15 N/mm going up, still acts with the stem or knife handle unheld (without credit), acts as feed only on a knife held still in the cut, and its moment about the knife's centre of mass is computed (logged, not applied); exact A/B starts pass and a 6 mm or 4 degree displaced start is invalid; 30 mm clearance, damage and release checks).

The scripted demo reads only the published layout (`lots.current()`), never simulated poses, cut flags or forces. The holding gripper takes the stem 75 mm behind the branch bases, leaning 45 degrees toward the head so its wrist stays clear of the cutting arm. The knife cuts the florets outermost first (6, 5, 4, 3, 2, 1 in A) with its tip 20 mm past each stalk toward the uncut florets; the first cut is turned 25 degrees (aimed 3 mm further out) so the knife hand stays clear of the holding hand. The edge comes down through each stalk over 4 s, commanded 30 mm below the board so the arm pushes through the cutting resistance, about 6 mm past its reference plane (about 20 mm of stalk kept), until the knife's flat underside meets the board; each cut's aim carries a fixed correction of -5 to +3.5 mm for its layout, measured in traces of this demo with the resistance on, because each loaded arm lands a few mm off its command. After each cut the holding hand lifts the head 15 mm, carries it 62 mm straight back along that floret's axis and sets it down, leaving the cut floret where it lies and giving the next cut room. Finally the knife is stood back at its start spot and both hands let go.

## Results

The first three rows use this folder's code with the cutting resistance on, two trials (A then B), 2400-step limit.

| Run | Lot A | Lot B | Notes |
|---|---|---|---|
| Scripted demo | 1.0 | 1.0 | All six florets cut in turn in both layouts, no crown damaged, success at 95 s. |
| Traced demo (same code, with the per-floret log) | 1.0 | 1.0 | Retained stalks: angled first cut 13.4-25.2 mm (A) and 13.5-24.6 mm (B), the other cuts 17.0-25.2 mm. Peak resistance 9.4 N; the moment it would have about the grip, logged but not applied, peaked at 0.9 N m. The knife turned at most 1.9 degrees in the grip. |
| Dream-1 | 0.0 | 0.0 | Both trials reached the step limit; the arms moved toward the florets and the knife but never held the stem and the knife together, and nothing was cut. |
| Dream-1 on earlier versions of this task | 0.0 | 0.0 | Two runs, both 0.0 in both layouts, with the same behaviour; a third was stopped unfinished when the task code changed. |

Code: MIT. Documentation and drawing: CC BY 4.0.
