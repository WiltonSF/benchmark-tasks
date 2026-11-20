# Move the cookies from the sheet pan onto the cooling rack with the turner

**Instruction:** Move the three cookies from the sheet pan onto the cooling rack with the turner, without touching
them, then put the turner back on its rest.

**Credit.** This task takes up the idea of PR #2311, **TK-7D26TA** ("Move baked cookies from a sheet pan to a cooling
rack with a turner"; its first round was TK-67G3V5), whose 48-hour hold lapsed without a revision. The scene, code and text here are new; the table
below answers every sentence of both of that task's reviews.

Lifting cookies off a baking sheet onto a wire cooling rack with a thin turner is how most cookie recipes end, and
anyone recognises it from a short clip. The hard part is physical: the turner's thin front edge has to get under a
cookie that lies flat on a bare pan without shoving it across the pan, the cookie has to be carried level over the
pan's rim, and it has to be put down on the wires without a drop, three times, with no hand ever on the food. Picking
the cookies up with the fingers scores 0.

The cookies are a non-perishable stand-in, so the same set can stay on the rig for months: birch discs of a baked drop
cookie's size and weight, with the underside's rim bevelled up the way a baked cookie's rim curls, and painted chips.

## Review of TK-7D26TA and what this task does

| Review sentence | What this task does |
| --- | --- |
| Round 2: "We are concerned about reproducibility with using perishable materials on the rigs." | No food is used. The three cookies are durable stand-ins (birch discs, tops oiled with food-safe oil), wiped and reused. |
| Round 2: "Can you please specify a non-perishable stand-in that maintains fidelity to the real context?" | Stand-in: a birch disc 3 in (76.2 mm) across and 1/2 in (12.7 mm) thick, cut with a 3 in hole saw from a Walnut Hollow 1/2 in birch circle project panel (Home Depot 322634074), about 37 g (birch at 0.65 g/cm3; estimate), its underside rim bevelled 6 mm wide and 1.5 mm high and its top edge rounded 3 mm (drawing in `fabrication/`). Fidelity was chosen on what the skill depends on: the diameter and mass match the baked cookie the original task specified (Otis Spunkmeyer item 58814: 72 to 82 mm, 35 g by the maker's specification), the bevel stands for the rounded underside rim of a baked drop cookie (an estimate, not a measurement), which is where a turner gets in, and the bare underside on bare aluminium grips the pan the way a baked cookie does (friction about 0.7 in the simulator; estimate). An acceptance check (75 to 78 mm, 32 to 40 g, bevel intact) is in the reset notes. What the stand-in cannot do, crumble or break, is replaced by rules a person can see on video: a dropped cookie, one that lay on the counter or one turned over is spoiled. |
| Round 1, Importance: "Bakers perform this work, but the submission does not establish that over 500,000 bakers and cooks regularly transfer baked cookies; the task fits societal relatability." | Category is Societal relatability. |
| Round 1, Clarity: "A same-batch instruction does not specify a repeatable source or quick preparation for cookies with consistent dimensions, brittleness and release from the pan." | No baking at all: the stand-ins are cut once from a named 1/2 in birch plywood panel with a 3 in hole saw, then the bevel is sanded and the tops oiled; this is done once, and every trial uses the same three. Release from the pan is fixed by the stand-in's bare underside on the pan's bare aluminium. Brittleness is not reproduced; the rules above replace it. |
| Round 1, What to change: "Please change the domain to Societal relatability and explain why this familiar cookie-transfer task is recognizable." | Done; the context explains it (the last step of nearly every cookie recipe, judged at a glance). |
| Round 1, What to change: "Specify a standard source or a preparation method that yields consistent cookies in a few minutes, including their size, brittleness, underside and handling on the pan, so another lab can reset trials reproducibly." | Source and preparation as above, with size, mass, underside bevel, pan friction and the acceptance check; reset is numbered in `reset_notes`. |

## Distinct from

TK-84YE4P (pilot) flips a toy egg in a pan with a spatula: one flip, nothing carried. This task never turns anything
over: it slides a thin blade under flat items on a pan that resist sliding, carries each level over a rim and sets it
down on wires. No board task found for cookies on a cooling rack or for lifting flat food with a turner.

## Run

From the yam-isaac-lab-arena checkout (the program's YAM embodiment for Isaac Lab-Arena), with this folder as
`tasks/move-the-cookies-onto-the-cooling-rack-with-the-turner`:

    inspect-robots run tasks/move-the-cookies-onto-the-cooling-rack-with-the-turner/task.py \
        --instruction "Move the three cookies from the sheet pan onto the cooling rack with the turner, without touching them, then put the turner back on its rest." \
        --max-steps 2400 --epochs 2 -E object_jitter=0

Add `--policy yam_scripted` for the scripted demo. Trial k uses lot k mod 2 (lot A first), so `--epochs 2` scores
each lot once and reports the mean. Set `TASK_LOT=A` or `TASK_LOT=B` to pin every trial to one lot. The scene is
deterministic; submissions run with `object_jitter=0`. A trial takes about 3 minutes on an L40S. With
`COOKIE_DEMO=hand` the scripted demo picks the cookies up with the fingers instead (scores 0).

Offline checks: `python near_misses.py` (every score level, both sides of each limit, and the shortcuts, in both
lots).

## Files

- `task.py`: scene, scoring, the step hook (the gripper-on-food rule, the spoil rules, the let-go distance and the
  stillness hold) and the scripted demo.
- `parts.py`: multi-shape rigid parts (this copy can also set a loose part's centre of mass, used for the turner).
  `lots.py`: per-trial layout lots. `tilt.py`: grip rotations for the scripted demo only. Each patches the stock
  embodiment in the task's own process; nothing is changed on disk, and `tilt.py` never affects Dream-1 or scoring.
- `near_misses.py`: scoring counterexamples.
- `fabrication/stand-in-cookie-and-rest.svg`: the stand-in cookie and the turner's rest.

No third-party assets are used: every object is built from boxes, prisms and convex pieces in `task.py`.

## Scene (all from task.py)

World frame: x forward from the line through the two arm bases, y to the robot's left, z up. The counter is the
station table. Positions in cm.

| object | real item | in the simulator |
| --- | --- | --- |
| sheet pan | Nordic Ware Naturals quarter sheet, bare aluminium, listed about 13 x 9.6 x 1 in outside; on a non-slip silicone mat | fixed; floor 21.6 cm front to back x 30.5 cm across, 1 mm, rims 2.5 cm tall, 3 mm thick; floor centred at 38.0, -10.25; floor friction 0.9 (pairs with the stand-in to about 0.7) |
| 3 stand-in cookies | 3 in x 1/2 in birch discs cut from a Walnut Hollow 1/2 in birch panel with a 3 in hole saw, bevelled, tops and sides oiled, underside bare (drawing); about 37 g | loose convex pieces, 76.2 x 12.7 mm, underside rim bevel 6 x 1.5 mm, top rounded 3 mm, 37 g, friction 0.5 |
| cooling rack | a quarter-sheet wire cooling grid, 30.5 x 21.6 cm, 2 mm wires 10 mm apart, 1.7 cm tall (estimates); on a non-slip mat | fixed; 23 wires front to back on 4 cross wires and corner legs; centred at 37.0, -39.2 |
| turner | small stainless offset cookie turner: blade 9.0 x 6.5 cm, 0.6 mm, front 4 mm ground to 0.15 mm; blade's underside 4.3 cm below the handle's middle; black polypropylene handle 11.0 x 2.2 x 1.4 cm; about 69 g (estimates) | loose; handle box, steel neck, blade as a convex piece with friction 0.05 (pairs with the stand-in to about 0.3); centre of mass 3.5 cm ahead of the handle's middle |
| turner rest | two hardwood posts 0.8 x 1.6 x 3.6 cm, 9.6 cm apart (drawing), with tape marks | fixed |

Lot A (the screenshot): cookies 1, 2 and 3 at 44.4, -19.0; 44.4, -10.0; 44.4, -1.0, in a row 6 mm from the pan's
far rim. Lot B: 44.0, -17.8; 44.0, -8.8; 44.0, 0.2. The turner lies on its rest pointing left, handle's middle at 16.5, -10.0, its blade flat on
the counter.

## Scoring (exact)

- Whole trial, 0 and it stays 0: any sampled point of either hand within 2 mm of a cookie (a gripper touching the
  food). Also 0 if the scene or the arms cannot be read at any step (fail closed).
- Spoiled cookie (for the rest of the trial): downward speed over 0.6 m/s (a free fall of about 18 mm), its
  underside within 3 mm of the counter with its middle off the pan's floor (on the counter), or tilted more than 60
  degrees (on its edge or over).
- At the end, nothing scores unless every sampled hand point (hull faces sampled every 3 to 5 mm) is 10 mm or more
  from the turner and from every cookie and nothing has moved more than 1 mm for 1 s.
- Good cookie: not spoiled, its middle within 114.4 mm front to back and 69.9 mm side to side of the rack's centre (so its whole outline is over the
  30.5 x 21.6 cm grid), its underside 1 mm below to 4 mm above the wire tops, top up and leaning 8 degrees or less.
- Score: 0.25 per good cookie; 1.0 when all three are good and the turner is home (handle's middle within 30 mm of
  its mark and within 10 mm of its height on the posts, pointing within 15 degrees, blade flat within 10 degrees).
- Lots A and B are averaged.

Real trials: one camera looks down on the pan and rack, one is at counter height in front. A person scores from the
video (touches, drops, cookies on the counter or turned over) and a photo plus ruler at the end (cookies on the rack,
flat, wholly over the wires; turner on its marks).

## The demo

The right arm takes the turner by the middle of its handle, turns it to point forward and, for each cookie, sets the
blade flat on the pan's floor 10 mm behind the cookie and slides it 9 cm forward: the thin front edge goes in under the bevelled rim and the cookie rides onto the
blade. It lifts the cookie level over the rim, carries it over the rack, lowers the blade to just above the wires,
rolls the turner 25 degrees about its handle so the cookie's rim comes down onto the wires, draws the blade out
sideways from under it and rolls back. Last it lays the turner back on its rest and lets go. The left arm stays at
rest. A cookie that is shoved forward instead of riding up stops against the far rim, as on a real pan. The plan is open-loop from the start layout the top camera shows (the lot's cookie positions); it aims 3 mm
high because the arm carries a light tool about that much below its target (measured on a trace), and uses
`tilt.py` for the roll.

## Fidelity and limits

- Rigid bodies in Isaac Sim with real contact; the thin blade, the stand-ins' bevel and the wires are colliders.
- The stand-in is wood, so it cannot crumble or break as a baked cookie can; drops, falls onto the counter and
  turning over are scored by rule instead.
- The turner's blade is rigid, while a real thin turner flexes a little.
- The pan and the rack are fixed (non-slip mats in real trials); they cannot be pushed.
- Friction values, the turner's and the rack's sizes, the stand-in's mass and the bevel are estimates.
- Lots stop a single fixed habit, not a memorised replay of both layouts.

## Scores

- Scripted demo (`--policy yam_scripted --epochs 2`): 1.0 in lot A and 1.0 in lot B (mean 1.0); every cookie rode onto
  the blade and was set down flat on the rack, the turner went back on its rest and the score reached 1.0 at about
  60 s of the 120 s limit.
- Dream-1 (same instruction, `--epochs 2`): 0.0 in both lots (mean 0.0); it ran to the 2,400-step limit in each and did
  not refuse.
