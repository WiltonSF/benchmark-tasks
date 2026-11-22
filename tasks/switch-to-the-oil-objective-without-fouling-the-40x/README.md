# Switch a focused microscope to its 100x oil objective: nosepiece half-turned, one drop on the lit spot, 40x kept dry

**Instruction:** Switch the microscope from the 40x objective to the 100x oil objective for the stained smear, put the
oil dropper back in its bottle, and take both hands away.

**Category:** Other (laboratory and teaching-lab microscopy).

## Why it matters

Gram stains, blood-smear differentials and thick and thin malaria films are read with a 100x oil-immersion objective,
in clinical microbiology and haematology labs and in every microbiology teaching lab. The slide is found and focused
at 40x, then the oil objective is brought in. A robot that reads slides unattended has to make the same switch, and
the step is physical and order-bound. The usual teaching-lab instruction (microscope manuals and Gram-stain lab
sheets) is: turn the revolving nosepiece by its knurled ring until it is halfway between 40x and 100x, put one drop of
immersion oil on the lit spot of the slide, then turn the 100x into the oil until the nosepiece clicks.

What goes wrong, and what makes it hard for a robot:
- the dry 40x objective works 0.6 mm above the slide, so oil put on with the 40x still in place, or a nosepiece turned
  back the short way after the oil is on, wets its front lens (it then images through an oil film until it is cleaned);
- the drop must reach the lit spot through the gap the half-turned nosepiece leaves, with the dropper's tip a few
  millimetres above the slide and clear of both objectives;
- a light squeeze gives nothing and a hard one squirts a stream that runs off the slide onto the stage;
- the nosepiece is turned by its ring, not by pushing the objectives (which loosens and misaligns them);
- the glass dropper comes out of and goes back into a 12/18 ground-glass neck, so it is drawn straight up and
  lowered straight in.

## Parts and scene (true size)

| Item | Product | Size and mass | Status |
|---|---|---|---|
| Microscope | Swift SW350B student compound microscope (4x, 10x, 40x spring, 100x/1.25 oil spring objectives on a quadruple revolving nosepiece; mechanical stage; LED condenser; binocular head that turns 360 degrees) | 215 x 150 mm base, stage top 120 mm up, about 404 mm tall; fixed in the scene | objective magnifications from the maker's listing; body sizes are estimates |
| Nosepiece | knurled ring 72 mm across and 18 mm thick, its axis 28 mm behind the light path and leaning 15 degrees toward the robot; click stops every 90 degrees | objectives' shoulders 45 mm above the slide (parfocal); working distances 4x 17 mm, 10x 7 mm, 40x 0.6 mm, 100x 0.2 mm | estimates (typical DIN student objectives) |
| Slide | stained smear on a 75 x 25 x 1 mm glass slide in the stage's slide holder | lit spot 4 mm across at the light path | slide size standard |
| Oil | Cargille Type A immersion oil in a Kimble 30 mL clear glass dropping bottle with a 12/18 ground neck (DWK 15035-30; sold with its ground-in glass pipet and rubber bulb as Fisher 02992A); drawn amber in the simulation so it shows | bottle 36 mm across, neck top 88 mm up, 75 g; dropper 114 mm long, 12 g, bulb 15 mm across and 32 mm long | bottle and dropper sizes are estimates |

The objectives carry coloured bands: 4x red, 10x yellow, 40x blue, 100x white. The microscope's head is turned so
its eyepieces face away from the robot (the head turns 360 degrees), which keeps the arms clear of them.

## Layout (fixed; two lots)

Millimetres from the midpoint between the two arm bases (x forward, y to the robot's left), heights above the bench.

| Object | Lot A (screenshot) | Lot B |
|---|---|---|
| Microscope light path | (300, 0); stage toward the robot | same |
| Nosepiece | 40x in the light path, focused on the smear | same |
| Dropping bottle, dropper seated in its neck | (205, -150) | (255, -190) |

Trials alternate A, B and are averaged. The microscope and its nosepiece never move between lots.

## Score 0 if (whole trial; putting things right afterwards does not undo it)

- oil touches the 40x, 10x or 4x front lens: a drop is put on under it, or it is turned through a drop (side video;
  real trial: wipe the 40x with lens paper after the trial and look for oil);
- oil lands anywhere but the slide (on the stage, the bench or an objective), more than two drops are put on, or the
  bulb is squeezed hard enough to squirt a stream;
- a robot finger or the dropper touches an objective (video; any contact; simulation: over 0.1 N);
- the oil bottle tips over (more than 45 degrees).

## Ladder (end state; every level needs the bottle and dropper still, under 5 mm/s and 0.1 rad/s, and the nosepiece not turning, under 4 degrees a second, for the last 1 s)

| Score | Condition | How it is read in a real trial |
|---|---|---|
| 0.25 | one or two drops on the slide, the first one's centre within 3 mm of the centre of the lit spot | a front camera looking 45 degrees down through the gap under the half-turned nosepiece at a 1 mm grid printed on clear film under the slide (the head and nosepiece hide the spot from straight above): the frame when the drop lands |
| 0.5 | and the 100x is in the light path at its click stop (within 2 degrees) with the drop under its front lens | photo of the nosepiece's index mark; front photo shows the oil bridging slide and lens |
| 0.75 | and the dropper is home in its bottle (shoulder within 3 mm above the neck, centred within 2 mm, within 10 degrees of the bottle's axis) and the bottle stands (within 10 degrees of upright, base within 3 mm of the bench) | side photo against a ruler |
| 1.0 | and nothing is held, both hands at least 30 mm from the nosepiece, objectives, bottle and dropper for the last 1 s | top and side video |

Otherwise 0. Trials on lots A and B are averaged.

## How the simulation scores it (task.py)

`update()` runs every control step from the simulator callback with the object states, the two finger pads' contact
forces on the dropper and the contact force of the hands and dropper on the objectives:
- the dropper is loaded with 3 drops each time its tip is down in the bottle's neck;
- out of the bottle with its bulb up (dropper within 60 degrees of vertical), a squeeze reaching 10 N on both pads of
  a hand whose grip centre is on the rubber bulb releases one drop (a hand holding the glass stem or shoulder squeezes
  nothing); the pads must ease below 5 N before the next; over 35 N on a pad squirts out the whole load (score 0);
- a drop falls straight down from the tip, where the tip is: onto any objective's nose if one is under the tip (score
  0), onto the slide if the tip is over it and above it by no more than 25 mm, otherwise off the slide (score 0). The
  25 mm limit stands for the maker sheets' advice to put the drop on from close: from higher up its landing point is not
  controlled and it can catch the ring or a barrel on the way down;
- drops are 5 mm across and 1.2 mm tall; an objective whose front lens (4.5 mm radius) comes lower than the drop's top
  and within 7 mm of its centre while the nosepiece turns (checked every 1 degree between control steps) is fouled;
- the nosepiece is a revolute joint about its tilted axis; it holds where it is left (friction, modelled as a joint
  target that follows it within 0.04 rad at 0.4 N m/rad, about 0.016 N m) and its click stops pull it into each 90-degree position
  from within 10 degrees; the knurled ring is a smooth 36-sided ring with friction 1.0;
- `score()` returns the ladder level once the bottle and dropper have been still and the nosepiece unturned for 1 s.

Known limit of the drop model: a squeeze held at 10 N or more while the dropper is drawn out of its bottle releases
a drop as soon as the tip clears the neck (over the bottle, so the trial scores 0 for oil off the slide). A real
dropper squeezed in the bottle empties into it instead. The scripted demo holds the bulb at about 4 N while lifting;
the fix (re-arm only after the squeeze eases outside the bottle, and count a drop over the neck as going back in) is
written down for a revision.

## Bypasses checked

`near_misses.py` drives the same `update()` step by step; every line below is asserted on import.

| Strategy | Score |
|---|---|
| Do nothing | 0 |
| Oil put on with the 40x still in the light path | 0 |
| Drop put on at the half turn, nosepiece turned back the short way, then on to the 100x | 0 |
| Turned the other way first (40x back over the oil) | 0 |
| Turned back only to -30 degrees (40x 20 mm away, 1.4 mm up) and on again | 1.0 |
| Turned past the 100x to the 4x (25 mm up) and back | 1.0 |
| Drop centre 3.1 mm off the lit spot / 2.9 mm off | 0 / 1.0 |
| Two drops / three drops | 1.0 / 0 |
| Bulb crushed (36 N) | 0 |
| Drop on the stage past the slide's end; squeezed 36 mm up | 0 |
| Squeeze of 9.9 N on one pad; squeezed with the tip up | 0 (no drop) |
| Nosepiece turned by pushing an objective (0.11 N of contact) | 0 |
| Turned straight to the 100x with no oil, then oil squeezed 6 mm beside its nose | 0 |
| Tip below the slide's top (under the stage) when squeezed | 0 |
| Grip centre on the glass shoulder or stem (not the bulb) | squeezes nothing |
| Bottle knocked over and stood up again | 0 |
| Drop on the lit spot, nosepiece left half-way | 0.25 |
| 100x 1.9 degrees short of its stop / 2.5 degrees short | 1.0 / 0.25 |
| Dropper left lying on the bench | 0.5 |
| Hands clear for only 0.5 s | 0.75 |

## How this differs from close tasks

- TK-WY2P72 (accepted): centre a slide's alignment cross and focus with the stage and focus knobs. Knob turning on a
  microscope; no objective change, no oil.
- TK-PPBUN2 (pending): load a slide into a microscope stage under its spring arm. Slide handling; no objective change.
- TK-EPG2T3 (accepted): one eye drop squeezed from a dropper bottle into a manikin's eye. The drop there is the whole
  task; here it must be placed through the gap of a half-turned nosepiece, and the hard part is the order the optics
  force: half turn, oil, then the oil objective in, with the dry 40x kept out of the oil.

## Fidelity and what is simplified

See `fidelity` in task.json. The oil is a declared model: drops are counted from pad forces and placed where the tip
points; there is no liquid. The nosepiece's friction and click stops are a joint-target rule. The slide is fixed in its
holder. The spring-loaded 40x and 100x fronts are modelled rigid, with their last 1.5 mm not colliding. The ring's
knurling is not modelled (friction 1.0 stands in for it), and the click stops' pull (10 degrees) is wider than a real
ball detent's.

Estimated figures and a one-minute check for each: the 10 N squeeze for one drop and 35 N for a stream (press the
loaded dropper's bulb between two fingers on a kitchen scale and note the reading when the first drop falls, and when
oil runs); the nosepiece's 0.016 N m holding friction and 10-degree click pull (hook a luggage scale on the ring and
note the pull that starts it turning); the 0.6, 0.2, 7 and 17 mm working distances (a feeler gauge under each
objective in focus).

## Real-world trial

Lay out the parts on the lot's tape outlines, focus the smear at 40x and tape the focus knobs so the stage cannot
move, and seat the dropper in its bottle. Set a camera level with the stage in front, with a 1 mm grid card standing
beside the slide. Fit the
gripper pads with nitrile finger cots so oil never reaches them. Run the policy for 60 s; then, without touching
anything, photograph the slide area and the nosepiece from above and the side, and wipe the 40x front lens with lens
paper to check it for oil. Reset: wipe the slide and the 100x with lens paper and lens cleaner, turn the nosepiece
back to 40x, refocus.

Cost, about US$260 (prices checked Nov 21): Swift SW350B microscope with its 100x oil objective, about $160-200
(listings); Kimble 30 mL dropping bottle with ground-in pipet and bulb (Fisher 02992A), about $30; Cargille Type A
immersion oil, 1/4 oz, about $15; a box of prepared stained smear slides, about $15; lens paper and finger cots,
about $10.

## Run it

yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), from this folder:

```sh
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Switch the microscope from the 40x objective to the 100x oil objective for the stained smear, put the oil dropper back in its bottle, and take both hands away." \
  --scorer task_score --max-steps 1200 --epochs 2
```

1,200 control steps (60 s at 20 Hz) per trial; trial k uses lot k mod 2 (`TASK_LOT=A` or `B` pins one lot). No seeds
change the scene. The scripted demo (`demo()`, policy `yam_scripted`) plays fixed waypoints from the visible layout of
each lot; its tilted hand poses use the included `tilt.py`, which affects only the scripted demo.

## Results

- Scripted demo (`yam_scripted`, `--epochs 2`, 1,200 steps): 1.0 in lot A and 1.0 in lot B (mean 1.0); full score at
  step 557 and 558 (about 28 s) of the 1,200-step limit, when the scorer ended each trial.
- Dream-1 (same instruction, `--epochs 2`, 1,200 steps): 0.0 in lot A and 0.0 in lot B (mean 0.0). Both trials ran to
  the 1,200-step limit; it did not refuse. The arms lifted and hovered, and the right hand came close to the stage's
  right side, but neither hand took the dropper or touched the nosepiece; no drop was made and no rule was broken.
- What the demo does: the left hand opens, closes its fingertips on the knurled ring either side of the 40x (never on
  an objective) and carries the ring 45 degrees round (it ran on to about 51 degrees in the traced run); the right hand
  draws the dropper straight up out of the bottle by its bulb, leans it 45 degrees, brings the tip 6 mm above the lit
  spot through the gap in front of the half-turned nosepiece and squeezes once (about 16 N per pad; in the traced run
  the drop landed 1.2 mm from the centre of the lit spot), then puts the dropper back in the neck; the left hand then
  carries the ring on until the 100x's click stop takes it, and both hands move away.
- Development runs on the same task (lot A traces): the turret was knocked round at the start by an objective
  touching the arm (fixed by moving the arm back); pushing the ring's side with closed fingertips slipped (the ring
  turned 16 of 45 degrees, then 25 of 49 with ribbed knurling), and pressing harder stalled the arm; holding the ring's
  rim between the fingertips and carrying it round turned it without slip; the dropper first jammed going back into
  the neck (fixed by allowing for its 2 mm offset in the hand); an independent review's scoring fixes (drop recorded
  where the tip is, any objective's nose catching oil scores 0, the tip must be above the slide, only a grip on the
  bulb squeezes, any objective touch over 0.1 N scores 0) were applied before the final runs.

## Licence

Code and the procedural geometry: MIT. Text: CC BY 4.0. No third-party meshes are included.
