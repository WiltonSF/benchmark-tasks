# Hang a hamster's water bottle on its wire cage with the spout through the bars

An empty wire-top hamster cage stands on the bench across from the robot, its front wall facing it. A filled 8 oz
small-pet water bottle in its steel hanger stands spout-down in a drip stand. The robot has to hang the bottle on the
outside of the front wall so that the sipper tube reaches in through a 12 mm gap between the bars, its tip at a
height a hamster drinks from, and the hanger's hook sits on the high horizontal wire. Then it lets go.

Instruction: "Hang the water bottle on the cage so the hamster can drink from the spout."

## Why this is not a plain hook-and-hang

The hanger holds the bottle 35 degrees from vertical with the spout toward the cage, so the tube runs down and in.
The front wall has a horizontal wire just above the base's rim, 10 mm below where the tube passes the bars when the
bottle is hung. Once the hook is on the high wire, the bottle can only swing about that wire, and every point of the
tube then crosses the bar plane straight below the wire, between 49 and 72 mm above the floor. The low wire and the
base's rim sit in that band, so a bottle hooked on first and then swung in (or let go to swing in by itself) ends
with the tube's tip resting on the low wire outside the bars. The tube has to go in first, along its own line through
a gap, with the hook carried above the wire, and only then can the bottle be lowered so the hook drops onto the wire.
Pushing the tilted bottle straight in at its hung height drives the hook into the high wire from the side.

The rule was checked before any simulator run with a rough MuJoCo model of the same geometry (rigid bodies, no
robot). Hooked first and let go from 15, 25 or 40 degrees out: the tip stopped 6.5 mm outside the bars, resting on
the low wire, every time. Hooked first and swung in by a stiff hand: the tip stopped outside. Pushed straight in at
the hung height and let go: the hook hit the high wire and the bottle fell. Tube in along its line 13 mm high, then
lowered: hung with the tip 18.6 mm inside, also with the bottle 2 mm off the gap's middle. These are probe results,
not measurements.

## Run

Install yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena) and run, from this folder:

    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
      --policy yam_scripted --no-prompt \
      --instruction "Hang the water bottle on the cage so the hamster can drink from the spout." \
      --scorer task_score --max-steps 800 --epochs 2

Trial k uses lot k mod 2 (lot A first); `--epochs 2` runs both lots and reports their mean as `task_score`.
`TASK_LOT=A` or `TASK_LOT=B` pins every trial to one lot. Use `--policy dream` for Dream-1. The task needs
`object_jitter=0`. No seeds are used.

- **Runtime per trial:** the scripted demo takes about 26 s of simulated time per trial.
- **Offline checks (no simulator):** `python near_misses.py` scores every near miss below and the history rules.
- **The demo** (`demo()` in task.py) is open loop. It reads only the published layout of the trial: where the cage
  and its door are and where the stand is (both lots are written in `setup` and plainly visible to the top camera).
  It never reads the simulator's states. Its tilted carrying poses include one fixed calibration for the arm's sag under the 0.39 kg
  bottle (4.8 degrees more lean, 12 mm farther, 2 mm left, 18 mm higher), measured once in a GPU trace and the same
  for both layouts. The right hand grips the standing bottle across its 44 mm depth, 35 mm below
  its closed end, lifts the spout out of the stand, leans it 35 degrees so it hangs as the hanger holds it, and brings
  it to the bar gap nearest its usual spot that keeps the bottle 6 mm clear of the door frame. It carries it 16 mm
  above the hung height and 60 mm back, pushes it in along the tube's own line so the tube passes through the gap,
  lowers it until the hook sits on the high wire, opens the fingers, draws back 70 mm along the bottle and returns to
  its rest pose. The gripper is turned with `tilt.py` (a small runtime patch so a scripted waypoint can carry a full
  gripper rotation; it affects only the scripted demo). `lots.py` moves the cage and stand between lots; `parts.py`
  builds the multi-shape bodies.

## Scene

World frame: x away from the robot, y to its left, z up; bench top z 0.7475 m.

- **Cage** (2.0 kg, free on the bench): a wire-top hamster cage 460 x 290 x 271 mm. Blue plastic base with an 8 mm
  floor and walls to a rim 69.35 mm above the bench. 2.5 mm zinc-plated steel wire; front wall bars 12.0 mm clear
  apart (14.5 mm pitch). Horizontal wires on the outside of the bars at 70.6 mm (just above the rim), 220.5 mm (the
  high wire) and 270.0 mm (top frame). A door 80 x 155 mm (40 to 120 mm left of the wall's middle, 95 to 250 mm up)
  framed by 8 mm flat strips standing 6 mm proud of the bars; the high wire stops at the door frame. Front bar plane
  at x 0.420 m. Only the base and the front wall collide; the side, back and roof wires are drawn but never reached.
- **Bottle** (Kaytee Clear 8 oz, filled): oval body 44 x 57 mm, 112 mm long; 30 mm cap; 8 mm stainless sipper tube
  50 mm long with a ball tip. 0.39 kg with water and hanger.
- **Hanger**: 4 mm black epoxy-coated steel; a band round the body 30 mm above the cap; a rigid arm up to a hook with
  an 18 mm top bar and a 12 mm lip. Hung on a wire it holds the bottle 35 degrees from vertical with the tube's tip
  50 mm above the cage floor and 15.8 mm inside the bars.
- **Drip stand** (0.6 kg): base 100 x 110 x 8 mm, two posts, plate 70 x 84 x 6 mm at 75 mm with a 34 x 36 mm hole.
  The bottle stands spout-down in it, hanger arm toward the cage.
- **Lot A**: cage middle at y -0.120 m, stand at (0.240, -0.200) m. **Lot B**: cage middle at y -0.316 m, stand at
  (0.240, -0.330) m. In lot B the door frame is where lot A's bottle hangs, so a replay of lot A's motion meets the
  door, where there is no high wire; the bottle has to go to a gap clear of the frame.

## Scoring

All measured in the cage's own frame (front bar plane x = 0, cage floor 8 mm above the bench), per trial:

- **1.0** when all of these hold together, with both hands more than 5 mm from the bottle and its hanger and the
  bottle and the cage still within 1 mm for 1 s:
  - the hook's top bar rests on the high wire: the wire lies under the 18 mm bar at least 2 mm in from its outer end
    and from its lip, the bar's underside is 1.0 mm below to 1.5 mm above the wire's top, and the hook is at least
    2 mm in from the wire's ends (so not at the door);
  - the tube's tip is at least 8 mm inside the bar plane, at least 10 mm in from the cage's sides and back, and 40
    to 60 mm above the cage floor;
  - the tube points down at least 20 degrees below level;
  - the body's lower edge on the cage side is within 6 mm of the bar plane (resting against the bars).
- **0** otherwise.
- **Whole trial**: 0 if the cage is ever pushed more than 10 mm or tipped more than 3 degrees from its starting pose,
  or if the hands, bottle or cage cannot be read at any step.
- Trials are run on lots A and B and averaged.

By hand: a ruler from the bar plane to the tip and from the cage floor to the tip, a protractor on the tube, a look
that the hook sits on the high wire and the bottle touches the bars, and the video for the cage not being shoved.

How the simulator checks it (task.py): each control step `_step()` reads the bottle, the cage and the hands. The hands
are the gripper and both fingertip links of each arm, sampled as their collision hulls' corners plus points every
3 mm over every hull face, and their distance is taken to every box of the bottle and hanger (round parts count as
their boxes, which only makes the 5 mm rule stricter). The 1 s hold starts when every condition holds with the hands
clear and restarts if any tracked point of the bottle or cage moves more than 1 mm. An unknown state at any step ends
credit for that trial. There is no partial credit: hooked but with the tube outside, or the tube in but the hook not
on the wire, are both failures to give the pet water.

Near misses (near_misses.py, all pass): the exact limits on both sides of hook height and of where the wire lies
under the hook's bar (the bottle settled back on its lower edge, wire 2.0 mm and 1.5 mm from the lip end); hooked
first and let go (tip on the low wire outside, 0); swung part way in (tip 1 mm outside, 0); swung out 1 degree (still
resting, 1.0) and 3 degrees (lower end 6.9 mm off the bars, 0); hooked on the top frame wire (tip 50 mm too high,
0); on the door's top strip (0); at the door with no wire (0); hook wedged 2 mm low (0); upside down (0); still in
the stand (0); a fingertip 4 mm away (0) and 5.5 mm away (1.0); 0.9 s versus 1.0 s of stillness; the cage shoved
10.5 mm (0) or 9.5 mm (1.0) at one step; tipped 3.5 degrees; unknown hands or bottle; and in lot B, lot A's place is
the door (no credit) while lot B's own place scores.

## Results

- **Scripted demo:** 1.0 in lot A and 1.0 in lot B (task_score 1.0, `--epochs 2`, 800 steps per trial). In both
  lots the bottle came to rest with the wire 4.6 mm along the hook's bar from its seat point, the tube's tip about
  19 mm inside the bars and 51 mm above the cage floor, the tube 52 degrees below level, and the hands 43 mm away.
- **Dream-1:** 0.0 in lot A and 0.0 in lot B (both reached the step limit).
- An earlier development trace with the same motion but no carrying calibration left the tip against the base's rim
  outside the bars (0 in both lots).

## Sources

- Bottle: Kaytee Clear water bottle, maker's page https://kaytee.com/all-products/small-animal/clearwater-bottle
  (2.25 x 1.75 x 6.87 in; stainless sipper tube; spring hanger for wire cages). Retailers list the 8 oz size in
  its package at 2.5 x 3 x 10.5 in. The 57 x 44 mm oval body follows the maker's 2.25 x 1.75 in; the split of the
  length into body, cap and tube, the tube's 8 mm diameter and the cap size are estimates to measure.
- Hanger: Kaytee water bottle holder, https://www.kaytee.com/all-products/small-animal/water-bottle-holder (steel,
  epoxy-coated, holds 4 and 8 oz bottles, 6.75 x 1.6 in). Modelled as a rigid hook; the real one is spring steel.
- Cage bar spacing: wire-top cages for Syrian hamsters are sold with about 12 mm between bars (8 mm for dwarf
  hamsters), https://en.wikipedia.org/wiki/Hamster_cage. The 2.5 mm wire, wire heights, base depth and door are
  estimates for a common small wire-top cage, not one product.
- Tip height: 40 to 60 mm above the floor is a common setting so a standing hamster reaches the ball without
  climbing; it is a care guideline, not a product figure.

## Known limits

- Water is a fixed mass in the body; it does not slosh or drip.
- The cage is one rigid body; real cage walls flex a little under a 5 N push.
- The wires are rigid cylinders; real wire tops have welded joints and slight bends.
- Two layout lots stop one fixed replay, not a policy that memorises both layouts.
