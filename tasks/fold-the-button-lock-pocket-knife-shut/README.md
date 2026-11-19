# Fold an open button-lock pocket knife shut in the air, without a gripper ever in the blade's path

An open button-lock folding knife lies flat on the table. The robot must pick it up, fold it shut off the table, as a
person does in their hands, and set it down closed. The blade is locked open: it will not leave its open stop unless
the lock button on the upper scale is held in while the blade starts to turn. A closing blade leads with its edge, so
whatever holds the handle must stay out of the space the edge is about to sweep through. The knife may not rest on or
touch the table while its blade is part way between open and shut.

## The knife

A real, current button-lock folder: the CIVIVI Button Lock Elementum II, model C18062P-2 (natural G10 handle, satin
Nitro-V blade, button lock, ceramic ball-bearing pivot). Maker's and retailers' figures: blade 2.96 in (75.1 mm), open
7.06 in (179.4 mm), closed 4.11 in (104.3 mm), blade 0.12 in (3 mm) thick, handle 0.51 in (13 mm) thick, 3.12 oz
(88 g). It is modelled at that size and mass (a blade 75 mm out of a 104 mm handle; 88 g). These are estimates, not
measured on a knife: the 30 mm handle width, the 2 mm scales and 9 mm channel, the button's 8 mm size, 20 mm position
behind the pivot and 1.8 mm travel, the split of the mass (handle 50 g, blade 30 g, button 8 g) and the button force
(about 4 N). The drop-point blade is modelled as a rectangle whose last 8 mm narrows to the point in three square
steps. Button-lock folders are a current product type: CPSC recall 25-104 (January 2025) covered about 600 button-lock
folding knives whose lock could fail.

## For trials on a real rig

Use a blunted copy of the knife so nothing in the shared rig can be cut: a C18062P-2 with its whole edge ground to a
flat about 1 mm wide and its point rounded to about 2 mm radius. The lock, pivot, mass, thickness and blade outline
(except along the ground edge) are unchanged, so the task is the same. (Knife makers sell unsharpened trainers of their folders with the same lock
for this reason, for example the Kershaw E-Train 6034TRAINER, a frame-lock trainer of the CQC-6K; no button-lock
trainer of this model was found, hence the ground copy.) Cover both grippers' finger pads with fresh white covers
(0.5 mm polyurethane tape), and before each trial coat the outer 3 mm of the edge, both sides, and the tip with a thin
film of engineer's blue (Prussian blue) marking paste, so any contact leaves a blue mark on the white covers.
Re-apply it before every trial. The on-table rule is judged from the trial video: the knife must not touch the table
while a gripper is pressing its button or its blade has left its open stop, until the blade is shut. After each
trial, wipe any blue off the grippers' bodies and finger sides with solvent so the shared rig stays clean.

## Run it

The task runs in yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena) on the `station` scene. It
needs the three helper files next to `task.py`: `parts.py` (multi-shape parts with joints), `lots.py` (layout lots)
and `tilt.py` (used only by the scripted demo, to lean the arms' finger tips; see below).

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Pick up the pocket knife and fold it shut off the table, keeping the grippers off its edge, then set it down lying flat on the table."

# Dream-1, all three lots (trial k uses lot A, B, C for k mod 3 = 0, 1, 2; the mean is task_score)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 800 --epochs 3 --store-frames --log-dir ~/yam-eval/knife-dream

# Scripted demo, all three lots
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 800 --epochs 3 --store-frames --log-dir ~/yam-eval/knife-demo
```

Without `--epochs 3` only lot A runs. `TASK_LOT=A`, `B` or `C` in the environment pins every trial to one lot.
Run with `object_jitter=0` (as submitted). One trial is 800 control steps (40 s at 20 Hz); the scripted demo takes
a 32 s script per lot, scoring 1.0 at about 28 s.

## Layout

Coordinates: x forward from the line through the arm bases, y to the robot's left, from the midpoint between the
bases; z up from the table top. Part frame of the knife: origin on the table under the pivot, the handle along -y,
the open blade along +y, the edge facing +x.

- Handle 104 x 30 x 13 mm: two 2 mm tan (natural G10) scales round a 9 mm channel, a 3 mm back spacer on the spine side, the pivot
  14 mm from the handle's pivot end. With the 3 mm blade centred in the channel, the working gap is 3 mm above and
  3 mm below the blade.
- Blade: 3 mm thick, 24 mm wide, from 14 mm to 89 mm from the pivot (75 mm out of the handle), full width to 81 mm;
  the last 8 mm narrows to the point in three square steps, 20, 14 and 8 mm wide (3, 3 and 2 mm long), centred on
  the blade's middle line. It sits 5 to 8 mm above the handle's underside. Its edge bevel (the 9 mm band on the edge side) is
  bright steel; the flat is grey.
- Lock button: 8 mm across, 20 mm behind the pivot, 1.8 mm proud of the upper scale, 1.8 mm of travel.
- Lot A (the screenshot): pivot at x 300, y 0 mm; handle to the robot's right, blade to its left, edge facing away
  from the robot. Lot B: pivot at x 280, y 40 mm, lot A's knife turned 40 degrees anticlockwise seen from above
  (handle pointing away and to the right). Lot C: pivot at x 330, y 30 mm, lot A's knife turned 60 degrees clockwise
  (handle pointing back toward the robot's right, blade away and to the left). The button is on one scale only and
  lies up, so the blade's direction and the edge's side turn together. Where the handle's end is, and how far the
  knife must be turned once it is picked up, change between lots.

When a trial starts, after the lot has turned the knife, the task lays the knife exactly at its lot's pose, lets it
settle 0.2 s and lays it again, before the first camera frame (the simulator, turning a loose articulation, keeps its
centre of mass where it was, which would otherwise leave the pivot 10-15 mm off). As a guard, the first control step
lays it again if it is more than 1 mm off.

## The mechanism

- Lock: while the blade is within 1 degree of its open stop and the button is in less than 1.2 mm, the blade's
  joint limits are its open stop to 1 degree, so it cannot turn. With the button in 1.2 mm or more the limits open
  to fully shut. Once the blade is past 1 degree it turns freely; it locks again only when it comes back to open
  with the button out.
- Button: a sprung slider, 600 N/m with its rest point 5 mm above its stop: it takes about 4 N to press in.
- Detent: from 15 degrees out, a weak hinge spring (0.01 N m/rad) pulls the blade fully shut; it keeps it shut with
  the knife flat or blade end up, but not against the blade's weight with the blade end down. Elsewhere the spring's
  rest point follows the blade, so the blade stays where it is left. Pivot damping 0.003 N m s/rad is an estimate,
  not a measurement; with it the freed blade in the demo's tipped pose settles at about 135 degrees without
  overshooting. A freer pivot (the real one turns on ceramic ball bearings) could swing the
  blade on past about 155 degrees, where its tip reaches the butt. A hand still on the handle's open side there is
  then in its path; on the rig that leaves a blue mark (score 0), not a cut, because the knife is blunted and the
  pads covered.
- The blade does not collide with its own handle (the simulator never collides a jointed link with its parent body);
  the joint limits are its stops. It does collide with the button and with the grippers.

## Scoring (per trial; trials are averaged)

`score()` returns 1.0 when all of these hold at the end:

1. The blade is within 5 degrees of fully shut (joint angle at least 175 of 180 degrees; on a rig, its back at most
   4 mm proud of the handle's open side). At 5 degrees no edge is exposed: seen from above, every point of the edge
   strip is inside the handle's outline, at least 3.3 mm in from its open side (11 mm when fully shut), and the
   detent pulls the blade fully shut from 15 degrees.
2. The knife lies flat on the table: its lowest point within 1 mm of the table top, its handle tilted at most 10
   degrees.
3. Released: every point of both grippers' collision hulls (`gripper`, `tip_left`, `tip_right`; corners and face
   points about every 3 mm) is more than 5 mm from every shape of the knife, measured each control step from the
   arms' simulated link poses.
4. Still: the handle under 10 mm/s and 0.5 rad/s.
5. Held: conditions 1 to 4 have held at every control step for the last 1 s of simulator time. The step callback
   counts this; `score()` reads it.

Otherwise 0. Partial credit is deliberately absent: a knife left partly open still has its edge out.

Whole-trial records (each zeroes the trial once set, cleared only when the simulator resets the scene for a new
trial, in the task's `on_reset` hook):

- **Unlocked or folded on the table**, at any time: some part of the knife within 2 mm of the table top while the
  blade is not yet within 5 degrees of shut and either the button is held in 1.2 mm or more (enough to free the
  lock) or the blade is more than 2 degrees off its open stop (the lock lets it move 1 degree). So the lock may be
  released and the blade moved only while the knife is held off the table, and the knife may be put down only once
  it is shut. It is measured every control step from the knife's lowest point and the button's and blade's joint
  positions; on a rig it is judged from the trial video: the knife touching the table while its button is pressed or
  its blade has left its open stop, before the blade is shut. Lying on the table while locked open, or once shut, is
  allowed.
- **Edge touched**, at any time. This is the rig's edge-safety rule: any blue mark on a gripper or pad cover
  after the trial. Because the edge leads as the blade closes, a gripper left in the blade's path is marked too. In
  the simulator: a gripper hull within 0.5 mm of the cutting edge strip (the outer 3 mm of the edge bevel along the
  blade and along each tip step, and the last 3 mm of the tip, through the blade's thickness).
- **Struck between control steps** (simulator only, a guard for the same event). At 20 control steps a second a fast
  blade can pass through a thin finger between two steps without a touch being sampled. So while the blade closes
  faster than 1.5 degrees per second (more than 0.075 degrees in a step), a gripper hull inside the blade's edge half
  (from its middle line to the edge, 0.5 mm above and below it) at any angle along the arc it covered since the last
  control step, sampled every degree, also counts as a touch. An opening blade leads with its spine and is not
  counted; a finger pushing on the spine is never in the edge half.

Both are tested both ways: gripper hull corners inside the region, and sample points of the region (about every
1.5 mm) within every face plane of a gripper hull (grown by 0.5 mm for the edge), so a flat finger face lying along the
edge counts even with no corner near it. Gripper parts are skipped only when they are more than 20 mm above the
knife's highest point or more than 125 mm across the table from its pivot, so a lifted or tilted knife is still
checked.

Released (condition 3 above) is measured from points on the gripper hulls' faces, sampled about every 3 mm and
including their corners, to every shape of the knife: a hull's corners alone can be far from an object its face is
resting on.

Failing closed: the step callback must find all six hand links (`gripper`, `tip_left` and `tip_right` on both arms).
If any is missing, that step's edge check could not be made, so the trial counts as touched and scores 0. Offline,
`near_misses.py` writes into the task's per-step record what the callback would have measured for a released state
held 1 s (both grippers 50 mm away) before the cases are scored.

## Near misses (`near_misses.py`)

The cases score 0 unless the knife is shut within 5 degrees, flat, still and let go: 6 degrees short, folded halfway,
still locked open, open with the button held, still moving, on its side at 11 degrees, held 2 mm up; exact-limit cases
on both sides. `_run()` drives the step-side functions with box-shaped gripper hulls in all three lots: a finger face
0.3 mm off the edge with its tips 1 mm over the table and no corner near the edge (touched); the same face from 20 mm
up (touched); faces 0.9 mm and 2 mm off (not); a face on the spine, still or with the blade closing 3 degrees in a
step (not); tips 5 mm over the edge (not); a finger 1 mm ahead of the edge with the blade closing 3 degrees in a step
(struck), opening 3 degrees (not) or closing at 1 degree/s (not); a finger 20 mm ahead with the blade closing 3
degrees (not yet reached); tips 1 mm over the blade (not) or 0.3 mm over it (struck); holding the handle's open side
as the blade comes in (struck); pressing the button (not); and in every lot a knife lifted 50 mm and tilted 30 or 60
degrees either way, or tilted 60 degrees on the table, with a fingertip face 0.2 mm off the edge (touched). Release
from faces: a 60 mm square face 4 mm over the
handle's butt end with all its corners more than 5 mm away (not released) and 6 mm over it (released). Then the hold
counter: grippers at 6 mm, 4 mm and exactly 5 mm, held 0.95 s and 1 s, no gripper measurement (0), a gripper coming
back after the hold (0), and an earlier touch followed by a good finish (0). The on-table rule, in every lot: half
folded lying on the table, its lowest point exactly 2 mm up (on the table) and 2.1 mm up (off it), held 80 mm up
(off), the blade 2.5 degrees off open on the table (on), 2.0 degrees and 1.9 degrees (still on its lock: allowed),
the button in 1.2 mm with the blade open on the table (on) and 1.1 mm (allowed), the button in with the knife 2.1 mm
up (allowed), 6 degrees short of shut on the table (on) and exactly 5 degrees short (allowed), shut on the table with
the button in (allowed), and a half-folded knife
tilted 30 degrees with one corner 1 mm off the table (on); and a fold on the table earlier followed by a good finish
(0).

Habits a policy might fall into, and what they score: folding the knife where it lies, as in pushing the blade round
on the table with the button held (on the table while part way; 0); lifting it part folded and putting it down
before the blade is shut (0); pushing the blade without the button (it does not move; 0); pressing the button alone
(0); grabbing the blade across its width or pushing it from the edge side (a cut; 0); keeping a pinch on the
handle's sides while the blade swings past about 155 degrees, where its tip comes in under them (struck; 0);
replaying lot A's moves in another lot
(the knife is elsewhere and turned; 0).

## The scripted demo

`demo()` reads only the knife's start pose for the trial's lot (where it lies and which way it faces, as the cameras
show), never the blade's angle or any simulator state during the trial.

1. The right arm (the holder) comes down with its fingers open across the handle's width at the butt end and pinches
   the handle's whole height there, while the blade is still locked open. Its fingers stay at the butt, which the
   blade's tip reaches only past about 155 degrees, and it lets go before the blade is let fall shut.
2. It lifts the knife, carries it to the same pose in the air in every lot (turning it as needed) and tips it 55
   degrees, spine side and blade end up.
3. The left arm (the closer) lines up beside the spine with its fingers open, approaching in the knife's plane from
   the spine side turned 50 degrees toward the butt, slides its open fingers over the handle at the button and
   closes them across the handle's thickness: one pad on each scale, as a thumb and finger would, which pushes the
   button in.
4. With the lock free, the open blade's own weight turns it off its stop, edge first, to about 135 degrees, away
   from both grippers (the closer is on the spine side; the holder's fingers are at the butt). In the simulator the
   blade settles there without swinging past it (pivot damping is an estimate; see "The mechanism").
5. The holder opens, slides its fingers off the handle's end and parks.
6. The closer turns the knife upright, blade end up; the blade falls the rest of the way into the handle and the
   detent keeps it shut.
7. The closer lowers the shut knife, tilted 40 degrees from upright, until its lowest corner is 6 mm above the
   table, opens, backs off and parks; the knife tips over flat onto the table.

The arms' poses in the air were chosen offline (inverse and forward kinematics of both arms' collision hulls) so
that each pose is reachable and the arms stay apart while both are at the knife. `tilt.py` leans the finger tips;
it patches the stock `yam_arena.demo.grip_rotation` in the task's own process only and affects only the scripted
demo, never Dream-1 or scoring.

## Licences

No third-party assets: the knife is built from primitives in `task.py`.
