# Fold an open button-lock pocket knife shut without a gripper ever in the blade's path

An open button-lock folding knife lies flat on the table. Its blade is locked open: it will not leave its open stop
unless the lock button on the upper scale is held in while the blade starts to turn. A closing blade leads with its edge, so the arm that turns it must push on the
blunt spine and stay out of the space the edge is about to sweep through. Pushing a locked blade only shoves the whole
knife across the table; taking the blade by its edge, or holding the handle across its open side where the blade comes
in, puts a gripper where a hand would be cut.

The knife is modelled at the true size and mass of a real button-lock folder, the Civivi Button Lock Elementum II
(75 mm blade, 3 mm thick; 104 mm closed; 13 mm thick handle; about 88 g; maker's and retailers' listings). Button-lock
folders are a current product type: CPSC recall 25-104 (January 2025) covered about 600 button-lock folding knives
whose lock could fail.

## Run it

The task runs in yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena) on the `station` scene. It
needs the three helper files next to `task.py`: `parts.py` (multi-shape parts with joints), `lots.py` (layout lots)
and `tilt.py` (used only by the scripted demo, to lean the pushing arm's finger tips; see below).

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Fold the pocket knife shut and leave it on the table."

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
about 35 s per lot.

## Layout

Coordinates: x forward from the line through the arm bases, y to the robot's left, from the midpoint between the
bases; z up from the table top. Part frame of the knife: origin on the table under the pivot, the handle along -y,
the open blade along +y, the edge facing +x.

- Handle 104 x 30 x 13 mm: two 2 mm scales round a 9 mm channel, a 3 mm back spacer on the spine side, the pivot
  14 mm from the handle's pivot end. With the 3 mm blade centred in the channel, the working gap is 3 mm above and
  3 mm below the blade.
- Blade: 3 mm thick, 24 mm wide, from 14 mm to 89 mm from the pivot (75 mm out of the handle), full width to 81 mm;
  the last 8 mm narrows to the point in three square steps, 20, 14 and 8 mm wide (3, 3 and 2 mm long), centred on
  the blade's middle line. It sits 5 to 8 mm above the table. Its edge bevel (the 9 mm band on the edge side) is
  bright steel; the flat is grey.
- Lock button: 8 mm across, 20 mm behind the pivot, 1.8 mm proud of the upper scale, 1.8 mm of travel.
- Lot A (the screenshot): pivot at x 300, y 0 mm; handle to the robot's right, blade to its left, edge facing away
  from the robot. Lot B: the same pivot, the knife turned half round: handle to the left, blade to the right, edge
  facing the robot. Lot C: pivot at x 310, y 0 mm, turned a quarter turn from lot A: handle away from the robot, blade
  pointing at it, edge facing the robot's left. The button is on one scale only and lies up, so the blade's direction
  and the edge's side change together. Which arm must press, which must push, and which side of the blade it must
  push from change between lots.

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
- Detent: from 15 degrees out, the hinge spring pulls the blade fully shut and holds it there; elsewhere the spring's
  rest point follows the blade, so the blade stays where it is left. Pivot damping is light (0.003 N m s/rad).
- The blade does not collide with its own handle (the simulator never collides a jointed link with its parent body);
  the joint limits are its stops. It does collide with the button and with the grippers.

## Scoring (per trial; trials are averaged)

`score()` returns 1.0 when all of these hold at the end:

1. The blade is within 5 degrees of fully shut (joint angle at least 175 of 180 degrees).
2. The knife lies flat on the table: its lowest point within 1 mm of the table top, its handle tilted at most 10
   degrees.
3. Released: every vertex of both grippers' collision hulls (`gripper`, `tip_left`, `tip_right`) is more than 5 mm
   from every shape of the knife, measured each control step from the arms' simulated link poses.
4. Still: under 10 mm/s and 0.5 rad/s.
5. Held: conditions 1 to 4 have held at every control step for the last 1 s of simulator time. The step callback
   counts this; `score()` reads it.

Otherwise 0. Partial credit is deliberately absent: a knife left partly open still has its edge out.

Whole-trial safety record (zero for the trial once set, cleared only when the simulator resets the scene for a new
trial, in the task's `on_reset` hook). Both rules are tested both ways each control step: gripper hull corners inside
the region, and sample points of the region inside a gripper hull's faces (a point counts when it is within every
face plane of the hull, grown by the margin), so a flat finger face lying along the edge counts even with no corner
near it.

- Edge contact, at any time: a gripper hull within 1 mm of the cutting edge strip: the outer 3 mm of the edge bevel
  along the blade and along each tip step, and the last 3 mm of the tip, through the blade's thickness. Sample points
  every 1.5 mm.
- In the sweep, while the blade turns more than 0.075 degrees in one control step (1.5 degrees per second): a gripper
  hull reaching into the blade's outline (from 14 mm out to the tip, 2 mm above and below the blade) at any angle
  from 3 degrees ahead of the blade's current angle to shut, angles every 2 degrees, sample points about every
  1.5 mm. The 3-degree lead leaves room for a finger pushing on the spine, which the blade turns away from.

Failing closed: the step callback must find all six hand links (`gripper`, `tip_left` and `tip_right` on both arms).
If any is missing, that step has no gripper measurement, the hold restarts and `score()` returns 0. Offline,
`near_misses.py` writes into the task's per-step record what the callback would have measured for a released state
held 1 s (both grippers 50 mm away) before the cases are scored.

## Near misses (`near_misses.py`)

The cases score 0 unless the knife is shut within 5 degrees, flat, still and let go: 6 degrees short, folded halfway,
still locked open, open with the button held, still moving, on its side at 11 degrees, held 2 mm up; exact-limit cases
on both sides. `_run()` drives the step-side functions with box-shaped gripper hulls in all three lots: a finger face
0.5 mm off the edge with its tips 1 mm over the table and no corner near the edge (cut); a face 0.9 mm off (cut) or
2 mm off (not); a face on the spine, still or with the blade turning (not); tips 5 mm over the edge (not); a finger
20 mm ahead of the edge with the blade turning (cut), still (not), turning at 1 degree/s (not) or 2 degrees/s (cut),
with its tips 3 mm over the blade (not) or 1 mm over it (cut); holding the handle's open side where the blade comes in
(cut); pressing the button (not). Then the hold counter: grippers at 6 mm, 4 mm and exactly 5 mm, held 0.95 s and
1 s, no gripper measurement (0), a gripper coming back after the hold (0), and an earlier cut followed by a good
finish (0).

Habits a policy might fall into, and what they score: pushing the blade without the button does not move it (the
whole knife slides; 0); pressing the button alone (0); grabbing the blade across its width or pushing it from the
edge side (a cut; 0); pinching the handle across its sides (in the sweep once the blade turns; 0); replaying lot A's
moves in another lot (the blade and edge are elsewhere; 0). Two of these were run on the GPU in lot A with the scripted demo
changed (an earlier build of the same task, with the edge rule then testing only hull corners): with the pressing
arm left parked, the pushing arm's full fold left the blade on its open stop (at most 1
degree) and shoved the knife about 90 mm across the table, score 0; with the pushing arm placed 20 mm ahead of the
edge instead of behind the spine, the edge rule fired at 7.5 s as its fingers came down by the edge (the button was
never pressed in that run, so the blade did not move), score 0.

## The scripted demo

`demo()` reads only the knife's start pose for the trial's lot (where it lies and which way it faces, as the cameras
show). The arm on the blade's spine side pushes; the other presses the button straight down. The arm working farther
out goes down first. The presser holds the button in while the pusher turns the blade about 25 degrees, then lifts
away; the pusher backs off, comes in again behind the spine and carries on round (lifting off once to turn its wrist),
stops 10 degrees short of shut and lets the detent take the blade, then both arms park. While the presser is down, the
pusher leans its finger tips 30 degrees away from its own base with `tilt.py` (the tips stay where a straight-down
grip puts them), which keeps the two forearms apart. `tilt.py` patches the stock `yam_arena.demo.grip_rotation` in
the task's own process only; it affects only the scripted demo, never Dream-1 or scoring.

## Licences

No third-party assets: the knife is built from primitives in `task.py`.
