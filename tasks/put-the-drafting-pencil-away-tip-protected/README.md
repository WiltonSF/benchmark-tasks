# Put the drafting pencil away with its tip protected

**Instruction:** Put the drafting pencil in the tray with its tip protected, then let go.

A drafting mechanical pencil modelled on the Pentel GraphGear 1000 (0.5 mm) lies across a two-post pen rest with
its 4 mm lead-guide sleeve sticking out. On the real pencil, pressing the rear button extends the tip and opening
the pocket clip stows it (maker's page, pentel.co.jp: "クリップを開くことでペン先が収納される", opening the clip
stows the tip; pentel.fr: the 4 mm guide pipe is retractable "grace au clip"). Pressing the rear button again only
advances lead. To open the clip the robot presses the clip's raised back end down to the barrel. That press is
behind the rear post, so a loose pencil tips up off its front post instead; one hand has to hold the pencil in the
rest while the other presses the clip. Then the pencil goes in the tray and is let go.

## Run

From the yam-isaac-lab-arena checkout (the program's YAM embodiment for Isaac Lab-Arena), with this folder as
`tasks/put-the-drafting-pencil-away-tip-protected`:

    inspect-robots run tasks/put-the-drafting-pencil-away-tip-protected/task.py \
        --instruction "Put the drafting pencil in the tray with its tip protected, then let go." \
        --max-steps 600 --epochs 2 -E object_jitter=0

Trial k uses lot k mod 2 (lot A first), so `--epochs 2` scores each lot once and reports the mean. Set
`TASK_LOT=A` or `TASK_LOT=B` to pin every trial to one lot. One trial takes about 1-2 minutes on an L40S. The scene
is deterministic; submissions run with `object_jitter=0` (jitter could push the pencil out of its notches).

Offline checks: `python near_misses.py` (every score tier, both sides of each limit, and the shortcuts).

## Files

- `task.py`: scene, scoring, step hook (release and stillness hold, whole-trial rules) and the scripted demo.
- `parts.py`: articulated rigid parts (hinges, spring sliders, tilted boxes) built from boxes and cylinders.
- `lots.py`: per-trial layout lots. `tilt.py`: tilted grips for the scripted demo only (it patches the stock
  grip rotation in the task's own process; it never affects Dream-1 or scoring). `observation/`: face-sampled
  gripper and fingertip surfaces for the let-go check.
- `near_misses.py`: scoring counterexamples.

## Scene (all from task.py)

World frame: x forward from the line through the arm bases, y to the robot's left, z up; table top z = 0.

**Pencil** (one articulated part, about 20 g; frame origin on the long axis 75 mm from either end, +x from the tip
to the button, clip on top). Lengths along the axis from the nose face (the front of the cone):

| part | shape | size |
| --- | --- | --- |
| lead-guide sleeve | metal tube | 1.2 mm across, 4.0 mm out of the nose when extended, flush when stowed |
| nose cone | three silver steps | 3, 5, 7 mm across; 4, 5, 5 mm long |
| grip | silver knurled cylinder | 10 mm across, 14 to 54 mm behind the nose |
| barrel | dark cylinder | 9 mm across, 54 to 137 mm |
| rear button | silver cylinder | 6 mm across, 137 to 146 mm (rigid; lead feed not modelled) |
| clip blade | silver strip, 3.5 mm wide, 1.2 mm thick | 81 to 117 mm, 0.5 mm above the barrel, with a bead at its front jaw |
| clip hinge | cross pin | 117 mm behind the nose, 5.6 mm above the axis |
| clip press end | 16 mm strip rising 0.22 rad (13 degrees) from the hinge | far end about 4 mm above the barrel |

Mechanism: the sleeve and its inner tube form a tip carrier (5 g) that slides 4.0 mm along the axis. Its spring
(20 N/m, free 19 mm past extended, so about 0.3-0.4 N) pulls it back into the nose. A shoulder on the tube rests
against a 1.5 mm tooth that hangs from the clip blade through a slot in the barrel (103-105 mm behind the nose),
1.5 mm of overlap, so the sleeve stays out. The clip (4 g) turns on its hinge between shut (0 rad) and 0.24 rad
open, where its press end meets the barrel; its spring (0.03 N m/rad, preloaded to -0.5 rad) holds it shut with
about 0.015 N m, roughly 1-2 N at the press end. Opening the clip by 0.115 rad or more (press end down about 2 mm)
lifts the tooth clear and the spring pulls the sleeve in; the tooth then drops in front of the shoulder. Pressing
the front half of the clip or the rear button does nothing. Mass split 11 g barrel, 5 g carrier, 4 g clip
(estimate). Friction 0.9 static, 0.8 dynamic.

**Pen rest** (loose, 0.15 kg): base 66 x 36 x 5 mm; two posts 8 mm thick, 50 mm apart, each with a 90-degree V
notch whose faces touch the 9 mm barrel, putting the axis 25 mm above the table. The front post is 61 mm behind
the nose, the rear one 111 mm, so the clip's press end (117-133 mm) overhangs the rear post.

**Pen tray** (loose, 0.12 kg): inner 190 x 40 mm, walls 3 mm thick and 15 mm high, floor 3 mm; parallel to the
pencil, centre 90 mm from the pencil's axis towards the robot.

**Lots.** The pencil and rest are the same in both lots: pencil centre x 340, y 0 mm, lying along y with the tip
to the robot's left (its +x pointing to the robot's right); rest centred under it 15 mm towards the button.
Lot A (screenshot): tray centre x 250, y 0 mm, parallel to the pencil, 90 mm nearer the robot. Lot B: tray centre
x 290, y 200 mm (to the robot's left), its length pointing away from the robot, so the pencil has to be turned a
quarter turn on the way. A fixed habit (always set the pencil down 90 mm nearer the robot) scores at most 0.5 in
lot B.

## Scoring (task.py `score`, timed in the step hook)

Per trial:

- **0.5, tip protected:** carrier travel at least 3.7 mm, so the sleeve sticks out no more than 0.3 mm.
- **0.5, in the tray:** every corner of every pencil shape's box inside the tray's inner walls (in the tray's
  frame), the pencil's axis 0 to 9 mm above the tray floor (lying on it, not held above it or propped on a wall),
  the long axis within 10 degrees of level, and the tray within 5 degrees of flat.
- **Release and stillness:** either half counts only after 1 s in which every sampled point of both grippers and
  all four fingertips (hull vertices plus faces sampled every 3 mm) is more than 5 mm from every pencil and tray
  shape, the pencil and tray move under 5 mm/s and turn under 0.05 rad/s (from their pose change over each
  control step), and the carrier and clip move less than 0.002 (m or rad) per control step. Unknown hand state, a non-finite state or a missed control step gives 0 (fail closed).
- **Whole trial:** 0 if the pencil ever leaves the table (x -50 to 600 mm, y -450 to 450 mm, or falls below the
  table top).

Trials are run on each of lots A and B and averaged.

Near misses (all in `near_misses.py`, all pass): housed and in the tray 1.0; 0.95 s hold 0; housed on the rest
0.5; in the tray with the sleeve out (rear button pressed instead) 0.5; sleeve 0.4 mm proud 0.5 and exactly
0.3 mm proud 1.0; no-op 0; clip held open with the sleeve out 0; rolled onto its clip side 1.0; turned 10 degrees
1.0, 30 degrees (end over the wall) 0.5; end past the end wall 0.5; against the inner side wall 1.0, 1 mm over it
0.5; axis exactly 9 mm up 1.0, 9.5 mm 0.5; propped 11 degrees 0.5; tray tipped 0.5; moving 0; left the table 0;
unknown hands 0; stale or NaN state 0; a hand point 0.5 mm from a face centre counts as touching; lot B: in lot
B's tray 1.0, laid in lot A's tray spot 0.5.

## Demo

Open loop from the published layout (the pencil's and tray's visible start poses for the lot), using `tilt.py`
for a 30-degree lean of the holding hand that keeps the two arms apart. The hand on the tip's side pinches the knurled grip in place on
the rest; the other hand, fingers closed, presses the clip's raised end down to the barrel and lifts off; the
holding hand lifts the pencil out of the notches, lowers it level into the tray 6 mm above the floor, lets go and
lifts clear. The arm that pressed parks. The demo reads no joint state, mass, friction or lot id.

## Fidelity

Rigid shapes at the listed length, width and about 20 g. The internal catch (a tooth on the clip against a
shoulder on the tip carrier), both springs, the clip's opening angle and the 11/5/4 g mass split are engineering
estimates: the maker confirms that the rear button extends the tip and opening the clip stows it, not the internal
profile or forces. The rear button is rigid (lead feed is not modelled, and nothing re-extends the sleeve within a
trial); the lead is not modelled. The knurling is a plain cylinder. Two lots stop a fixed habit, not a memorised
replay of both layouts.

## Sources

- Pentel GraphGear 1000 product page (tip extends with the rear button, stows when the clip is opened):
  https://www.pentel.co.jp/products/mechanicalpencil/graphgear1000/
- Pentel France PG1013 page (4 mm guide pipe, retractable by the clip, metal clip):
  https://www.pentel.fr/catalogue/pg1013/
- Size and mass (about 150 x 10 x 9 mm, about 20 g) are from retailer listings of the PG1015; Pentel does not
  publish them on these pages.

No third-party assets are used; every shape is a primitive written in task.py.
