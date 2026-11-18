# Set a printer paper tray's guides snug to a fresh A4 stack

A laser printer's paper cassette has been pulled out and set on the desk to be refilled. A fresh pile of A4 has been
dropped in loosely. The paper is four loose 25-sheet bundles, staggered in 17 mm steps along their length, slightly
skewed, and short of the feed edge. The tray's guides are still
set for Letter width and Legal length. The robot must set the guides to the paper, so the stack is square against
the feed edge with every guide touching it, without squeezing it, and then let go.

## Credit

This task takes up an idea first submitted by another team and closed:

- PR [#1781](https://github.com/robocurve/benchmark-tasks/pull/1781), "Add paper-tray-side-guides", TK-WGL4TM, by
  Bhuvan-rayani. Closed Nov 12 as final. On importance, the reviewer wrote that loading printer paper is an
  occasional chore for office workers and "the scenario is more suited to societal relatability". On construct
  validity, a solid block with a grab handle had stood in for the paper, and a featureless frame for the printer.
- PR [#1782](https://github.com/robocurve/benchmark-tasks/pull/1782), "Set a paper cassette's two guides snug against
  the stack", TK-LL8AMQ, by matteoroda05. Closed Nov 12 as a duplicate of TK-WGL4TM.

This version is a new build from scratch. No code or assets from either pull request were used. Each review point is
addressed as follows:

- **Category.** Societal relatability (home and office printing), as the 1781 reviewer suggested. It is not framed as
  anyone's job.
- **No solid paper block.** The paper is four separate 25-sheet bundles of A4 that slide and turn on each other.
  They start staggered 3 to 54 mm short of the feed edge and slightly turned, and the robot must square the stack during the trial. There is no handle and
  nothing is grasped on the paper.
- **A real tray, not a featureless frame.** The cassette is true size, modelled on the 250-sheet Tray 2 of
  HP LaserJet Pro M404-class printers (A4, Letter and Legal). It has linked width guides and a rear length guide,
  each locked until its squeeze latch is pinched, a feed edge and walls. Exact tray dimensions are estimates.
- **Clarity.** Every dimension, mass and start offset is in `task.json` and below.

## Objects

| Object | Size | Notes |
|---|---|---|
| Cassette | 304 x 417 mm outside, 3 mm floor | dark grey plastic, fixed on the table; the front panel is 65 mm tall and 12 mm thick, with a handle lip; the side and back walls are 30 mm tall |
| Width guides (2) | 36 mm tall, 100 mm long (y -70 to 30) | light grey; linked, so they close and open together about the tray's centre line (x 270); widest setting 218 mm apart |
| Length guide | 140 mm wide, 36 mm tall | light grey; starts at the Legal setting, its face 356 mm from the feed edge |
| Guide pads | 3 mm thick on each guide's face | sprung, 1,500 N/m; pressed in 3 mm, a pad is flush with its guide |
| Squeeze latches (2) | a 30 mm wide post and a 4 mm lever, 6 mm apart, standing 26 mm above the guide | blue; on the near (robot-side) width guide and on the back of the length guide |
| Paper | four bundles of 25 sheets, each 210 x 297 x 2.5 mm, 125 g | A4 80 g/m2 copy paper, 100 sheets in all; paper-on-paper friction 0.45 static / 0.35 sliding, plastic 0.35 / 0.30 |

Coordinates: x forward from the line through the two arm bases, y to the robot's left, from the midpoint between the
bases; table top 0. The feed edge (the inside face of the front panel) is at y -160 mm. The paper path is centred on
x 270 mm.

**How the latches work.** Each latch lever is hinged at its foot and springs open (0.4 N m/rad, about 4 N at the
fingertips to close it). While the lever is squeezed 0.10 rad or more toward its post, the guide slides freely, with
30 N s/m of drag. Once the lever opens, the guide is held where it is by a stiff drive (50,000 N/m). The far width
guide follows the near one through a stiff drive, as the rack under a real tray does.

**Lots.** Lot A (the screenshot): the bundles, from bottom to top, lie 3, 20, 37 and 54 mm short of the feed edge (17 mm apart).
They are offset across the tray by +0.8, -0.6, +0.5 and -0.8 mm and turned +0.4, -0.5, +0.7 and -0.3 degrees. Lot B:
the bundles lie 54, 37, 20 and 3 mm short, with the offsets and turns reversed in sign. Trial k uses lot k mod 2.

## Scoring

Scored at the end of the trial.

- **No credit (0)** unless every gripper and fingertip face (sampled about every 3 mm, not only hull corners) is more
  than 5 mm from the paper, every guide and both latches, both latches are let go, and the paper and guides have been
  still for 1 s. If the arms cannot be read, the trial scores 0.
- **0** if any guide's pad is pressed in more than 2 mm (3 N). This is the stand-in rule for a squeezed stack
  that bows and lifts at its edge. It also scores 0 if any bundle is tilted more than 2 degrees, or lies higher than
  the stack (riding on a guide).
- **0.5** when the length guide's face is within 1 mm of the stack's rear edge, and every bundle is square (turned
  1 degree or less) with its leading edge within 1 mm of the feed edge.
- **+0.5** when each width guide's face is within 1 mm of the stack's side.

The gaps are measured from the bundles' corners. On a real trial, a 1 mm feeler gauge should not enter between any
guide and the stack, and the pad's press can be read from a scale on the guide.

## Running it

    inspect-robots run --embodiment yam_arena --policy dream -E task=tasks/set-the-printer-tray-guides-snug-to-the-paper/task.py \
      -E object_jitter=0 -E control_hz=20 --instruction "<instruction from task.json>" --scorer task_score \
      --max-steps 600 --epochs 2

`--epochs 2` runs lot A, then lot B, and reports the mean. Without it only lot A runs. Set `TASK_LOT=A` or `TASK_LOT=B`
to pin every trial to one lot. Use `--policy yam_scripted` to play the scripted demo. One trial takes about 1-2 minutes
on an L40S. `object_jitter=0`: the layout is fixed per lot.

The demo (`demo()`) is open loop. It uses the published layout: the guides' visible start settings and the printed
paper size (A4). It never reads the lot, joint states or object poses. The left arm pinches the length guide's latch
and slides the guide toward the feed edge, pushing hard enough to slide every bundle up to the edge (the lowest sheets
take a few newtons to slide under the others). It then eases the guide back to just touching, lets the latch go and
moves away. The right arm then does the same with the width latch, closing the linked width guides onto the stack.
Letting a latch go while still pushing hard leaves the pad pressed in more than 2 mm, which scores 0.

## Files

- `task.py`: the scene, the latch rule, the scorer and the demo.
- `parts.py`: a kit for jointed multi-shape parts (guides, pads, latch levers) and per-step callbacks.
- `lots.py`: the two start layouts.
- `near_misses.py`: near-successes and unintended strategies, with their expected scores. Run it directly
  (`python near_misses.py`) to also check lot B's start and the release check against a gripper face 0.5 mm from a
  latch whose hull corners are far away.

Code MIT.
