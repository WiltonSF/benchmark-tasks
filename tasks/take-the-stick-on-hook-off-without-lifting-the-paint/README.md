# Take a stick-on wall hook off without lifting the paint

A stick-on utility hook is stuck to a painted wall-board panel with a stretch-release foam strip. The strip's grip
tab hangs over the panel's edge. The robot must get the hook off without tearing the paint, and put it in a tray.

Stretch-release strips (3M Command and similar) let go because the foam is stretched along the wall: the maker's
product sheet says to grab the removal tab and slowly stretch it straight down along the wall at least 12 inches until
the hook and strip release, and never to pull the strip toward you; its product page also says to hold the hook while
pulling. Pulling the hook off the wall, or pulling the tab out and away from the wall, peels the bond instead and takes
paint (and often the drywall's paper face) with it. That is the most common way these "damage-free" hooks damage a
rental wall.

## Real-world layout and why the panel lies flat

The panel is a true-size section of 1/2 in painted drywall (250 x 250 mm). It lies on its back on two battens, held
at its corners by clips. The hook sits with its
"up" end toward the middle of the panel and its tab hanging over the near edge, so "down the wall" is the direction
from the hook toward that edge, along the face. A pull "straight down along the wall" is a pull along the panel's face
past that edge; a pull "toward you" is a pull up off the face. Gravity on the hook is the only thing that changes,
and the hook's weight (15 g) is far below the bond's strength either way. The hook sits 3 mm from the panel's edge
so that the tab hangs free over it and can be pinched; on a real wall the tab lies flat against the wall below the
hook.

## Objects

| Object | Size | Notes |
|---|---|---|
| Painted panel | 250 x 250 x 12.7 mm, 0.50 kg | 1/2 in gypsum board, eggshell paint on the face; centre at x 270, y 0 mm; face 62.7 mm above the table |
| Battens | two, 30 x 250 x 50 mm | pine, under the panel at x 185 and 355 mm |
| Corner clips | four | dark steel; at each corner two 30 x 8 mm posts 0.5 mm off the edges and a lip over the corner 0.5 mm above the face |
| Hook | plate 24 x 62 x 4 mm; stem 14 x 14 x 21 mm; 15 g | white plastic; a medium utility hook (3 lb rating); its stem rises 44 mm from the plate's lower end, its arm runs on to the plate's upper end and ends in a 5 mm lip |
| Adhesive strip | 19 mm wide, 1.2 mm foam; 50 mm bonded under the plate, 19 mm more past the bond to the panel's edge | white; the medium strip is 3/4 x 2-3/4 in (19 x 70 mm) |
| Grip tab | 19 x 15 x 2 mm | red, on the strip's free end, hanging over the panel's edge |
| Tray | 100 mm (x) by 150 mm (y) inside, 4 mm walls 25 mm high, 3 mm floor | grey plastic, centre x 78, y 0 mm |

Coordinates: x forward from the line through the two arm bases, y to the robot's left, from the midpoint between the
bases; table top 0.

**Lot A** (the screenshot): the hook near the panel's right edge (robot's right): the plate's lower end 3 mm from
that edge at y -122 mm, centred on x 270 mm; the tab hangs over the right edge. **Lot B**: the panel turned half
round in its clips, so the hook is near the left edge, its tab hanging over the left edge. Trials alternate A, B.

## How the strip and bond are modelled

- The 50 mm bond is five 10 mm cells, released one at a time from the tab end. The strip is five chained
  stretching segments (one per cell, 8 g each, 120 N/m each) and the hinged tab.
- A bonded cell's segment cannot stretch. The front cell's segment stretches under the pull, and the cell lets go once
  it has been stretched 61 mm. Then the next cell becomes the front. The five cells need 305 mm (12 in) of pull in
  all, and the strip pulls about 7 N while it lets go.
- A cell that lets go while the tab is lifted more than 30 degrees off the face takes its paint with it.
- With the tab lifted more than 45 degrees, the front cell peels off with its paint once its segment carries half the
  release pull.
- The hook's plate is held on the face by the bond, as springs: 4,000 N/m sideways and 3,000 N/m off the face. If
  the plate is pulled more than 2 mm off the face (about 6 N) or 4 mm sideways (about 16 N) while any cell is still
  bonded, every cell still bonded tears away with its paint.
- Once every cell has let go, the hook is free. Torn cells show as brown bare paper on the face.
- The hook moves on three hidden slides (the bond's springs, slack once it is free) and does not turn. The used strip
  stays on the plate and springs back once the tab is let go, and its tab then lies back flat along it (a light
  spring on the tab's hinge), where a real spent strip comes away in the hand.

## Scoring

Per trial:
- **0** if paint came away under any cell at any time in the trial.
- **0.5** once the strip has let go of the whole bond with no paint lifted.
- **1.0** when, in addition, the hook's plate lies inside the tray's walls, its lowest point within 4 mm of the
  floor, let go (every gripper part more than 5 mm from the hook, its strip and tab, gripper faces sampled about every
  3 mm) and still (under 10 mm/s) for 1 s.
- If the grippers cannot be measured, the trial scores 0.

The record is cleared only by the simulator's reset for a new trial. Trials are run on lots A and B and averaged.

## Run it

The task runs in yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena) on the `station` scene. It
needs `parts.py` (multi-shape parts with chained joints) and `lots.py` (layout lots) next to `task.py`. Run with
`-E object_jitter=0`.

```bash
I="Take the hook off the painted panel without damaging the paint, and put it in the tray."

# Dream-1, both lots (trial k uses lot A, B for k mod 2 = 0, 1; the mean is task_score)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 800 --epochs 2 --store-frames --log-dir ~/yam-eval/hook-dream

# The scripted demo, both lots
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 800 --epochs 2 --store-frames --log-dir ~/yam-eval/hook-demo
```

Set `TASK_LOT=A` or `TASK_LOT=B` to run every trial in one lot. A trial is 800 steps (40 s at 20 Hz).

The scripted demo runs open loop from the lot's layout (which edge the hook is on, as the top camera shows it). One
hand pinches the hanging tab by its edges and draws it out level with the face, past the edge, about 0.4 m in all.
The other hand holds the hook by its stem from early in the pull until the strip has let go, then lifts the hook and
lays it in the tray. Holding the hook follows the maker's advice; the scoring does not require it (the bond holds the
hook on its own while the strip is pulled along the face). What stops a pull toward you is the scoring's tab-angle
rule (a cell that lets go with the tab more than 30 degrees off the face, or more than 45 degrees under half the
release pull, tears) and, for a hard pull on the hook or the strip, the 2 mm plate-lift limit.

## Real trials

Use a fresh medium hook and strip each trial, on a fresh primed and painted drywall coupon (or repaint and let dry).
Clean the face with isopropyl alcohol before applying, press the hook for 30 s and wait at least one hour, as the maker
directs. After the trial, look under the footprint: any patch where paint is missing or the brown paper shows counts
as paint lifted.

## Sources

- 3M Command product information sheet: "Grab removal tab and slowly stretch straight down along the wall at least
  12 inches ... Never pull the strip towards you." Also the 3M product page for medium hooks.
- Medium refill strips (17021): 3/4 in x 2-3/4 in, 3 lb rating (retailer listings of the 3M product).
