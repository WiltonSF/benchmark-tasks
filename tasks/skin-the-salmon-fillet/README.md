# Skin a salmon fillet with a filleting knife without cutting into the flesh, and put the flesh on the plate

**Instruction:** Skin the salmon fillet with the knife, put the flesh on the plate, and put the knife down.

A chilled skin-on salmon portion lies skin down on a cutting board, its tail end to the left, with a 40 mm tab of
bare skin sticking out past the flesh at that end (the usual starting cut, made before the trial). A flexible
filleting knife lies on the counter beside the board and a side plate stands on the other side. The robot takes the
knife by its handle, pins the skin tab to the board with its other gripper, runs the edge along the boundary between skin and
flesh for the whole 140 mm of the fillet, puts the knife down, and lifts the skinned flesh onto the plate, leaving
the skin on the board.

## Why this task

- Skinning a fillet is one of the commonest fish-preparation jobs in a home kitchen (before pan-searing, poaching,
  cubing for a curry or a poke bowl), and a routine step for food preparation workers (O*NET 35-2021.00) who cut
  and portion fish.
- The skill is keeping a blade on a boundary between two attached layers while the other hand pins the skin.
  Edge angled up: flesh stays on the skin and is thrown away. Edge angled down or pressed steeply: it cuts through
  the skin, which then has to be cut off in strips. Skin not pinned: the skin bunches or the fillet moves with the
  knife. Both the skin and the flesh have to come out whole.

## Scene (true size, one fixed layout per lot)

Coordinates are metres in the station frame: x away from the robot, y to its left, z up. Counter top z = 0.7475.

| Item | What it is | Size and source |
|---|---|---|
| Cutting board | OXO Good Grips Carving & Cutting Board 11272900, polypropylene, non-slip feet | 535.9 x 368.3 mm (maker), 20 mm thick (estimate); fixed, centre (0.550, 0), top z = 0.7675 |
| Salmon portion | Chilled (0-4 C) skin-on Atlantic salmon fillet portion, skin down, length along y, tail end to the left | flesh 140 x 62 mm, 11-24 mm thick (thin belly edge, thinner tail end), 164 g (volume x 1.05 g/cm3); flesh centre (0.335, 0) |
| Skin | The portion's own skin | 1.2 mm thick, 62 mm wide, 180 mm long, 14 g; the last 40 mm (y 0.070 to 0.110) is a bare tab past the flesh's tail end |
| Knife | Victorinox Fibrox Pro 5.3763.20, 8 in (20 cm) flexible filleting knife | 340 mm long, 200 mm blade, 85 g (maker); blade 16 mm wide and 1.6 mm thick, handle 140 x 30 x 23 mm (estimates from maker photos); lying flat on the counter, blade pointing +x, edge toward -y (the robot's right) |
| Plate | White stoneware side plate | 200 mm across, 6 mm base, 10 mm rim; fixed at (0.370, 0.310); base top z = 0.7535 |
| Wall | Kitchen wall behind the counter | at x = 0.87 |

- **Lot A:** knife handle centre (0.205, -0.235, 0.759).
- **Lot B:** knife handle centre (0.215, -0.265, 0.759).
- The portion and plate are the same in both lots. No jitter (`-E object_jitter=0`), no seeds. The screenshot shows
  lot A.
- Drawing: `fixture/portion.svg` (portion, tab and strip thicknesses).
- The tab: the tail-end 40 mm of flesh is cut off down to the skin before the trial, as cooks do to get a grip on
  the skin. That trimmed flesh is not part of the trial; the 164 g main piece is.

## Score

**Score 0 for the trial, whatever happens later, if at any moment:**
- the salmon, the skin or the knife falls off the counter;
- the knife's cutting edge comes within 10 mm of the gripper that is not holding the knife by its handle, or any
  part of the blade touches the gripper that is holding it (real trial: overhead and side video, stepped frame by
  frame);
- the edge cuts into the flesh more than 6 mm above the skin (real trial: any cut into the underside or through the
  portion deeper than 6 mm, ruler in the cut);
- the skin is cut through (real trial: any slit in the skin, held up to the light; the simulation stands in for this
  with the blade pressed on the skin more than 25 degrees edge-down or with more than 40 N);
- more than 10% of the flesh mass is left on the skin (real trial: pink flesh scraped off the skin and weighed).

**Otherwise it is judged at the end, when both grippers are at least 30 mm from the flesh, the skin and the knife;
each of these rests on the counter, the board or the plate; and nothing has moved more than 1 mm for 1 s. If not,
the score is 0.**

| Score | Condition | How it is read in a real trial |
|---|---|---|
| 1.0 | Skin completely off the flesh; flesh on the plate, inside its rim; skin not on the plate; at most 5% of the flesh mass (8 g) left on the skin | top photo of the plate and board; skin laid flesh side up; flesh scraped off the skin and weighed on a 0.1 g kitchen scale |
| 0.75 | Skin completely off, but any other 1.0 condition is not met: the flesh is not on the plate, the skin is on the plate, or 5-10% of the flesh mass is left on the skin | same photos and scale |
| 0.5 | 50-99% of the skin-flesh interface separated | skin lifted back and the separated length measured with a ruler along the fillet (or the attached area counted on a 5 mm grid) |
| 0.25 | 25-49% separated | same |
| 0 | Under 25% separated, or any end condition not met | |

Lots A and B are averaged. "Flesh left on the skin" is the pink flesh: the thin grey fat layer that stays on salmon
skin after a clean cut is not counted (the simulation allows the first 1 mm of edge height above the interface for
it, the 1 mm clean-edge allowance). "Held by its handle" means the two gripper pads on opposite faces of the
handle: its two narrow sides (as when picked up lying flat) or its two flat sides (a side pinch); either counts.

## How the simulation scores it (`task.py`)

PhysX cannot cut, so cutting is a declared rule, checked every control step (20 Hz):

- The flesh is one rigid body (so it cannot split; the 6 mm rule is what protects it). The skin is a chain of 1.2 mm boxes: the 40 mm tab, then seven 20 mm links under the
  flesh, joined by hinges across the fillet so the skin can bend. Each link is held to the flesh by a fixed joint
  (a "bond") until the rule releases it. Pulling on the skin alone never releases a bond.
- The knife passes through the flesh (knife-flesh contact is filtered) but collides with the skin, the board, the
  counter and the grippers, so a blade pressed down rides on the skin.
- Cutting resistance. While the stroke is going on and uncut interface lies within 8 mm of the edge line (in the
  separation band), a force is applied to the knife at its centre of mass, opposing the edge's motion relative to
  the flesh: 40 N per metre of engaged edge against edge-first (feed) motion plus 15 N per metre against motion
  along the blade, each reaching its full value at an edge speed of 5 mm/s (speed measured over the last 0.25 s;
  a stopped knife is not pushed back). With the whole 62 mm width engaged that is 2.5 N of feed resistance. The equal and opposite force
  is applied to the skin link under the edge, so it pulls the skin (and, through the bonds, the portion) along with
  the knife: the knife arm has to push through it and the gripper on the tab has to hold the skin against it.
  These magnitudes are estimates of the push a cook feels skinning a chilled fillet with a sharp flexible knife
  (a few newtons over a 60 mm cut), not measurements.
- The interface under the flesh is a grid of 12 x 28 cells (5.2 x 5.0 mm). A cell separates when the knife's
  sharpened edge (the line 10 mm past the handle to the tip, on the blade's edge side) crosses it, interpolated in
  1 mm steps in the flesh's own frame, at a height from 1 mm below to 6 mm above the interface, and in that step:
  the cutting resistance above was acting on the knife; the knife is held by its handle (pads on opposite faces);
  the edge leads (over the last
  0.5 s the edge moved at least 1 mm, within 45 degrees of the blade's plane, not spine first); the blade presses
  on the skin with at least 0.5 N (PhysX contact force between knife and skin, at some step in the last 0.25 s); and the
  other gripper touches the skin tab (within 2 mm), pinning the skin. This is a rule, not a measured tension.
- A link's bond is released when all 48 cells over it have separated. 100% means every cell, so every bond.
- Flesh left on the skin: for each separated cell, the edge height above 1 mm when it separated, times the cell
  area, at 1.05 g/cm3, as a share of the 164 g flesh.
- The end gate uses face-sampled gripper hulls for the 30 mm clearance, "resting" (under 0.02 m/s and 0.5 rad/s,
  lowest point within 3 mm of the counter, board, skin or plate surface) and one anchor pose per object for the
  1 mm, 1 s stillness. `score()` and `success()` are the same function, so a trial that becomes complete in its
  last second scores 1.0.
- History (every whole-trial zero) is kept for the trial and reset only by the simulator's reset.

## Shortcuts and what they score

| Shortcut | Score | Why |
|---|---|---|
| Lift the whole portion onto the plate | 0 | Bonds hold; 0% separated |
| Tear the skin off by pulling the tab | 0 | Pulling never releases a bond |
| Cut without pinning the tab | 0 | No cell separates (rule) |
| Slice down through the flesh, or lift the knife up through it | 0 | Edge more than 6 mm into the flesh |
| Hold the blade flat but high (3 mm or more into the flesh) | 0 | More than 10% of the flesh left on the skin |
| Saw with the edge steeply down on the skin | 0 | Skin cut through (over 25 degrees, loaded) |
| Push the knife spine first, or press it straight down | No credit for that motion | Edge not leading |
| Leave the knife in the gripper, or hold the flesh over the plate | 0 | End gate: released, clear, still 1 s |
| Skin both off, but flesh left on the board | 0.75 | Flesh not on the plate |

`near_misses.py` checks the ladder boundaries (24.99% / 25% / 49.99% / 50% / 99.9% / 100%), the separation band
edges (-0.9 / -1.1 / 5.9 mm), a 4 mm-high pass (more than 10% left), a 7 mm cut into the flesh, spine-first and
flat motion, the hold, and the start state (`python near_misses.py`).

## One hand is not enough

Without the skin pinned, the skin bunches in front of a real blade or the fillet moves with it. The simulation has
both a rule (no cell separates unless the other gripper is on the tab) and the physics: the cutting resistance pulls
the skin and portion along with the knife, so a weak hold fails. In a development run with a lighter press on the
tab, the tab slid out from under the gripper within 1.5 s and the portion was dragged 54 mm across the board; the
cut stopped at 1%. Both grippers are needed: one pushing the knife, one holding the tab down against the pull.

## How this differs from close tasks

- Peeling a cucumber (an accepted task) shaves a thin layer off an exposed surface; here the blade has to stay between two
  attached layers for the whole length and both have to come out whole.
- Cutting a taut cord with a knife, sawing a slice off a loaf and slitting a taped carton seam (accepted cutting
  tasks) cut through or along a fixed line; none keeps a blade on a boundary between two layers while the other
  hand pins one of them.
- Scaling a fish removes scales and keeps the skin; pulling pin bones removes small parts with tweezers. Neither
  separates the skin from the flesh. Opening a salmon package for thawing is packaging work, not knife work.

## Running a trial

    # two trials (lots A and B), each up to 1,200 steps (60 s at 20 Hz)
    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=tasks/skin-the-salmon-fillet/task.py -E object_jitter=0 -E control_hz=20 \
      --instruction "Skin the salmon fillet with the knife, put the flesh on the plate, and put the knife down." \
      --scorer task_score --max-steps 1200 --epochs 2

The scripted demo is `--policy yam_scripted` with the same arguments. It runs open loop from the published layout:
it reads only the lot's published knife position, never an object pose, force, bond or cell state. Runtime: about
45 s of simulated time per trial for the demo. Simulator: yam-isaac-lab-arena (the program's YAM embodiment for
Isaac Lab-Arena).

## Results

- **Scripted demo (`yam_scripted`, two trials, cutting resistance on): 1.0** (lot A 1.0, lot B 1.0). In both trials
  the whole interface separated (100% of cells) against a feed resistance of 2.4 N on average (2.65 N at most)
  over the 140 mm stroke; the gripper on the tab held it with up to 6.4 N of contact force and the portion moved
  less than 3 mm; flesh left on the skin was 0.6% (A) and 2.8% (B) of the flesh mass; the highest blade load on the
  skin was 5.3 N; the flesh came to rest on the plate. Each trial reached full score at about 40 s (about 800
  steps).
- **Dream-1 (two trials, 1,200 steps each): 0.0** (lot A 0.0, lot B 0.0). Both trials reached the step limit.
  The arms moved but stayed at least 11 cm above the counter; Dream-1 never touched the knife, the skin tab or the
  salmon. It did not refuse.

## Fidelity and estimates

- True size throughout. Board size, knife length, blade length and knife mass are the makers' figures; the board
  thickness, blade width and thickness and handle section are estimates from product photos.
- The portion's size and mass are typical of a supermarket 5-6 oz centre portion; flesh density 1.05 g/cm3 and skin
  thickness 1.2 mm are typical values, not measurements of one fish.
- The flesh is rigid, so it does not sag where the skin has come away. The skin bends only across the fillet (along
  the hinge axes). Friction: blade 0.30, knife handle 0.9, skin 0.6 (scale side, on the board), skin tab 0.9 (its
  flesh side, which the gripper presses: tacky raw membrane), board 0.7, flesh 0.6 (PhysX averages each pair),
  estimates for wet steel, a textured handle, wet skin and polypropylene, not measurements.
- The separation band (1 mm below to 6 mm above the skin), the 1 mm clean-edge allowance, the cutting resistance
  (40 N/m feed, 15 N/m along the blade), the 0.5 N minimum and 40 N maximum blade load, and the
  25 degree edge-down limit are stated estimates for a sharp filleting knife on chilled salmon, not measured
  values. A real trial judges the outcome (separated length, flesh weighed off the skin), not these numbers.
- The knife's centre of mass is modelled inside the handle (uniform density), so in the simulation it lies level on
  its handle with the blade 11 mm above the counter. A real knife balances nearer the blade and tips onto its tip;
  in a real trial lay it with the blade tip on a 10 mm spacer (a small plastic block) so it starts level.
- Edge heights up to 1 mm above the interface count as a clean cut (the grey fat layer that stays on salmon skin);
  higher edge heights count as pink flesh left on the skin.

## Real trial safety

Fit cut-resistant covers over both grippers' pads, with disposable food-safe covers over them changed every
trial, and run the knife arm with its speed limited; the 10 mm edge clearance and the blade-on-gripper rule above
end the trial before a cover is reached. Use a fresh portion per trial and keep raw fish chilled; wash the board,
plate and knife between trials.

## Licences

All geometry is built from boxes and cylinders in `task.py`; no third-party assets.
