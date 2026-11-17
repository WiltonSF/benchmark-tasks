# Empty a bedpan into a measuring graduate without spilling

Nursing assistants empty bedpans into a graduate to measure and record urine output (O*NET 31-1131.00 core tasks
"Supply, collect, or empty bedpans" and "Measure and record food and liquid intake or urinary and fecal output"; the
NNAAP nurse aide skill "Measures and records urinary output": pour into the graduate without spilling or
splashing, with the graduate on a flat surface). A bedpan is wide, shallow and has no spout. Tipped over one of its
straight sides, the urine leaves as a sheet as wide as that side (28 or 36 cm), much wider than the graduate's
11.7 cm mouth. Tipped quickly, the urine sloshes over the rim. Tipped slowly so that a rounded corner is the lowest
point of the rim, held over the mouth, the stream is narrow and lands inside.

## Run

    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=tasks/empty-the-bedpan-into-the-graduate/task.py -E object_jitter=0 -E control_hz=20 \
      --instruction "Empty the bedpan into the measuring graduate without spilling, then leave the graduate standing on the table and the bedpan on its pad." \
      --scorer task_score --max-steps 1800 --epochs 3 --store-frames

Use `--policy dream` for Dream-1. `--epochs 3` runs lots A, B and C (trial k uses lot k mod 3) and `task_score` is
their mean; `TASK_LOT=A|B|C` in the environment pins every trial to one lot. Submissions run with
`-E object_jitter=0`. A trial runs at most 1,800 control steps (90 s); the demo takes about 64.5 s per trial (1,290 frames).

Files: `task.py` (scene, liquid model, scoring, demo), `parts.py` (multi-shape parts, visual-only meshes, a constant
wrench on a loose part), `lots.py` (layout lots), `tilt.py` (tilted grips for the scripted demo only),
`near_misses.py` (offline checks of the scorer and the liquid model). All are plain Python on the official
yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena) stack; nothing on disk is patched. `parts.py`,
`lots.py` and `tilt.py` patch the embodiment and the demo's grip rotation in the task's own process only.

## Objects (sizes from the products)

| Object | Size | Mass | Source |
|---|---|---|---|
| Bedpan, polypropylene | 35.6 x 27.9 x 10.2 cm (14 x 11 x 4 in) | 145 g (0.32 lb) | Medline stackable bedpan listing |
| Graduate, stainless 1 quart measure | 13.1 cm tall (5.15 in), 11.7 cm usable top (4.59 in), 9.9 cm base (3.90 in) | 310 g (0.68 lb) | Vollrath 95320 listings |
| Underpad | 39 x 50 x 0.2 cm | fixed | |

The pan's shape is modelled from boxes and one convex floor: 6 mm walls leaning out 30 degrees (55 mm over their
96 mm height) from a 16.9 x 24.6 cm floor, corners rounded 6.5 cm (outside) as 4 flat facets each, and a solid
handle block at one end, and a 3 mm seat plate on the rim over the handle half. The liquid's cavity is the whole
shell up to the rim (about 5.9 L), and the seat covers the handle half of the rim, so liquid can leave only over the
open half; the real pan is listed at 1.6 L. The graduate is
18 wall panels leaning out 3.8 degrees, a floor, a handle and four dark marks (250 to 1000 mL).

## Lots

| Lot | Handle | Urine | Pouring corner |
|---|---|---|---|
| A (screenshot) | robot's left | 250 mL | front right |
| B | robot's right | 150 mL | front left |
| C | robot's right | 400 mL | front left |

The handle block sits where a grip on that end wall would go, so a replay of lot A's grips fails in lots B and C. Lots
stop that fixed habit, not a memorised replay of all three layouts. The demo reads only which end the handle is at
(visible), never the volume.

## The liquid model (what is simulated)

The urine is not a fluid; it is a tracked volume read and moved by `step_liquid()` each control step (ten
substeps):

1. **Surface.** A flat free surface inside the pan. Outflow is only over the open half of the rim; the seat closes
   the handle half. The inside is cut into 4 mm columns; each column's bottom is the
   floor or the flared wall. The surface's height is found so the columns hold the tracked volume.
2. **Slosh.** The surface's tilt relative to the pan is a damped oscillator (damping ratio 0.15) that settles to
   level under gravity plus the pan's own acceleration (filtered over 0.1 s, at most 5 m/s²), at the shallow-water
   frequency of the first sloshing mode for the liquid's current footprint (ω = π √(g h) / L). Tipping fast
   overshoots and slops over the rim; tipping at the demo's pace does not.
3. **Outflow.** Every rim point the surface stands above loses liquid at the sharp-crested weir rate
   Q = 0.62 · (2/3) · √(2g) · h^1.5 per metre of rim (h = head above the rim), limited to what stands above the rim.
4. **Where it lands.** Each outflow point's stream leaves the rim horizontally outward at (2/3) √(2 g h) and falls
   freely to the plane of the graduate's mouth. It is caught if it lands at least 3 mm inside the mouth
   (5.55 cm inner radius) while the graduate leans under 20 degrees, otherwise it is spilled. A pan on its side or
   upside down empties at once over its whole rim.
5. **The graduate** keeps what fits below its lowest rim point; if it leans, the excess is spilled.
6. **Weight.** The remaining urine's weight (1 g/mL), at its centroid, is applied to the pan as a force and torque.

Drawing: yellow layers in the pan (10 levels, drawn in the pan's own frame, so they tilt with it rather than staying
level) and graduate (20 mL steps), a stream over the mouth while liquid is
landing in it, and puddles on a 5 cm grid where spilled liquid landed.

## Scoring (exact)

Per trial, from the tracked volumes V0 (start), V_grad (in the graduate) and V_spill (spilled, cumulative):

- 0 once V_spill >= 0.02 · V0 at any step, and 0 while the hands' state is unknown (fails closed).
- 1.0 when V_grad >= 0.95 · V0, V_spill < 0.02 · V0, and for 20 control steps (1 s) in a row: the graduate leans at
  most 5 degrees with its underside within 3 mm of the table; the pan leans at most 5 degrees, its underside within
  3 mm of the pad's top, its centre within 40 mm of its start and its yaw within 15 degrees of its start or turned
  round 180 degrees; both under 0.01 m/s and 0.5 rad/s; and no point of either arm's finger tips or gripper body is
  within 5 mm of any shape of the pan or the graduate. Their collision surfaces are sampled at every vertex and across
  every face about every 3 mm, so a flat face resting on a part counts as touching it. The hands are measured only
  while the rest of the finished state holds; when unmeasured or unknown they count as not clear. Each comparison
  allows 1e-6.
- 0.5 when V_grad >= 0.95 · V0 and V_spill < 0.02 · V0 without that end state.
- 0.25 when V_grad >= 0.50 · V0 and V_spill < 0.02 · V0.
- 0 otherwise.

The liquid history is reset by the simulator's reset hook at the start of each trial, never by anything the robot
does. In a real trial, weigh the graduate after the trial (1 g = 1 mL) and the pan; spill = start minus both.

## Demo

Both arms pinch the pan's walls near the front corner at the open end, one on the front wall and one on the end wall,
12.5 cm from that corner. Each grip leans 30 degrees with the wall so the pads lie flat on its top 30 mm (the demo
uses `tilt.py` for these leaning grips). The arms lift the pan 3.5 cm, carry and turn it 45 degrees so that corner
points toward the graduate, tip it to 24 degrees behind the graduate, bring the corner over the mouth, and tip on to
60 degrees about the corner over about 16 s, lowering it, then hold for 5 s. They tip it back, carry it back, set it
down 6 mm above its pad and let go, back their hands up the walls and lift them clear (the front-wall hand also
moves off to its own side, away from the graduate). The grip points stay at most 0.20 m above the table. Offline (`near_misses.py`,
the pan's poses only), the demo's pour puts all 250 mL in the graduate; tipped slowly over the long side 145 mL
spill, over the open end 102 mL, and the corner pour tipped in 3 s spills 32 mL.

## Known limits

- The liquid is a model with stated rules, not a fluid simulation; there is no splash, no dribble down the outside
  of the pan and no film left on the walls.
- The rounded corners are flat facets; the seat is a plain plate without the real seat's opening or contour.
- Spilled liquid is measured, not traced: the puddle grid only shows where it landed.
