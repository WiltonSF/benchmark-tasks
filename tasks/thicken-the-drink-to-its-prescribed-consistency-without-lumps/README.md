# Whisk a premeasured scoop of thickener into a resident's 200 mL drink without lumps

Nursing assistants (O*NET 31-1131.00, about 1.3 million US workers: "Feed patients or assist patients to eat or
drink") and home health and personal care aides (31-1122.00: "Prepare and serve meals ... according to the
patient's diet") make up thickened drinks for people with dysphagia (swallowing difficulty) to the level on the
care plan, using the IDDSI framework (International Dysphagia Diet Standardisation Initiative). A thin or patchy
drink, or one with lumps, is a choking and aspiration risk. The label of Thick-It Original (a modified food starch
thickener sold in the US) says to "slowly add level measured thickener to liquid, stirring with fork or whisk as
you pour". Tipped in at once, or poured onto still juice, the powder wets only on its outside and traps dry lumps.

The robot has a premeasured dose of thickener in a flour scoop, a 200 mL drink in a tumbler and a whisk in a
utensil cup. Two hands are needed: one tips the scoop slowly over the drink while the other keeps the whisk moving
through it, and the whisk keeps going until the drink is even.

## Run

    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=tasks/thicken-the-drink-to-its-prescribed-consistency-without-lumps/task.py -E object_jitter=0 \
      -E control_hz=20 \
      --instruction "Thicken the drink with the thickener in the scoop so it has no lumps, then put the whisk back in its cup and the scoop down on the mat." \
      --scorer task_score --max-steps 2400 --epochs 3 --store-frames

Use `--policy dream` for Dream-1. `--epochs 3` runs lots A, B and C (trial k uses lot k mod 3) and `task_score` is
their mean; `TASK_LOT=A|B|C` in the environment pins every trial to one lot, and `TASK_DEBUG=1` prints the powder's
state every half second. Submissions run with `-E object_jitter=0`. A trial runs at most 2,400 control steps
(120 s); the demo reaches full score after about 95 s in each lot. The first control step of a run also samples the gripper's collision
hulls for the hands-clear check, which takes about 40 s of wall time once.

Files: `task.py` (scene, powder model, scoring, demo), `parts.py` (multi-shape parts, visual-only meshes), `lots.py`
(layout lots), `tilt.py` (tilted grips for the scripted demo only), `near_misses.py` (offline checks of the scorer
and the powder model). All are plain Python on the official yam-isaac-lab-arena (the program's YAM embodiment for
Isaac Lab-Arena) stack; nothing on disk is patched. `parts.py`, `lots.py` and `tilt.py` patch the embodiment and the
demo's grip rotation in the task's own process only.

## Objects (true size)

| Object | Size | Mass | Notes |
|---|---|---|---|
| Placemat, navy | 45 x 60 cm, 2 mm | fixed | centred 30 cm forward on the centre line |
| Coaster, cork | 9 cm across, 4 mm | 6 g | the tumbler's mark |
| Tumbler, clear polycarbonate, 300 mL | 8.6 cm across (8.0 inside), 6.5 cm tall, 5 mm base | 35 g + 200 g juice | 200 mL apple juice, 4.0 cm deep |
| Utensil cup, white ceramic | 5.2 cm across, 3.6 cm bore, 6.0 cm tall, 8 mm base | 200 g | holds the whisk |
| Mini whisk | 16 cm: steel wire head 6 cm long, 2.6 cm across at most; red handle 1.6 cm across, 10 cm long | 30 g | stands head down in the cup |
| Flour scoop, red polypropylene | flat floor 8 x 6 cm, open front, back wall 4.5 cm, side walls 4.5 cm at the back falling to 2.0 cm at the front, 2 mm walls; hollow handle (an inverted U, 1.4 x 1.4 cm) 10 cm long, its top level with the back wall's top | 30 g | lies flat, open front toward the robot |
| Thickener, Thick-It Original (starch) | 30 mL = 2 level tablespoons (lot A), 45 mL = 3 (lots B, C) | | in the scoop against its back wall |

Doses from the product's chart for apple juice (same as water) (per 4 fl oz: mildly thick 3 1/2 to 4 teaspoons, moderately thick about
4 1/2 to 5 1/2), scaled to 200 mL (6.8 fl oz): 2 tablespoons for IDDSI level 2 (mildly thick), 3 for level 3
(moderately thick). A care plan or the tin in use may differ; the real trial checks the result with the IDDSI flow
test, not the dose.

## Lots

Positions are of centres, in cm forward from the line between the arm bases (x) and to the left of their midpoint
(y).

| Lot | Tumbler and coaster | Utensil cup and whisk | Scoop's front edge (middle) | Dose |
|---|---|---|---|---|
| A (screenshot) | x 22, y -3 | x 30, y -18 | x 27, y +10 | 30 mL (level 2) |
| B | x 22, y +3 | x 30, y +18 | x 27, y -10 | 45 mL (level 3) |
| C | x 19, y +4 | x 30, y -18 | x 27, y +10 | 45 mL (level 3) |

Lot B is lot A's mirror image, so the hands swap jobs. Lot C moves the drink 7.6 cm from lot A's place: a replay of
lot A's motions pours the thickener onto the mat and drives the whisk into the tumbler's rim. Lots stop that fixed
habit, not a memorised replay of all three layouts. The demo reads only where the tumbler, the utensil cup and the
scoop are (visible); it never reads the dose or the lot.

## The powder model (what is simulated)

The simulator is rigid, so the thickener and the juice are tracked volumes moved by `step_powder()` each control
step (20 Hz). Every rule:

1. **The scoop.** Its inside is cut into 2.5 mm cubes. It keeps the powder that lies below a plane through the
   lowest point of its rim (the open front's edge, in any normal pour), rising at the powder's angle of repose
   (40 degrees) toward the scoop's inside. Anything above that plane runs out over that point at up to 200 mL/s, so
   a scoop tipped over empties in a fraction of a second. The scoop holds 112 mL level and nothing once its floor is
   tipped 40 degrees; tipping it slowly from about 25 to 40 degrees pours about 3 mL per degree.
2. **Where it lands.** Powder falls plumb from that rim point. It lands on the drink if that point is above the
   juice and at least 3 mm inside the tumbler's inner wall, with the tumbler leaning under 20 degrees; otherwise it
   is spilled (counted, and drawn as cream spots on the mat on a 4 cm grid).
3. **The drink** is 7 regions of equal area (a centre disc and 6 sectors), each 28.6 mL of juice holding dry
   powder (floating, not yet wetted) and wetted powder (dispersed and taking up water).
4. **Stirring** is measured from the whisk's motion, not named: 30 points on its head (6 heights, 5 points across
   each) are tracked relative to the tumbler over a sliding 0.25 s window. A point's speed is the distance it
   travelled across the whisk's axis in that window divided by 0.25 s, at most 0.2 m/s, and it counts only if it got
   at least 3 mm from where it was at the window's start (a tremor or jiggle is not a stroke). Each point under the
   juice sweeps (its share of the head's width) x (1 cm) x (that speed) per second, shared equally among the regions
   whose centres are within 3 cm of it. A region's **local stirring** g is the swept volume it gets per second
   divided by its volume. The **swirl** follows the whole head's swept volume / 200 mL with a 1.5 s lag.
5. **Wetting** is done by the whisk itself: each step, dry powder in a region is wetted at the rate 40 x g (per
   second), but at most 5 mL/s in any one region, however hard it is stirred. The swirl left behind after the whisk
   stops wets nothing.
6. **Lumps.** Dry powder above 1.5 mL in one region (a layer over 2 mm deep on 7.2 square cm) becomes lumps at once.
   Dry powder in a region whose local stirring stays under 0.02 per second for 3 s also becomes lumps. Lumps never
   break up or dissolve (stricter than real starch, where hard whisking breaks some up). Together with the 5 mL/s
   cap, powder landing faster than one region can take it up lumps whatever the whisk does.
7. **Evening out.** Wetted powder moves between regions toward their mean at the rate 2 x (g + swirl) per second,
   so a drink stirred only briefly stays patchy.
8. **Juice.** The juice is part of the tumbler's mass. It spills if the tumbler ever leans more than 25 degrees (a
   level surface 2.0 cm below a rim 4.0 cm out reaches the rim at 26.8 degrees). Splashing is not modelled; whisk
   speeds above 0.2 m/s earn no more stirring.

The rates and thresholds (repose angle, 200 mL/s, 0.25 s and 3 mm, 0.2 m/s, 3 cm, 40, 5 mL/s, 1.5 mL, 0.02 and 3 s,
2, 1.5 s) are chosen to give the behaviour described, not measured. The powder's weight is not applied to the scoop
(at most 23 g).

Drawing: the powder in the scoop as a cream slab (9 sizes), a falling stream where it lands, each region of the
juice in 4 shades from clear amber to cloudy by how much wetted powder it holds against an even share, a cream
patch for floating dry powder, white beads for lumps, and cream spots on the mat for spilled powder.

## Scoring (exact)

Per trial, with V the dose (mL):

- 0 for the trial once lumps reach 0.05 mL (about one 4.6 mm lump), once the juice spills, or once 5% of V or more
  has been spilled, at any step; and 0 while the hands' state is unknown: if the simulator cannot report the finger
  tips and grippers, or the check fails, that step records the state as unknown and the trial scores 0 until it is
  known again (fails closed).
- **Wetted share** W = wetted powder in the drink / V. **Even**: every region's wetted powder within 10% of the
  regions' mean, and less than 1% of V still dry and floating.
- 1.0 when W >= 0.95, the drink is even, and for 20 control steps (1 s) in a row: the tumbler leans at most 5
  degrees, its centre within 15 mm (horizontally) and 3 mm (vertically) of its start; the whisk's tip is inside the
  utensil cup's bore (within its 18 mm radius of the cup's axis, below its rim); the scoop leans at most 10 degrees
  with its underside within 3 mm of the mat; all three move under 0.01 m/s and 0.5 rad/s; and no point on the faces
  of either arm's finger tips or gripper body (their collision hulls, sampled every 3 mm) is within 5 mm of any shape
  of the tumbler, the whisk or the scoop. Each comparison allows 1e-6.
- 0.75 when W >= 0.95 and the drink is even, without that end state.
- 0.5 when W >= 0.95 but the drink is not even.
- 0.25 when W >= 0.50.
- 0 otherwise.

Where the arms end up is never scored. The powder's history is reset by the simulator's reset hook at the start of
each trial, never by anything the robot does.

**Real trials.** Weigh the scoop before and after and sweep spilled powder off the mat onto tared paper (spilled
5% or more scores 0). Draw a spoon along the bottom and round the wall, then pour the drink through a 2 mm kitchen
sieve: any lump of 4 mm or more scores 0. After 1 minute standing (the label's wait for water and juice), run the
IDDSI flow test (a 10 mL slip-tip syringe filled to 10 mL, flowing for 10 s) on a sample from the middle and one from
the wall: level 2 leaves 4 to 8 mL, level 3 more than 8 mL. Both in the band: even (0.75 tier and above). One in
the band, or both below it but thicker than the plain juice (over 1 mL left): 0.5. A juice spill, the tumbler
tipped past 25 degrees on the video, scores 0.

## Demo

The hand on the utensil cup's side lifts the whisk out by its handle, leans it 20 degrees away from the scoop's side
and stirs in an ellipse 2.4 cm across and 0.8 cm deep, once a second, near the bottom of the juice and a little
toward where the thickener will land. The other hand pinches the scoop's handle 5 cm behind the bowl, lifts it,
carries it over the tumbler already tipped 18 degrees, and tips it on to 36 degrees over 34 s and to 42 degrees over
a further 24 s (the last layer on the floor slides off near 39 degrees), with its front edge held 2.8 cm from the
tumbler's centre and 1 cm above the rim, so the thickener runs in at about 1 to 2 mL/s. The whisk
goes on for about 20 s after the scoop is empty. The scoop goes back down on the mat where it started and the whisk
back in its cup, and both hands let go. The demo uses `tilt.py` for the leaning grips.

Offline (`near_misses.py`, the objects' poses only, as if held rigidly), the demo scores 1.0 in every lot, and
still does with 40% of its stirring amplitude or the pour twice as fast; with 30% of the amplitude, or the pour
three times as fast, lumps form and it scores 0. These also score 0: the scoop jumped to 42 degrees in one step, or
tipped from 18 to 42 degrees in 1 s, under a 1 cm, 2 Hz whisk; whisking hard, holding the whisk still through such
a dump and restarting it 3 s later; pouring at the demo's pace while the whisk only trembles 1 mm either way each
step; pouring with no stirring; stirring only after the pour; and lot A's motions in lot C's layout.

## Known limits

- The powder and juice are a model with stated rules, not a fluid or granular simulation. Thickening over time
  (viscosity rising) is not modelled; the score reads how much powder was wetted and how evenly it is spread.
- Only the whisk stirs; the scoop or a finger dipped in the juice does not.
- The tumbler and utensil cup are 20- and 16-sided; the whisk's head is a solid convex body, not wires.

## Closest tasks

- TK-AUSTZF (a lab polymer stock sprinkled into a stirred beaker) poses the same physical problem in a specialist
  setting and was closed for importance and scoring, not as a duplicate. This task is a core duty of a large care
  occupation, gives partial credit for each real step (half the dose wetted, all of it, an even drink, the tools
  put down), never scores where the arms end up, and its real-trial checks (scoop and spill weights, a spoon and
  sieve for lumps, the IDDSI flow test) measure the same things the simulator scores.
- TK-6RMQTA (working flour into a dough with a spoon while steadying the bowl) works a whole batch already in the
  bowl; nothing is added at a rate, and there is no lump rule.
- BEHAVIOR-1K's `make_instant_coffee`, `make_hot_cocoa`, `making_a_drink` and `mixing_drinks` put a powder or
  drinks into a cup and stir or combine them; none has a rule for the rate the powder goes in, lumps, or how even
  the result is, so tipping the powder in at once succeeds there and fails here.


Source for the label wording and doses: Thick-It Original guide, https://www.vitalitymedical.com/pub/pdf/guide-thick-it-original.pdf
