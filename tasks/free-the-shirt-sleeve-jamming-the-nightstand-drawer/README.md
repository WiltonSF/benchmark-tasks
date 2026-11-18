# Open a clothes drawer jammed by a T-shirt sleeve caught behind its rail

A low nightstand's drawer, full of folded clothes, has stuck about 10 cm open. The sleeve of the T-shirt on top has
bunched up and gone in behind the rail above the drawer, so the drawer catches after about 1.5 cm more of pulling.
Pulling harder strains and tears the sleeve. The drawer opens fully only once the bunched sleeve is pressed back down
into the drawer through the gap, below the rail. The robot is told only to open the drawer all the way without
tearing anything.

**Credit.** This task builds on the idea of PR 579, "Bimanual recovery from a clothing-obstructed drawer"
(TK-YBZC8L), which was closed as final. Its review raised three points, each addressed here:

| Review point | What this version does |
|---|---|
| The garment was "a rigid spring-mounted proxy constrained to vertical motion". | The sleeve is a hinged strip of five segments that bends at every fold line and keeps the bend it is pushed into. The robot really has to move the sleeve's segments below the rail. The snag on the rail is a stated stand-in rule (below), allowed under the board's rule on material stand-ins, and the same geometry drives the simulator and the score. |
| The instruction gave the grasp, diagnosis, compression and arm roles. | The instruction is the goal and one constraint: "Open the drawer all the way without tearing anything." |
| The only full-score demo used privileged simulator state. | `demo()` is fixed waypoints from the published layout. It reads no state. |

There was also no importance line. The context now names the everyday situation: clothes drawers at home sticking
part-open on a caught sleeve. Other changes from the original: the setting is a real nightstand at true size; the
tear is scored (a strain limit judged over the whole trial); there is a "hands clear" release check; and there is a
0.5 tier for freeing the sleeve intact without opening the drawer. All of the code here is new.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. Other files:
- `parts.py` (in this folder) builds the nightstand as one articulation: a fixed carcass, the drawer on a prismatic
  slide, and the five sleeve segments chained by hinges from the drawer. It also runs the per-step callbacks.
- `near_misses.py` scores near-successes and wrong outcomes without a simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Open the drawer all the way without tearing anything."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/sleeve-dream

# Scripted demo (reaches 1.0 at about 21 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/sleeve-demo
```

- **Runtime per trial:** at most 40 s of simulated time (800 steps at 20 Hz). On an L40S that is about a minute of
  wall time once Isaac Sim has started.
- **Seeds:** none. There is one fixed layout, with no loose objects. Run with `-E object_jitter=0`.
- `TASK_DEMO=naive` in the environment swaps the demo for a straight pull on the handle. It is for traces only (see
  below).

## Scoring in the simulator

`score()` and `success()` in `task.py` read a record that an `on_step` callback updates every control step.

- **Opening:** 0.10 m plus the drawer slide's position. "Fully open" means at least 0.22 m. The slides stop at
  0.25 m.
- **Sleeve points:** a 5 x 3 x 2 grid of points over each segment's box.
- **Caught:** any sleeve point is at least 0.185 m above the table (within 5 mm of the rail's underside at 0.19 m)
  and at or behind the carcass's front face (x 0.38 m). Freed means no point is caught.
- **Snag strain:** let *g* be the distance from the foremost caught point to the rail's back face (x 0.40 m). The
  strain is 4,000 N/m x max(0, 10 mm - *g*). Once the strain passes 20 N at any step, the sleeve is torn and the
  trial scores 0 from then on.
- **Inside the drawer:** every sleeve point lies between the front board's inner face and the box back, within the
  inner sides, and no more than 3 cm above the side tops (0.19 m).
- **Hands clear:** every sampled point on both grippers and all four fingertips is more than 5 mm from every box of
  the drawer link (front, box, clothes, handle) and of the sleeve. The hand meshes are sampled at every vertex and
  across each triangle face at about 3 mm spacing, so a flat face resting on the bar is seen even when no vertex is
  near it.
- **Still:** over the last 1 s, neither the opening nor any sleeve point moved more than 2 mm.
- **Fails closed:** if the nightstand or hand state is missing or not finite at any step, the trial scores 0.
- **Score:** 1.0 when the sleeve is freed and inside the drawer, the hands are clear, everything is still, the sleeve
  was never torn and the opening is at least 0.22 m. 0.5 when the same holds with the opening under 0.22 m. 0
  otherwise.

### The stand-in rules (`_apply_fabric` in `task.py`)

- **Fold lines.** Each sleeve hinge has a 1.5 N m/rad spring. When a hinge is bent more than 0.12 rad from its
  spring's rest angle, the rest angle follows it, so the hinge stays within 0.12 rad of where it was pushed. This
  stands in for cloth holding the shape it is pressed into. The segments weigh 12 g each, more than jersey, to keep
  the solver stable.
- **Snag.** While a sleeve point is caught and *g* < 10 mm, the drawer slide gets a 4,000 N/m spring whose rest
  point is the opening at which *g* = 10 mm. Otherwise its stiffness is 0. The spring therefore carries exactly the
  strain the scorer computes. It is one-sided: it never resists closing. The rail is also a real collider.
- **Slide friction.** The drawer slide has 20 N s/m damping and about 4 N of static friction (a loaded drawer).

## Scripted demo

`demo()` uses fixed waypoints only.
1. The left hand, fingers shut and pointing down, comes in over the 8.2 cm gap between the drawer front and the
   nightstand. Its 6.8 cm finger pair is centred in the gap.
2. It presses the bunched sleeve flat onto the clothes, which swings the part behind the rail down with it.
3. It lifts and leaves the gap sideways. Leaving toward the robot would drag the drawer front with it.
4. The right hand, turned 90 degrees, pinches the bar handle across its 12 mm thickness and pulls the drawer out,
   aiming at 24 cm.
5. It lets go, rises off the bar, and leaves sideways.

## What was traced on the GPU (L40S, trace mode)

- **Straight pull (`TASK_DEMO=naive`):** the right hand pinched the handle and pulled out to 24 cm, then held. The
  drawer moved 1.5 cm and caught at 12.2 cm open. The snag strain passed 20 N at about 7 s (torn) and levelled at
  about 35 N, the most the arm pulled. Score 0.
- **The demo:**
  - Before the press, the highest point of the sleeve stood at 0.203 m, 1.3 cm above the rail's underside.
  - The press brought it to 0.168 m, and the sleeve stayed there to the end.
  - The drawer was then pulled to 22.9 cm open, the hands cleared, and the trial ended at 1.0.
  - The snag strain stayed at 0 N throughout.
- **Dream-1:** 0.0 in 1 completed trial (second trial did not run: API access ended). In the completed trial the
  arms moved toward the nightstand but did not touch the drawer or the sleeve, and the trial ran to the 800-step
  limit.
- **An earlier version of the demo:** that version left the gap toward the robot, which dragged the drawer front 7 cm
  open. The other hand then landed on the moved handle and pushed the drawer back in. Both hands now leave sideways.
  That trace also showed that sleeve springs of 0.4 N m/rad let the sleeve sag free at reset, so they are now
  1.5 N m/rad.

## Limits a reviewer should know

- The sleeve bends in one plane only (hinges across it). It cannot twist, stretch or crumple sideways.
- The tear is judged only from the snag strain. A hand pulling or pinching the sleeve directly is not measured for
  tearing.
- The folded clothes under the shirt, and the shirt's folded body, are one rigid block fixed in the drawer. Only the
  sleeve moves.
- There is one fixed layout, so a memorised replay of the demo's waypoints would pass.
- The nightstand is a low (25 cm) model. On a taller one, the YAM's forearm meets the carcass above the drawer while
  the hand reaches into the gap.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable
```
