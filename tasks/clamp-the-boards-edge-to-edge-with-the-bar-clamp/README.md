# Clamp two pine boards edge to edge with a one-hand bar clamp

Two planed pine boards lie flat side by side on a workbench, 2 mm apart. A 6 in one-hand bar clamp lies open in
front of their ends. The robot has to get the clamp's jaws round the boards and close it on them until the boards
are pressed together edge to edge, then let go, with the clamp holding them on its own.

On a one-hand clamp the clamping force comes from the trigger. The sliding jaw can be pushed up to the work by
hand, but that gives little force. Each squeeze of the trigger against the grip drives the bar about 3 mm
through the handle, and a brake holds the bar while the trigger springs back for the next stroke. So the robot has
to hold the grip and squeeze the trigger, stroke after stroke, with one hand.

**Category: Societal relatability.** Most people know this from home DIY: gluing a split shelf, a loose chair rail
or a small panel, or a school woodwork project. The one-hand bar clamp is one of the most common clamps sold for
home use. The task makes no claim about any occupation.

**Credit.** This task takes up the idea of PR 771 on this repository (TK-U8LDP5, last revised as TK-DZW6AB), whose 48-hour revision hold
lapsed. The scene, code, wording and scoring here are new. Every review point raised on that pull request is
answered below.

## Review points on the original, and how this version answers them

1. *"Require a specified, practical minimum clamping force rather than treating gaps under 1 mm as gripping; explain
   how to verify it in a real trial, update the simulated scoring, and provide a full-score demo under the revised
   criterion."* Clamped means at least **60 N** between the jaws, with both pads on the boards' outer edges and the
   joint closed. A gap alone never counts. A pad that only touches reads 0 N.
   - **In the simulation:** the fixed jaw's pad is a 25 N/mm spring, and the force is its compression times that
     rate.
   - **In a real trial:** put a thin load cell or force-sensing pad (0 to 200 N, 3 mm or thinner) between the fixed
     pad and the board. It must read 60 N or more once both hands are clear. A 0.5 mm feeler gauge checks the joint.
   - The scripted demo reaches 1.0 under this rule (scores below).
   - **Why 60 N:** it is a firm hand clamp-up for this size of clamp, well short of its rated ~620 N, and three
     times the 20 N the second review asked for. Jaws pushed up to the work with no force score 0.5 (`near_misses.py`);
     60 N takes the trigger, or a push on the sliding jaw as hard, which the brake then holds as on a real clamp. A full glue-up of wide stock is often clamped harder;
     this task asks for the firm, even closure a one-hand clamp gives on a small glue-up, not the maximum force.
2. *"Model a realistic ratchet trigger and provide a full-score demonstration that operates it to achieve at least
   20 N, rather than building force automatically when a finger is near the sliding jaw."* The trigger is a real
   mechanism, and nothing in the code builds force on its own (see Mechanism). In the mechanism, force reaches the bar
   through the trigger's pin and the drive plate (or a direct push on the sliding jaw, as on a real clamp). The bar's load comes back through the pin onto the trigger, so the finger
   has to push harder as the force rises. The demo works it stroke by stroke with one hand (see Demo).
3. *"Change the domain to Impact on jobs and explain how clamping is part of carpenters' regular work."* This was
   superseded by the third review, below.
4. *"The carpenter occupation total does not establish how many workers regularly edge-glue boards into panels...
   Please use Societal relatability for the recognizable act of clamping boards and explain its everyday
   relevance."* The category is now Societal relatability, and the context field is about the everyday home and craft act.
   No occupation count is claimed.

## Is the trigger within the YAM's strength?

The board puts the YAM's fingers at roughly 40 N each, and the stock simulator caps them at 50 N. The trigger's
lever is 4.6 to 1: a finger at the trigger's middle sits 46 mm from the pivot, and the drive pin sits 10 mm from
it. Its return spring holds the trigger open with 0.20 N m and resists with 0.275 N m when it is fully squeezed.

60 N at the jaws therefore needs about (60 x 0.010 + 0.275) / 0.046 = 19 N at the finger. 40 N would give about
150 N.

For comparison, mini one-hand clamps of this size (6 in) are rated at about 620 N (140 lbf). Through the same lever
that rating needs about 140 N of squeeze, a strong human grip. No maker publishes trigger-effort figures, so the
lever ratio and spring are estimates for this size of clamp, not measurements of a named product. The force asked
for here, 60 N, is within the arm's grip by a wide margin. The grip-to-trigger span is 52 mm open and 38 mm
squeezed, well inside the gripper's 94 mm opening.

## Mechanism

Everything is in `task.py`.

| Part | What it is |
|---|---|
| Body | The handle around the bar, the fixed jaw's arm and the 24 mm pistol grip, in one rigid body. The clamp lies flat on its side. |
| Trigger | A lever hinged in the handle, 0 to 0.30 rad. A spring holds it open; squeezed, it turns toward the grip. A 5 mm pin 10 mm from its pivot sits in the drive plate's slot (0.3 mm play each side). |
| Drive plate | Rides on the bar on its own sliding joint. It may slide back along the bar freely, but forward it stops 0.1 mm ahead of where it last was on the bar, and at that stop it pushes the bar. Each control step the stop follows the plate back. So a squeeze drives the bar about 3 mm, and the trigger springing open leaves the bar where it is. |
| Bar | A 12.7 x 5 mm steel bar on a sliding joint through the handle, with the sliding jaw and its pad on the far end. |
| Brake | The bar's opening limit follows the bar shut, 0.1 mm behind it, every control step. The bar slides shut freely but never opens. The real clamp's release lever is not modelled and is never needed. |
| Fixed pad | A black rubber pad on a stiff sliding joint (25 N/mm, 10 mm travel). Its compression is the clamping force. |

The brake and drive plate act through moving joint limits, which is how a one-way grip is built in this simulator.
Each limit only follows its joint, never drives it, so neither can add force. With no squeeze and no push on the jaw,
the bar does not move.

## Run it

The task runs in yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena) on the `station` scene. It
needs two helper files next to `task.py`:
- `parts.py`: the multi-shape parts with sliding and hinged joints (the clamp).
- `lots.py`: the two layout lots. At the start of each trial `task.py` writes every object exactly at its lot's
  pose, with the clamp open and its joints at 0.

Submissions run with `-E object_jitter=0`.

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Clamp the two boards together edge to edge with the bar clamp, then let go."

# Dream-1, both lots (trial k uses lot A for even k, lot B for odd k; task_score is the mean)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 2 --store-frames --log-dir ~/yam-eval/clamp-dream

# Scripted demo, both lots
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --epochs 2 --store-frames --log-dir ~/yam-eval/clamp-demo
```

`TASK_LOT=A` or `TASK_LOT=B` in front of a command pins every trial to one lot. A trial takes about 4 minutes of
simulation on an L40S. There are no random seeds: with `object_jitter=0` the start is fixed for each lot.

## Scoring (exact)

These terms are used in the table:
- **Pads on the edges:** each board's outer edge, where it crosses the pads' centre line (49 mm from the bar),
  within 1 mm of its pad's face.
- **Bear:** both boards' near ends between the bar's edge (6 mm) and 53 mm from the bar, so that each pad bears on
  at least 15 mm of its edge.
- **Joint closed:** the right board's inner corners, measured square to the left board's inner edge at both ends of
  the joint, 0.5 mm or less away.
- **Force:** the fixed pad's compression times 25 N/mm.
- **Let go:** every sampled point of both grippers' collision hulls (corners plus face points about 3 mm apart) more
  than 5 mm from the clamp's shapes and both boards, checked every control step.
- **Still:** no board corner, and neither the clamp's origin nor its bar direction, moves more than 2 mm during the
  hold.

| Score | When |
|---|---|
| 1.0 | Pads on the edges, bear, joint closed, force 60 N or more, near ends within 3 mm of each other, both boards flat (under 2 degrees) with their tops within 1 mm, the clamp flat (within 5 degrees). Let go and still for 1 s, the whole time in this state. |
| 0.5 | Pads on the edges, bear and joint closed, but not all of 1.0 (for example under 60 N, or ends or tops out). Let go and still for 1 s. |
| 0.25 | The clamp round the panel: both boards' outer edges between the pad faces (within 1 mm) at the pads' centre line, and bear. Let go and still for 1 s. |
| 0 | Otherwise. Also 0 for the whole trial if a board or the clamp ever drops 5 cm below the bench top or tips past 30 degrees, or if any object, clamp joint or gripper could not be read at any step. |

Every tier needs the hands clear, so nothing is credited while the clamp is held. The hold restarts whenever the
tier changes or anything moves. `score()` returns 1.0 whenever `success()` holds.

`near_misses.py` checks the following, and `factory/check.py` runs it:
- every tier and both sides of each boundary (59/60 N, 0.5/0.6 mm, 3/4 mm ends, 15/14 mm of pad, the bar's edge);
- the jaw pushed shut with no force;
- a pad off its edge;
- a twisted board, a proud board and a tilted clamp;
- the 1 s hold, a finger at 4 mm and at 4.9 mm, and full force while still held;
- a sliding panel, and force falling during the hold;
- the whole-trial fall rules and unreadable grippers;
- face sampling;
- lot B.

## Demo

`demo()` runs open-loop from the published start layout of the trial's lot (the clamp's start position only).
1. The left hand comes down over the clamp's grip and trigger, fingers either side.
2. It closes, one fingertip on the back of the grip like a palm and the other on the trigger's middle. That first
   squeeze also takes hold of the clamp.
3. Squeezing, it slides the clamp 62 mm toward the boards until the jaws are round their near ends.
4. It works the trigger 14 times. Each stroke relaxes the fingers to 4 mm clear of the grip and the sprung-open
   trigger, then squeezes, commanding the fingers about 6 mm past the fully squeezed faces (about 31 N). The hand's
   middle shifts 7 mm toward the grip as the trigger closes.
5. It lets go, lifts straight up and parks, then waits about 8 s.

The right arm stays parked. Nothing reads the trigger angle, the bar position, the force or any object pose.

## Scores

- **Scripted demo, both lots:** task_score 1.0, 1.0 in lot A and 1.0 in lot B. In a lot A trace of the same demo,
  the jaws closed on the boards on the 7th squeeze and the force rose over the next three, reaching 85 to 89 N.
  It held 87 N after the hand let go, with the joint shut (0.1 mm or less) and the near ends 2.4 mm apart. Full
  score came at 31.5 s.
- **Dream-1, both lots, 1,200 steps each:** 0.0 in each. Both arms lifted and hovered over the bench. Neither
  touched the clamp or the boards.
- **The simulator is not deterministic** (PhysX `deterministic=False`). One earlier demo run of an older version
  scored 0 in lot A. That version had a shorter final wait, and it may not have finished its 1 s hold before the
  plan ended. Since then the plan waits 8 s more, and every run has scored 1.0 in both lots.

## Fidelity

- The boards and the clamp's parts are rigid. The real pads and wood are stiffer than the 25 N/mm pad spring,
  which stands in for both.
- The lever ratio and spring are estimates for this size of clamp, as above.
- There is no glue, so there is no squeeze-out and no slip on wet glue. The real trial is a dry clamp-up.
- Simulation settings, all in this folder: the boards and the clamp have a rigid-body angular damping of 1/s, and
  the loose boards solve contacts with 32 position iterations, like the articulated clamp (`parts.py`). Without
  them, the clamped, released panel crept round on the bench at about 0.6 degrees a second, which a real one does
  not. Neither setting adds any force.
- The two lots stop a single fixed replay, but not a policy that memorised both.

## Files

| File | |
|---|---|
| `task.py` | scene, mechanism, scoring, demo |
| `near_misses.py` | offline scoring checks |
| `parts.py` | multi-shape articulated parts for yam_arena task files |
| `lots.py` | per-trial layout lots |
| `task.json` | the board entry |

No third-party assets are used: every shape is a box or cylinder primitive.
