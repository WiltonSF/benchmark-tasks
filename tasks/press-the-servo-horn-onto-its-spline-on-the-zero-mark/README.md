# Press a servo horn onto its 25-tooth output spline with the arm on the zero mark

The right arm picks a white single-arm horn off a foam pad, turns it so its arm lies along a white zero mark painted
on a hobby servo's case, sets its bore over the output shaft, lets go, and presses it home onto the boss. The shaft
has 25 teeth, so the horn can only go on at 25 angles, 14.4 degrees apart. Pressed on at the angle it was picked up,
it stands on the tooth crests and does not go down. Turned while pressed, it drops in at the first tooth position it
passes, which is not the mark. Robot builders do this for every hobby-servo joint they assemble or repair: a horn one
tooth off puts the joint's zero 14.4 degrees out.

Instruction (goal only):

    Fit the white horn fully onto the servo's output shaft with its arm on the white zero mark, then let go.

## Credit

This task takes up the idea of PR 1864 (TK-3RLQU3, revised as TK-58UVQV and TK-9QFHJE), "Fit a servo horn onto its
splined output shaft with the horn arm on the zero mark". Its 48-hour revision hold after the TK-9QFHJE review lapsed
with no revision. The scene, scoring and demo here are new code; nothing was copied from that PR. The crest-on-crest
stand-off (3.5 mm) and seating force (32 N) are the values that PR reports as measured on an MG996R with its stock
horn. They were not re-measured for this task.

## Why this is not "press a snap-fit until it clicks"

A snap-fit locates itself. Push it roughly straight and its own shape guides it to the one position where it
latches, and the click tells you it is done. A spline has 25 positions that all accept the same press equally well.
Nothing guides the horn to the right one, and nothing in the press tells you whether it is right. The clocking has to
be done before contact: the arm turned to within 2.5 degrees of a tooth position (about 1.3 mm sideways at the arm
tip) and held there while the bore goes over the shaft. Once the horn is pressed on a wrong tooth, it stays wrong
until it is pulled off. The force is the second half of the problem: about 32 N straight down, close to what one YAM
arm can push (up to about 39 N, measured below), on a 13 mm hub, without tilting it more than 4 degrees.

It also differs from meshing gears (TK-YNPRML), where the teeth only engage at certain phases and the mesh itself
shows when the phase is right. Here every tooth position engages equally; only the external zero mark says which one
is correct, so the arm angle must be read and held, not found by feel.

## How each review point is handled

TK-3RLQU3:
1. *A horn seated on a wrong tooth zeroed the whole run, even if it was removed and refitted.* The score reads the
   final state. A horn pulled off a wrong tooth and refitted on the mark scores 1.0 (near-miss case "then refitted
   on the mark (no damage)"). Only real damage scores 0 for the whole trial: a horn twisted more than 4 degrees off
   its tooth position while on the teeth (stripped teeth), or forced more than 2.3 mm off the axis while on them.
2. *Select and check a specific servo and horn against the stand-off and seating force.* The pair is the TowerPro
   MG996R (25-tooth output) with its own white single-arm horn. The simulation uses the stand-off and force reported
   as measured in TK-9QFHJE: 3.5 mm and 32 N. The fidelity note lists what differs.

TK-58UVQV:
1. *Final-state scoring was claimed, but an earlier 0.25 rung was kept from any earlier step.* There is no rung that
   reads the past. The tiers read only the last control step, plus the last second for stillness and release. The
   only things carried over from earlier steps are which tooth the horn went on at (it cannot change while the horn
   is on the teeth) and the damage flag. Both are cleared when the horn is pulled off or the scene is reset.
2. *Measure the stand-off and seating force on the selected MG996R.* No physical measurement was made for this
   task. It uses the measured values reported in TK-9QFHJE (3.5 mm, 32 N) and says so in setup and fidelity. A real
   trial should check them first with a 1.5 mm feeler gauge and a kitchen scale under the servo (see Real trial).

TK-9QFHJE:
1. *The Dream-1 and demo videos were from an earlier revision.* Both videos were made from this `task.py`. The
   SHA-256 prefix under Results matches `eval.environment_revision` in both logs.
2. *A full-score demo of the 3.5 mm, 32 N scene within 1,200 steps.* See Results.
3. *Outdated reset and revision text.* This README describes only the current code. Reset is under Reset.

## Scene

Positions are in cm, from the midpoint between the arm bases: x forward, y to the robot's left. Heights are above the
table top.

- **Servo (fixed):** a black MG996R-size case, 40.7 x 19.7 x 37 mm, with mounting ears 54 mm across. It is screwed
  to a 70 x 40 x 3 mm aluminium base plate, long side along x. Its 25-tooth output shaft is 10 mm from the far (+x)
  end, at x 30.0, y -9.0. The shaft rises from a 13 mm round boss whose top is 42 mm above the table. A white paint
  line 3 mm wide runs along the case top from 7 mm to 28 mm out from the shaft, towards the robot (-x): this is the
  zero mark.
- **Horn (loose):** a white nylon single-arm horn with a 25-tooth bore. Its hub is 13 mm across and 6.5 mm tall,
  with a 6.6 mm bore 5 mm deep. Its arm reaches 30 mm from the axis, 5 mm wide and 2 mm thick, its top flush with the
  hub's. It lies hub down on the foam pad, centred at x 21.0, y -20.5, with its arm pointing 112 degrees from +x.
  That is 68 degrees from the mark, 4 degrees from a tooth position.
- **Foam pad (fixed):** black, 45 x 45 x 25 mm, centred under the horn.

All shapes are boxes and cylinders built by `parts.py` (a kit for multi-shape and jointed parts, next to `task.py`).
There are no third-party assets.

## The press fit (simulation)

The shaft's core is a 3.2 mm steel pin, 5 mm tall, inside the horn's bore. It keeps the bore within about 2 mm of the
axis. It is thinner than the real 5.9 mm spline so that it also serves as the lead-in the real chamfers give. The tooth crests the horn stands on are an 11 mm black collar round the shaft, 3.5 mm tall. The collar
sits on a straight-down slider with a 40,000 N/m spring. Each control step, an `on_step` rule sets where the spring
rests:

- The rest point follows the collar down while all of these hold:
  - the horn's hub stands on it (within 0.5 mm);
  - the bore is within 2 mm of the axis;
  - the horn is tilted at most 4 degrees;
  - the arm is within 2.5 degrees of one of the 25 tooth positions;
  - the push is at least 32 N.

  The collar stays wherever it is pushed.
- Within 0.3 mm of full depth, the collar goes to full depth. The hub then stands on the boss.
- Otherwise the rest point stays put, so the collar does not give at any force. This covers a horn off a tooth
  position, a tilted horn and a light push.
- When the hub is lifted more than 1 mm off the collar, the collar returns to the top: the horn is off the teeth.

The teeth themselves are not modelled, so a seated horn is not held against turning. The scorer stands in for this.
It keeps the tooth the horn went on at, and it counts a horn turned more than 4 degrees off that tooth while on the
teeth as stripped. The arm is soft, about 500 N/m at the fingertips, so the demo commands its press 75 mm below the
seated position. In the traces it pushed up to 39.5 N, measured from the collar spring.

## Scoring (exact definitions)

The horn's pose is measured at the centre of the hub's underside, in the servo's frame.

- **a:** the arm's angle from the zero mark. **Tooth:** a / 14.4 degrees, rounded; tooth 0 is the mark.
- **On the teeth:** the collar pushed at least 0.5 mm and the hub standing on it (within 0.5 mm). The tooth the
  horn goes on at is recorded. It is cleared once the hub is more than 1 mm above the collar (pulled off).
- **Damage (whole trial):** while on the teeth, the horn is turned more than 4 degrees off its recorded tooth
  (stripped), or is more than 2.3 mm off the axis. The trial then scores 0. This is cleared only by a scene reset:
  the horn back within 0.5 mm of its start, at rest, after a jump of over 20 mm in one control step.
- **On the mark:** the recorded tooth is 0 and |a| is at most 2.5 degrees.
- **Seated:** on the mark; the collar within 0.3 mm of full depth; the hub at most 0.3 mm above the boss; tilt at
  most 2 degrees; bore within 2 mm of the axis.
- **Started:** on the mark, pressed at least 1 mm onto the teeth, tilt at most 4 degrees.
- **Let go:** no point of either arm (link3, link4, link5, gripper, both fingertips; hull faces sampled every 3 mm)
  within 5 mm of the horn. This is measured in the simulator every control step. Until it is measured, or if any
  link cannot be read, the horn counts as held (fail closed).
- **Still:** the horn's pose within 0.2 mm and 0.2 degrees of the previous step's, for 20 steps in a row (1 s).
- **Score:** 0 if damaged. Otherwise, when let go and still: 1.0 if seated, 0.5 if started, else 0. A horn on a
  wrong tooth, standing on the crests, or anywhere else scores 0. Success is the 1.0 state; the episode ends once it
  has held 1 s.

`near_misses.py` checks every tier and limit offline:
- on the crests on the mark, and on the crests as picked;
- 0.9 mm and 1 mm in;
- seated 0.3 mm and 0.4 mm high;
- tilted 2.0 and 2.1 degrees;
- held, unmeasured, and a flat arm face 0.5 mm away;
- one tooth either way, and five teeth (the wiggle-on from as picked);
- pulled off a wrong tooth and refitted on the mark (1.0);
- 2.5 and 2.6 degrees off the mark;
- twisted from a wrong tooth onto the mark (stripped, 0 even after a refit);
- turned 4 degrees (not stripped);
- shoved 2.4 mm sideways;
- 2.0 and 2.01 mm off the axis;
- standing on the shaft's tip, dropped on the case, and left on the table.

## Run and score a trial

    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "Fit the white horn fully onto the servo's output shaft with its arm on the white zero mark, then let go." \
        --scorer task_score --max-steps 1200

- **Scripted demo:** add `--policy yam_scripted`. The results below used `--epochs 2`.
- **Dream-1:** add `--policy dream`.
- One trial takes about 2 minutes on an L40S GPU.
- Use `object_jitter=0`. With it, the seed changes nothing in the layout.
- `parts.py` must sit next to `task.py`. This copy also gives loose parts the same solver iteration counts as jointed
  ones.

The demo runs open loop from the published start layout. It uses fixed waypoints and reads no object poses, joint
states or press-fit state. It has two fixed constants, each measured once in a trace: a carrying offset of the
loaded right arm over the shaft (0.6 mm, 0.14 mm), and a 2 degree wrist correction for the horn turning in the pads
while it is carried. The steps:

1. The right arm grips the hub across its sides, with the arm between the pads.
2. It lifts the horn and turns the wrist 68 degrees over the servo.
3. It lowers the bore over the shaft's tip onto the crests.
4. Still gripping, it presses down (commanded 75 mm past seated). The pads slip on the hub below 32 N, so this
   mostly holds the horn level as it starts on the teeth.
5. It opens slowly over 2 s and rises straight up along the shaft's axis.
6. It closes the fingers and presses on the hub with the fingertips (commanded 75 mm past seated). This press seats it.
7. It eases off, rises and moves away. The left arm stays parked.

`DEMO_VARIANT` (traces only) runs the same plan with the wrist left as picked (`as_picked`) or turned one tooth past
the mark (`one_tooth`).

## Real trial

- Use a TowerPro MG996R screwed to a base plate, with the horn from its own horn set, and paint the zero mark with a
  paint pen at the output's centre position.
- Before trials, check the two simulation values. With the horn standing on the crests, a 1.5 mm feeler gauge
  should pass under the hub. Pressing the horn home with the servo on a kitchen scale should read about 3.3 kg
  (32 N).
- Judge each trial:
  - a top photo with a ruler for the arm against the mark (14.4 degrees is about 7.5 mm at the arm tip);
  - a 0.3 mm feeler gauge that should not pass under the hub;
  - the video for the release and the 1 s hold.
- Pulling the horn off a tooth does not damage it. A horn turned while on the teeth should be checked for stripped
  teeth with a loupe.

## Reset

1. Pull the horn straight off the shaft by hand or with a horn puller.
2. Check its bore teeth with a 10x loupe, and replace it if any are stripped.
3. Lay it hub down on the foam pad's centre, with its arm 112 degrees from +x (68 degrees from the mark's direction,
   toward the robot's left).
4. Nothing else is consumed. The servo stays screwed down.

## Results

Runs in Isaac Sim (Isaac Lab-Arena, yam_arena, one L40S). The demo and Dream-1 runs used this `task.py` (SHA-256
prefix 5df0c8bbe16c, equal to `eval.environment_revision` in both logs). The traces were earlier steps of the same
scene.

| Run | Policy | Result | task_score |
|---|---|---|---|
| Demo (`--policy yam_scripted --epochs 2`, 1,200-step limit) | scripted | Both trials ended on success at step 536. The horn was carried on the mark, pressed home on tooth 0, let go and still. | 1.0, 1.0 (mean 1.0) |
| Dream-1 (`--policy dream`, 1,200 steps, 320 x 176) | Dream-1 | The right arm moved over the servo and back without picking up the horn. The horn stayed on its foam pad. Hit the step limit. | 0.0 |
| Earlier trace, press with the fingertips only (an older build of this scene) | scripted | The collar gave way at 32 N. The arm pushed up to 39.5 N against a horn standing off a tooth position. | - |
| Earlier traces, older demo (released before pressing, 4.6 mm core) | scripted | 3 of 6 trials scored 1.0. In the others, the opening fingers knocked the horn off the mark, or its bore caught on the shaft's tip. That led to the slow release and the thinner core used here. | - |

## Licence

MIT. The code in this folder is new for this task. No third-party assets are used.
