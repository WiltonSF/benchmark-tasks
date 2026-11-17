# Calibrate a dial stem thermometer to 32 °F in ice water

A bimetal dial thermometer has just been stood in a tumbler of ice water, hanging from its hex calibration nut in a
pot clip. Its dial is off by a few degrees. The robot must let the reading settle, read the dial, and correct it: hold
the nut still (a wrench lies on a riser beside the cup) and turn the dial head on the stem until the pointer is on
32 °F, keeping the stem in the ice water; then let go, put the wrench down and leave the thermometer in its clip.

Built for yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), station scene. `parts.py` (the parts
kit: multi-colour parts, hinged links, visual meshes) and `lots.py` (per-trial starting errors) must sit next to
`task.py`, which imports them.

## Run and score a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Calibrate the dial thermometer in the cup of ice water so it reads 32 °F, and leave it hanging in its clip with the stem in the ice water."

# Dream-1 (lots A, B and C)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1600 --epochs 3 --store-frames --log-dir logs/dream

# Scripted demo (lots A, B and C)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1600 --epochs 3 --store-frames --log-dir logs/demo
```

- **Lots:** trial *k* of a run uses lot *k* mod 3 (A, B, C), so run with `--epochs 3` (or a multiple) and average.
  `TASK_LOT=B` in the environment pins every trial to one lot. Lot A: the dial reads 6 °F high and the room is 75 °F;
  lot B: 9 °F low, room 69 °F; lot C: 4.5 °F low, room 72 °F. Nothing else changes. Because the room temperature differs
  too, the starting needle (room + error: 81, 60 and 67.5 °F) does not give the error away; it shows only once the stem
  has settled in the ice.
- **Runtime per trial:** at most 80 s of simulated time (1,600 steps at 20 Hz). The demo takes about 70 s per lot.
- **Seeds:** runs use `-E object_jitter=0`; the layout is fixed (the thermometer hangs in a 7 mm ring, so jitter would
  move it out of its clip).

## Objects (simulation)

Positions: "in front" is ahead of the line through the arm bases, "left"/"right" from the midline, heights above the
table (world frame: x forward, y left).

- **Tumbler** (loose, 0.45 kg with its ice and water): clear plastic, 13.1 cm tall, 8.6 cm across, 3 mm wall and floor,
  centred 26.0 cm in front, 8.0 cm right. Water to 10.0 cm above the table (drawn, not simulated as a liquid). Six
  2.2 cm ice cubes at the water line, at least 12 mm from the centre, and three 2.0 cm cubes lower down (drawn only);
  the cubes are fixed to the tumbler.
- **Pot clip** (part of the tumbler): stainless strip over the far rim, an arm to the centre, and a ring with a 7 mm
  bore, 10 mm outside, 2.5 mm thick, its top 6 mm above the rim (13.7 cm above the table).
- **Thermometer** (loose, 33 g): a 4 mm stainless stem reaching 12.7 cm below the nut (tip 1.0 cm above the table, 9 cm
  under water); a brass hex calibration nut 10.6 mm across the flats, 7 mm tall, resting on the ring, flats facing left
  and right; 4 mm of bare stem; a hollow stainless dial head 4.6 cm across and 1.3 cm thick (15 g) on a revolute joint
  about the stem. The head's friction fit holds it on the stem against up to 0.08 N m and slips above that: the
  joint's drive holds the head at a rest point (8 N m/rad, capped at 0.08 N m, so at most 0.01 rad of give), and once
  the head is pushed past that, the rest point moves with it, so it stays where it slipped to (light damping,
  0.005 N m s/rad). PhysX's own joint friction was tried first and did not behave usefully here. The nut resting on the ring can only resist about 0.06 N m even
  under a 20 N push (an estimate), so one hand twisting the head turns the whole thermometer; with the nut held by the wrench, a
  pinch on the head's rim turns the head on the stem. The white face reads 0 to 220 °F clockwise over 300 degrees (1.36 degrees
  per °F), with ticks every 10 °F, every 2 °F from 12 to 58 °F and a red tick at 32 °F.
- **Pointer:** on the bimetal coil's spindle, which turns with the stem, not the head. The coil is not simulated; the
  pointer is drawn as one of 131 visual-only needles on the stem (one per 0.5 °F from 20 to 85 °F), and each control
  step the needle nearest (stem temperature + the dial's error) is shown. The drawn needle therefore steps by 0.5 °F;
  the scorer uses the exact value.
- **Wrench** (loose, 40 g): open-end steel wrench with a 10.8 mm jaw, 15.8 cm long, 4 mm thick, jaw tips flared 20
  degrees, blue dipped handle 9.5 x 1.2 x 0.8 cm. It lies flat on the riser, jaw centre 30.0 cm in front, 17.0 cm left,
  handle toward the robot.
- **Riser** (fixed): wood, 12 x 5 x 3 cm, centred 17.5 cm in front, 17.0 cm left.

## The one scripted catch: the wrench holds the nut while seated

An open-end wrench on a hex nut is modelled as a rigid hold while seated. In the simulator the jaw's contact on the
nut alone let the nut turn inside the jaw, so each control step the task checks whether the jaw is seated: its centre
within 1.5 mm of the stem's axis, its mid-plane within 2.5 mm of the nut's mid-height, lying flat within 10 degrees
and square to the nut's flats within 6 degrees (modulo 60). While the jaw is seated, the nut's turn is locked to the wrench's, so the head's turning torque (the fit slips above 0.08 N m) passes through the nut into the wrench and the hand holding it; the head turns on the stem and stays where it was turned. In the simulator this is done by taking the fit's torque off the stem and putting the same
torque on the wrench each control step. When the jaw is not seated, the stem takes the fit's torque, so one hand
twisting the head turns the whole thermometer. If the handle is turned while seated, the nut turns with it (it stays seated as it turns).
`near_misses.py` checks the seated test: fully on, 1.5 mm short (seated), half on (5.5 mm short, not seated), 3 mm
high, 8 degrees off the flats, turned 20 degrees together with the nut (seated), and on the next flats.

## The stem's temperature and the dial

- The stem starts at the lot's room temperature (just put into the ice water). Each control step its temperature moves
  toward 32 x f + room x (1 - f) °F with a 6 s time constant (an assumed value; real stems are often slower, and the
  guides' 30 s wait is the reference), where f is the share of the bottom 51 mm of the stem (the sensing
  area) that is under the water line inside the tumbler. Fully immersed, it is within 0.3 °F of 32 °F after 30 s.
- The dial shows stem temperature + the lot's error + (head angle on the stem) / 1.36 degrees per °F. Turning the head
  clockwise (seen from above) lowers the reading. Turned with nothing holding the nut, the whole thermometer turns in
  the ring (its friction there is below the head's 0.08 N m), and the reading does not change. A GPU check (one hand,
  no wrench, lot B: pinch the head, command the hand 25 mm down onto the ring, twist 12.3 degrees over 10 s; run with a
  temporary one-hand plan in place of `demo()`) turned the whole thermometer 12.4 degrees in the ring while the head
  stayed on the stem (joint angle 0.000 degrees throughout), so the reading did not change. That check ran with
  PhysX joint friction of 0.10 N m on the head; the final model (a 0.08 N m capped hold) was not rechecked with one
  hand.

## Scoring (exact)

Judged at the end of the trial (`task_score`: 1.0 once success has held for 1 s, otherwise `score()` at the end).

- **Released and still:** for at least 1 s (20 control steps) without a break, no finger tip or gripper of either arm
  within 5 mm of any shape of the thermometer or the wrench (measured from points covering the hand links' collision
  hulls: their corners plus every face sampled about every 3 mm, so a flat face resting on a part counts), and the thermometer, wrench and tumbler at rest
  (under 1 cm/s and 0.5 rad/s). If the simulator cannot report the hands, nothing scores.
- **In its clip:** the nut's underside between 1 mm below and 2 mm above the ring's top, the stem's axis within 2.5 mm
  of the ring's centre at the ring, the stem within 8 degrees of upright.
- **In the ice water:** the whole 51 mm sensing area under the water line, inside the tumbler, for the last 30 s
  without a break.
- **Wrench down:** its bounding box at least 10 mm from the thermometer's.
- With all of these: 1.0 if the dial reads 32 °F within 2 °F (30 to 34 °F), the Food Code tolerance for a thermometer
  scaled in Fahrenheit (FDA Food Code 4-203.11(B), "accurate to ±2°F in the intended range of use"); 0.5 within 4 °F
  (28 to 36 °F); else 0.
- **Whole trial:** 0 if the tumbler ever tilts more than 10 degrees. The record resets only with the scene reset at the
  start of each trial.

`near_misses.py` checks every tier and its boundaries (34.0 °F exactly: 1.0; 34.1 and 36.0 °F: 0.5; 36.1 °F: 0; doing nothing in each lot: 0), doing nothing, turning the
wrong way, the whole thermometer spun, setting the dial while it is still cooling, setting it out of the water and
putting it back, the stem lifted, a hand left near, a 0.95 s hold, the wrench left on the nut or put down 8 mm away, a flat hand face 0.5 mm above the wrench with no corner near it,
the tumbler tipped and stood back, a swinging thermometer, the thermometer lifted out, held 2.5 mm up, off-centre and
tilted. Each scores as declared. Run it with `python near_misses.py` (needs yam_arena).

Fixed habits: in lot A the right correction is a clockwise turn, in lots B and C counter-clockwise, by 6 to 12 degrees
of head turn. The errors are more than 4 °F apart, so no single fixed turn brings two lots within 2 °F, and doing nothing
scores 0 in every lot (each starts more than 4 °F off). By arithmetic: a turn fixing lot A leaves lot B 15 °F off and
lot C 10.5 °F off; one fixing lot C leaves lot A 10.5 °F and lot B 4.5 °F off. Lots stop a fixed turn, not a memorised replay of all three.

## The scripted demo

The left arm picks the wrench up by its handle, slides its jaw onto the nut from the robot's side and holds it. The
right arm pinches the dial head's rim. After 32 s (the dial has settled), it reads the dial and turns the head by the
error, then reads again and trims, five times in all, 5 s apart (no turn once a reading is within 0.3 °F). The demo reads the drawn needle and the head pose
from the simulator, the same information the dial shows (the angle between the needle and the red 32 °F tick); it
does not process the camera image. It does not use the lot's error or room temperature directly. Then both arms let go, the
wrench goes back on the riser, and the arms move away. The demo's left arm path is offset 1 mm to the robot's right to
allow for that arm's measured settling error.

## Why this task, and the closest tasks

Cooks and food preparation workers check and correct probe thermometers at the ice point as a routine food-safety
step (county health departments ask for it weekly and after a drop). An earlier board task, TK-297ARY (PR 1010), held a dial
thermometer in ice water and put it away; its review closed it because full credit never needed the reading or the
correction ("a meaningful calibration task would need to represent and assess the reading and correction
themselves"). Here the correction is the scored step: the dial starts several degrees off in either direction, and
full credit needs it brought to 32 ± 2 °F (the Food Code's accuracy for a Fahrenheit thermometer), read after the stem has settled, with the stem kept in the ice water.

Holding one part while the other hand turns it is well covered on the board, and TK-C9N6KA (zeroing a dial-indicator bezel) was closed as not a meaningfully different twist task, with the advice to model the whole measurement. This task does that. The turn has no stop or mark (unlike TK-3SHVWE, a coupling ring turned until tight): its size and direction are read off the dial, and the reading only means something after the stem has settled in the ice for about 30 s with its sensing area kept under water. A policy has to wait, read, turn a few degrees either way, and read again; turning early, turning by a fixed amount, or setting the dial with the stem lifted out all fail (see near_misses.py).

## Licences

All geometry is built from primitives in `task.py`; no third-party assets.
