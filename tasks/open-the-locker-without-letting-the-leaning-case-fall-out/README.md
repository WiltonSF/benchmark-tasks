# Open a valuables locker whose door a tipped perfume box is leaning on, and take the box out upright

The setting is a row of three small valuables lockers, the kind people use at gyms, pools, schools and workplaces
for a phone, keys or toiletries. They are modelled at true size: grey steel walls, a pale yellow door on each
compartment with vent slots, a number plate and a key lock. Real valuables lockers have compartments about 15 to
17 cm tall, 16 to 30 cm wide and 35 to 48 cm deep; these are 16.5 x 24 x 35 cm. The right-hand door is unlocked,
hinged on the right and held by a magnetic catch; the other two are locked. Inside stands a slim gift box holding a
flat glass bottle of perfume, on end and tipped forward past its base, so that its top rests on the inside of the door.
Opening a locker or cupboard and having its contents fall out is a common everyday mishap, and this is that
situation. Once the door is pulled past the catch, the box pushes it open and leans further with it; opened in one
go, the box falls out. The ability tested: hold the door partly open against the load and take the box out through
the gap. The box holds glass and is printed this way up, so tipping it past 45 degrees scores 0 even if it is laid
down slowly, and dropping it scores 0 even if it lands upright in place. The instruction only asks for the box to
be taken out and stood on the mat.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), station scene. `parts.py` (a small parts kit) builds the lockers and the hinged door; `tilt.py` lets the
scripted demo tilt the gripper. Both files must sit next to `task.py`. `near_misses.py` checks `score()` and
`success()` on near-successes and wrong outcomes without a simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Take the blue box out of the right-hand locker and stand it upright on the green mat."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/locker-dream

# Scripted demo (reaches 1.0 in about 21 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/locker-demo
```

- **Runtime per trial:** at most 60 s simulated (1,200 steps at 20 Hz), about a minute of wall time on an L40S once
  Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0` (the box starts against the door; jitter would push it
  into the door or the walls). The layout is then the same every trial.

## How the scene is built

- **The lockers.** One fixed part: a floor plate, a back wall, four upright walls and a roof, all 10 mm thick,
  making three compartments 24 cm wide, 35 cm deep and 16.5 cm tall in a row (y -10 to 65 cm inside). The two left
  doors are locked, so they are fixed slabs in their openings. Each door carries four vent slots, a number plate
  and a key lock, standing 0.6 to 5 mm proud of its outer face; the slots are dark strips, not holes. The open
  compartment is the right-hand one.
- **The tipped start.** The task-file API places loose objects upright only. A reset hook (`parts.on_reset`) sets
  the box's pose after every reset: tipped 12.5 degrees forward about the front edge of its base, its top 0.5 mm
  behind the shut door, so it settles onto the door in the first steps. The box is 25 mm thick and 160 mm tall,
  so it tips past its base at 8.9 degrees and is held up only by the door.
- **The door.** A hinge with damping 0.3 N m s/rad. An on-step hook makes the catch: within the first 0.05 rad the
  door is pulled shut with 0.045 N m; past that it turns freely. Traced on the GPU in this scene: with the door
  held at 0.92 rad the box leaned 34 degrees. In an earlier trace (a single compartment with the same door and
  box, not re-measured here): opened past about 1.1 rad unheld, the box passes 45 degrees and falls out.
- **The trial record** (box ever past 45 degrees; box ever dropped) is cleared by the same reset hook, so nothing
  the robot does can erase it.
- **Clearance.** Checked offline with forward kinematics along the demo, no arm link comes within 7.5 mm of the
  moving door or its fittings, and none enters the fixed lockers. A second row of lockers above was tried and
  dropped: the right arm's forearm passes through it while reaching down to the pull.

## Dropping

The box counts as dropped if, at any control step of the trial, its centre moves downward faster than 0.45 m/s
(`DROP_SPEED`). A free fall from about 1 cm reaches this; it is checked at each 50 ms control step, so falls from about
4.5 cm or more are always caught and shorter ones only sometimes. A box let go from 5 cm up (about 1 m/s on
landing), one that slips out of a grip, and one that falls out of the locker when the door opens all count. Like tipping, it is
judged over the whole trial: a dropped box scores 0 even if it ends upright on the mat. Lowering the box faster
than 0.45 m/s in a grip also counts as dropping it, which no careful handling of a glass bottle needs.

## Release check

A standing box earns credit only once it is let go: an on-step hook measures, every control step, the distance from
both arms' gripper, `tip_left` and `tip_right` link hulls to the box, and every one must be at least 5 mm
(`RELEASE_GAP`) away. Until the first measurement after a reset, nothing is credited (it fails closed). Offline,
`near_misses.py` sets `LOCKER_CASE_TASK_OFFLINE_CHECK=1` so its cases stand for states with the grippers away; its
own runner also checks a box standing on the mat while gripped (0), a finger 4.9 mm away (0), every link exactly
5 mm away (1.0) and no measurement yet (0). Set down gently, the box is well under the
limit: in a GPU trace of the demo (positions logged every 5th step), its fastest downward speed between
samples was 0.04 m/s, and the run scored 1.0, so the drop check, which reads the velocity every step, never fired.

## Scripted demo

The right arm grips the handle (4 cm from the hinge), pulls the door past the catch and opens it slowly to
0.95 rad, holds it while the left arm takes the box, then swings it on to 1.5 rad, clear of the opening, and lets
go. The left arm uses `tilt.py`: with its fingers closing front to back across the box and its fingertips leaning
30 degrees away from the robot, it reaches down into the gap between the open door and the locker front and
pinches the box's top. When the door is clear it lifts the box, draws it out, turns the grip back so the box
hangs upright, sets it on the mat from 5 mm up, lets go and rises straight up clear of it. The demo uses fixed waypoints only; the mat target is offset by
the 16 mm and 21 mm the box hangs from the pinch point, measured in a trace.

`tilt.py` replaces `yam_arena.demo.grip_rotation` at import, in the task's own process, so a waypoint can carry a
tilted gripper rotation. It changes only the scripted demo; Dream-1 and scoring do not use it.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable
```

Cases include: standing on the mat (1.0), front face on the mat's edge (1.0), leaning 4 degrees on the mat (1.0),
overhanging the mat by 5 mm (0.5), standing on the table beside the mat (0.5), standing on the table in front of
the middle locker (0.5), standing on the lockers' roof (0), moving (0), held 1 cm up (0),
leaning 8 degrees (0), the starting lean (0), pushed upright inside the locker (0), a 44-degree lean (not tipping),
set down gently at 0.3 m/s then standing on the mat (1.0), lowered at exactly 0.45 m/s then standing on the mat
(1.0), dropped from 5 cm then upright in place on the mat (0), falling out of the locker as the door opens then
standing on the table (0), the door opened to 90 degrees and held (0), lying on its face (0), and stood on the mat
after tipping (0). The cases run in order on one copy of the task, so the drop and tipping cases come last.

## Habits that score 0

There is one box, and only the right-hand door opens (the other two are locked and fixed), so there is no
"nearest" or "leftmost" choice to guess. The habits that fail are
ways of opening: pulling the door open in one go, opening it slowly but all the way before reaching in, or opening
it with one arm and letting go (the box then pushes it open). Each lets the box lean past 45 degrees or fall
out, which scores 0 for the rest of the trial; `near_misses.py` has a case for each, and for tipping the box out onto the
mat. The layout is fixed (no seeds), so a memorised replay of the demo's waypoints cannot be ruled out.
