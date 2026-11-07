# Lift a service door off its lift-off hinges once it is open past about 80 degrees

A small unit cabinet has a service door hung on two lift-off pins. The door is unlatched and stands 30 degrees open.
The upper hinge sleeve has a small tab, and a fixed keeper plate sits 6 mm above it. While the tab is under
the keeper the door can rise only 6 mm, and its sleeves need 25 mm to leave the pins. The keeper has a notch that the
tab reaches only with the door between about 80 and 95 degrees open; there the door lifts off. A door stop holds the
swing to about 125 degrees. The robot has to swing the door to that angle, lift it off both pins, and stand it in the yellow rack's slot.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), on the `station` scene. It uses `parts.py` (in this folder) to build the door, which is one rigid body
made of several boxes, and to read the grippers' positions for the release check. `near_misses.py` lists
near-successes and wrong outcomes with the score each must get. They are plain-Python checks of `score()` and
`success()` and need no simulator.

## Run a trial

From this folder, on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Take the service door off the unit and stand it in the door rack."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/door-dream

# Scripted demo (reaches 1.0 in about 22 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/door-demo
```

The run prints `task_score`. Scoring is `success()` and `score()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about a minute of wall time on an
  L40S once Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0`: the door must start with its sleeves on the pins, and
  jitter would shift it off them.

## Scoring details

- **Off the pins:** a sleeve counts as on its pin while its centre is within 19 mm of the hinge axis sideways and it
  overlaps the pin's height to within 5 mm. Both sleeves must be off.
- **In the rack:** the panel's lowest corner is at most 3 mm above the table, the panel's top and bottom edges are
  within 10 degrees of level, and the middle of its bottom edge lies between the walls (within 9 mm of the slot's
  centre line) and between the rack's ends. Either way up and either end first is accepted.
- **Released:** after every control step, `task.py` measures how far the robot's finger and gripper collision
  shapes are from points spaced every 5 mm or less over the door's surface. Released means no point is within 5 mm.
  Offline (in `near_misses.py`) there are no grippers, so the door counts as released.
- **Still:** under 1 cm/s and 0.5 rad/s, held for 1 s (`SUCCESS_HOLD_S`). `score()` is 1.0 exactly when `success()`
  holds, and 0 otherwise.

## Scripted demo

`demo()` drives the right arm with fixed waypoints. Nothing is read from the simulator. The arm grips the door's
top edge 81 mm out from the hinge axis, over its centre of mass, with the fingers closed 3 mm past each face. It
swings the door from 30 to 88 degrees in 6-degree steps, turning the grip to stay square to the panel. Then it
lifts the door 32 mm, clear of the 25 mm pins. A larger lift there would jam the lower sleeve under the upper leaf.
It backs the door 35 mm toward the robot, away from the leaf and the keeper, lifts it to carry height, squares it
to the rack and carries it across. Over the slot it lowers the door to 6 mm above the table and opens. The release point is shifted 2.3 mm to
correct the arm's measured tracking error while carrying.

## Checked on the GPU: lifting too early, and prying

Two variants of the demo with the swing left out were each run once, at 30 degrees open. In the first, the arm grips
over the centre of mass and lifts. In the second, it grips 30 mm from the hinge and lifts the hinge end, so the free
end tips down. Each time the door rose about 6 mm until the tab met the keeper, and it stayed on both pins (at 32 and
41 degrees open). Both scored 0.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable
```

The cases cover the full-score state either way up and either end first, against one wall, the 3 mm height limit
(and 4 mm), lean at 9.9 and 10.5 degrees, the slot's end, beside the rack, still moving, lying across the walls,
dropped flat on the table, and the door's tab jammed under the keeper after an early lift at 30 degrees. They also cover
the door still hung at 90 degrees, lifted short of clearing the lower pin, lifted clear and held over the pins,
and tilted 10 degrees on its pins at 40 degrees open.
