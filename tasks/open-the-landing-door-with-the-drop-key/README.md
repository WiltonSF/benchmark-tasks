# Open the lift landing door with the drop key

A landing wall stands across the table with a narrow key slot. Behind it, a sliding door is held shut by a gravity
latch: an orange bar whose catch hangs in front of a lug on the door's back. The robot has to pick up the drop key,
pass its hook through the slot under the latch bar, lift the bar and hold it up while the other arm pulls the door
open by its handle.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), run through Inspect Robots, on the `station` scene. It uses `parts.py` (hinged and sliding parts) and
`near_misses.py` (offline scoring checks) from this folder.

## Run a trial

```bash
I="Open the sliding door."
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/landing-dream
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/landing-demo
```

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz).
- **Seeds:** none needed with `-E object_jitter=0`.
- **Checked shortcut:** yanking the handle without the key (scripted check, 125 mm commanded in 1 s) moves the door
  2 mm; the catch holds it and the latch stays on its stop (score 0).

## Scoring details for the simulator

- The wall and the latch are separate fixed parts, so the latch bar collides with the door's lug.
- Every step an on_step callback measures both arms' fingers and gripper bodies: within 5 mm of the door's shapes
  counts as holding it; any of them inside the space behind the wall (x 0.37 to 0.50 m, between the wall's ends, up
  to 30 cm) zeroes the trial. "Still" means the door moved under 0.5 mm in the last control step.
- "The key back in front of the wall": the key's whole bounding box at x 0.36 m or less.
- The behind-the-wall record is kept over the trial and cleared by the simulator's reset hook (offline: the key back
  within 0.5 mm of its start at rest, door closed and latch down, after a jump of over 20 mm in one step).
- The latch's hinge (55 mm up) is below the catch's contact with the lug (60 to 75 mm), so pulling the door turns the
  bar down onto its stop; it cannot be cammed up by yanking the door.

## Scripted demo

Left arm (wrist turned -90 degrees, fingers closing front to back on the grip block): lift the key level, line it up
with the slot, push the hook in under the bar, raise it 28 mm and hold. Right arm: grip the handle, pull the door 95
mm to the right, let go. Left arm: lower the key, draw it out and set it down.

## Checks without a simulator

`python near_misses.py` (needs yam_arena importable). Licences: no third-party assets.
