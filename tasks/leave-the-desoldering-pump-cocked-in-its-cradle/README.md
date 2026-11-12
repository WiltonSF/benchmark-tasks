# Leave a desoldering pump cocked and ready in its bench cradle

A manual desoldering pump lies on a mat beside its bench cradle. Pushing its plunger knob 70 mm in, until a catch
holds it, cocks it; the red button on the barrel releases the catch and the plunger shoots back out. The robot must
leave the pump cocked, lying in the cradle with its nozzle against the end stop. In lot A the pump starts released;
in lot B it starts cocked, and handling it must not fire it.

Instruction: "Leave the desoldering pump cocked and ready, lying in its cradle."

## Run

On the program's Brev launchable (yam-isaac-lab-arena, the program's YAM embodiment for Isaac Lab-Arena), from
this folder:

    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 \
      --instruction "Leave the desoldering pump cocked and ready, lying in its cradle." \
      --policy yam_scripted --scorer task_score --max-steps 900 --epochs 2 --store-frames

Use `--policy dream` for Dream-1. `--epochs 2` is required: trial k uses lot k mod 2 (lot A first), and the score is
the mean. `TASK_LOT=A` or `TASK_LOT=B` pins every trial to one lot. Runs use `object_jitter=0`. A trial is at most
900 control steps (45 s at 20 Hz).

Files: `task.py` (scene, scoring, demo), `parts.py` (multi-shape parts and joints), `lots.py` (per-trial start
states), `near_misses.py` (offline scoring cases). `parts.py` and `lots.py` patch the yam_arena embodiment class in
the task's own process at import; nothing is changed on disk. No third-party assets are used.

## Lots

- Lot A (screenshot): released, the knob 70 mm behind the rear cap. The pump must be cradled and cocked.
- Lot B: cocked. The pump must be cradled without firing it (or re-cocked if it was fired).

Pressing the button never cocks the pump; doing nothing leaves it on the mat (0 in both lots); cradling it without
cocking fails lot A. One replay of the demo passes both lots: the lots do not test reading the pump's state.

## Scoring (exact)

Pass/fail per trial; the run's score is the mean over trials.

- **Cocked:** the plunger pushed in at least 67 mm of its 70 mm (the knob within 3 mm of the rear cap) and held
  there by the catch.
- **In the cradle:** the barrel's axis within 8 degrees of the channel, nozzle forward; the axis within 4 mm of the
  channel's centre line; the underside of the barrel's middle within 2 mm of the channel floor; the barrel's front
  end within 10 mm behind the end stop.
- **Released:** an `on_step` callback measures, every control step, the distance from every finger and gripper hull
  of both arms to every shape of the pump (button and plunger included); all must be over 5 mm. Until it has
  measured, nothing is credited (fail closed).
- **Still:** the pump under 1 cm/s and 0.5 rad/s (the task.json text says "still").
- **Held:** all of the above continuously for 1 s.

Offline (`near_misses.py`) a plunger at 67 mm or more counts as caught, and each case is taken as held 1 s.

## Mechanism as modelled

- Plunger: a slider along the barrel, 70 mm of travel, its spring a drive of 50 N/m toward 50 mm beyond the out
  stop (2.5 N out, 6 N fully in).
- Catch: once the plunger is 67 mm in (and the button up), the drive's rest point moves past the in stop, so the
  plunger stays in. Pressing the button 2.5 mm moves it back out, and the plunger shoots out.
- Button: a slider 4 mm deep, sprung up with 1.5 N, rising to 2.25 N at the 2.5 mm firing point.
- Lot B's cocked start is set by an `on_reset` callback before the trial's first frame.

## Demo

Scripted and open loop from the published layout, the same plan in both lots; it reads no simulator state. The left
arm pinches the barrel at its middle, clear of the button, and lays the pump in the cradle with its front 3 mm short
of the stop. The right arm then pushes the plunger knob forward with closed fingers until the catch holds
(commanded 30 mm past the stop, since the soft arm gives against the spring) and backs off. In lot B the knob is
already in, so the push only presses the pump against the end stop.

## Seeds

Deterministic with `object_jitter=0`.
