# Coat the pipe band and socket with cement without leaving a pool or a bare patch

A plumber's bench with a 1-1/2 in PVC drain joint ready for solvent cement. One bench vise holds the pipe upright with
its socket-depth line drawn on; a second vise holds a pipe stub with a coupling cemented on top, its upper socket
open. The robot takes the dauber from the cement can, coats the pipe end down to the line, puts a thin coat inside the
socket, gives the pipe a second coat, and puts the dauber back in the can. Nothing may go into either bore, the swab
must not be squeezed so hard that cement runs, and the socket gets one thin coat (re-coating a quarter or more of its
wall from another dip puddles at the stop).

## Run

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=tasks/coat-the-pipe-band-and-socket-without-leaving-a-pool/task.py -E object_jitter=0 -E control_hz=20 \
      --instruction "Cement-coat this joint as the can directs: the pipe end down to the black line, a thin coat inside the coupling's open socket, then a second coat on the pipe. Leave no bare patch and no cement in the bore, then put the dauber back in the can." \
      --scorer task_score --max-steps 2400 --epochs 2

`--epochs 2` runs lot A then lot B (trial k uses lot k mod 2); `TASK_LOT=A` or `TASK_LOT=B` in the environment pins
one lot. Use `--policy yam_scripted` for the scripted demo. One trial is 2,400 control steps (120 s at 20 Hz); the
demo finishes in about 84 s. On one shared L40S a trial takes about 6 minutes of wall time. `object_jitter=0`: the
layout is fixed. `python near_misses.py` replays 20 scripted strategies through the coating rule offline.

The task uses three helper files next to `task.py`: `parts.py` (multi-shape parts; one addition: shapes in
`VISUAL_ONLY` are drawn without a collider, used for the swab's soft outer layer, the dauber wire, the drawn line, the can's label and the cement surface), `lots.py` (two
fixed layouts). No third-party assets; everything is built from boxes and cylinders.

## Results

Scripted demo (`--policy yam_scripted --epochs 2`): 1.0 in lot A and 1.0 in lot B. Dream-1 with the instruction above
(`--epochs 2`, 2,400 steps): 0.0 in both lots; it ran to the step limit without refusing, reached over the can and the
vises, and coated nothing.

## Scene (millimetres; x away from the arms, y to the robot's left, heights above the bench)

| Object | What it is | Size and place |
|---|---|---|
| Pipe | 1-1/2 in Schedule 40 PVC (Charlotte Pipe, ASTM D1785/D2665): OD 48.26 (1.900 in), minimum wall 3.68 (0.145 in) | 100 long, standing on the vise base, top end 115 above the bench, axis at (290, -220). Black line round it 18.3 below the top (the socket depth). |
| Pipe vise | small cast-iron bench vise, jaws closed on the pipe (sizes estimated) | base 120 x 100 x 15; jaws 70 x 18, tops 68 above the bench. Fixed. |
| Coupling | Charlotte Pipe PVC DWV coupling, part no. 100, 1-1/2 in, hub x hub: 39.7 long (1-9/16 in) with a 3.2 (1/8 in) centre stop, so each hub is 18.3 deep (maker's catalogue). Socket 48.3 across, outside 56 (estimates, ASTM D2665 class). | Lower hub cemented onto a 1-1/2 in pipe stub held in the second vise. Mouth 105 above the bench, stop face 86.7, axis at (200, -100). |
| Coupling vise | same vise, jaws 58 high | Fixed. |
| Cement | Oatey Rain-R-Shine medium blue PVC cement, 8 fl oz (237 mL), item 30891 | Round can 70 across, 87 to the shoulder, neck 36 across up to 95 (sizes estimated); about 140 mL left, surface 40 above the can bottom; 0.25 kg. Centre (180, -270). |
| Dauber | the can's own lid dauber: blue lid 40 across x 16 tall, 2.5 mm wire 45 long, wool swab 19 across x 25 long (3/4 in; sizes estimated); 12 g | Lid resting loose on the neck (not screwed down), swab hanging in the cement. |

Lot B moves the can (with the dauber in it) to (245, -345). Trials run on lots A and B and are averaged; the screenshot
shows lot A. Oatey's directions ask for "a suitable applicator at least 1/2 the size of the pipe diameter"; the 3/4 in
swab on 1-1/2 in pipe meets that.

## Scoring

**How a real trial is read.** The overhead and wrist videos (20 fps, stepped frame by frame) are the record of
everything that happens during the trial: each trip from the can, where the swab goes, and whether it is pushed flat.
Right after the trial: the pipe band is photographed from four sides with a transparent sleeve printed with a 6 x 48
grid (3.0 x 3.2 mm windows) held 2 mm off the wet surface; the socket wall is photographed through the mouth with a
ring light and compared window by window with the same grid printed on a strip inside a dry reference socket; both
bores and the stop are checked with a torch and an inspection mirror (a bead or puddle on the stop, or blue below it).

**Score 0 for the whole trial if, at any moment after the dauber has been in the cement:**

1. the swab goes into the pipe end or past the coupling's stop (cement in a bore; video, then torch and mirror);
2. the swab is pushed flat against the pipe, the socket wall or the coupling's outside, so that its stiff 9 mm core
   meets the surface: more than 5.0 mm of squeeze on the 19 mm swab (cement squeezed out as a bead or run; video and
   photos);
3. a quarter or more of the socket wall is coated again from another trip to the can (a second coat inside the
   fitting puddles at the stop; video, then the mirror);
4. the swab touches the bench outside the can (cement on the bench);
5. the can tips over 15 degrees or slides more than 20 mm, or either vise moves more than 3 mm.

**Coating rule.** A window is coated when blue shows in it. In the simulation: the swab, carrying cement, overlaps
the window while squeezed 1.0 mm or more against that surface (the pipe's outside with the swab outside the pipe, or
the socket wall with the swab inside the socket). One dip (the swab end at least 8 mm into the cement) loads enough
for about 3,200 mm2 of new windows; the band and the socket wall are about 2,770 mm2 each. Then the dauber is dry
until dipped again. Going over windows again on the same dip uses no cement and does not add a coat. A coat on the
band counts once per trip from the can (in a real trial: the band wiped all round on that trip, from the video).

**Every nonzero score needs, for the last 1 s:** the dauber let go, both hands at least 30 mm from the dauber and the
can (sampled over every face of both grippers), and the dauber and the can at rest.

| Score | Condition |
|---|---|
| 1.0 | in the can's order: every band window coated on a trip before the socket's and again on a trip after it; every socket window coated; the dauber back in the can (lid resting on the neck within 3 mm, centred within 6 mm, hanging within 10 degrees of upright) |
| 0.75 | every band window and every socket window coated at least once (including the right coats in the wrong order) |
| 0.5 | one surface fully coated and at least half of the other |
| 0.25 | one surface fully coated |
| 0 | anything else |

In `task.py`: `apply_pose` (coating, load, dips and the socket re-coat rule), `violations` (rules 1, 2, 4), `step`
(rule 5, release, hands), `in_order` (the can's order), `tier` (the ladder), `stowed` (dauber back in the can).

### Shortcuts and near misses (replayed by `near_misses.py`; all 22 pass)

| Strategy | Score |
|---|---|
| scripted demo, lot A / lot B | 1.0 / 1.0 |
| lot A's demo replayed in lot B | 0 (dips where the can is not) |
| demo 0.4 to 1.6 mm low (the GPU runs ran about 1 mm low; replays assume 1 mm), or 1.4 mm off sideways | 1.0 |
| demo 3.5 mm too low in the socket (swab end past the stop) | 0 |
| loaded dauber pushed straight down to the bottom of the socket | 0 (into the bore) |
| swab pushed into the pipe end | 0 |
| pressed 3 mm harder round the pipe | 0 (squeezed out) |
| socket coated all round again from a second dip | 0 (puddle) |
| dauber kept in the hand at the end | 0 |
| pipe, pipe, then socket; or socket, then pipe twice (wrong order) | 0.75 |
| no second pipe coat | 0.75 |
| no re-dip before the socket (runs dry part-way) | 0.25 |
| pipe only / socket only, dauber put back | 0.25 |
| swab held 2 mm off the pipe or off the socket wall | 0.25 (only the other surface coated) |
| band coated 8 mm short of the line | 0.5 |

## Fidelity

True size: the pipe's outside and wall and the coupling's length and stop come from the maker's catalogue; the socket
and outside diameters, the vises, the can and the dauber are estimates. Cement is a stated rule, not a fluid: the
loaded 19 x 25 mm swab coats what it overlaps while squeezed 1 to 5 mm (about 0.2 to 1 N at an estimated 0.2 N/mm);
one dip coats about 3,200 mm2 (an estimate: about 0.5 mL at a 0.15 mm film). The swab's soft outer 5 mm and the
2.5 mm wire have no collider; its 9 mm core does, as do the lid, can, vises, pipe and coupling, so the swab cannot pass
through a wall and the core meeting a wall is the squeezed-out limit. Entering a bore or re-coating a quarter of the
socket counts as a run or pool. Solvent softening, cure, bond strength and joining are not simulated. Both layouts are
fixed: lot B stops one fixed replay, not a memorised pair of layouts.

## Checking the estimates before real trials (about 15 minutes, no robot)

The coating numbers are estimates, stated so they can be checked and changed in one place (`task.py`, top):

- **Load per dip (`LOAD_AREA`, 3,200 mm2).** Weigh the dauber on a 0.01 g scale dry, after one dip, and after
  coating one band of a spare pipe end with a single dip until it stops leaving cement. The coated area per dip
  replaces `LOAD_AREA`; the cement used per band (grams) is recorded with it.
- **Squeeze limits (`PRESS_MIN` 1.0 mm, `PRESS_MAX` 5.0 mm).** Press the loaded swab sideways against a pipe standing
  on a kitchen scale while a ruler shows the squeeze. Note the squeeze at which a stroke first leaves a continuous film
  and the squeeze at which cement beads and runs; these replace the two limits (0.2 N/mm is the current estimate of the
  swab's stiffness).
- **Sizes.** Measure the can, neck, lid, wire and swab with a caliper and correct the drawing in `task.py`.

If a change moves the demo outside the limits, the fixed corrections at the top of `demo()` are re-measured from one
traced run.

## Why it matters

Joining PVC drain, waste and vent pipe with solvent cement is routine work for plumbers and pipefitters: O*NET
47-2152.00 lists "Assemble pipe sections, tubing, or fittings, using couplings, clamps, screws, bolts, cement, plastic
solvent, caulking, or soldering, brazing, or welding equipment" as a task (about 500,000 US jobs, BLS). The can's own
directions (Oatey 30891 technical specification) say: "Apply liberal coat of cement to pipe to the depth of the socket,
leave no uncoated surface"; "Apply a thin coat of cement to inside of fitting, avoid puddling of cement. Puddling can
cause weakening and premature failure of pipe or fitting. Apply a second coat of cement to the pipe." A bare patch is
a leak path; cement wiped or squeezed into the bore runs down inside the pipe.

The robot skill: carry a dauber with a limited load, keep a light, steady squeeze while going all the way round the
outside of a pipe and all the way round the inside of a socket 18 mm deep, stop above the 3 mm stop without touching
the bore below it, and plan dips so that the pipe gets two coats and the socket one thin coat.

## How this differs from close tasks

- TK-7P97ME (finish a painted panel with a roller): flat panel, no paint load, no bore to protect.
- TK-F6JFF7 (pending; load a paint roller on a tray): loading only, no application.
- TK-4T97N9 (filling with a putty knife): a rigid blade packing and striking a flat filler; no curved surface or bore.
- Thread-tape wrapping on pipe threads: tensioned winding, not coating; no socket.
- PVC repair and assembly tasks (a slip coupling dry-fitted across a repair, pipes pushed into fittings): aligning,
  sliding and inserting. Joining is excluded here; this task is the coating step before it.
- TK-RB2759 (pending; nail polish inside the lines): a convex surface only, no load to manage, no inside surface.

## Design notes

- **Why the coating step on its own.** It is the step that decides whether a solvent joint leaks: a bare window is a
  leak path, and cement wiped past the stop or pushed into the bore runs inside the pipe. It is also done against the
  clock (the can's specification gives a set-up time of 1 to 2 minutes at 70 to 90 F), so it is one continuous unit of
  work. Pushing the joint together afterwards is a separate insertion skill.
- **Bores.** A bore is the whole open region inside the pipe below its end and inside the stop below its face, down to
  the bench; any part of the swab there fails the trial.
- **Order.** Full credit follows the can's directions: pipe, socket, pipe again, with one thin coat in the socket
  (re-coating a quarter or more of the socket wall from another dip counts as a puddle).
- **Estimates.** The load per dip and the squeeze limits are estimates; the section above gives a 15-minute check that
  replaces them with measured values.

## Real trials

Use the products named above. Wear nitrile gloves and eye protection, ventilate, keep ignition sources away (the
can's precautions), and lay a solvent-proof tray under the can. The grippers touch only the dauber's lid; cover each
finger pad with a disposable nitrile finger cot so solvent on the lid cannot reach the pads. Rehearsals can use a
water-based blue gel in the same can; scored trials use the named cement. Reset with a new pipe offcut and a new coupling on a
new stub each trial; the dauber can be reused. Scoring photographs are taken within 30 s of the end, before the
cement dulls.

## Reset

1. Clamp a new 100 mm offcut of 1-1/2 in Schedule 40 PVC upright in the first vise, standing on its base, top end
   115 mm above the bench, axis at (290, -220); draw the line round it 18.3 mm below the top.
2. Clamp a pipe stub upright in the second vise with a new Charlotte no. 100 coupling cemented on top, mouth 105 mm
   above the bench, axis at (200, -100).
3. Stir the cement; it should be about 140 mL (surface 40 mm above the can bottom). Stand the can at (180, -270)
   (lot B: (245, -345)) and rest the lid dauber loose on the neck with the swab in the cement.
4. Arms at their ready pose, grippers open. Start within 15 s.
