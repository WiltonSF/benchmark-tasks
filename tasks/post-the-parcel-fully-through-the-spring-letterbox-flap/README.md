# Post a small parcel fully through a gate's spring-flap mail slot

A section of a wooden garden gate stands on the table with a brass mail slot through it and a locking sheet-steel
catch box behind it. The robot posts a small corrugated mailer box through the slot so that it drops into the catch
box and the flap falls shut behind it.

The gripper's fingers are about 26 mm wide (front to back) at the pads but up to about 68 mm wide higher up, so the
hand cannot enter the 62 mm opening: it stops with its centre about 34 mm in front of the plate. How far the parcel
can be pushed in therefore depends on where it is held:

- held at its trailing end (the demo), the parcel goes in far enough that its centre passes the throat's inner edge
  (the gate's inner face, 22 mm behind the plate); released, it tips onto the chute, slides into the catch box,
  and the flap closes (1.0);
- held about 50 mm or more from its trailing end, its centre stops over the slot floor or in front of the plate;
  released, it stays lying in the slot with its end sticking out, or falls back out (0). Measured on the GPU for a
  grip 45 mm from the trailing end: see "GPU traces while building". Pushing that end again with the closed
  fingertips may still send it in (not tested).

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment
for Isaac Lab-Arena), on the `station` scene. It uses `parts.py` (in this folder) for the multi-shape gate with its
hinged flap and for the parcel's shapes and friction. `near_misses.py` lists near-successes and wrong outcomes with
the score each must get, and tests the release check on synthetic hand surfaces; it needs no simulator. (In the
code the gate and box are the object `letterbox` and the parcel is `packet`.)

## The real objects

**Mail slot.** Salsbury Industries 4045 mail slot, standard magazine size, solid brass (4045B): unit 13 x 3-1/2 in
(330 x 89 mm), rough opening 11-1/4 x 2-1/2 in (286 x 64 mm), spring-loaded mail flap 11-3/8 x 2-3/8 in (289 x 60
mm across its pins). Source: Salsbury's 4045 product listing ("Mail Slot - Standard - Magazine Size",
mailboxes.com). Not published, so estimated here: the plate's opening (286 x 62 mm, just clear of the
flap), plate thickness (3 mm), leaf thickness (0.8 mm), the rolled lower edge (3 mm), and the spring (0.005 N m per
rad; Salsbury publishes no spring rate). The flap's mass is brass (8.5 g/cm³) at those sizes, 130 g: weigh the real
flap and set `FLAP_T` to match. The flap is closed mainly by its own weight: held 70 degrees open, as it is while
the parcel passes, its weight gives about 0.036 N m about the pins and the spring about 0.006 N m.
The flap hangs in the opening on two pins at its top corners and swings inward; the hinge pins are shown as short
bosses at the leaf's ends.

**Gate and catch box.** Three 1x6 cedar fence boards (19 x 140 mm, 3/4 x 5-1/2 in nominal) with a cap rail, the
slot screwed through them. Behind the slot is a locking steel receiving box of the kind sold for through-the-wall
mail drops, which pair a spring-loaded slot flap with a sloped anti-fishing drop chute leading down into a locked
box: for example the AdirOffice 631-10 through-the-wall drop box (all steel, "anti-fishing drop chute", anti-fish
baffle and spring-loaded flap, piano-hinged rear door; listing for Home Depot SKU 308321499), whose chute's angle,
the seller notes, is chosen to limit the length and flexibility of anything pulled back out. That box's own slot is
only 1 in tall, so it is not used whole here: the catch box is a box of that design fitted to the 4045 slot. It is
2 mm painted sheet steel (inside 220 deep, 364 wide, 262 high), fixed to the back of the gate, with a rear door, a
bottom hinge and a round cam lock. Its chute is a 2 mm steel plate 120 mm long across the box's width, from the
lower inner edge of the slot down at 35 degrees, so posted items slide down into the box and a hand reaching in
through the slot meets the sloped plate. No seller publishes a chute angle; 35 degrees is this design's choice.
In `task.py` the chute is a hinge held hard against a stop at 35 degrees by a stiff spring set at every reset,
because parts.py shapes cannot be tilted; it does not move in any trial (the traces log it at 0.6109 rad
throughout). The gate section stands on the table so the slot is at the arms' working height; a real gate's slot
is higher, but the slot, flap and box are at true size.

**Parcel.** A 6 x 2-1/2 x 1 in (inside) white crush-proof corrugated mailer, roll-end tuck-front, 200#/ECT-32-B
board, sold for example as Interplas CP-060201 and Partners Brand MRX-1L (listed bundle weights give about 26 to
40 g each).
Outside size from the inside plus the board (about 3.2 mm B-flute, doubled at the sides and the tuck end): 162 x 76
x 32 mm. Packed to 150 g with its contents centred. A packed mailer box barely flexes, so it is one rigid body. A
padded bubble mailer was considered: the narrowest common size (#000, 3-7/8 to 4 in wide) is wider than the
gripper's 94 mm opening, so the arm could not pinch one lying flat. The shipping label, barcode, address block,
lid fold and tape are thin shapes (0.2 to 0.4 mm) on the box.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Post the white parcel through the mail slot."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/mailslot-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/mailslot-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 30 s of simulated time (600 steps at 20 Hz); about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started.
- **Seeds:** none needed. Runs use `-E object_jitter=0`, so every object starts exactly where `task.py` puts it.
  With jitter on, seeded resets shift only the parcel.

## How the simulator scores it

- "Inside" uses the parcel's world-aligned bounding box (label and tape included), which is never smaller than the
  parcel: its near side must be at or behind the gate's inner face (x 0.382 m), its far side in front of the back
  wall, its sides between the side walls and its top under the roof.
- "Flap shut" is the hinge angle: lower edge at most 5 mm inside its hanging position, asin(5/60.5) = 0.084 rad.
- "At rest" is moving under 1 cm/s and turning under 0.5 rad/s.
- "Let go": after every control step, `on_step` measures the distance from the gripper and both fingers of both arms
  to the parcel's shapes. Each collision hull of those links is sampled over its faces about 3 mm apart (kept at one
  point per 1 mm cell), not just at its corners, so a broad face resting on the parcel counts. Over 5 mm is let go.
  If the arms cannot be read, there is no measurement and the score is 0.
- The 1 s hold is the embodiment's: success() must hold on 20 consecutive control steps to end the trial at 1.0.
- Every limit is compared with a 1e-6 margin. Scoring is pass/fail: a parcel stuck in the slot scores 0.
- The top camera cannot see the box floor (the gate hides it). A photo for scoring is taken through the rear door.

## Scripted demo

`demo()` drives the left arm only. It grips the parcel's trailing end from above across its 76 mm width (pads flush
with the end, tips 3 mm above the table), lifts it to the slot's height (aimed 4 mm high to cancel the arm's sag
while carrying), lines it up 10 mm short of the flap, pushes it in level until the hand is 2 mm short of the plate,
opens the fingers and backs off. The parcel's leading end opens the flap. On release its centre is about 6 mm past
the gate's inner face (measured); it tips onto the chute, slides off its end into the catch box, and the flap swings
shut. On the GPU it scores 1.0 about 12 s into the trial.

## GPU traces while building (this version)

- A first build had a catch box with no chute (a flat drop from the slot floor). Released from the end grip, the
  parcel pivoted on the slot floor's inner edge and wedged at about 41 degrees, its top against the plate's upper
  lip and its underside on the inner edge, with the flap resting on it (0). A 22 mm deep, 62 mm tall throat and a
  32 mm thick parcel meet at that angle. Pushing its trailing end in a further 24 mm with the closed fingertips only
  wedged it again at the same angle. With the drop-box chute at 35 degrees, the parcel's top stays below the slot's
  ceiling while it lies on the chute, so it cannot wedge.
- With the chute, end grip (the demo): 1.0.
- Held 50 mm further from its trailing end, otherwise the same moves: the parcel rolled onto its side as the fingers
  closed and ended leaning on the table in front of the gate (0). It never reached the slot.
- Held 45 mm further from its trailing end, with the pad tips 9 mm above the table and a slower close so it did not
  roll, otherwise the same moves: the hand stopped at the plate and the parcel was released with its centre 22 mm in
  front of the plate. It stayed lying flat on the slot floor, 103 mm of it sticking out in front of the plate and
  its leading end 38 mm past the gate's inner face, the flap resting on it about 63 degrees open, still at the end of
  the trial (0).

## GPU traces while building (earlier version: 20 mm panel, 60 mm slot, 160 x 60 x 25 mm 100 g packet)

- Slot 40 mm tall: the packet tipped and jammed diagonally in the slot. The slot was made 60 mm tall.
- Slot floor 90 mm up: the packet came to rest as a ramp into the box. The floor was raised to 140 mm and a smooth
  floor added.
- Held 47 mm from its trailing end and released 7.5 mm above the slot floor: it stayed lying in the slot.
- Flap springs of 0.075 and 0.05 N m per rad with a 50 g flap: the flap pinned the packet's trailing end and it hung
  about 48 degrees in the slot. That case is in `near_misses.py` (0).
