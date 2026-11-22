# Fixture files

- `finger_pf_torsion_t42.STL`: the original Yale OpenHand Model T42 pin-flexure finger scaffold, unmodified
  (openhand-hardware, commit b10197dcd79e7bcfa92c6d5a0bfa479e6a789fba, `model t42/stl/finger_pf_torsion_t42.STL`,
  SHA-256 358e3fe7e5048424da4f0c83f9b365bfb7f7693802dcf09b757cc3c5fe2b4347). Licence: Creative Commons
  Attribution-NonCommercial 3.0 (the repository's LICENSE.md); credit: Yale GRAB Lab, OpenHand project.
- `finger_geometry.py` / `occupancy.npz`: a 0.5 x 0.5 x 0.25 mm solid-occupancy grid made from that STL by vertical
  ray casting (29.94 cm3 against the mesh's 30.19 cm3). `python fixture/finger_geometry.py` rebuilds it.
- `calibration.json`: two static offsets per arm added to the demo's commanded tool-centre point (see README).
- `bushings.py` / `bushing.stl`: the replaceable pivot-bore bushing used in real trials, one in each ear (below).
  `python fixture/bushings.py` rewrites the STL. The simulation does not read it.

## Features read from the STL (mm, STL frame)

| Feature | Where | Size |
|---|---|---|
| Pivot bore (running joint, 6.35 mm pin) | axis x 21.79, y 9.64, along z | CAD 6.70 mm; upper ear z 24.25-27.25, lower ear z 4.50-7.50, open spring pocket between |
| Routing-pin seat (press fit, 3.175 mm pin) | axis x 19.63, y 21.82, through z 0-31.75 | CAD 3.325 mm, 12.4 mm from the pivot axis |
| Knuckle | y 0.5-19.5, z 4.5-27.25 | overhangs the vise jaws by 19.5 mm |
| Proximal scaffold (clamped) | y 19.5-63.5 | 21 mm wide at the back face x = 10, rigid corners at x = 31 |
| Cast pad (urethane, estimate) | x 22.5-30.5, y 24-58 | black |
| Cast flexure (urethane, estimate) | x 15-26, y 64-79 | amber |

## Custom parts (drawings in words; all mild steel, sizes in mm)

**Tool stand.** Foot plate 58 x 90 x 10 (its front edge 12 mm behind the hanging reamer). Post 16 x 16, 70 tall,
standing on the foot. Top plate 60 x 60 x 6 at 74-80 above the bench with a 12 mm wide slot from its front edge to
6 mm past the wrench axis (slot length 36). The wrench hangs by its chuck nut on the plate with the reamer in the
slot, its tip 7 mm above the bench. Mass about 1.5 kg (estimate).

**Parallel spacer.** One 6 x 25 x 36 mm ground steel parallel between the fixed jaw and the finger's back face
(x = 10), so the jaws bear only on the rigid proximal scaffold and leave the flexure, the distal segment and the
knuckle free.

## Real-trial finger: reusable carrier with replaceable bore bushings

A trial reams two small press-in bushings, not a whole finger. The cast finger (the carrier) is made once and reused;
the bushings are printed in batches ahead of time. The whole reset, bushing swap and enrolment included, takes about
5 minutes per trial.

**Carrier (made once; make two, one spare).** The maker's STL, printed and cast exactly as the maker describes (ivory
ABS printed flat, pivot axis vertical, 0.2-0.25 mm layers; Vytaflex 30 pad and PMC-780 flexure cast per the HDM
steps), with one CAD change: each ear's 6.70 mm bore is opened to a 9.30 mm socket through the full 3 mm of the ear
(lower ear STL z 4.50-7.50, upper ear z 24.25-27.25). Nothing else changes: the ear faces, thickness, 16.75 mm
spacing and axis, the routing-pin seat and the scaffold are the maker's. Measured on the STL, the ear's outer edge is
at least 8.8 mm from the axis, so at least 4.1 mm of ear remains outside each socket. The spring pocket between the
ears is clear to 5.0 mm from the axis and open on one side (toward the pad side and the finger body), so a 9.40 mm
bushing lying flat can be put into it or taken out of it sideways. Measure the printed sockets once (drill blanks or a
caliper) and draw the bushing 0.10 mm larger. Check the routing-pin seat once: the finger's 3.175 mm (1/8 in) dowel
pin goes in only as a press fit. The spare carrier replaces the first when enrolment check 4 finds a mark on it, or
when a socket is worn (two fresh bushings in a row fail check 1 in it).

**Bushing (stock).** `bushing.stl`: a tube 9.40 mm OD x 3.00 mm long with the maker's 6.70 mm CAD bore, so it prints
tight in the same way as the one-piece ear, and a 0.3 mm x 45 degree lead-in chamfer on the outside edge of one end
(pressed in first). Print it with the axis vertical, in the same ABS and with the same settings as the finger, solid
(the 1.35 mm wall is all perimeters); about 40 fit on a build plate. Mass 0.11 g. Tune once with a test batch: if a
pressed-in bushing passes the 6.350 mm go pin, draw the bore 0.1 mm smaller; if it fails the 50 N check, draw the
outside 0.05 mm larger; if the 6.20 mm pin no longer passes after pressing, draw the outside 0.05 mm smaller. Pressing
in closes the bore by a few hundredths of a millimetre, which is why the enrolment checks are made after pressing.

**Why the cut should be the same (reasoned, not measured).** The reamer removes the same ring of material from each
ear: a 3 mm length of the same printed ABS, laid down in the same orientation, with a bore drawn and printed the same
way, held all round by the press fit as the surrounding ear holds it. The cutting torque and push should therefore
match a one-piece ear; the simulation models ear and bushing as one piece (task README, "Declared cutting model").
Pressing a bushing in at 50 N does not split an ear because a straight push on the bushing's face does not wedge its
bore open; the 25 N limit is for the reamer's 45-degree lead wedging a tight 3 mm ear (estimate).

**Tools.** An 8 mm flat-ended pin punch with a shank at least 30 mm long (5/16 in also works), an M6 flat washer
(6.4 mm hole, 12 mm OD), a seating block (40 x 16 x 4.50 mm, printed or steel, with a 12 mm hole on its centre line
10 mm from one end; 4.50 mm is how far each ear's outer face sits in from the finger's faces). The block lies across
the finger under the knuckle, its 40 mm length along the knuckle's width; 16 mm fits between the knuckle's end, 9.1 mm
from the axis, and the finger body's end face, 9.2 mm from it on the other side, a 10 kg kitchen scale and a paint pen.

**Swap, finger out of the vise and standing the right way up on the scale, knuckle on the block.**
1. Out: with the block's hole under the lower ear, push the upper bushing down into the pocket with the punch and take
   it out of the pocket's open side. Push the lower bushing down through its socket into the block's hole with the
   punch reaching through the empty upper socket.
2. In: slide the block so its solid end is under the lower ear. Put a bushing, chamfered end down, into the pocket
   through its open side, onto the lower socket, and press it with the punch (through the empty upper socket) until
   it stops on the block: it is then flush with the ear's underside and, being 3.00 mm long, with its top.
3. Slide the block so its hole is under the lower ear, tare the scale, and push the lower bushing's top face down with
   the punch until the scale reads 5.1 kg (50 N): it must not move (enrolment check 1).
4. Put a second bushing, chamfered end down, on the upper socket, the M6 washer on it, and press the washer with the
   punch until it meets the ear's top face (flush). Then push the bushing's own top face down at 50 N with the punch,
   as in step 3: it must not move.
5. Draw a paint-pen witness line across each bushing's outer face and its ear's face (the upper ear's top face, the
   lower ear's underside).

**Enrolment (every trial, after the swap), so both ears start the same.**
1. Each bushing held a 50 N push on its top face, in the reaming direction, with nothing under it, and is flush with
   both faces of its ear (no step felt with a fingernail). Held by friction on a 4.7 mm radius, it then resists at
   least 50 N of push (twice the 25 N limit) and about 0.2 N m of torque (above the 0.15 N m cutting torque estimate).
2. The 6.350 mm go pin does not enter either bushing at 5 N.
3. One 6.20 mm pin passes through both bushings in a single push at 5 N or less: both bores are at least 6.20 mm and
   their axes are in line within about 0.15 mm.
4. Under the 10x loupe: no cut, burr or chamfer on either ear face, the scaffold's top face or the routing-pin seat's
   mouth; no cut or dent in the pad or flexure.
A bushing that fails 1-3 is discarded and replaced. A carrier that fails 4 is replaced by the spare.

**After the trial.** A bushing found out of flush was pushed harder than its 50 N hold: this is read as the
over-force rule (score 0), since a one-piece 3 mm ear would have split. A witness line that is offset with the
bushing still flush means the bushing turned in its socket instead of being cut: the trial is void and is run again
with a new pair (the simulated ear cannot turn).

## Load plate (real trials: the 0.25 s force average)

The vise stands on a Phidgets KIT4007_0 weighing-scale frame (156 x 156 mm anodised aluminium top and bottom
plates) fitted with the kit's 0-25 kg FRC4116_0 load cell. A Phidgets DAQ1500_0 Wheatstone Bridge Phidget reads the
cell at a 20 ms data interval (50 samples a second, its fastest rate on one channel) through a HUB0007_0 USB VINT hub,
logged on a laptop by the Phidget Control Panel (Windows) or a short script with the Phidget22 library. Calibrate it
once with a 2 kg mass (19.6 N).

Placement: the plate's centre is at (0.2985, -0.067) m in lot A and (0.2815, -0.103) m in lot B, under the vise
base and offset toward the bore; it turns with the vise about the bore between lots, and stays at least 9 mm clear of
the tool stand's foot in both lots. The plate stands on the bench. Measure its
assembled height h once and raise everything else by h (the two arm bases on h shims, the tool stand and the back
bench items on h spacers), so every height in the layout is measured from the plate's top as the bench.

Reading: tare the plate with the vise and finger on it before each trial and start the log. The logged value is then
the downward push on the finger, which in a trial comes from the reamer (held within 5 degrees of vertical, so it is
the axial force). The trial scores 0 if the mean of any 13 consecutive samples (0.26 s) exceeds 25 N. Load in use is
under 5 kg (vise 2.0 kg plus at most about 2.5 kg of push), well inside the cell's range.
