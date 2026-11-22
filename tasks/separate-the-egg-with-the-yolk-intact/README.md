# Separate an egg with a squeeze separator, laying the yolk unbroken in its own bowl

**Instruction:** "Use the egg separator to move the yolk, unbroken, into the small bowl and leave the white in the large bowl. Then put the separator down and let go."

A raw large egg has been cracked into a 12 cm glass bowl. The robot uses a squeeze egg separator (a silicone bulb) to
lift the whole yolk out of the white and lay it in a second, smaller bowl. The yolk is held together only by its thin
membrane: the separator's mouth has to meet the yolk without pressing on it, the bulb has to be let open while the
mouth is on the yolk, and the yolk has to be lowered close to the bowl floor before it is let out, not dropped from
height or squirted out. Drawing white up with the
yolk carries it into the yolk bowl, and letting it out anywhere else spills it.

Category: Societal relatability. Separating eggs is an everyday home cooking and baking step (meringue, mousse, sponge
and custard recipes call for separated eggs) that most people have done by hand, and it is rare among robot tasks to
move a fragile food item by suction rather than by a grasp.

## Run it

Simulator: yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), station scene, 20 Hz control,
240 Hz physics. Copy the folder next to the stock kit and run, with `object_jitter=0`:

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=<this folder>/task.py -E object_jitter=0 \
      --instruction "Use the egg separator to move the yolk, unbroken, into the small bowl and leave the white in the large bowl. Then put the separator down and let go." \
      --scorer task_score --max-steps 1200 --epochs 2

`--epochs 2` runs lot A then lot B (trial k uses lot k mod 2); the score is their mean. `TASK_LOT=A` or `TASK_LOT=B` in
the environment pins every trial to one lot. Runtime: about 2 minutes per trial on an L40S. `--policy dream` runs
Dream-1 with the same instruction. `python near_misses.py` (in this folder) runs the offline rule and score checks.

Files: `task.py` (scene, rules, scoring, demo), `parts.py` (multi-shape parts, sliders, visuals, physics-step hook),
`lots.py` (the two mirrored layouts), `near_misses.py` (offline checks). No third-party assets; every shape is a
primitive written in `task.py`.

## Scene (lot A; lot B is its mirror image across the centre line)

World frame: x away from the robot, y to its left, z up; the counter top is z = 0.7475 m.

| Object | Real item | Simulated as |
|---|---|---|
| Egg bowl | Duralex Lys 12 cm stacking bowl, tempered glass, 120 mm across, 49 mm tall, 310 mL (retail catalogue size) | loose, 230 g (estimate); 56 mm inside floor (estimate), 33 degree flared wall of 16 flat 3 mm panels, 4 mm floor; centre (0.355, -0.150) |
| Yolk bowl | Duralex Lys 10.5 cm, 105 mm across, 40 mm tall, 205 mL | loose, 160 g (estimate); 50 mm floor (estimate); centre (0.245, -0.035) |
| Egg | one USDA Grade A Large egg (about 57 g whole): yolk about 17 g, white about 33 g | yolk: rigid 36 x 36 x 24 mm dome (16.3 mL), 17 g, in the middle of the egg bowl; white: 31.5 mL tracked volume, drawn as a pale layer 12 mm deep |
| Separator | Peleg Design YolkFish, food-grade silicone, about 100 x 40 x 70 mm (retail listing) | loose, 38 g: lip ring 32 mm outside, 22 mm opening, 6 mm tall; head, body and tail fin; two 56 x 52 mm side walls on sprung sliders, 40 mm apart at rest. Stands mouth-down on the saucer, flat sides facing left and right; centre (0.215, -0.255) |
| Saucer | plain 12 cm white porcelain saucer | loose, 150 g, 120 mm across, 6 mm thick |
| Backsplash | tiled wall | fixed 14 mm panel, 80 cm wide, 30 cm tall, 57.5 cm out |

Separator sizes other than the overall size are estimates: check the mouth with a caliper and the wall stiffness by
pressing the bulb in 10 mm on a kitchen scale (about 4 N expected).

## How the egg is modelled (declared rules)

The simulator has no fluids or soft bodies, so the egg is a rigid yolk plus these stated rules. Every rule is in
`task.py` (`bulb_step`, `monitor_physics`, `on_step`) and tested in `near_misses.py`.

1. **The bulb.** Each side wall slides in up to 10 mm on a 400 N/m spring (estimate for a 1.5-2 mm silicone wall).
   Pressing the two walls in by a total of d mm pushes 1.5 x d mL out of the mouth; letting them back draws the same
   volume in.
2. **Drawing the yolk up.** While the separator is upright (mouth down within 25 degrees), its mouth's centre within
   6 mm of the top of the yolk sideways and the underside of the lip ring no more than 1 mm above the yolk's top (the
   dome rises 2.5 mm into the 22 mm opening before the ring meets it; the code accepts down to 3.5 mm below the top,
   which can only be reached by pressing on the yolk, where rule 5's 0.5 N limit applies), the volume drawn in goes
   to the yolk. Once
   16.3 mL (the yolk's volume) has been drawn in this way, the yolk is inside the separator, with 1.0 mL of thick
   white that clings to it (estimate). Moving the mouth off the yolk before that resets the count: the yolk sinks
   back. While the yolk is inside, the simulator keeps the yolk body out of the scene.
3. **Drawing white.** Volume drawn in with the mouth below the white's surface in the egg bowl, and not on the yolk,
   is white (up to what is left). The surface height follows the bowl's shape and the volume left. Volume drawn in
   anywhere else is air.
4. **Squeezing out.** With the yolk inside, the yolk leaves the mouth after 8 mL has been squeezed out, and is placed
   just under the lip ring (resting on the floor below if that is higher), moving with the separator plus the exit
   speed (volume rate / 3.8 cm2 opening). The clinging white and then any other white inside come out after it. Material
   lands straight below the mouth: inside the small bowl's rim it is carry-over, inside the egg bowl's rim it is back
   in the white, anywhere else it is spilled.
5. **Bursting** (whole trial, never undone): the separator pressing on the yolk with more than 0.5 N (240 Hz contact
   force); any robot surface (every link of both arms, sampled every 4 mm) within 2 mm of the yolk; the yolk
   moving faster than 0.8 m/s at any physics step (a free fall of about 33 mm, or a squirt), checked from the step after
   it leaves the mouth; squeezing it out with less than 22 mm between the lip ring and the surface below it. A burst
   shows as a yellow pool around the yolk. Basis: the vitelline membrane's puncture strength in egg-quality testing is
   a few to about 25 grams-force for a small probe; a 22 mm lip spreads the load, so 0.5 N (51 gf) is taken as the
   burst load. These thresholds are estimates to check in the real trial below: before a test series, press a spare
   yolk in a bowl with the separator's lip on a kitchen scale and note the reading at which it bursts, and drop spare
   yolks from 2, 3 and 5 cm into the small bowl. A real yolk deforms to pass the 22 mm mouth; the rigid yolk cannot,
   so the simulator holds the yolk body out of the scene while it is inside (rule 2).
6. **Bowls.** A bowl tipped more than 15 degrees or lifted more than 20 mm at any time pours raw egg over its rim: the
   trial scores 0.

## Scoring

Per trial, in this order (`score()`):

- **0** if at any time the yolk burst (rule 5), a bowl was tipped or lifted (rule 6), or the simulator state could not
  be read (fail closed); or if at the end the yolk is not lying whole in the small bowl (its centre within 25 mm of the
  bowl's axis, its underside within 4 mm of the bowl's floor, still), or the last time it left the separator it was
  not squeezed out over the small bowl (a yolk pushed, carried on the lip ring, or squeezed out elsewhere and then
  moved does not count), or any point of the separator's box is within 5 mm of it, or it has not been so for 1 s
  (yolk under 10 mm/s and 0.5 rad/s); or if more
  than 5.0 mL of white is outside the bowls (white still inside the separator counts: it drips out when the separator
  is put down), or more than 10.0 mL is in the small bowl (nothing was separated).
- **1.0** if the carry-over (white in the small bowl) is at most 2.0 mL, the white outside the bowls at most 1.0 mL,
  the separator is put down (resting, its lowest point within 5 mm of the saucer or the counter, every corner of its
  box clear of both bowls' rims, under 10 mm/s), both grippers are at least 30 mm from the separator, the yolk and
  both bowls, and this whole state has held for 1 s (its own counter).
- **0.75** if the amounts are as for 1.0 but the separator is not put down or a gripper is within 30 mm.
- **0.5** if the carry-over is at most 5.0 mL and the white outside the bowls at most 2.0 mL.
- **0.25** otherwise (a whole yolk in the small bowl).

The trial's score is the mean over lots A and B. Limits pass exactly at the stated values (1e-6 margin).

### Reading it in a real trial

- Burst: watch the 30 fps overhead video, stepping frame by frame through each moment the separator meets the yolk
  and the yolk is laid down; any yellow running outside the yolk's outline is a burst (a whole yolk sags; that is not a
  burst). Photograph
  both bowls at the end against a white card.
- Amounts are weighed on a 0.1 g kitchen scale; raw egg white is 1.04 g per mL (2 mL = 2.1 g, 5 mL = 5.2 g,
  10 mL = 10.4 g).
- Carry-over: weigh the small bowl dry before the trial. 60 s after the end, lift the yolk out with a slotted spoon,
  hold it over the bowl for 10 s so the white clinging to it drains back in (the simulator counts that clinging
  1.0 mL as carry-over too), and weigh the bowl again; the gain is the white carried over.
- White outside the bowls: weigh a folded paper towel before the trial; afterwards wipe the counter and the saucer
  with it, squeeze the separator out into it, and weigh it again.
- Hands clear and stillness: the video, with a 30 mm card held beside each gripper at the last frame if in doubt.
- Bowl tipped or lifted: the video against the backsplash tiles (15 degrees is about a 3 cm rise across the 12 cm rim).

## Reset (real trial)

The bowls stand directly on the laminate counter. Raw egg is wiped up and the bowls, saucer and separator are washed
in hot soapy water between trials; the assessor wears gloves to handle the egg and washes the counter after a series.

1. One USDA Grade A Large egg straight from a 4 C refrigerator, 7 to 21 days after its pack date.
2. Crack it on the counter and open it into the 12 cm bowl at the lot's position; if the yolk breaks or shell falls
   in, discard it and use another egg.
3. Tilt the bowl gently until the yolk sits within 10 mm of the bowl's centre.
4. Weigh the dry 10.5 cm bowl and a folded paper towel; place the bowl and the saucer on their counter marks (lot A as
   listed; lot B mirrored).
5. Rinse the separator, squeeze it dry and stand it mouth-down on the saucer, flat sides facing left and right.
6. Start within 2 minutes of cracking; measure 60 s after the end.

## Unintended strategies (each checked in near_misses.py or by the rules)

| Strategy | Result |
|---|---|
| Do nothing | 0 (yolk not in the small bowl) |
| Pick the yolk up with the gripper, or push it with any arm link | 0 (robot within 2 mm of the yolk: burst) |
| Push or roll the yolk out of the egg bowl into the small bowl, gently enough not to burst it | 0 (it was not squeezed out of the separator over the small bowl) |
| Draw up white several times and squeeze it into the small bowl, then the yolk | 0 above 10 mL carried over |
| Pour the egg bowl into the small bowl, or slide the bowls together | 0 (bowl tipped past 15 degrees or lifted 20 mm; a yolk pushed across the rims was not squeezed out over the small bowl) |
| Push the yolk with the separator's head, or press the mouth down on it | 0 above 0.5 N; a yolk pushed over a rim falls more than 33 mm: 0 |
| Squeeze the yolk out from high above the small bowl, or fast | 0 (speed over 0.8 m/s) |
| Squeeze it out with the mouth jammed on the floor | 0 (no room: burst) |
| Mouth 4 mm above the yolk, 7 mm to the side, or tipped 30 degrees while letting the bulb open | nothing drawn up |
| Mouth pushed down into the white and let open | white is drawn up; it ends up carried over or spilled |
| Keep extra white inside the separator to avoid carry-over | counts as spilled |
| Squeeze the yolk back out over the egg bowl | yolk not in the small bowl: 0 unless drawn up again |
| Leave the separator held over the small bowl | at most 0.75 |
| Lay the separator down on the yolk (its 38 g is under the 0.5 N burst load) | 0 (separator within 5 mm of the yolk) |
| Fixed habit (replay lot A in lot B) | lot B is mirrored, so a lot-A replay reaches for the wrong side and scores 0 there |

One hand is enough for this task; either arm may be used. The demo uses the arm on the separator's side.

## Reference demo

`demo()` is open-loop from the published layout of the current lot (no simulator state): pick the separator up by its
flat sides with the pads 2 mm into each wall, squeeze the walls in 9.5 mm away from the egg, bring the mouth down until
the yolk's dome is 1 mm inside its opening (not touching the lip ring), let the walls back out to 2 mm over 2 s (22.5 mL drawn in: the yolk), lift, move over the small
bowl, lower the mouth to 25.5 mm above its floor, squeeze slowly over 2.5 s (exit speed about 0.03 m/s), lift, stand
the separator back on the saucer, let go and move the hand clear.

Results (task.py unchanged since 12:15 UTC, before both runs): scripted demo 1.0 in lot A and lot B (mean 1.0; the
yolk drawn up at 14 s, squeezed out at about 0.03 m/s at 22 s, 1.0 mL carried over, no white outside the bowls, peak
press 0 N). Dream-1 with the same instruction: 0.0 in lot A and 0.0 in lot B (mean 0.0), both at the 60 s step limit;
the separator and the yolk never moved, and there was no refusal. A first Dream-1 run on the same source scored 0.0 in
lot A (step limit, nothing moved) and ended in a simulator crash at 48 s in lot B (0.0, nothing moved at that point),
so it recorded no mean; the second run is the reported one.

## How this differs from close tasks

- TK-GA22SQ (pending, crack an egg into a bowl with both hands): ends where this task starts; it does not separate
  yolk and white.
- TK-PPNWGA (accepted, charge a three-valve pipette filler and dispense 5 mL): squeeze-bulb control of a liquid volume
  through valves; here the bulb must capture one fragile body whole, and the failure is bursting it, not a volume
  error.
- TK-A2LXP9 (accepted, fill four tubes with a transfer pipette): a squeeze bulb drawing and dispensing a liquid
  volume; here the bulb must capture one fragile body whole, and the scored failures are bursting it and carrying
  white with it.
- TK-L6C8J2 (accepted, drain the water off dumplings) and TK-LBRPTL (accepted, decant a liquid keeping its stir bar):
  pour liquid off while a solid stays behind; here the liquid stays and the fragile item is lifted out of it.
- TK-65UQMJ (accepted, pour soup into a tureen): spill-free pouring of a heavy pot; no fragile item.

## Simplifications

Rigid yolk (a real yolk sags and flattens, and deforms to pass the mouth); while the yolk is inside, the separator does
not carry its 17 g (the real head is opaque, so the yolk is not seen either way); each draw-up takes its 1.0 mL of
clinging white from the egg bowl's account, even when a yolk is re-drawn from the small bowl; a separator lying across
the saucer's edge onto the counter is judged against the saucer only (it then scores as not put down); white as a volume with stated rules, not a fluid (no strands, no
dripping in transit); the separator's body is boxes around a hollow head and the bulb is two flat sprung walls, not a
moulded fish; the yolk body is held out of the scene while inside the separator; thresholds (0.5 N, 0.8 m/s, 3 mm,
1.0 mL cling, wall stiffness, bowl masses) are estimates stated above. Bowls are clear glass in reality and drawn
pale blue-grey.
