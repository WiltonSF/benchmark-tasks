# Clear the lint blocking a Galaxy S24's charging port with a soft toothbrush, without packing it deeper, and wipe it off

A Samsung Galaxy S24 stands upside down in an aluminium desk phone stand on a kitchen counter, so its USB-C
charging port is on top. Four small balls of loose lint sit inside the port, wedged beside the flat tongue in its
middle, 4.5 mm down. A soft toothbrush lies bristles-down on the counter and a folded paper towel lies beyond it.
The robot must brush the lint out of the port, wipe it off the bristles onto the towel and lay the toothbrush down.

Instruction: "Brush the lint out of the phone's charging port with the toothbrush, wipe the lint off onto the paper
towel, then lay the toothbrush down."

## Why it matters

- Samsung's support page *Basic charging tips for your Samsung Galaxy* (Canada), section "Remove debris from the
  charging port": "Dust or foreign material inside the charging port is one of the most common causes of charging
  issues." It continues: "Use a flashlight to inspect the charging port. If dust or debris is visible, carefully
  remove it without using metal objects or liquids."
  https://www.samsung.com/ca/support/mobile-devices/galaxy-device-not-charging-or-charging-slowly/
- Samsung UK, *What to do if your phone is not charging*: "If dust or debris is stuck in the charging port or cable
  connector, charging may be blocked for safety." and "Remove debris using a soft brush."
  https://www.samsung.com/uk/support/mobile-devices/what-to-do-if-your-phone-is-not-charging/
- Almost everyone who carries a phone in a pocket meets this. Lint that has just collected in the port is loose and
  a soft brush lifts it out; lint that is pushed down, by a jab with a tool or by the plug, is pressed against the
  back of the port and packs, and then the cable no longer seats (that packed lint no longer brushes out is this
  task's stated assumption). So the job is to clean the port early, gently, and
  without pushing the lint further in. Fine work inside a 2.56 mm slot with a soft tool is rare among robot tasks.

## What the robot has to do, and why each part is hard

1. **Pick up the toothbrush by its handle** (13 mm wide) where it lies bristles-down, and carry it bristles-down.
2. **Find the port and line the bristles up on it.** The opening is 8.34 x 2.56 mm on the phone's 7.6 mm top edge,
   about 151 mm above the counter, and it leans back 12 degrees with the phone.
3. **Control the depth.** The lint sits 4.5 mm down: tips that stop short of 4.0 mm miss it; tips more than 6.5 mm
   in, at any moment, push lint against the back wall where it packs (the trial scores 0). That leaves a band of
   about 2.5 mm for the bristle tips, set by hand position with a 190 mm tool held 86 mm from its bristles.
4. **Hook the lint out with a sideways stroke, and stop.** Pushing straight in and pulling straight out moves
   nothing: the bristles have to move at least 2 mm sideways along the opening while at the lint's depth. But a
   stroke carried on after the lint is caught drags it into the end of the slot, where it is scraped off and stays.
   Then the bristles come straight out.
5. **Get the lint off the brush.** Lint carried on the bristles goes back into the port if the loaded bristles are
   pushed in again, so it has to be wiped off: drawn at least 10 mm across the towel.
6. **Lay the toothbrush down** clear of the phone and move both hands away.

## Answers to questions a reviewer may ask

- **Why a toothbrush?** Samsung prescribes "a soft brush" and rules out metal objects and liquids. A clean, dry,
  soft toothbrush is the soft brush most homes have, and its handle (13 mm) can be held by the arm's parallel
  gripper. Its 25 x 10 mm bristle field covers the whole opening: the bristles over the opening go in, beside the
  tongue on both sides, and those over the frame bend on it.
- **Why is the phone in a stand, upside down?** The port then faces up under the light, which is how it is
  inspected; the stand is an ordinary desk phone stand. The phone is not fixed: it rests in the stand, and the trial
  scores 0 if it is knocked (moved more than 6 mm or tipped more than 4 degrees). In a real trial the bristles push
  on it; in the simulation they do not (the port end has no collider, see below), but a hand or the brush's rigid
  parts do.
- **How is the lint set up so it is the same every time?** Four 2 mm balls rolled from grey cotton/polyester dryer
  lint. Each is pushed into its gap beside the tongue (0.93 mm wide) with a 0.5 mm wire marked at 4.5 mm until the
  mark meets the mouth; it wedges there, so it stays put with the port facing up. One that sinks past 5.5 mm is
  redone. Photos before and after, square-on under a flashlight with a 0.5 mm rule, record it.
- **Is the lint simulated?** No. It is a stated rule (next section), drawn as four small grey dots that ride out on
  the bristles and stay on the towel. Accepted tasks score with stated rules in the same way, for example a hub
  counted as scrubbed by rub travel (TK-F37LCS) and a SIM tray that stays latched until the pin lifts its catch
  (TK-MLJF7T).
- **One hand or two?** One. The left arm stays parked. A second hand steadying the phone earns nothing and is
  allowed.
- **What is not covered?** Charging, the electrical contacts, damage to the contacts below the gross level a
  macro photo shows, and other cleaning methods (air, liquids, picks).

## The bristles and the lint (simulation)

**Bristles.** The bristle field is one rigid 10 x 25 x 11 mm box. It collides with the counter, the towel, the
stand and the phone's body, so the brush lies on its bristles and is wiped across the towel physically. The phone's
top 14 mm (the port end: the frame round the opening, the tongue and the cavity floor) is drawn but has no collider,
so the bristle tips can enter the 2.56 mm opening, where they are scored by the rule below; the bristles over the
frame are read as bending on it. A soft-contact version (PhysX compliant contact on the bristle box, with the
frame, tongue and floor colliding) was tried in two simulation runs: the contact drew the brush 1.5 to 2.5 mm further
into the port and 2 mm sideways during the stroke, so it was not used. Whatever touches the port end other than the
bristle tips is caught by the hard-object rule below.

**Lint.** Each control step, `record()` in `task.py` places a 0.5 mm grid of 1,071 points on the bristle tips and
expresses them in the port's frame (u through the phone's thickness, v along the opening's 8.34 mm side, d depth
below the mouth). Tips inside the opening (|u| < 1.28, |v| < 4.17, d > 0) are "in the port"; those in the gaps
beside the tongue (|u| > 0.35) can reach lint, each on its own side. For each tuft (u = +/-0.9, v = +/-2.2,
d = 4.5):

- it is **reached** on a step when gap tips on its side, within 1.5 mm of it along v, are 4.0 mm deep or more;
- while it is reached on consecutive steps, the bristles' sideways movement in the port's cross-section (the mean
  u-v displacement of the tips; a step over 3 mm counts nothing) is added up; at 2 mm it is **hooked** and rides on
  the nearest gap tip;
- a hooked tuft dragged past the opening's end (|v| beyond 4.17) while below the mouth is **scraped off** and is in
  the port again;
- it is **out** once it is 0.5 mm above the mouth; carried back into the opening (0.5 mm down) it is left there
  again;
- a carried tuft within 2.5 mm above the towel, over it, that slides 10 mm across it (counted between consecutive
  steps, 3 mm a step at most) **stays on the towel**;
- tips more than 6.5 mm into the opening at any moment **pack** the lint: score 0 for the trial.

The thresholds (4.0, 6.5, 2 and 10 mm) are stated estimates, not measurements. A robot finger, or the brush's
rigid head, neck or handle, in the phone's top 14 mm or within 1 mm of its top edge scores 0 for the trial (a hard
object at the port: Samsung's "without using metal objects").

## Scoring

Every trial is scored by `score()` in `task.py`; `task.json` has the short text.

**Score 0 for the whole trial, at any moment (it stays 0):**

- packing: bristles more than 6.5 mm into the port. Real trial: a pen line drawn round the bristles 6.5 mm from
  their tips must stay above the mouth in the side close-up video (30 fps, stepped frame by frame);
- a hard object at the port: a robot finger, or the brush's head, neck or handle, touching the phone's top edge
  (side video; simulation: points sampled every 3 mm on the hand links' faces and every 2 mm on the brush's rigid
  parts, inside the top 14 mm or within 1 mm of the top);
- the phone knocked: moved more than 6 mm or turned more than 4 degrees from its place in the stand (side video
  against tape marks on the stand);
- the phone or the toothbrush falls off the counter;
- the scene cannot be read (the scorer fails closed).

**Every score also needs, for the last 1 s:** the toothbrush let go and lying on the counter or the towel (its lowest
point within 4.5 mm of the counter top) at least 5 mm from the phone and the stand; both hands at least 30 mm from
the phone and the toothbrush; the phone and the toothbrush still (under 10 mm/s and 0.5 rad/s).

**Counting lint.** The opening is read in four quarters (each side of the tongue, near each end), one per tuft. A
tuft is left in the port if any lint fibre 0.5 mm or longer remains in its quarter in the end macro photos.

| Outcome at the end | Score |
|---|---:|
| No lint left in the port, and all 4 tufts on the paper towel (photo of the towel) | 1.0 |
| No lint left in the port (some still on the bristles, or wiped off elsewhere) | 0.75 |
| 1 or 2 tufts left in the port | 0.5 |
| 3 tufts left in the port | 0.25 |
| 4 left, or any rule above broken | 0 |

Lots A and B are averaged.

## Shortcuts and naive moves, and what they score

All of these are run by `near_misses.py` (30 behaviour cases and 10 end states, all passing):

| Attempt | Score |
|---|---:|
| Bristles pushed 4.6 mm in and pulled straight out, no sideways stroke | 0 |
| Strokes with the tips short of the lint (field centre 3.0 or 3.2 mm deep) | 0 |
| Brushing across the outside of the mouth | 0 |
| A short stroke, 1.2 mm of travel | 0 |
| Jumps of 4 mm a step (not brushing) | 0 |
| Jabbing 7 mm deep (packing), before or after cleaning | 0 |
| All the lint out, then the loaded bristles pushed back in | 0 (the lint goes back) |
| Stroke carried on 3.5 mm past centre after the lint is caught | 0.5 (two tufts scraped off at the slot's end) |
| Bristle field 15 mm off along the opening (two tufts out) | 0.5 |
| All the lint out but left on the brush when it is laid down | 0.75 |
| Loaded bristles set on the towel without sliding, or slid only 8 mm | 0.75 |
| Lint wiped on the counter beside the towel | 0.75 |
| Full clean, but the brush still held, or still for under 1 s | 0 |
| Full clean, but the brush left lying across the phone or against it on the stand | 0 |
| Full clean, but the phone knocked 8 mm or tipped 6 degrees | 0 |
| Full clean, then the bristles pushed 12 mm in (the brush's head reaches the port) | 0 |

## Nearest tasks, and how this differs

- **Pop a phone's SIM tray out with the eject pin (TK-MLJF7T, accepted):** a rigid pin pushed straight into a 1 mm
  hole on a docked phone to trip a latch. Here a soft tool must stop within a depth band, move sideways inside the
  slot, and carry debris out.
- **Scrub an IV connector's hub before flushing (TK-F37LCS, accepted):** a pad rubbed on the outside of a hub,
  scored by rub travel. Here the dirt is inside a 2.56 mm slot, past a tongue, and pushing too far ruins the trial.
- **Clean an SC fibre connector's ferrule on a cassette cleaner (TK-6Z2YW3, accepted):** a connector end wiped
  across a fixed cleaner. Here the tool goes into the socket and the debris must come out with it.
- **Clean a car battery's tapered post with a terminal brush (TK-4Z8LF9):** brushing the outside of a large post
  with a brush made for it.
- **Scrape stuck-on debris off a griddle into its tray (TK-63RTRA, accepted):** an open flat surface; here the debris
  is hidden in a narrow socket and is lifted out, not pushed along.
- **Take a screenshot on a phone in a desk stand (TK-QJFSFZ, accepted):** the same kind of scene (a phone in a desk
  stand) for pressing buttons; here the skill is tool use inside the port.

## Run a trial

From this folder, in the Isaac Lab-Arena environment of the program's simulation walkthrough (yam-isaac-lab-arena,
the program's YAM embodiment for Isaac Lab-Arena). Trial k uses lot k mod 2, so `--epochs 2` runs both lots and
`task_score` is their mean:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Brush the lint out of the phone's charging port with the toothbrush, wipe the lint off onto the paper towel, then lay the toothbrush down." \
  --scorer task_score --max-steps 1800 --epochs 2 --store-frames
```

- The scripted demo: the same command with `--policy yam_scripted -E cam_width=640 -E cam_height=360`. Its plan
  lasts about 32 s of the 90 s limit; a trial ends earlier, once the full score has held for 1 s.
- `TASK_LOT=A` or `TASK_LOT=B` in front pins every trial to one lot (debugging only). Runs use `object_jitter=0`.
- The simulation prints one `[lint]` line a second: each tuft's state (0 in the port, 1 on the bristles, 2 on the
  towel), the sideways travel and towel slide counted for each, the deepest tip in the opening, the rule flags and
  the score.
- `python near_misses.py` runs the scorer's behaviour cases offline.

## Products and sizes

| Item | Source | Used here |
|---|---|---|
| Samsung Galaxy S24 SM-S921B | Samsung specification: 147.0 x 70.6 x 7.6 mm, 167 g | Body, glass, frame and keys as boxes; the port end drawn with its opening, tongue and floor |
| USB-C receptacle | USB Type-C specification: opening 8.34 x 2.56 mm, tongue 6.69 x 0.70 mm | Depth 7.5 mm and the tongue's top 2.0 mm below the mouth are estimates (check with a 0.5 mm wire) |
| Oral-B Indicator Contour Clean 35 Soft | oralb.com product page (no dimensions published) | 190 mm long, handle 13 x 9 mm, head 28 x 11 x 5 mm, bristle field 25 x 10 mm, bristles 11 mm, 16 g: estimates, a one-minute ruler check |
| Aluminium desk phone stand (Lamicall type, listed 3 x 3 x 4 in, 120 g) | retailer listings | 76 x 76 mm base, ledge with an 8 mm lip, 85 mm back rest at 12 degrees; on a non-slip pad, fixed in the simulation |
| Bounty paper towel, one sheet folded in quarters | retail | about 140 x 130 mm, flat |
| Dryer lint, grey cotton/polyester | any tumble dryer's filter | four 2 mm balls |

Real-trial cost: the phone stays switched off for the whole trial and only its body and USB-C port matter, so any
used Galaxy S24 (SM-S921B, or the US SM-S921U with the same 147.0 x 70.6 x 7.6 mm body), in any cosmetic grade,
will do. Refurbished units start at about $220 at [Back Market](https://www.backmarket.com/en-us/p/galaxy-s24)
(checked Nov 21), against $799.99 new. The aluminium stand is about $10, the toothbrush and towel a few dollars,
and a 0.5 mm wire and rule about $5: about $250-350 in all.

## Results

- Scripted demo (open loop from the visible layout, right arm only): 1.0 in lot A and 1.0 in lot B. In both, the
  four tufts are hooked during the single 2 mm stroke (about 13 s in), drawn straight out, and left on the towel by
  a 20 mm wipe (10.2 and 10.5 mm of it counted within 2.5 mm of the towel); the toothbrush is laid back where it lay
  and both hands move clear; each trial ends at about 28 s. The deepest bristle tip in the opening, in the
  once-a-second log, was 5.3 mm (lot A) and 5.65 mm (lot B); the packing rule (checked every step) never fired and
  the phone did not move in its stand.
- Dream-1, same instruction and source, lots A and B: 0.0 in each (mean 0.0). Both trials ran to the 1,800-step
  limit without refusing; the toothbrush was not picked up, no lint moved and no rule was broken.

## Licence

Task code MIT; text CC BY 4.0. Robot assets keep their upstream licences.
