# Press the Pemko Q102BL seal home along the door head jamb

Status: NEEDS-MORE. The final Isaac Lab-Arena scripted demo scored A=0/B=0.
The single frozen-source Dream-1 beta evaluation scored A=0/B=0 at the step
limits. This is an implemented native simulation with unsuccessful reference
validation, not a measured hardware result. Exact-product physical insertion
forces remain unavailable; the related-product estimate is explicit.

Homeowners replace worn exterior-door weatherstripping to keep drafts out.
This trial fits a full-length loose seal to a head jamb during frame-parts
assembly, before installation. Two hands align and steady the seal, then press
its fin home progressively. One hand may hold the loose length while the other
works. A valid one-hand result has the same end-state score.

## Products, size and real workholding

- [Pemko Q102 manufacturer drawing](https://s3.amazonaws.com/s3-absupply-net/pdf/pemko/q102_cutsheet.pdf),
  a maker-authored PDF hosted by a distributor: Q102BL foam kerf-in seal with a
  rigid PVC fin, nominal 1/8-inch retainer width, 3/8-inch retainer height,
  5/8-inch reach, and 36-inch stock option. The maker drawing lists BL as Canada-only; current Canadian stock
  availability still needs confirmation. The dynamic maker listing was inaccessible.
- [Pease exterior head jamb](https://peasedoors.com/products/exterior-head-jamb):
  select the 31.75-inch-door / 32-inch-long option and 4-9/16-inch depth,
  primed poplar. SKU TJH324 appears across variants; select by dimensions.
  The displayed starting price is $19, not a selected-variant quote.
- [BESSEY CM30 maker brochure](https://besseytools.com/getmedia/56927ce4-da1b-447a-ac20-976b2026ac6b/BESSEY_C-clamps_EN):
  76.2 mm opening and 44.45 mm throat. Two clamps secure the back of the stop
  to an assembly deck; their bottom jaws fit below the deck, above the table.
  [Retail listing](https://www.homedepot.com/p/205512967): $7.49 each.
- [Therma-Tru frame assembly instructions](https://www.thermatru.com/globalassets/customer-support/how-to-center/documents/hpdsingle.pdf),
  section 4, fits weatherstrip to frame parts before assembling the frame:
  “Press in kerf with hand.” This documents the assembly setting, without
  asserting product interchangeability.
- [Pease fitting guide](https://peasedoors.com/pages/weatherstrip) supports a
  1/8-inch kerf and a smooth wood block for seating.

The whole 812.8 mm jamb is retained. It rests on a reusable 950 x 160 x 18 mm
plywood deck on two 40 mm-high battens. CM30 pad bottoms meet the stop top at
89.75 mm above the table; the deck-plus-stop stack is 49.75 mm. No clamp or
support touches or inserts the seal. See `fixture/fixture.md` and `fixture/jamb.svg`.
Planning cost is about $65-135 excluding an existing bench and any force gauge:
seal $10-25, jamb $20-40, clamps about $15, deck/block $10-25, simple gauges
$10-20. These are allowances, not verified complete procurement quotes.

Coordinates: x away from arm bases, y left, z up. Table z=0.7475 m; deck top
z=0.8055 m. Jamb length/depth are 812.8/115.8875 mm. Estimated board and stop
thicknesses are 19.05/12.7 mm. Slot face x=265 mm, center 25.4 mm above deck;
kerf 3.175 mm wide and 9.525 mm deep. Gauge purchased specimens before use.

The 812.8 mm seal has 34 numerical spans of 23.9059 mm. Its estimated bulb
is 15.875 x 12.7 mm, stem 9.525 x 1.5 mm, and total mass 45 g. A tapered
barb ridge is 3.5 mm long and widens from 1.5 to 3.6 mm thickness; its exact
profile is an estimate. The graspable bulb exceeds 10 mm; the thin fin is not
pinched. Ink span marks travel with the seal, away from contact surfaces.

The loose seal initially overhangs the right jamb end by 20 mm; fin tips are
6 mm short of the slot. Connected parts have the same start pose in A and B.
Only the loose 40 x 30 x 20 mm wood block moves from (175,-190) to
(175,-230) mm. Seed 0, zero object jitter, layouts A then B.

## Force estimate and physical work

The native estimate is **12 N per 23.906 mm span**. The basis is
[US8474189B1, Table 6](https://patents.google.com/patent/US8474189B1/en):
FOAM-TITE 12403 measured 11.65 N for a 25.4 mm sample in a 0.080-inch kerf.
Length scaling gives 10.96 N, rounded upward to 12 N. The paper also reports
3.16-16.15 N across related weatherseals. These are related-product data;
**none is a Q102BL measurement in a 1/8-inch kerf**. This estimate does not
close the original measurement-specific verdict.

Bulb, fin, tapered barb and slot walls have physical colliders. A compliant
contact material supplies insertion resistance and retention. D6 connections
lock stretch and provide finite estimated bending/torsional resistance:
0.03 Nm/rad weak-axis bending, 1.21 Nm/rad strong-axis bending and
0.01 Nm/rad torsion, with damping and angular limits. The strong/weak ratio
is the rectangular carrier-section ratio (9.525/1.5)^2=40.32. The bending estimate is a
0.5 N hand load at a 20 mm lever producing about 0.33 rad of bending, rather
than a measured modulus. Fold/twist and plastic damage are approximated;
a greater than 30-degree relative kink or a fin-edge opening above 3 mm
fails the whole trial.

No mechanism captures, welds, teleports or advances the seal. PhysX solves
contact and equal reaction into the clamped jamb. Per-physics-step contact
forces are read from each seal span against the actual kerf-wall colliders;
the horizontal rabbet-floor collider is excluded. Contact stiffness is
350,000 N/m with 10 Ns/m damping, selected as an engineering model gain
after native force traces, rather than a measured material property.
A span needs at least 12 N over at least 0.25 mm first forward travel, and at
least 0.00425 J insertion work before reaching home. Work uses the smaller of two consecutive interval-mean contact forces
times first forward displacement. A jump greater than 0.5 mm per 240 Hz
physics tick or an isolated impact supplies no work credit. The 0.00425 J
is an engineering estimate of a triangular 12 N force profile over 0.708 mm
axial travel: 0.2125 mm compression per barb side divided by the taper slope
0.3. The force threshold is unchanged.
Only tip depths 2-8.5 mm count; rear-stop impacts, stationary force and repeated
pumping cannot replace insertion work. Complete withdrawal permits a new
attempt. No state rule performs insertion after a force observation.

The script is open-loop from the published layout. Closed pads drag the bulb
to align the ends; the right hand presses its half while the left steadies the
loose length, then the left continues after the right withdraws. Compensation
for measured arm lag and finite-speed presses are published waypoints, not
object-state feedback. Actual load, work, deformation and release still need
native validation. Force/profile estimates and reset time remain unmeasured
on hardware.

## Scoring and controls

Score 0 after a greater than 30-degree kink or a connection opening more than
3 mm at either the bulb centerline or fin edges, and while
hand/object/force observations are unknown. Missing observations
invalidate the trial. No credit for held or moving end states. Damage is a
finite-fold proxy; actual fin crushing, yielding and wood damage are not fully
modeled and must be checked in physical enrollment.

A span counts only if both ends of its fin are geometrically home, its bulb
front gap is at most 1.5 mm, and its native force/work certificate is complete.
Mark the seal into 34 equal lengths; count those moving seal spans, not fixed
jamb intervals. Read seating with a photo/ruler and feeler gauge.

- 0.25: 9-30 spans seated.
- 0.5: at least 31 seated, including all 34 when an end is not flush or the
  final one-second hold has not completed.
- 1.0: all 34 seated, each end within 2 mm of flush, hands and tool clear by
  at least 10 mm, and the scene still continuously for 1 second.

All nonzero tiers require release and stillness. Simulation stillness is below
3 mm/s and 0.05 rad/s. Pad/palm faces are sampled at 3 mm spacing, with a
6 mm guard for sampling/thinning error. The loose tool uses exact convex-box
clearance including edge/edge approaches. Missing or stale certificates fail
closed; a pose-only completion earns 0.

`near_misses.py` includes tier boundaries, held/moving states, pad-face contact,
rotated tool proximity, pitched fins, stale and nonfinite states, hold timing,
and synthetic force-series controls. Synthetic states are not robot demos.

## Original claim and every verdict

Core idea credited to [PR2302](https://github.com/robocurve/benchmark-tasks/pull/2302),
TK-SKUUNX, revised as TK-XRZU6Q and TK-9FWS7C. This implementation is independent.
The latest original revision names FOAM-TITE 12403, while its older description
names Q102BL. This build explicitly uses Q102BL, as the candidate entry proposes.
The last verdict is preserved verbatim in `verdict.md`.

| Review request | Current treatment |
|---|---|
| Initial: simulate firm insertion resistance, measure the specified strip/kerf, show full-score demo | Actual collision resistance and contact/work scoring implemented. Exact physical measurement absent; final native A/B scores are 0, so full-score evidence is absent. |
| Second: 4 N estimate does not satisfy a measurement request | The estimate is explicitly labeled. No synthetic or related-product number is represented as a Q102BL measurement. Original request remains open. |
| Last: 0.080-inch measurement does not establish resistance in 1/8-inch kerf | Kerf remains 3.175 mm. Related data supplies an explicit engineering estimate only; exact measurement remains open. |
| Last: full-score demonstration within limit | Final native scripted A/B scores are 0; full-score requirement remains unmet. |
| Flag: 0.5 requires at least 31 links seated | Exactly 31 of 34 force-qualified spans, with 30 capped at 0.25. |
| Pre-review: real bench setting, full length, workholding | Maker assembly workflow, complete 812.8 mm head, deck and named CM30 clamps. |
| Pre-review: two-handed hold/operate, no capture fixture | Coordinated alignment/support; physical contacts and angular resistance; no seating/carry assistance. |
| Pre-review: real force and reaction | Explicit 12 N estimate, native contact force/work observations and solver reaction. Exact physical calibration remains open. |
| Pre-review: stock specimens ready in minutes | Commercial pre-cut strip; no casting, curing or per-trial fabrication. Reset target under 3 minutes is unmeasured. |

## Reset, rivals and reproducibility

Peel the strip out from an end; inspect for torn/flattened barbs or kerf damage.
Reject any kerf outside 3.175 +/-0.025 mm width and 9.525 +/-0.1 mm depth. Check
actual profile, length 812.8 +/-1 mm, and clamp seating. Lay the seal with
20 +/-1 mm axial offset and 6 +/-1 mm initial fin-tip gap; put the wood block
on its A/B outline. A measured reuse-force envelope remains unavailable.

Nearby tasks: PR3072 closes/locks a window; PR2172 operates a deadbolt; PR7 seats
a cord in an open channel; accepted PR2277 progressively pinches a zip-top track;
PR2337 closes a paint can. PR2738's rejected screen-spline repair is close and
its importance/novelty objections remain relevant. Distinction depends on
registering two free ends while inserting a separate continuous fin into a
retaining kerf, with actual resistance, not product names alone.

`versions.json` records installed native distribution metadata and framework
revisions. This does not establish compatibility or physical accuracy. CPU
component diagnostics used a different preliminary profile and are not final
native-task validation. Native trials are 2,400 steps at 20 Hz, 120 seconds;
scripted plans are under the limit. Use `--epochs 2` for A/B.

```sh
python near_misses.py
bash "$HOME/factory/gpu_run.sh" "$PWD" demo 2400 "Align the Pemko Q102BL seal with the head-jamb ends, press its fin home along the kerf, then let go." --epochs 2
bash "$HOME/factory/gpu_run.sh" "$PWD" dream 2400 "Align the Pemko Q102BL seal with the head-jamb ends, press its fin home along the kerf, then let go." --epochs 2
```

Run Dream-1 only after a full-score A/B demo, with the same final frozen source
and instruction. The native trace/demo budget is eight calls; Dream-1 is once.
Recheck lapse and rivals immediately before native work. A failed run or refusal
is reported honestly. No submission is authorized from this build lane.

## Native validation result

Final scripted source: SHA256 `3981a67724a7aab4e2721582c306e17e82c6607f8f4373c702e0ad9dde132783`.
Its native A/B demos and single Dream-1 beta evaluation used the same Python,
calibration, scene/scoring parameters and instruction. Only result/media text
and this README were updated afterward. The exact pre-results package is
archived with the local handoff.

| Evaluation | A | B | Outcome |
|---|---:|---:|---|
| Scripted reference | 0.0 | 0.0 | Policy finished; no force-qualified or geometrically home spans |
| Dream-1 beta | 0.0 | 0.0 | Both reached 2,400 control steps; no seal/kerf contact |

Dream used `dreamlabs/dream-1-beta`, `dry_run=False`, 50 inference requests
per lot. There were no errored trials or recorded abstentions. No verbal
refusal is reported. This failed-source diagnostic does not satisfy the
full-score prerequisite for a READY package.

The final scripted A/B conservative insertion peaks were 10.612/10.316 N,
work maxima 1.690/2.631 mJ, and loaded travel above the 12 N gate was zero.
Every span exceeded the 1.5 mm all-corners bulb-gap limit. Alignment also
overshot flush; final A end offsets were about +22/+24 mm. These are native
model observations, not physical force measurements.

The frozen source passed the full offline checker in both layouts and 199
synthetic scoring/evidence controls. These checks do not establish native
feasibility. A remaining defect allows a brief >3 mm fin-edge opening between
control callbacks to escape the historical damage latch. Aggregate kerf-wall
reaction and the work threshold also do not establish the whole insertion
force curve or fin/barb contact provenance. The original exact-product
measurement-specific request remains open.

Fetched failure evidence: [setup](../../../media/press-the-pemko-q102-seal-home-along-the-head-jamb/setup.png),
[scripted demo](../../../media/press-the-pemko-q102-seal-home-along-the-head-jamb/demo.mp4),
[Dream-1](../../../media/press-the-pemko-q102-seal-home-along-the-head-jamb/dream.mp4).
No public upload or submission is claimed.

Code: MIT. Documentation, original drawings and task data: CC BY 4.0.
No manufacturer PDF, photograph or third-party mesh is redistributed. Local
parts/lots derive from the standard framework, with local compatibility and
import-reentrancy fixes; shared factory files are unchanged.
