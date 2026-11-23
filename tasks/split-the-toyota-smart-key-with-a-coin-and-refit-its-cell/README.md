# Split a Toyota smart key with a coin

Drivers open a depleted smart key to replace its CR2032. The task uses an unmeasured Toyota HYQ14FBA-referenced simulation stand-in, starting with the case snapped closed. A loaded coin twist must split it while the other hand holds it clear of the table; cell exchange alone earns zero.

Current frozen native demo completed at **A0.25/B0.25, mean0.25**. Both lots reached a released0.5 from78.1 to83.1s, then lost the seated replacement during the folding/closure sequence. Neither contact latch reclosed. The single Dream-1 completed at **A0.0/B0.0, mean0.0**,2400 steps per lot, two epochs, both ending at the step limit. Its evaluation status is success with zero errored trials/abstentions, but98 expired-response warnings limit interpretation; no explicit refusal was recorded. Policy transcript counters report1882/1867 action rows beyond limits out of2400 returned rows in A/B respectively. No instruction reword or rerun occurred. The launch wrapper returned143 after completed evaluation and media generation; the saved evaluation and artifacts were checked independently. No hardware trial or RF result exists.

| Frozen scripted demo | A | B |
| --- | ---: | ---: |
| Independent final score |0.25|0.25|
| Peak paired-jaw coin torque |0.1369N m|0.1496N m|
| Contact work at detent release |3.67mJ|4.32mJ|
| Spent cell in tray at end |yes|yes|
| Replacement retained / contact latch closed |no / no|no / no|
| Sampled simulator finger-load diagnostic |39.2N|324.0N|

The324.0N sampled simulator load exceeds the intended40N diagnostic threshold, which is not a verified hardware rating and is a known feasibility failure. A0.5 snapshot is not retained as the final score. Force samples are not a hardware certificate, and no calibrated physical equivalence is claimed. The complete refit remains unsolved. Runtime source SHA256: `f98def23e78ef5304089f2733279c5d1f4b4a3ea190d82d17e02ed55c8463586`.


## Products and trial cost

- [Stallion R-SHELL-79 / Toyota HYQ14FBA shell](https://stallionkeys.com/products/2012-2020-toyota-3-button-smart-key-shell-hyq14fba-aftermarket), $8 checked Nov23. The current seller title and product record identify four buttons; the legacy URL still says three-button. Confirm the four-button SKU when enrolling a physical specimen. The70x40x15mm envelope and30g case/board mass are estimates, not a measured OEM drawing.
- [Toyota electronic-key procedure](https://techinfo.toyota.com/t3Portal/document/om-s/OM33A28U/pdf/OM33A28U.pdf) describes removing the mechanical key, prying the smart-key cover with a protected flat screwdriver, fitting a CR2032 plus up and closing. The quarter is an alternative flat edge, not a claimed Toyota recommendation for this exact shell.
- [US Mint quarter](https://www.usmint.gov/learn/coins-and-medals/circulating-coins/quarter):24.26mm diameter,1.75mm thick,5.670g. It is gripped across its rim while upright in a slotted rest; there is no added gripping tab.
- [Energizer CR2032](https://energizer.com/wp-content/uploads/2025/01/cr2032.pdf):20x3.2mm,3g. The green-marked replacement sits on an8mm square pedestal for a real rim grasp. The old red-marked cell starts inside. Neither cell is sprung.

Five shells$40, cells about$8, trays/mat/rests about$15 and a quarter$0.25 give about$65 for the inert specimen kit. Robot and enrollment instruments are separate. No purchase has been made. Fix the spent tray and support blocks to the mat with removable tape during one-time preparation; coin/new-cell trays remain movable and ballasted to150g. Do not attach the case, coin or cells. Reset target is under two minutes, not a measured human baseline.

## Current model and physical work

Asset/configuration version6 is in fixture/model-v6.json, notch-v6.svg and rests.svg. Older snapshots are retained for development history. The shape is a functional stand-in: two ABS halves with a captured end hinge instead of detachable halves, a26-to-20mm flared notch with2.8mm opposed-jaw gap, and a21.2mm open-sided circular cradle on an inert46x26x1.6mm board. Its top is at the case seam. These details need comparison with a stock shell; cell retention and electronics are simplified.

Opening estimate:10N at the quarter's12.13mm radius gives approximately0.12N m. Across a16mm coin contact span this is7.5N separating load; at the modeled64mm cover lever it gives0.48N m hinge preload. The native drive has8N m/rad stiffness and a-0.06rad target before release. Every release requires actual contact with both notch jaws, opposing coin moment>=0.12N m, useful cover-opening moment, loaded left/base and right/coin contacts, the entire case>50mm above the table, >=2degrees net turn during continuous contact, and >=1.984mJ signed contact work. Tiny reversals return work. The hinge transmits reaction to the held base. No weld, pose write or proximity-only latch performs the split.

Closing estimate: a1.2mm elastic snap deflection at a60mm lever gives0.02rad travel. Native implicit drive stiffness36N m/rad and damping0.05N m s/rad yield0.72N m elastic torque at closure and7.2mJ stored energy. The damping is near critical for the estimated12g cover inertia1.7e-5kg m^2. The contact latch additionally requires>=12N signed compressive right-hand cover contact and>=0.72N m closing moment at q<=0.0045rad (maximum seam gap0.5mm). Reopening after scored reclosure is not modeled. Shell forces, dimensions and friction remain unmeasured estimates.

The scripted plan uses an end pinch for the loaded split. It then levels and sets the case on two passive10x30x21.5mm table rests, pauses before releasing, and regrips its back end corners while lifting the freed cover by its rim. The right wrist turns with the cover. A small fixed wrist correction and lowering step seat the base on the broad rests before full jaw release. The left withdraws for cell insertion, then steadies the case during folding and a gradual closing press. All actions are fixed waypoints from the published visible layout. The demo never reads live object poses, joint states or latch flags. Fixed layouts permit memorized replay.

Rigid bodies and articulation use128 position/16 velocity solver iterations, with1m/s maximum depenetration velocity. Numerical settings are not extra mechanical force. The quarter is a32-sided convex prism. No RF/LED operation, calibrated shell equivalence, hardware feasibility or measured robot result is claimed. Enroll five stock shells with calipers and opening/closure force measurements before interpreting this as a physical benchmark.

## Scoring and failures

The final state decides the tier. Score0 without a witnessed loaded coin split, after a whole-trial mat-bounds violation, with missing/stale observations, or without final release/rest. Then0.25 for the split;0.5 also for the old cell removed and replacement seated plus up;0.75 also for a contact-latched flush cover;1 also for the old cell in its tray. Wrong replacement polarity caps at0.25. No best earlier score is retained.

Seating tolerance is2mm radially,1mm in height and12degrees in face-normal alignment relative to the case. The actual maximum seam gap must be<=0.5mm. Hands must clear all task parts by5mm. A1mm/1degree rolling pose window over1.1s, part speeds<=3mm/s and0.05rad/s, cover speed<=0.02rad/s, and a continuous1s clear/rest hold are checked. Video is needed for the loaded split; a final photograph alone cannot establish its force history. The old cell must have been witnessed >30mm from its seat and must finally have its centre >=11mm outside the70x40x15mm base envelope. Native telemetry verifies simulated contact force/work; an instrumented edge/press measurement and stock-shell enrollment would be needed for physical equivalence. No such measurement has been made. The21 standalone counterexamples use fabricated states and validate grading only, never simulated task success.

## How this differs from close tasks

- PR 2410 / TK-3RDX7X (clock, final as overlap with dexhand Remote Battery Swap): removing and inserting a cell alone scores zero here. The prerequisite is loaded coin-edge torsion that splits a held snap case; a sliding door or open cell socket bypass does not satisfy it. This distinction is a proposal, and reviewers may still judge overlap.
- PR 1838 / TK-CDVB8C (server CR2032, final for oversimplification): that task already included a board, latched socket, polarity and a full-score demo. Those details alone do not distinguish this task. The proposed difference is opening the enclosing shell through loaded coin-seam torsion while the other hand resists it. The shell and retention geometry remain unmeasured stand-ins, so the same fidelity objection remains unresolved.
- PR 1165 / TK-DXM2J8 (smoke alarm, final): no abstract battery blocks or best-score retention; the old disc starts inside the estimated70x40x15mm stand-in case, the replacement must occupy its well plus up, and the released final state controls credit.
- PR 2816 / TK-HLT4ZT (accepted Safety): starts open, uses a screwdriver engaging a physical slotted bayonet cover, then tests safe remote placement. This task starts snapped closed and requires the loaded coin-pry split against the holding hand. Tool-driven compartment work and closure overlap; the proposed difference is the two-handed seam split, not the later placement or closure. The category is Societal relatability.


## Reproduction

Recorded runtime: IsaacSim6.1.0.0, IsaacLab17.0.2, IsaacLab-Arena0.3.0, yam-arena0.1.0 (source0059d02a1a91bc6834c1d0846022891df0c78730), Inspect Robots1.0.0, inspect-robots-yam0.38.0 and inspect-robots-dream0.1.0. Native physics dt=1/240s; control20Hz. The evaluation record has seed0. The task itself has no RNG and uses fixed A/B arrays with object_jitter=0. Offline checker library source isbd62771e60320433893bd1d2df0fc79b5ddcd6d1. Use the station embodiment and two epochs. A and B move only the rigid coin/new-cell presentations; the jointed case has the same start pose. Coordinates in task.json are nominal reset positions; gravity settles parts onto the mat and supports before manipulation.

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 --max-steps 2400 --epochs 2 --instruction "Hold the key in the left hand and split it with the coin in the right. Replace the CR2032 plus up, press shut with the right, old cell in tray, then release all." --scorer task_score
```

The run limit is2400 control steps (120s) per lot, averaged acrossA/B. See fixture/README.md for drawings and enrollment. Geometry uses generated primitives; no third-party mesh is bundled. Code MIT; documentation CC BY4.0.
