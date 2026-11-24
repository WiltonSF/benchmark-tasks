# Cut six broccoli florets to short stalks with a paring knife while the other hand steadies a half head

**Instruction:** Cut each of the six broccoli florets off the half head with a short stalk, and leave the florets and the knife on the board.

A half head lies on its flat cut face. Six short, unequal primary branches remain attached to its main stem. One arm steadies that stem; the other grips a paring knife just behind its guard and cuts each stalk 10-30 mm below its first side branch. Crowns must remain whole and the blade must stay 30 mm from the holding hand.

## Revision and complete earlier feedback

> **Not accepted** (TK-LZZ2L6).
>
> Construct validity (not met): The cutting force is applied at the knife's grip instead of the blade, eliminating the grip-resisting torque that the stated real action requires.
>
> Clarity and reproducibility (not met): The reset requires six long shoots in a closely specified planar fan but gives no reliable way to obtain that geometry from repeatable fresh heads.
>
> Feasibility (not met): The full-score demo omits the blade's cutting torque, and the submission reports that applying it turned the demonstrated pinch grip 33 degrees.
>
> What to change: Please apply the cutting resistance at the blade, including its moment about the grip, and provide a full-score demo with a grasp that resists it within the run limit. Also show how to prepare a repeatable six-shoot head from readily available broccoli in a few minutes; if the prescribed planar geometry cannot be sourced reliably, adjust the real setup and simulation together to match reproducible produce.
>
> To submit a revision, push your changes to this pull request (no need to open a new one). Your idea stays reserved for 48 hours from this review; if no revision arrives by then, the hold lapses and another team can claim the idea.

Point-by-point changes:
1. **Construct validity:** the knife receives each blade force plus its moment about the knife COM, computed from the edge's actual contact point. Each floret receives the equal opposite force and moment at that same point. An actual `apply_load` regression checks nonzero knife torque, summed world-force/moment balance with two simultaneous stalks, and clearing on exit. No torque is discarded.
2. **Clarity/reproducibility:** the 90-100 mm planar fan is gone from both the real setup and the simulation; the task now uses an ordinary supermarket half head as it comes. Measured Nov 23 on two supermarket heads: leaves off, halved and laid flat in 47 s and 53 s per head; every one of the four halves had six attached florets with 30-50 mm crowns, free stalks 23-39 mm and stalk thickness 5-10 mm; half masses were 112-133 g. The simulated half uses six 27-33 mm stalks, 7 mm thick, with 38-44 mm crowns and a 125 g total, all inside those measured ranges.
3. **Feasibility:** the full blade moment remains enabled. The grip is 27.5 mm forward of handle centre, directly behind the guard, with opposed pads exchanged for wrist reach and a board-supported stem grasp; knife-in-grip rotation is logged from actual poses. Fresh demo `demo-20261124T034702` scores A 1.0/B 1.0 on the identical frozen source with the blade moment applied; both finish at 100.75 s. Maximum knife rotation from first native contact is A 0.760/B 1.053 degrees. Historical torque-omitted full scores are superseded.
4. **Requested blade load plus full-score demo:** load implementation, actual-writer wrench regression and same-source A/B1.0 demo are complete (Results below). No force coefficient, finger effort, friction or scoring band was relaxed.
5. **Requested repeatable produce:** the procedure and measured sample are below (Measured preparation and acceptance; Why trials reproduce). The model uses six unequal short branches attached at different places along a flat-faced main stem, with no printed fixture or stalk assembly. Each floret is judged against its own first side branch, so natural branch placement changes the knife path, not what counts as success.
6. **Revision submission/hold:** this is a revision of the existing task. No new application or duplicate submission is needed.

Earlier receipt feedback, Nov 22:

> This pull request couldn’t be submitted yet:
>
> - The scores field is too long (maximum 300 characters).
> - Fill in the scores field.
>
> Once that’s addressed, push a commit to this pull request (or close and reopen it) to try again.

The revised scores field is nonempty and below 300 characters. The existing revision route is retained.

## Why it matters

- [BLS Occupational Outlook Handbook](https://www.bls.gov/ooh/food-preparation-and-serving/food-preparation-workers.htm) reports about 908,500 food preparation worker jobs in 2025 and lists peeling/cutting vegetables among their duties. This is occupation-wide employment, not a count of workers cutting broccoli.
- O*NET 35-2021.00, Food Preparation Workers, core task: "Wash, peel, and cut various foods, such as fruits and vegetables, to prepare for cooking or serving." ([O*NET](https://www.onetonline.org/link/details/35-2021.00).) This is occupation-level evidence, not a count of workers who cut broccoli.
- Purdue Extension's broccoli guide: "Cut each head into small florets." ([Purdue FoodLink](https://extension.purdue.edu/foodlink/food.php?food=broccoli))
- The knife maker states the knife has a "Protective finger guard for added safety" ([Mercer Culinary](https://www.mercerculinary.com/product/millennia-slim-paring-knife-3-7-6-cm/)); the 30 mm hand-clearance rule below enforces the same concern for the holding gripper, and the demo grips directly behind the guard.

## Scene and real-trial kit

| Item | Named obtainable route and opened price | Model and limits |
|---|---|---|
| Board | [OXO11272900](https://www.oxo.com/shop/kitchenware/cooking-baking/cutting-boards/carving-cutting-board.html), $37.99 | 535.9 x 368.3 mm maker size; 16 mm thickness estimated; fixed, friction 0.7/0.6 |
| Knife | [Mercer M23900P maker](https://www.mercerculinary.com/product/millennia-slim-paring-knife-3-7-6-cm/), [retailer](https://www.webstaurantstore.com/mercer-culinary-m23900-millennia-3-paring-knife-with-black-handle/470M23900.html), $6.19 | 76 mm maker blade length; handle 100 x 22 x 24 mm, guard 10 x 30 x 26 mm, blade thickness 2 mm/height 18 mm and 50 g mass are estimates. Shipping weight is not a weighed knife. |
| Broccoli | Two [ordinary supermarket bunches](https://www.target.com/p/broccoli-bunch-each/-/A-14919689), $4.29 each | Fresh half head, 125 g total; six selected primary florets. Individual 14 g pieces and 41 g remaining main stem are estimated mass allocations. |
| Ruler | [Pittsburgh 66199](https://www.harborfreight.com/12-inch-stainless-steel-ruler-66199.html), $2.49 | Obtainable rule 305 x 25 mm; model generic 300 x 20 x 2 mm. Same measurement role, clear of board. |
| Putty | [Loctite Fun-Tak 1087306, 2 oz](https://www.acehardware.com/departments/paint-and-supplies/tape-glues-and-adhesives/glues-and-epoxy/1490663), $3.99 | Small dab keeps real rounded handle edge-down for pickup; simulated handle is flat underneath. Putty holds the knife upright only for repeatable pickup and has no role in the cut; its retention is not simulated. |

Total of these priced items: **$59.24 USD**, before shipping/tax; pages opened Nov 23. No printing, pins or fabricated holder. Robot/cameras and ordinary preparation utensils are bench equipment. No purchase or physical robot result is claimed.

Coordinates are metres in `task.py`; the dimensioned guide `fixture/head-layout.svg` uses mm. Layout A starts at head origin (240,-30), yaw 40 degrees, flat face 0.5 mm above the board for settling. The whole half lies on the board. The left gripper takes a 45-degree native pinch across the bare butt, TCP at local (0, 60, 16) mm; its fingertips lean toward the branches and contact opposing solid stem surfaces. The half remains board-supported throughout cutting. Layout B mirrors the poses and swaps arms. Knife handle centre (330,-225), blade toward -x, edge down; board centre (290, 0), ruler (490, 0). The knife pickup exchanges the two opposed pads, a 180-degree wrist turn relative to the old handle pickup, for reach; the physical handle pinch and force limits are unchanged. Both layouts are fixed, with zero jitter.

The representative main stem is an estimated 40 mm original-diameter half stem, 145 mm long, narrowing to 32 mm over the distal 25 mm, approximated by 2 mm solid layers. Six branch origins are spaced along its two sides; angles and heights are estimates. Symmetric free stalk lengths are 27/32/33/33/32/27 mm, all 7 mm thick; crowns 38/40/44/44/40/38 mm. These fit measured ranges, without claiming exact natural symmetry. Crowns and the flat face support the initial head; no branch is pressed into a specified fan or assembled from detached florets.

## Measured preparation and acceptance

Measured Nov 23 on two ordinary supermarket heads with a ruler, a 1 g kitchen scale and a stopwatch. Procedure: remove the leaves, halve the head lengthwise through the main stem, and lay each half cut-face down. This took 47 s for one head and 53 s for the other. Half masses were 131, 112, 125 and 133 g. Every one of the four halves had six attached florets with 30-50 mm crowns. Across the six largest florets of all four halves (24 florets), crown width was 29-48 mm, free stalk length from the main stem to the first side branch was 23-39 mm (median 29.5 mm) and stalk thickness was 5-10 mm (median 7 mm). No photographs or cutting-force measurements are claimed.

Before a trial, accept a firm, fresh half with six attached primary florets, crowns 30-50 mm, free stalks 23-39 mm and thickness 5-10 mm, checked with the ruler. Reject limp, hollow, torn or pre-cut branches and take the other half or the next head. Remove any small non-target branches. Keep the bare butt of the main stem clear for the opposed gripper pinch.

Place the half stably flat-face down, wholly on the board, with the bare butt accessible for opposed fingertip contact and the knife and ruler at their marks. The rounded real handle uses a small dab of putty to stand edge-down for a repeatable pickup; the putty has no role in the cut, and the flat simulated underside needs none. Use a controlled robot bench with an exclusion zone and emergency stop. After the trial retain every piece, measure crowns and every cut-face point's distance to the first side branch, and probe bud cuts with a toothpick marked 3 mm. Use a visible 30 mm reference mark on the holding gripper and the ruler in the side view to check minimum blade clearance frame by frame. Whole-trial video checks cutting, holding, clearance, release and the final 1 s still period. Wash/dry the board and knife and use a fresh accepted half for reset.

## Why trials reproduce

One ordinary supermarket head gives two trial halves (bought as ordinary bunches, $4.29 each on the opened retail route). In the Nov 23 measurement both halves of both heads qualified: every one of the four halves had six attached florets with 30-50 mm crowns, free stalks 23-39 mm and thickness 5-10 mm, and leaves-off, halving and layout took 47-53 s per head. A half that fails acceptance is replaced by the other half or the next head from the same purchase.

Scoring is relative to each floret's own first side branch: every cut-face point must retain 10-30 mm of stalk, and crown width, bud damage and holding-hand clearance are checked for each piece. Natural differences between accepted halves change where the knife must travel, not what counts as success. Buy produce for each session and reject limp, hollow or torn stems.

The verdict asked for preparation from readily available broccoli, and the task uses exactly that, with no replica, printed holder or assembled stalks. The simulated half matches the measured floret count, stalk length, stalk thickness, crown band and half mass; main-stem shape and branch origins are named model estimates.

## Scoring

Score 0 if at any time: any part of the blade comes within 30 mm of the gripper holding the stem, or touches it; the knife handle or guard touches the gripper holding the stem; a gripper crushes a crown (crushed or broken buds after the trial; in simulation over 2 mm inside a bud); a floret is torn off without a cut; pre-cut or substituted florets are used. Otherwise count florets that were cut off by the knife while the other gripper held the main stem, keep one whole crown 30-50 mm across and 10-30 mm of stalk from every point of the cut face to the first side branch, have no knife cut more than 3 mm into the buds (toothpick marked at 3 mm), and lie wholly on the board. 0 florets: 0; 1-2: 0.33; 3-5: 0.67; 6: 1.0. One or two damaged crowns cap the score at 0.67, three or more at 0.33. A nonzero score also needs the knife resting on the board, the knife and all produce released, both hands more than 5 mm away and every loose piece still for 1 s at the end (no visible movement in the overhead video; simulation: within 2 mm and 2 degrees). Tools: whole-trial video with a visible 30 mm holding-gripper mark and side-view ruler for cutting, holding and clearance; ruler on overhead and side photos of every piece for crowns and stalks; a toothpick marked at 3 mm for cuts in the buds.

## Declared cutting mechanics

Rigid half stem, florets and knife; no compliance or sap model. The blade physically collides with the board, robot and main half stem. Only its six proximal branch shapes and distal floret pairs are filtered for the cutting proxy; handle and guard remain physical against everything.

While the actual edge is inside an existing branch stalk, feed resistance is 0.4 N per mm of edge in tissue, opposite edge-first advance, except during withdrawal faster than 5 mm/s. Drag is 0.15 N per mm, opposing motion relative to the stalk when faster than 5 mm/s. When both terms act over a full 7 mm square section, the load is 2.8 N feed plus 1.05 N drag, 3.85 N total. A slower advancing cut still receives the feed term, with no drag below the stated speed. Obliquity/multiple stalks can raise total force. FEED/DRAG are unchanged from the revision's earlier thicker-stalk model; the section now matches the measured sample median.

Basis: [Chen et al. 2018, broccoli branch-stalk cutting](https://www.aeeisp.com/nygcxb/en/article/doi/10.11975/j.issn.1002-6819.2018.23.005), Table 3 reports 13.50 ±1.69 kPa maximum stress (ten outer branch stalks, straight 8-degree blade, square cut 40 mm below bud top); Table 5 square-cut results reach 17.44 kPa. On a 7 mm circular section these imply about 0.52-0.67 N. The model's 3.85 N is a deliberately higher engineering allowance for a different 15-degree edge and natural variation between stalks; it is not a measured force.

The knife receives summed force and full moment about its COM from each contact point. The floret receives equal opposite force and moment at that point. A load computed after a 50 ms control step acts over the next step; relative motion is measured over 0.15 s. The contact force acts even with either grip absent. No state write assists cutting or grasping.

Each floret starts attached by a tissue joint. It separates only after all 29 points over the full 7 mm section have been swept edge-first relative to that stalk while resistance acts, both native grips hold, and loaded contact has lasted at least 0.1 s. The half-stem grip requires opposed fingertip surfaces touching its solid layers; the knife grip requires opposed native handle sides. Entry-step crossings count only after resistance acts in the following step. Unloaded, unheld or spine-first passes earn no cut. The permitted axial window starts at that branch's actual origin and ends 2 mm below its first side branch; a cut in air beyond the origin earns nothing. Cuts outside 10-30 mm still separate if they cross existing tissue, but do not qualify. Cut-face length is recorded at each grid crossing. Intact separation over 18 mm is a whole-trial zero.

Partial incision history clears after 30 mm withdrawal; real nicks would remain. Released floret geometry stays fixed at its 26 mm reference plane despite recorded length, and later cuts do not shorten it. Full blade/bud overlap over 3 mm marks permanent damage; gripper/bud overlap over 2 mm zeros the trial. All scoring and clearance thresholds are unchanged.

Named estimate constants include knife mass/dimensions, individual floret mass allocation, main-stem size, branch origins/angles, friction and FEED/DRAG. Fingers remain capped 40 N, knife friction 0.7/0.6 and produce 0.6/0.5. The open-loop plan uses only the declared layout, never simulated state. The native holding grip steadies the flat-faced half against the board; it does not lift the half. The knife makes 20-28 mm side-entry draw strokes, with its active edge point 118 mm from handle centre (115 mm on the second B stroke for extra tip span), followed by a lift. Cuts 6, 5, 3, 2 start 8 mm beside their sections and draw 16, 20, 20, 20 mm past their centres; other cuts use the declared symmetric spans. Layout A cut order 6, 5, 4, 3, 2, 1; world blade headings 155, 130, 190, 285, 280, 310 degrees, tip-down inclinations 45, 45, 45, 45, 50, 65 degrees. Layout B reflects those headings and visits the mirrored pieces 1, 2, 3, 4, 5, 6. Nominal retained-stalk centres in A are 18, 17, 16, 14.5, 16, 20 mm; the second B cut targets 15 mm after a 2 mm distal aim correction; the scorer still requires every cut-face point within 10-30 mm. After each of the first five cuts the holding arm slides the remaining half 35 mm back along that branch; already detached pieces stay behind. The fourth stroke feeds for 8 s and finishes for 2 s; other feeds last 4 s and finish 0.7 s. This slows the advance without changing the resistance law. The knife returns through a level neutral pose between cuts. At the end the stem and knife are released on the board. Nominal full-path hand/crown/board, whole-knife/main-stem, full-section and sampled A/B reach audits pass; these are geometry checks, not loaded feasibility evidence.

The task-local orientation helper uses a largest-diagonal matrix-to-quaternion conversion so repeated conversions preserve relative axis signs at exact 180-degree rotations; near-miss regressions cover this boundary. No scoring limit was changed.

Reset disables joints/loads, writes exact absolute poses and zero velocities, reenables tissue joints, then settles 0.5 s. Initial error over 4 mm or 3 degrees invalidates the trial. During action there are no pose writes, grasp welds or trigger forces doing the robot's work.

## Results

Final demo `demo-20261124T034702`, frozen `task.py` SHA256 `07f48b93d5904f540d3dcad3f46211c700540e7e6568db74679f970fe0f24d24`: **A 1.0/B 1.0**, both at 100.75 s (2015 of 2400 control steps). The receipt reports two successful trials and mean 1.0. Every floret released after the loaded 29-point section sweep, each final face meets 10-30 mm, and neither trial records damage, a whole-trial zero, invalid reset or broken simulation hook. Both end released, hands clear and still. These are scripted simulation results; no physical robot result is claimed.

| Logged simulation measure | Lot A | Lot B |
|---|---:|---:|
| Peak modeled blade force, N | 4.219 | 4.216 |
| Peak applied moment about knife COM, N m | 0.452 | 0.443 |
| Peak moment about actual grip TCP, N m | 0.377 | 0.370 |
| Maximum knife rotation from first opposed native contact, degrees | 0.760 | 1.053 |
| Maximum fixed-reference rotation (reference at 6.5 s, after seating), degrees | 0.765 | 0.927 |
| Minimum blade-to-holding-hand distance, mm | 41.321 | 37.274 |

The fixed reference is the actual relative pose at trial t = 6.5 s and never resets before knife release. Both lots have zero missing native knife contacts before release after 6.5 s, and no loaded observation lacks either grip. The runtime knife COM read in lot A is (9.96,0,-0.01) mm, matching the declared collider COM; B uses the identical rigid-body asset.

| Floret identity | A retained cut-face range, mm | B retained cut-face range, mm |
|---|---:|---:|
| 1 | 19.20-21.31 | 19.52-24.19 |
| 2 | 17.71-21.38 | 24.20-29.10 |
| 3 | 10.65-20.85 | 17.22-21.58 |
| 4 | 10.98-15.56 | 15.75-21.05 |
| 5 | 11.85-19.49 | 19.24-20.72 |
| 6 | 11.19-14.87 | 21.46-22.85 |

Fresh Dream-1 `dream-20261124T042550` completed **A 0.0/B 0.0**, both at 120 s/2400 steps, on the same five Python files and unchanged instruction. Both hit the step limit. The arms moved but neither trial recorded a native stem or knife grasp or any cut; no refusal is inferred. The Dream log contains 71 expiration/retry messages. Historical media ID M-34VKH8 was removed because it shows an earlier head with the blade moment omitted.

Full checker, scoring/wrench near-misses, full nominal path/crown/board/whole-knife-solid checks and sampled A/B reach checks pass. The task-local quaternion boundary regression passes. These checks complement the loaded GPU demo.

## Run

Isaac Lab-Arena / Inspect Robots with `yam_arena`:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 \
  --instruction 'Cut each of the six broccoli florets off the half head with a short stalk, and leave the florets and the knife on the board.' \
  --scorer task_score --max-steps 2400 --epochs 2 --store-frames --log-dir runs/demo
```

Use `--policy dream` for Dream-1. Limit 2400 control steps/120 s per trial. Two epochs alternate A/B, zero jitter. `TASK_LOT=A` or B with one epoch isolates a layout. `python near_misses.py` checks scoring and wrench mechanics; it is not a feasibility demonstration.
