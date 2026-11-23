# Secure the flexible cord under the terminal-box saddle

Electricians install and repair electrical wiring and equipment; cord anchorage keeps an external pull off the terminal connections. [O*NET 47-2111](https://www.onetonline.org/link/details/47-2111.00) reports 821,000 US electricians in 2025. [HSE HSG150](https://www.hse.gov.uk/pubns/priced/hsg150.pdf), paragraph 484, requires that “the outer covering (sheath) of the cable is gripped” where it enters equipment. This task covers that anchorage step, with the circuit disconnected. It does not test electrical certification or claim that all electricians use this particular saddle.

A connected cord springs outward unless held. One hand advances and holds its sheath while the other alternately tightens two captive screws with a socket driver. The finished assembly must be released, retain the cord under a measured pull, and leave the driver in its tray. Category: Impact on jobs. The task does not test a hazardous instruction or robot-building procedure.

This is an independent rebuild of PR 2691, TK-3U8RC9; none of that submission's code or assets is used. The supplied code is an Isaac Lab-Arena simulation candidate, with no hardware trials. Native evaluation status appears below.

## Parts and real-trial route

- [Hammond 1550G](https://www.hammfg.com/part/1550G), [maker drawing](https://www.hammfg.com/files/parts/pdf/1550G.pdf): 222 x 146 mm footprint, nominal 51 mm product-page height. The drawing's section and lid dimensions differ; verify the actual lidless envelope before fixture construction. The scene uses a 51 mm wall envelope, simplified 4 mm walls and a 4 mm floor stack (2 mm casting plus 2 mm liner). Planning allowance US$35.
- [Prysmian H05VV-F catalogue](https://de.prysmian.com/sites/default/files/PGDE_POWERCORDS_EN_24pgA4_v2_lowres.pdf): 3G4 stock, nominal 10.90 mm OD, 215 g/m. Cut 295 mm once, remove 65 mm outer sheath without nicking the cores, leaving a 230 mm sheathed tail. Allow US$10 for spare prepared stock.
- [Wera 811/1 Stubby](https://hybris-media.wera.de/download/pdfgenerator-datasheets/en/05008880001.pdf), 65 mm holder, 54 mm handle, 44 g; [869/4 M 7x50 nutsetter](https://hybris-media.wera.de/download/pdfgenerator-datasheets/en/05060422001.pdf), 20 g, 11 mm OD. Estimated 10 mm insertion gives a 105 mm assembled tool. Verify assembled length, 34 mm handle envelope and socket fit before enrollment. Allow US$30.
- The custom aluminum saddle, steel bar, mounting pad, terminal anchor, tool rests, tray and M4 hardware are specified in [fixture/fabrication.md](fixture/fabrication.md), [assembly drawing](fixture/assembly.svg) and [driver profile](fixture/driver-profile.svg). The saddle is not a Hammond accessory. Machine the reusable socket entrance once to the specified 1.5 mm taper; do not substitute an unspecified stock chamfer. Fixture/material allowance US$70; terminal hardware and measuring tools including a dial indicator US$55. Approximate total US$200, planning allowances rather than purchase quotes.

The mounting pad is recessed flush in the bench and clamped down; enclosure, saddle and terminal anchor remain bolted throughout. The enclosure walls constrain access. Both tool rests and the tray exist in both lots. Only the driver's initial rest changes: A (155,185) mm; B (215,205) mm. The jointed cable and screws retain the same initial pose.

Reusable pieces are fabricated once. Keep several pre-cut cord specimens. The planned reset and enrollment is 2-4 minutes per trial, not yet timed on hardware: disconnect, inspect, restore the screw-head undersides to 36.3 mm; allow the free bar to settle and check approximately 0.58 mm head gaps and 9 degrees roll, form the insulated-core bows on the template, check 3-6 N holding force at +18 mm, and check three releases return below +5 mm within 2 s. Restore the sheath to -1 mm. Reject a specimen that fails enrollment; do not change scoring or add an external spring. There is no curing or fabrication per trial. See task.json and the fixture document for the full sequence and dimensions.

Use only room-temperature disconnected parts. Deburr metal edges and inspect tool, cable and insulation before each trial. The pads grasp the smooth tool handle and PVC sheath; reject any sharp or damaged specimen before using the shared rig. This is an inspection/preparation requirement, not a simulated damage guarantee.

## Scoring

The full numeric contract is in task.json. Score zero if the bar leaves its screw guides, a part falls off the table, hand/contact state is unknown, or the sheath fails to cover the terminal-side bar edge by 5 mm. Physical trials also reject visible insulation cuts or exposed conductor away from terminals. No force or turn count predicts insulation damage.

Every tier requires hands and tool clear of the clamp by 6 mm, with cable, bar and driver released and still for 1 s. A ruler and close photograph read coverage; a 0.2 mm feeler checks head seating. Tiers are 0.25 for one seated head, 0.5 for both, 0.75 for retaining a 30 N pull ramped over 1 s and held 0.5 s with at most 1 mm movement in either direction, and 1.0 for that result with the bar ends within 1 mm in height and the driver wholly inside its tray. Use a spring scale and before/after marks for slip, and a depth gauge for bar heights. Score each lot independently and average them.

The proof starts only after the one-second released hold. The prescribed load may cause elastic motion within the unchanged 1 mm slip limit; all tiers again require one second still after load removal. Later hand/tool approach, screw motion or assembly movement invalidates the completed proof. The simulator resolves fine motion at 240 Hz; equivalently measuring the 0.1 mm post-proof displacement rule on hardware requires a dial indicator, in addition to the ruler, photograph, feeler and scale. A plain video alone is insufficient for that fine invalidation rule. The proposed measuring equipment and physical specimen enrollment remain unvalidated.

## Contact model and controller

The cable's connected-core bow is approximated by an axial 250 N/m spring: 4.75 N at sheath +18 mm (19 mm travel from the -1 mm start), to be checked against physical stock. The 230 mm tail is rigid and guided through the entry; only physical contact moves it inward. The fixed terminal root reacts the restoring force. This is not a validated flexible-cable material model.

M4 screw travel is coupled natively to rotation at 0.7 mm/turn. The joints permit five turns (3.5 mm); the slider permits 3.6 mm. Native bidirectional Coulomb friction supplies an estimated 0.05 Nm resistance with zero advancing effort. Limits remain fixed during manipulation; there is no one-way ratchet. [NASA RP-1228](https://ntrs.nasa.gov/api/citations/19900009424/downloads/19900009424.pdf) describes the KFd relation; estimated K=0.2, d=0.004 m and 62.5 N preload motivate 0.05 Nm. This is not a measured tightening specification. Six colliding hex-socket walls transmit torque through real contact; straight axial pushing would require approximately 449 N to backdrive the ideal thread at that resistance.

Bar/cable friction is estimated at 0.60 static / 0.55 dynamic; [LANXESS material selection guide, Table 3-1](https://techcenter.lanxess.com/scp/americas/en/docguard/Material_Selection_Guide.pdf?docId=77017) provides a PVC/steel basis, not a measurement of this cord. Estimated normal compliance is 100 kN/m with 100 Ns/m damping, representing jacket, strand bedding and bar compliance together. Carrier links are 5 g; the bar is 16.5 g; the tool is 64 g. TGS uses 64 position / 1 velocity iterations. No mass inflation, extra supporting constraint, pose correction or capture trigger performs the fastening.

The scripted controller uses fixed Cartesian waypoints from the published visible lot layout. It holds the sheath against spring-back, alternates thirteen 120-degree strokes on each screw with physical disengagement between strokes, releases the cable and returns the driver. Fixed alignment compensation comes from prior visible alignment measurements, not hidden completion or live joint-state feedback. The full plan, including the runtime ready motion, takes at most 165.5 simulated seconds (3,310 actions), within the 3,400-step limit; evaluation can stop earlier on validated success.

The scorer reads native contacts and checks every robot collision link at each 240 Hz physics step. Stillness uses unsmoothed successive body poses and cable displacement at the original 3 mm/s and 0.05 rad/s limits, with a transient-motion latch across control samples. Native solver velocity fields are retained diagnostically: under loaded compliant contacts they reported much larger motion than the measured per-step transforms. Pose-update counts are diagnostic; freshness relies on the native PhysX buffers read immediately after each physics step. Nonfinite data fails closed. No independent stale-buffer detector is claimed. This measurement change is not a loosened stillness threshold.

The pull applies a physical 30 N outward force to the cable. The enclosure/terminal mounting reacts it. Starting the pull never grants credit: actual slip, guide retention, screw motion, hand clearance, post-load settling and unchanged final geometry determine the score. No fracture or electrical damage is simulated.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "Seat the cable sheath beyond the clamp, secure it with both screws, return the driver to its tray, and leave the cable and driver released." --scorer task_score --max-steps 3400 --epochs 2 --store-frames
```

For Dream-1 replace `--policy yam_scripted` with `--policy dream` and `--max-steps 3400` with `--max-steps 2400`; keep instruction and `--epochs 2`. Evaluation images are 320 x 176 pixels per camera for both policies; the three-view videos are 960 x 176. Control is 20 Hz, physics 240 Hz, seed 0 and object jitter 0. Lots cycle A then B. Videos and setup screenshot show A; scores cover both lots.

## Earlier verdict and distinctness limits

PR 2691's November 19 review said:

> Scoring (not met): The specified real-trial first-contact marks and 3 N core-clamping event cannot reliably be observed while the robot operates the screws.

> Clarity and reproducibility (not met): The custom saddle and ribbed clamp bar lack real fabrication drawings or an identified off-the-shelf equivalent, and the cable's required spring-back is not prepared or verified for real trials.

The review requested dimensioned fabrication/mounting, reproducible cable preparation and spring-back checks, observable scoring, and validation or removal of damage thresholds. Drawings and enrollment now specify the proposed physical trial; outcome checks replace hidden core-contact events and turn counts. Damage is inspected directly, with no claimed calibrated force/turn threshold. Physical enrollment has not been demonstrated.

Nearest tasks include accepted hose-clamp PR 1892 (TK-TGHDCE), accepted battery-cable PR 1947 (TK-6WXUHG, accepted after reconsideration of an overlap FINAL), and conductor-terminal PR 2144 (revisable in the inspected history). These share loaded screw fastening. Here the outer sheath must be held against connected-core spring-back while two independently loaded bar ends are balanced, followed by direct retention testing. That distinction may still be judged a variant; a different fixture or second screw alone does not establish novelty. No claim of guaranteed acceptance is made. Accepted PR 2018 (TK-W8FDKM) fits two lockset halves through a door bore and counter-holds their orientation while two screws close them; accepted PR 1283 (TK-BRLBMB) seats and clamps a DIN-rail end bracket while holding the loose rail. Both reinforce the overlap in holding and fastening. Here the scored object is the cable sheath under a shared two-end bar, with independent retention measurement. More faithful contact physics alone does not establish a new core skill; distinctness remains a review risk.

| Unintended strategy | Consequence |
|---|---|
| Position sheath, then release before tightening | Core-bow restoring load removes coverage |
| Hold cord without tightening | No seated head; zero |
| Turn driver away from screws | No contact torque or thread advance |
| Push screw axially | Native friction and thread coupling resist backdrive |
| Keep supporting the finished assembly | Whole-robot clearance and released hold fail |
| Remove bar, then replace it | Whole-trial guide violation remains latched |
| Clamp only one end | At most 0.25 after release and coverage |
| Park driver on clamp | Tool-clearance condition fails |
| Move assembly after proof | Proof invalidated; it must be repeated |

## Evaluation results and limits

The corrected runtime (task.py SHA256 bbefd2fea0482937de4d1d2db3428f58bc3cbdaa1d8883e4f98e70b01107ca1d) completed one scripted and one Dream-1 invocation, both with lots A/B, seed 0, object jitter 0, 20 Hz control and 240 Hz physics.

| Run | A | B | Mean | End conditions |
|---|---:|---:|---:|---|
| Scripted demo | 1.0 | 0.0 | 0.5 | A success after 3,075 actions; B policy stop after 3,310 |
| Dream-1 | 0.0 | 0.0 | 0.0 | Both reached 2,400 steps; 98 expired-response warnings |

A completed the physical 30 N load and the released settling hold. Its final successful proof displacement was 0.059 mm. B seated both heads, retained sheath coverage, leveled the bar and parked the driver. Its 30 N load phase completed with 0.403 mm maximum displacement, but the post-load stillness hold never passed: final 240 Hz peaks were approximately 3.1 mm/s bar translation, 2.61 rad/s bar rotation and 5.37 mm/s cable travel. The slower control-sample cable speed was small; the physics-step guard correctly rejected the intervening oscillation. B ended at zero, despite briefly earning 0.5 before loading. Its proof remained in settling, not passed. No threshold was relaxed or peak score substituted for the final score.

Both lots reset near zero screw angle and approximately 0.578/0.577 mm head gaps. There were no reported evaluator errors. Dream-1 had no abstention termination; its expired responses limit interpretation of the zero scores. No refusal was reworded around and no second new Dream invocation was made.

The previous packaged source scored demo A=0/B=0 and Dream A=0/B=0. It never completed the pull, and B reset preloaded. Those results remain separate from the corrected-source runs. This corrected package still lacks a full-score B demonstration, physical enrollment and hardware calibration. It is not a validated hardware task. The primary videos and screenshot show A; the reported scores cover both lots.

Original task code is MIT; documentation CC BY 4.0. No maker CAD, photographs or textures are redistributed. Runtime station attribution and external wrapper license scope are in [ASSET-SOURCES.md](ASSET-SOURCES.md).

The scorer also requires coverage relative to the actual bar bounding edge, retaining the original +12 mm minimum; bar translation cannot reduce the required 5 mm overlap. Pull displacement is checked symmetrically. The 8 mm sampled-hand margin conservatively guarantees the stated 6 mm physical clearance at 3 mm surface-sample spacing; it was retained rather than loosened. Initial head gaps are logged in both lots; the nominal level construction gap is distinct from the approximately 0.58 mm gravity-settled start.
