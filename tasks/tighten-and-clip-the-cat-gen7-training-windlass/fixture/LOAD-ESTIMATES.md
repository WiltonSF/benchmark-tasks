# Estimated mechanical loads

These are engineering estimates for simulation, not measurements of a C-A-T trainer. No physical force reading is present in this folder. The November 19 verdict's request for measurements on NAR 30-0033 remains open.

## Winding

[Fultz et al., JSOM 2026, 26(2), p.61](https://jsomonline.org/product/tourniquet-pressure-magnitudes-and-distribution-during-limb-applications/) report 3.31 N m median thigh occlusion torque at 335.56 mmHg for a **SOF Gen 5**, a sibling 38 mm windlass device. The source PR scaled this to a 400 mm training cylinder using 545 mm as a representative thigh circumference. That circumference and proportional scaling are assumptions; the abstract does not establish either for a C-A-T.

The calculation is 3.31 x (250/335.56) x (400/545) = approximately 1.81 N m at 250 mmHg. At 270 mmHg it gives approximately 1.95 N m. The model uses a 0.5293 N m/rad winding spring, reaching 1.94 N m at the candidate 210-degree lock and 1.76 N m at 190 degrees. It is a physical implicit joint spring, with 0.20 N m s/rad damping; there is no motor that winds the rod for the robot. Joint and finger contact reactions carry the load. A 65 mm lever needs approximately 30 N of tangential force at the final lock, before allowance for grip slip.

## Rod tilt

The tilt joint represents out-of-plane bending of the woven band, distinct from the resisted winding. Its 0.10 N m/rad stiffness is an estimate for a 38 mm wide, approximately 1.5 mm combined woven-layer thickness flexible woven strip with a 10 mm bend length and effective bending modulus around 95 MPa: EI/L is approximately 0.10 N m/rad. Effective woven bending modulus, thickness and bend length are assumptions, not maker values. This spring tries to level the rod and therefore resists seating; it never pushes the rod down into the clip. The earlier 0.30 N m/rad estimate stored enough energy to defeat the model clip; the softer estimate permits retention in the independent CPU contact probe while preserving the full winding torque.

## Clip

The [NAR maker page](https://www.narescue.com/combat-application-tourniquet-c-a-t-tb.html) describes a bilateral beveled windlass clip but publishes no snap force or resin specification. The model's two elastic lips are a lumped cantilever approximation, not a claim about the actual polymer.

For an explicit material prior, [BASF Ultramid A3K data](https://download.basf.com/p1/8a8082587fd4b608017fd64108ab6d3b/en/ULTRAMID%25C2%25AE_A3K) give a dry flexural modulus of 2,900 MPa. Assuming a nylon-like 10 mm wide, 1.5 mm thick, 7.9 mm effective cantilever gives k=Ebt^3/(4L^3), approximately 49.6 kN/m. The model uses 50 kN/m for each lip, with 8 g effective moving mass and 40 N s/m damping. Actual clip resin, elastic length, moisture response, mass and stiffness are unmeasured. The 8 g masses stabilize the reduced elastic modes; they are not weighed lip specimens.

A 12 mm rod must spread an approximately 11 mm mouth. The minimum credited snap is a stated **10 N rod-to-lip normal-load estimate**, with at least 1 mm of downward traversal and 1 mJ of actual contact work. The scorer reads the rod's native force matrix filtered separately to the two lips, and counts work only during advancing rod-lip contact on the insertion path, with a contact witness on each lip in the same entry. Pressing a lip with a finger cannot set that record. The physical lips, channel floors and walls must retain the rod after both hands withdraw; no latch flag creates a constraint or writes a pose. Insertion and extraction curves still need comparison with hardware; an estimated force gate is not that comparison.

## Revised contact geometry and modal damping

The stated clip prior now uses a 13 mm channel, 10.7 mm entry mouth and 12 mm sidewalls. The mouth was adjusted by 0.3 mm from the approximately 11 mm prior to meet the stated approximately 10 N insertion prior without lowering the force gate. These dimensions remain estimates rather than NAR CAD. A 50 kN/m spring and 8 g effective mode have critical damping 2sqrt(km)=40 N s/m; that damping replaces the earlier 2 N s/m numerical mode. At the slow entry speed it adds little quasistatic force. The 40 N spring-drive cap remains. Full-load CPU tests now retain the winding degree of freedom, actual anchor and surrounding plate/cylinder. Native confirmation is still required.

Normal qualification on the 10.9 mm estimate retained physically in both lots, but the sampled insertion peak was 10.34 N in A and 9.40 N in B; the latter correctly failed the unchanged 10 N gate. The mouth estimate is now 10.7 mm (bead centres +/-6.10 mm, bead radius0.75 mm) to provide physical force margin. Springs, caps, damping, winding load and contact-work requirements are unchanged. This is a simulation sensitivity adjustment within the unpublished geometry estimate, not a hardware measurement.
