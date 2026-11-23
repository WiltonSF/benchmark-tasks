# Reusable specimen and one-person reset protocol

This is a proposed physical enrollment protocol for the existing native Eisco PH0122B bottle and perforated stopper. It has not been demonstrated on hardware. The simulation uses exact nominal layouts and the twelve nominal pocket positions; it does not establish repeatability over the proposed physical position windows below.

Use one bought-to-spec, unmodified bottle/stopper pair for both lots. The glass, bath, stock shelf, open rack, rest and inspection pad are reused. Do not attach artificial bubbles, coat the inner wall, add a wetting agent, or replace water with a gel. Such a substitution would change the manipulation. Qualify a spare pair separately; catalogue dimensions alone do not qualify its ground joint or wetting behavior.

| Quantity | Nominal and proposed physical enrollment window |
|---|---|
| Water | One batch of ordinary water,20 +/-0.5 degrees C for reference and trial;no added wetting agent |
| Initial interior water |44.0 +/-0.1mL,measured before bubble placement |
| Individual adhering pockets |Twelve,each0.10 +/-0.01mL;sum1.20 +/-0.10mL;no coalescence |
| Pocket position |Polar67degrees and azimuth90+k*180/11degrees,k=0..11;within1degree per angle;two-view registered reference images |
| Pocket retention |All twelve independently visible and retained throughout10s upright after withdrawing the syringe |
| Bath waterline |80 +/-1mm above table datum |
| Bottle bottom A/B |(180,30,28)/(180,75,28)mm,each coordinate within1mm |
| Stopper tip A/B |(270,-150,4)/(270,-180,4)mm,each coordinate within1mm;tip supported on4mm rack base |
| Rest posts |Radius24.4mm,locations within0.2mm;actual glass has positive clearance |

The angle window is a proposed reproducibility requirement,not a manufacturer tolerance or a native simulation sweep. For the23mm inner-radius estimate,1degree corresponds to about0.4mm along the wall. A separate deterministic CPU sensitivity probe (seed20261123) checked36 prescribed pocket configurations within these angle/volume windows:576 upright/10degree heading-retention probes passed,and each configuration cleared under the paired30degree motions. These are reduced fluid-model controls,not robot or hardware runs;they do not establish physical noncoalescence,pose-jitter robustness,glass chemistry,wetting hysteresis or reset time. The reduced model keeps pockets independent and does not resolve their merging.

The reset syringe needs a blunt flexible plastic extension that can reach the shoulder through the native neck;do not assume a straight rigid tip can reach all twelve sites. During qualification,verify access to every site without scratching the glass,adding water or depositing residue,and calibrate the actual delivered air volume at that tip after priming it with air. Reject the preparation method if any site is inaccessible or delivery cannot meet the stated volume bounds. This is a reusable reset aid within the existing syringe/small-tools allowance,not an object manipulated in the robot trial.

Before a hardware session,calibrate the two close views,bubble-volume bounds,overflow collection and same-pair exterior-dry reference mass as in README. With spare resets spanning0.09,0.10 and0.11mL per pocket,check the stated upright/0-10degree retention and paired30degree clearing bracket. Record all preparation attempts,failed enrollments and elapsed qualification time. Reject the pair/water/preparation method if the bracket fails;do not silently choose a different liquid,roughen the glass or adjust scoring thresholds.

For each trial one operator:

1. Start a stopwatch before the first rinse. Rinse using the same water batch,inspect for chips,free the native capillary,and inspect leak-tested sleeves covering both gripper pads,fingers and distal wrists. Stop if glass is damaged or covers are breached.
2. Keep the bottle upright on its raised start shelf with its mouth above the bath waterline,and measure44.0mL interior water. Insert a1mL graduated syringe with a blunt flexible plastic extension through the open neck;only the tip goes below the bottle's interior water surface. Deliver twelve0.10mL air volumes against the inner shoulder at the published positions,then withdraw the syringe. Do not immerse the open mouth during preparation:that would admit extra bath water and violate the44mL starting inventory. No water is added during the injections. Use calibrated external image references rather than adding features inside the bottle. Pocket placement and correction are preparation time.
3. Withdraw the syringe and place the bottle upright. Record two orthogonal close views continuously for10seconds. Reject if a pocket merges,moves outside its enrollment window,escapes,is obscured,or cannot be bounded individually. Reject if the total volume,water inventory or temperature falls outside the windows. Rejected attempts and rework stay on the clock.
4. Set the named lot,waterline and loose stopper using the ruler/datums. Verify rest clearance,hands clear,readable images and dry exposed rig. The evaluator confirms the enrollment record before allowing the policy to start;no operator correction is allowed during the timed robot trial.
5. After scoring,rinse/drain as needed,dry and inspect sleeves,pads and rig,and include this cleanup in the trial reset log. Record elapsed preparation,cleanup and rejected-attempt time,plus the separately recorded session qualification.

The3-5minute reset by one person is an unmeasured estimate that includes every per-trial step,not a demonstrated result. A native-glass wetting failure or inability to complete the full enrollment reliably means the physical task is unvalidated. Keep simulation results separate and revise/qualify the protocol before hardware scoring. There is no claim that the twelve nominal pockets can already be prepared reliably or in the estimated time.
