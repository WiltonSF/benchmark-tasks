# Current revision qualification

current-source-manifest.json lists SHA-256 hashes of the frozen runtime files. Both runs below used exactly these files and the published instruction, 20 Hz control, zero layout jitter and --epochs 2 (lot A, then lot B).

- Scripted demonstration, 3200-step limit (current-runner-receipt.json, native observations current-native-audit.jsonl.gz): lot A 1.0 in 2231 actions; lot B 0.5 in 2381 actions. In A all three load signals stayed at or above 4.5 Nm for 3.30 s (peaks: modeled 4.574 Nm, spanner-cap contact 4.782 Nm, cap reaction 4.758 Nm, all below 5 Nm), then three 50 N, 1 s pulls passed with 0.447 mm residual. In B the cap peaked at 148.40 deg (modeled 4.279 Nm), short of the 150 deg / 4.5 Nm certificate, so B earns the 0.5 tier. No damage or invalid state in either lot.
- Dream-1, 2400-step limit (current-dream-receipt.json, current-dream-audit.jsonl.gz): A 0.0, B 0.0; both trials reached the step limit.
- offline-check-current.txt: the full offline checker on these files (reach, joint limits, fixed-object clearance in both layouts, 30 synthetic history and 7 endpoint scoring controls, text fields).

first-grip/ and retention-sweep/ hold reset-only simulation calibrations on the same physical model (scores 0 by design). historical/ keeps the pushed-source package, the superseded final18 package and the superseded 0.41 mm first-grip calibration. No physical robot result is claimed.
