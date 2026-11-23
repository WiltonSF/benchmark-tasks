# Evidence

All results are from the simulation (Isaac Lab-Arena with the yam_arena embodiment); no hardware trial has been run.

| Run | Policy | Lot A | Lot B | Notes |
|---|---|---|---|---|
| demo-20261121T105029 | scripted demo, `--epochs 2` | 1.0 | 1.0 | both trials ended on success at 63.2 s (1,265 steps) |
| dream-20261121T110718 | Dream-1, `--epochs 2` | 0.0 | 0.0 | both trials ran to 2,400 steps; no refusal |

- [demo-results.json](demo-results.json): per-lot times (first grain freed, all 20 in the pan, full score), peak bristle
  load, damage and off-counter flags, read from the task's own per-step log during the demo run.
- [demo-A-sheet.png](demo-A-sheet.png), [demo-B-sheet.png](demo-B-sheet.png): overhead frames every 9 s of the demo.
- [dream-A-sheet.png](dream-A-sheet.png): Dream-1 frames, lot A (overhead and both wrist cameras).

Both runs used the same task files. SHA-256 at run time: task.py 1a50150e3e69271867b90f5977224b2d545f7a5c995b2ba540c8cd56b0cee093,
parts.py a583f60684b2c7518cfa963dad55c52bd15d087ed13e8212368c65961bf3527b, residue/__init__.py
f26257b6c92d400c4454f2bb9aa9f345c7a7e46ec98c1c12ac914386ef4e9816, observation/__init__.py
9a67aa041b0e9d81308bbc241a345e1945cc744b71c0923f4f15b74e0afd2a10, lots.py
f2a660eddea564c7c9562864872c00cbaaa15874ec60e9345bd58eeb851bea77, tilt.py
138c4d1b3a499c95b72d4e1070c9066fd96d914c9bbaa13637e16d5ca75eb567.
