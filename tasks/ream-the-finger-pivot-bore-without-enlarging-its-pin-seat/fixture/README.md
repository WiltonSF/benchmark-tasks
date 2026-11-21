# Fixture files

- `finger_pf_torsion_t42.STL`: the original Yale OpenHand Model T42 pin-flexure finger scaffold, unmodified
  (openhand-hardware, commit b10197dcd79e7bcfa92c6d5a0bfa479e6a789fba, `model t42/stl/finger_pf_torsion_t42.STL`,
  SHA-256 358e3fe7e5048424da4f0c83f9b365bfb7f7693802dcf09b757cc3c5fe2b4347). Licence: Creative Commons
  Attribution-NonCommercial 3.0 (the repository's LICENSE.md); credit: Yale GRAB Lab, OpenHand project.
- `finger_geometry.py` / `occupancy.npz`: a 0.5 x 0.5 x 0.25 mm solid-occupancy grid made from that STL by vertical
  ray casting (29.94 cm3 against the mesh's 30.19 cm3). `python fixture/finger_geometry.py` rebuilds it.
- `calibration.json`: two static offsets per arm added to the demo's commanded tool-centre point (see README).

## Features read from the STL (mm, STL frame)

| Feature | Where | Size |
|---|---|---|
| Pivot bore (running joint, 6.35 mm pin) | axis x 21.79, y 9.64, along z | CAD 6.70 mm; upper ear z 24.25-27.25, lower ear z 4.50-7.50, open spring pocket between |
| Routing-pin seat (press fit, 3.175 mm pin) | axis x 19.63, y 21.82, through z 0-31.75 | CAD 3.325 mm, 12.4 mm from the pivot axis |
| Knuckle | y 0.5-19.5, z 4.5-27.25 | overhangs the vise jaws by 19.5 mm |
| Proximal scaffold (clamped) | y 19.5-63.5 | 21 mm wide at the back face x = 10, rigid corners at x = 31 |
| Cast pad (urethane, estimate) | x 22.5-30.5, y 24-58 | black |
| Cast flexure (urethane, estimate) | x 15-26, y 64-79 | amber |

## Custom parts (drawings in words; all mild steel, sizes in mm)

**Tool stand.** Foot plate 58 x 90 x 10 (its front edge 12 mm behind the hanging reamer). Post 16 x 16, 70 tall,
standing on the foot. Top plate 60 x 60 x 6 at 74-80 above the bench with a 12 mm wide slot from its front edge to
6 mm past the wrench axis (slot length 36). The wrench hangs by its chuck nut on the plate with the reamer in the
slot, its tip 7 mm above the bench. Mass about 1.5 kg (estimate).

**Parallel spacer.** One 6 x 25 x 36 mm ground steel parallel between the fixed jaw and the finger's back face
(x = 10), so the jaws bear only on the rigid proximal scaffold and leave the flexure, the distal segment and the
knuckle free.
