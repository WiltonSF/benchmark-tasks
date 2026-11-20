# Custom parts: rail section, feet, slot blocks, mat

All dimensions in mm. The rail is a stock 20 x 20 mm V-slot aluminium extrusion; the slot blocks and feet are made to
these drawings. Side view: `bank-side.svg` (looking along the rail, the robot's left to the right of the drawing).

## Rail and feet

| Part | Size | Notes |
|---|---|---|
| Rail | 20 x 20 x 100, black anodised V-slot | its top-face slot holds the feeders' hook noses; in the simulation the slot is a plain 4.1 wide, 4 deep channel |
| Foot post (2) | 10 (along the rail) x 20 x 96 | screwed to the rail's underside 5 mm from each end, centres 90 apart |
| Foot plate (2) | 26 (along the rail) x 60 x 6 | under each post, clamped to the bench |
| Rail centre line | 112 above the bench | the reels then clear the bench by 8 |

## Slot block (3, printed PETG, yellow)

| Feature | Size | Position |
|---|---|---|
| Block | 14.5 (along the rail) x 6 (thick) x 26 (tall) | on the rail's left face, from 16 below to 10 above the rail centre line; centres 15.5 apart |
| Lip | the block's bottom face | the feeder lever's two teeth hook under it, 3.4 to 7 mm either side of the block centre |
| Peg | 5 (along the rail) x 5.5 (out from the block) x 6 (tall) | centred on the block, from 2 below to 4 above the rail centre line |

## Feeder features that meet the bank (as modelled; estimates from the maker's photos)

| Feature | Size | Position (from the rail centre line, at the slot centre) |
|---|---|---|
| Body | 15 wide x 158 long x 42 tall | bottom 0.05 above the rail top, from 18 outboard (left) to 140 inboard (right) |
| Hook nose | round, 4 diameter, 14 long along the rail | in the rail's top slot, centre 8.3 above the rail centre line |
| Clip prongs (2) | 4.3 x 7 x 22, either side of the peg | in front of the slot block's face |
| Bearing pads (2) | on the prongs, 3 tall | touching the slot block face, 3 to 6 below the rail centre line |
| Release lever | yellow, two round teeth (5 diameter) under the slot block lip, thumb tab 3 x 13 outboard | pivot 20 outboard, 10.3 below the rail centre line; spring preloaded (estimate 0.03 N m) |
| Reel | 178 diameter x 13 | hub 125 outboard, 15 below the rail centre line |

## Mat

Blue anti-static bench mat, 300 x 420 x 3, 0.5 kg, cut from a stock roll; lot A centre (350, 360), lot B (320, 360)
in the robot frame given in task.json.
