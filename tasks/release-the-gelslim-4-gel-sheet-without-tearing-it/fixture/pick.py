"""iFixit IF145-123-2 pick in a reusable grasp handle. Native loose colliding tool; no captures."""
import numpy as np
# Handle frame x toward pick apex, y across blade, z through thickness. Metres.
TIP = np.array([.020,0.,0.])
TRIANGLE = np.array([TIP,[-.018,.018,0.],[-.018,-.018,0.]])
VERTICES = np.concatenate([TRIANGLE-[0,0,.00025],TRIANGLE+[0,0,.00025]])
FACES=[(0,2,1),(3,4,5),(0,1,4,3),(1,2,5,4),(2,0,3,5)]
# 30 x 18 x 26 mm two-piece printed handle: captures rear 30 mm, leaves 8 mm tip exposed.
# Handle runs x=-18..+12, i.e 30 long; 18 wide and 26 thick.
CLIP_SIZE=(.030,.018,.026)
CLIP_AT=(-.003,0.,0.)

# Proposed physical stops: top/bottom of each side pocket.20mm opening z=-11..+9mm.
RAIL_SIZE=(.030,.004,.002)
RAIL_AT=tuple((-.003,y,z) for y in (-.011,.011) for z in (-.012,.010))
