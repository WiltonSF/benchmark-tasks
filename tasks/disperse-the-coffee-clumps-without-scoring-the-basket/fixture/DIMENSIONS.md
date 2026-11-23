# Passive enclosure drawings

Complete counter surroundings, millimetres. No part is a tool holder or a modification to the scored products. `enclosures.scad` defines estimated simple outer enclosures: espresso330wide x310deep x400high including hopper, grinder160wide x200deep x380high. Model coordinates and member list are in task.py. Build from3mm printed/plywood panels, weighted or fixed at the background coordinates; estimate$50 for2kg material and fittings. Seal edges and keep units unpowered. Use original scored Normcore/Breville parts, never printed substitutes for the needles, basket or stand.

These are original envelope approximations based on complete BES870/BCG600 product photographs, not maker CAD. The live US BES870 page gives15.9x13.1x12.4in under WxDxH; its labeling is inconsistent with the pictured upright machine, so the drawing deliberately declares330x310x400mm as an estimated upright clearance envelope rather than claiming exact CAD. BCG600 page lists8x6.3x13.5in; the380mm-high approximation includes a conservative hopper. Verify actual counter clearances if using original unpowered appliances. No operation inside either appliance is scored.

The modeled floor/wall and native portafilter support are the task's real contact geometry. Nothing added to the counter fixes the basket or tool to a favorable pose.

Drawing corrected after independent review: generated directly from task.py background Shape dimensions; espresso actual bounding envelope310deep x330wide x400high, grinder200deep x160wide x380high. The 160 mm width includes its native base; the body alone is150 mm. Runtime geometry was unchanged. Generated member shells are3 mm and keep the simulated outer surfaces.
Regenerate with `python fixture/generate_enclosures.py` in an environment that imports the task declarations; this reads the same Shape values used by the simulation without starting it.
