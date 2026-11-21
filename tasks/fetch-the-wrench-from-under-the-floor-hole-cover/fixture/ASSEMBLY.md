# Building the deck and the hole cover

Written by `make_drawing.py` from the simulation's numbers. Drawings: `deck-1to1.svg` and `cover-1to1.svg` (full
size; each has a 100 mm check bar). Pieces: `cut-list.csv`. Tools: a saw (a jigsaw for the opening), a drill with a
countersink, a square, a plane or file, a rule. Nothing in this package has been built yet; sizes are the simulated
ones.

Not simulation numbers, and free to change: the batten width (38 mm), the screw sizes and positions,
and the edge fillers. The simulation fills the whole space under the plywood, so only the deck's outline, its
80 mm height and the opening matter.

## Deck

1. Cut the deck top, 440 x 340 mm, from 18 mm plywood.
2. Mark the opening on it: a 200 x 200 mm square in the middle, 120 mm in from each
   340 mm edge and 70 mm in from each 440 mm edge. Drill a starter hole inside
   each corner, saw the square out and file the edges straight and square to the face. Check both widths measure
   200 mm and the two diagonals are equal.
3. Cut the battens (62 mm tall, 38 mm wide): two long battens 440 mm, two cross
   battens 200 mm, two end battens 200 mm. If the stock is not 62 mm tall, plane it
   to 62 mm, or build with it as it is and record the deck's built height with the results.
4. Lay the top face down. Stand the two long battens on it along the 440 mm direction, one each side
   of the opening, inner faces flush with the opening's edges. Fit the two cross battens between them, flush with
   the opening's other two edges. Fit the two end battens between the long battens under the top's
   340 mm edges. Check with the square that all four inner faces are flush with the opening all the
   way down: the opening must be a straight-sided pocket with no ledge or gap for the wrench to slide under.
5. Turn it over and screw down through the top into the battens (16 screws at the marked positions, or as
   suits), countersunk so every head is flush with or below the deck's top: the cover slides on this face.
6. Optional: glue or pin the two edge fillers (440 x 32 x 62 mm) under the
   overhang on the two 440 mm sides, so those sides are solid as modelled.
7. On the deck's top draw a 230 mm square round the opening, 15 mm outside each edge.
   It is used to score the cover's lap.
8. Stand the deck on the table with its 440 mm sides across the robot's view (along y) and the
   opening's middle 260 mm in front of the midpoint between the arm bases, on the centre line. Fix
   it to the table (clamps or screws; it must not move when the cover is pushed against the opening's edges).
9. Check: the deck's top is 80 mm above the table, and the opening is 200 x 200 mm
   and 80 mm deep down to the bare table.

## Hole cover

10. Cut the plate, 270 x 270 mm, from 18 mm plywood. Cut the handle,
    150 x 40 x 40 mm, from softwood. Cut two cleats, 180 x 20 mm, from the
    18 mm plywood (18 mm tall as they hang under the plate).
11. Plane or file a 6 mm bevel at 45 degrees on all four bottom edges of each cleat (both long edges
    and both ends), leaving a flat foot 168 x 8 mm.
12. On the plate's underside mark its two centre lines. Fix the cleats parallel to one centre line, their own
    centre lines 80 mm either side of it, so their outer faces are 180 mm apart, and
    centred end to end (45 mm in from the plate's edges). Screw down through the plate
    into each cleat (three screws each, pilot holes drilled, heads flush with the plate's top), with glue if
    wanted.
13. Fix the handle on top of the plate at its middle, parallel to the cleats: two screws up through the plate
    from the underside, heads flush.
14. Paint the cover yellow and mark it HOLE.
15. Check: outer faces of the cleats 180 mm apart; cleats 180 mm long. Seat the cover on
    the deck: the plate lies flat on the deck all round, and the cover can be pushed 20 mm from stop
    to stop both along and across the handle (10 mm of play each way). It seats the same turned a
    quarter turn. With the cover pushed to any stop the drawn square stays hidden.
16. Weigh the finished cover and note the reading with the results. The modelled mass is 0.94 kg,
    an estimate from typical densities; the arm's payload is about 1 kg, so use a lighter plywood if the built
    cover comes out heavier than that.
