# Real-trial bench drawing (mm)

`bench.scad` draws the trial bench as the simulation builds it: core axis along y, 80 mm above the bench; bought
parts are drawn as envelopes only. Export with, for example, `openscad -D 'part="jaw_gauge"' -o gauge.stl bench.scad`.

| Part | Size and material | Datum |
|---|---|---|
| Core | 13.0 mm round 304 stainless rod, 220 mm long, both ends chamfered 0.5 mm x 45 deg | centre y=-25; bare 50 mm segment y=-135..-85; jaw contact spans y=-127.7..-102.3 |
| Actuator wall | NewAge Silcon tube 12.7 mm ID x 15.9 mm OD (1.6 mm wall), Shore A50, cut square to 170 mm | centre y=0; far end flush with the core tip at y=+85 |
| Vise | Yost BV-4 steel-jaw bench vise (serrated steel jaw pads, 3 in opening), bolted through its three mounting holes; drawn as a 25.4 x 50.8 mm jaw-contact envelope | jaw centre y=-115; core across the jaw pads |
| Jaw gauge | 3 mm printed plate with a 50.0 mm slot | slot ends register rear rod end y=-135 and wall start y=-85; not a jaw-face datum |
| Tray | 100 x 200 mm interior, 6 mm floor, 12 mm walls | centre A (320, 205), B (345, 195) |
| Ruler | 20 x 220 mm, 10 mm marks | centre (440, 240) |

The old cast-actuator mould drawing (actuator.scad) was removed in this revision: no specimen is cast or cured for
a trial any more. The drawing has not been printed or test-fitted.

Vise: [Yost BV-4, Penn Tool Co. SKU 56489](https://www.penntoolco.com/yost-model-bv-4-4-inch-utility-bench-vise-56489/). The jaw envelope is the simulated contact patch, not the BV-4 jaw geometry; the simulated housing/post and 80 mm axis height are simplified, not maker-certified or test-fitted. Jaw forward edge y=-102.3 mm leaves 17.3 mm to the wall start at y=-85 mm.

Named core stock: [Zoro RD013304-1](https://www.zoro.com/zoro-select-stainlesssteelrod-13mm-od-12in-l-2pk-rd013304-1/i/G913866509/), 13 mm diameter 304, 12 inch stock, diameter tolerance 0/-0.0017 inch (0/-0.043 mm), mill finish. Cut to 220 mm and chamfer once for fixture assembly; measure and record the working diameter/finish before enrolling specimens.
