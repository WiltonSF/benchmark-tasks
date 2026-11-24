"""Generate the trial skin's casting tooling from the same skin geometry as task.py.

Outputs (all in this folder, millimetres, frame F of task.py: origin at the dome's centre, +z from the apex
toward the skin's rim; the apex is at z = -20):
  pins.csv                    the 532 pins: id, ring, polar angle from the apex, azimuth, root and tip points
  pin_atlas.svg               numbered polar map of the pins (the scorer's pin map)
  master.stl                  print once in resin: the skin's outer surface with its 532 pins, a locating plug
                              (its end face, 38 mm across, is the face to print on) and four vent ribs; the
                              silicone mould is poured over it inside the box
  box_tube.stl, box_base.stl  print once: the pouring box for the silicone mould (the master's plug end sits
                              in the base's 1 mm recess; the tube stands in the base's ring). The tube is also the
                              sleeve that holds a slit mould closed for each skin pour
  core.stl                    print once per mould: the skin's inner surface (the painting former's r 19 dome and
                              body), a flange that seats on the mould face, a 3 mm sprue to the apex, vent groove
  silicone_mould_reference.stl  what the silicone mould becomes (box inside minus master); for checking only
  skin_reference.stl          the cavity between the mould and the seated core, i.e. the cast skin; for checking
  geometry.json               volumes and counts written by this script

Run: python make_mould.py   (needs numpy and manifold3d; tested with numpy 2.5.5, manifold3d 3.x)
The skin constants and _pin_layout() below are copied verbatim from task.py; check_specimen.py imports task.py
and fails if they differ.
"""

from __future__ import annotations

import json
import math
import pathlib
import struct

import numpy as np

try:
    from manifold3d import Manifold
except ImportError:  # check_specimen.py reads the constants below without the mesh library
    Manifold = None

HERE = pathlib.Path(__file__).resolve().parent
SEG = 160  # facets round a full circle: 0.004 mm chord error on the 20 mm radius

# ---- skin geometry, copied from task.py (metres there; millimetres here) ---------------------------------------
R_FORMER = 0.019
R_SKIN = 0.020
PIN_L, PIN_D = 0.002, 0.001
R_TIP = R_SKIN + PIN_L
N_PINS = 532
SKIRT_TOP = 0.025


def _pin_layout():
    """532 pin directions (unit vectors in F, pointing out of the dome) in rings round the apex. The cast skin's
    exact layout is not published: rings of near-equal 2.2 mm spacing out to 87 degrees from the apex."""
    rings = 15
    d_theta = math.radians(87.0) / (rings - 1 + 0.5)
    thetas = [0.0] + [d_theta * (k + 0.5) for k in range(1, rings)]
    raw = [1] + [2 * math.pi * math.sin(t) / d_theta for t in thetas[1:]]
    scale = (N_PINS - 1) / sum(raw[1:])
    counts = [1] + [max(6, int(round(r * scale))) for r in raw[1:]]
    while sum(counts) > N_PINS:
        counts[-1 - (sum(counts) - N_PINS) % (rings - 1)] -= 1
    while sum(counts) < N_PINS:
        counts[-1 - (N_PINS - sum(counts)) % (rings - 1)] += 1
    dirs, ring_of = [], []
    for k, (t, n) in enumerate(zip(thetas, counts)):
        for j in range(n):
            a = 2 * math.pi * (j + 0.5 * (k % 2)) / n
            dirs.append((math.sin(t) * math.cos(a), math.sin(t) * math.sin(a), -math.cos(t)))
            ring_of.append(k)
    return np.array(dirs), np.array(ring_of), [round(math.degrees(t), 2) for t in thetas], counts


def _orth(d):
    a = np.array([1.0, 0.0, 0.0]) if abs(d[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(d, a)
    u /= np.linalg.norm(u)
    return u, np.cross(d, u)


PIN_DIRS, PIN_RING, RING_DEG, RING_COUNT = _pin_layout()
assert len(PIN_DIRS) == N_PINS

# ---- in millimetres ------------------------------------------------------------------------------------------
RF, RS, RT, PD, ZS = R_FORMER * 1e3, R_SKIN * 1e3, R_TIP * 1e3, PIN_D * 1e3, SKIRT_TOP * 1e3
PIN_ROOT_INSET = 0.3  # mm the pin solids start below the skin surface, so the union is one solid
PLUG_TOP = 33.0  # the locating plug (r = RF) runs from the skirt's rim to here: an 8 mm socket in the mould
BASE_T = 3.0  # box floor and core flange thickness
CUP_R_IN, CUP_R_OUT = 33.0, 36.0  # silicone mould outside radius; box wall
COVER = 10.0  # silicone over the apex pin tip
CUP_BOTTOM = -RT - COVER  # the open end of the cup (the mould's back when poured)
SEAT = 1.0  # the master's plug runs 1 mm past the mould face into a recess in the box base
FIT = 0.15  # radial clearance of the base's recess round the plug
FLANGE_R = CUP_R_IN - 0.2  # the core's flange slides into the tube with 0.2 mm clearance
VENT_R, VENT_AT = 0.6, RF + 0.5  # four vent ribs on the plug, from the skirt's rim to the mould face
SPRUE_R, FUNNEL_R, FUNNEL_D = 1.5, 4.0, 3.0
GROOVE = (RF, RF + 2.5, 1.0)  # core flange underside: vent groove inner r, outer r, depth
NOTCH = 1.0  # index notch in the skin's rim at azimuth 0: 1 mm wide, 1 mm deep, through the wall


def cyl(r, z0, z1, x=0.0, y=0.0):
    return Manifold.cylinder(z1 - z0, r, r, SEG).translate((x, y, z0))


def ball(r):
    return Manifold.sphere(r, SEG)


def halfspace_below(z, big=200.0):
    return Manifold.cube((2 * big, 2 * big, big)).translate((-big, -big, z - big))


def skin_outer_solid():
    """The skin's outside without pins: r 20 dome (z <= 0) and r 20 skirt from z 0 to the rim."""
    dome = ball(RS) ^ halfspace_below(0.0)
    return dome + cyl(RS, 0.0, ZS)


def former_solid(top):
    """The skin's inside (the painting former's dome and body): r 19 dome and body up to `top`."""
    return (ball(RF) ^ halfspace_below(0.0)) + cyl(RF, 0.0, top)


def pins_solid():
    hexes = []
    for d in PIN_DIRS:
        u, v = _orth(d)
        ring = [(math.cos(2 * math.pi * k / 6) * u + math.sin(2 * math.pi * k / 6) * v) * PD / 2 for k in range(6)]
        pts = [(RS - PIN_ROOT_INSET) * d + r for r in ring] + [RT * d + r for r in ring]
        hexes.append(Manifold.hull_points([tuple(map(float, p)) for p in pts]))
    return Manifold.batch_boolean(hexes, _add_op())


def _add_op():
    from manifold3d import OpType
    return OpType.Add


def notch():
    return Manifold.cube((RS - RF + 1.0, NOTCH, NOTCH)).translate((RF - 0.5, -NOTCH / 2, ZS - NOTCH))


def build():
    skin_pos = (skin_outer_solid() + pins_solid()) - notch()  # the skin's outside, as cast
    plug = cyl(RF, ZS - 0.01, PLUG_TOP + SEAT)
    vents = [cyl(VENT_R, ZS - 0.01, PLUG_TOP, VENT_AT * math.cos(a), VENT_AT * math.sin(a))
             for a in (math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4)]
    master = Manifold.batch_boolean([skin_pos, plug, *vents], _add_op())

    # the pouring box in two prints: a tube (also the sleeve that closes a slit mould for every skin pour) and a
    # base plate with a 1 mm recess that centres the master's plug, and a 2 mm ring that locates the tube
    box_tube = cyl(CUP_R_OUT, CUP_BOTTOM - 3.0, PLUG_TOP) - cyl(CUP_R_IN, CUP_BOTTOM - 4.0, PLUG_TOP + 1.0)
    box_base = (cyl(CUP_R_OUT + 4.0, PLUG_TOP, PLUG_TOP + BASE_T)
                + (cyl(CUP_R_OUT + 2.0, PLUG_TOP - 2.0, PLUG_TOP + 0.01) - cyl(CUP_R_OUT + 0.2, PLUG_TOP - 3.0,
                                                                                 PLUG_TOP + 0.5))) \
        - cyl(RF + FIT, PLUG_TOP - 1, PLUG_TOP + SEAT)

    silicone = cyl(CUP_R_IN, CUP_BOTTOM, PLUG_TOP) - master

    core_body = former_solid(PLUG_TOP) + cyl(FLANGE_R, PLUG_TOP, PLUG_TOP + BASE_T)
    groove = cyl(GROOVE[1], PLUG_TOP - 0.01, PLUG_TOP + GROOVE[2]) - cyl(GROOVE[0], PLUG_TOP - 1, PLUG_TOP + 2)
    holes = [cyl(1.0, PLUG_TOP, PLUG_TOP + BASE_T + 1, (RF + 1.25) * math.cos(a), (RF + 1.25) * math.sin(a))
             for a in (0.0, math.pi / 2, math.pi, 3 * math.pi / 2)]
    sprue = cyl(SPRUE_R, -RF - 1, PLUG_TOP + BASE_T + 1) + cyl(FUNNEL_R, PLUG_TOP + BASE_T - FUNNEL_D,
                                                                 PLUG_TOP + BASE_T + 1)
    core = Manifold.batch_boolean([core_body, groove, *holes, sprue], _subtract_op())

    # the cast skin: the mould cavity below the rim minus the seated core (the sprue's gate stub is trimmed off)
    skin = (cyl(CUP_R_IN, CUP_BOTTOM, ZS) - silicone) - former_solid(PLUG_TOP)
    bits = sorted(skin.decompose(), key=lambda b: -b.volume())
    assert all(b.volume() < 1e-3 for b in bits[1:]), [b.volume() for b in bits]
    skin = bits[0]  # drop zero-volume slivers where the notch meets the plug's faceted edge
    return {"master": master, "box_tube": box_tube, "box_base": box_base, "core": core, "silicone_mould_reference": silicone,
            "skin_reference": skin}


def _subtract_op():
    from manifold3d import OpType
    return OpType.Subtract


def write_stl(path, man):
    m = man.to_mesh()
    v = np.asarray(m.vert_properties, dtype=np.float64)[:, :3]
    t = np.asarray(m.tri_verts, dtype=np.int64)
    tri = v[t]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-30)
    rec = np.zeros(len(t), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tri
    with open(path, "wb") as f:
        f.write(b"trial skin tooling, mm, frame F".ljust(80, b" "))
        f.write(struct.pack("<I", len(t)))
        f.write(rec.tobytes())


def count_pins(man):
    """Separate solids left when everything inside r 20.05 or above the dome's equator is cut away."""
    shell = (ball(RT + 1.0) - ball(RS + 0.05)) ^ halfspace_below(0.0)
    return len((man ^ shell).decompose())


def write_csv():
    lines = ["id,ring,theta_deg,phi_deg,root_x_mm,root_y_mm,root_z_mm,tip_x_mm,tip_y_mm,tip_z_mm"]
    for i, (d, k) in enumerate(zip(PIN_DIRS, PIN_RING)):
        theta = math.degrees(math.acos(max(-1.0, min(1.0, -d[2]))))
        phi = math.degrees(math.atan2(d[1], d[0])) % 360.0 if theta > 1e-9 else 0.0
        r, t = RS * d, RT * d
        lines.append(f"{i},{k},{theta:.6f},{phi:.6f}," + ",".join(f"{x:.6f}" for x in (*r, *t)))
    (HERE / "pins.csv").write_text("\n".join(lines) + "\n")


def write_atlas():
    """Polar map seen from outside the apex: radius = angle from the apex (4 px per degree), azimuth as in F with
    0 degrees at the rim's index notch. Viewed looking at the apex (from -z), so +y appears to the left of +x."""
    s, c = 4.4, 430.0
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{2 * c:.0f}" height="{2 * c + 60:.0f}" '
           f'font-family="Helvetica, Arial, sans-serif">',
           f'<rect width="100%" height="100%" fill="white"/>',
           f'<text x="10" y="18" font-size="13">Trial skin pin atlas: 532 pins, 15 rings. Looking at the apex '
           f'(pins toward the viewer); azimuth 0 = rim index notch; radius = degrees from the apex.</text>']
    for k, (deg, n) in enumerate(zip(RING_DEG, RING_COUNT)):
        if k:
            out.append(f'<circle cx="{c}" cy="{c + 20}" r="{deg * s:.1f}" fill="none" stroke="#bbb" '
                       f'stroke-width="0.6"/>')
    out.append(f'<line x1="{c}" y1="{c + 20}" x2="{c + 90 * s + 10:.1f}" y2="{c + 20}" stroke="#c00" '
               f'stroke-width="1"/><text x="{c + 90 * s - 40:.1f}" y="{c + 14}" font-size="10" fill="#c00">'
               f'notch, 0 deg</text>')
    for i, (d, k) in enumerate(zip(PIN_DIRS, PIN_RING)):
        theta = math.degrees(math.acos(max(-1.0, min(1.0, -d[2]))))
        phi = math.atan2(d[1], d[0]) if theta > 1e-9 else 0.0
        x, y = c + theta * s * math.cos(phi), c + 20 + theta * s * math.sin(phi)  # seen from -z: y mirrored
        out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="3.2" fill="black"/>'
                   f'<text x="{x:.1f}" y="{y - 4.2:.1f}" font-size="6" text-anchor="middle">{i}</text>')
    yk = 2 * c + 30
    legend = "; ".join(f"ring {k}: {deg:g} deg, {n}" for k, (deg, n) in enumerate(zip(RING_DEG, RING_COUNT)))
    for j, chunk in enumerate((legend[:len(legend) // 2], legend[len(legend) // 2:])):
        out.append(f'<text x="10" y="{yk + 14 * j}" font-size="10">{chunk}</text>')
    out.append(f'<line x1="{2 * c - 10 - 10 * s:.1f}" y1="{yk - 40}" x2="{2 * c - 10}" y2="{yk - 40}" '
               f'stroke="black" stroke-width="2"/><text x="{2 * c - 10 - 10 * s:.1f}" y="{yk - 45}" '
               f'font-size="10">10 deg</text>')
    out.append("</svg>")
    (HERE / "pin_atlas.svg").write_text("\n".join(out) + "\n")


def write_section(parts):
    """Section through the axis (the y = 0 plane) of the silicone mould, the seated core and the cast skin, as
    poured: apex down, sprue and vents up."""
    s, cx, cy = 6.0, 260.0, 60.0 + (PLUG_TOP + BASE_T) * 6.0
    out = ['<svg xmlns="http://www.w3.org/2000/svg" width="520" height="440" font-family="Helvetica, Arial, '
           'sans-serif">', '<rect width="100%" height="100%" fill="white"/>',
           '<text x="10" y="18" font-size="11">Section on the axis, as poured (apex down). Grey: silicone mould; '
           'orange: core; black: cast skin.</text>',
           '<text x="10" y="32" font-size="11">Inject through the funnel and 3 mm sprue to the apex; air leaves by '
           'the rim vents</text>',
           '<text x="10" y="46" font-size="11">(at 45 degrees to this section) into the flange groove and its '
           'four holes. Scale bar 10 mm.</text>']
    for name, fill in (("silicone_mould_reference", "#c8c8c8"), ("core", "#ee7319"), ("skin_reference", "#111")):
        polys = parts[name].rotate((90.0, 0.0, 0.0)).slice(0.0).to_polygons()
        d = " ".join("M " + " L ".join(f"{cx + p[0] * s:.2f} {cy + p[1] * s:.2f}" for p in poly) + " Z"
                     for poly in polys)
        out.append(f'<path d="{d}" fill="{fill}" fill-rule="evenodd" stroke="#333" stroke-width="0.3"/>')
    out.append(f'<line x1="20" y1="420" x2="{20 + 10 * s}" y2="420" stroke="black" stroke-width="2"/>')
    out.append("</svg>")
    (HERE / "mould_section.svg").write_text("\n".join(out) + "\n")


def main():
    parts = build()
    write_section(parts)
    for name, man in parts.items():
        assert man.status().name == "NoError", (name, man.status())
        write_stl(HERE / f"{name}.stl", man)
    write_csv()
    write_atlas()
    hexa = 3 * math.sqrt(3) / 2 * (PD / 2) ** 2
    geo = {
        "units": "mm, mm3, g",
        "ring_deg": RING_DEG, "ring_count": RING_COUNT, "n_pins": int(N_PINS),
        "volume": {k: round(v.volume(), 3) for k, v in parts.items()},
        "genus": {k: v.genus() for k, v in parts.items()},
        "pins_on_master": count_pins(parts["master"]),
        "pins_on_skin_reference": count_pins(parts["skin_reference"]),
        "skin_volume_analytic_no_notch": round(2 / 3 * math.pi * (RS ** 3 - RF ** 3) + math.pi * (RS ** 2 - RF ** 2) * ZS
                                               + N_PINS * hexa * (RT - RS), 1),
        "skin_mass_g_at_sg_1.04": round(parts["skin_reference"].volume() * 1.04e-3, 2),
        "silicone_volume_cm3": round(parts["silicone_mould_reference"].volume() / 1e3, 1),
        "silicone_mass_g_at_sg_1.18": round(parts["silicone_mould_reference"].volume() * 1.18e-3, 1),
    }
    (HERE / "geometry.json").write_text(json.dumps(geo, indent=1) + "\n")
    print(json.dumps(geo, indent=1))
    write_manifest()


def write_manifest():
    import hashlib
    names = ["make_mould.py", "check_specimen.py", "pins.csv", "pin_atlas.svg", "mould_section.svg", "geometry.json",
             *sorted(p.name for p in HERE.glob("*.stl"))]
    lines = [f"{hashlib.sha256((HERE / n).read_bytes()).hexdigest()}  {n}" for n in names]
    (HERE / "MANIFEST.sha256").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
