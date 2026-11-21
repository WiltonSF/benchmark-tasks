"""Writes the build package for the deck and the hole cover from the same numbers the simulation uses (../task.py):

    deck-1to1.svg / .png     the deck: plan and two sections, full size, millimetres, with a 100 mm check bar
    cover-1to1.svg / .png    the hole cover: plan, end view, side view and the cleat's section, full size
    cut-list.csv             every piece: stock, size, number
    ASSEMBLY.md              numbered build steps and the checks to make on the finished parts

Run from this folder with the Python that has yam-isaac-lab-arena installed (task.py imports it):

    python make_drawing.py

The PNG previews are written only if Pillow is installed. Nothing here has been built yet: the drawings state the
simulated sizes, and the choices that are not simulation numbers (batten width, screws) are marked as such.
"""

import csv
import pathlib
import sys

sys.dont_write_bytecode = True
HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
import task as T  # noqa: E402
from drawing_sheet import GREY, Sheet, View, fmt  # noqa: E402

MM = 1000.0


def mm(v):
    return round(float(v) * MM, 3)


# -- simulation numbers (task.py) -----------------------------------------------------------------------------------
PLY = mm(T.PLY_T)
BAT_H = mm(T.BATTEN_H)
DECK_H = mm(T.DECK_H)
H = mm(T.OPEN_HALF)  # half the opening
X0, Y0 = (mm(v) for v in T.DECK_LO)  # deck outline about the opening's middle (x forward, y to the robot's left)
X1, Y1 = (mm(v) for v in T.DECK_HI)
OPEN_AT = tuple(mm(v) for v in T.OPEN_XY)
PLATE = mm(T.PLATE)
HANDLE = tuple(mm(v) for v in T.HANDLE)  # length, width, height
CL, CW, CH, BEV = mm(T.CLEAT_L), mm(T.CLEAT_W), mm(T.CLEAT_H), mm(T.BEVEL)
CY = mm(T.CLEAT_Y)
LAP = mm(T.OVERLAP_MIN)
PLAY = H - (CY + CW / 2)  # cleat outer face to the opening's edge
END_PLAY = H - CL / 2

# -- build choices that are NOT simulation numbers (the simulation fills the whole space under the plywood) ----------
BATTEN_W = 38.0  # batten width; any width works if the four battens round the opening are flush with its edges
FILLER_W = X1 - H - BATTEN_W  # strip that closes the overhang on the two 440 mm sides

C_PLY, C_BAT, C_YEL, C_WOOD = "#f1e2c3", "#d9b98c", "#fbe7a1", "#e6cfa5"
BROWN, RED = "#8a5a2b", "#c62828"


def battens():
    """(name, x0, y0, x1, y1) of every batten under the deck, about the opening's middle."""
    w = BATTEN_W
    return [
        ("long batten", H, Y0, H + w, Y1), ("long batten", -H - w, Y0, -H, Y1),
        ("cross batten", -H, H, H, H + w), ("cross batten", -H, -H - w, H, -H),
        ("end batten", -H, Y1 - w, H, Y1), ("end batten", -H, Y0, H, Y0 + w),
        ("edge filler", H + w, Y0, X1, Y1), ("edge filler", X0, Y0, -H - w, Y1),
    ]


def deck_screws():
    w = BATTEN_W
    ys = [Y0 + 40 + i * (Y1 - Y0 - 80) / 3 for i in range(4)]
    out = [(sx * (H + w / 2), y) for sx in (-1, 1) for y in ys]
    out += [(x, sy * (H + w / 2)) for sy in (-1, 1) for x in (-0.6 * H, 0.6 * H)]
    out += [(x, sy * (Y1 - w / 2)) for sy in (-1, 1) for x in (-0.6 * H, 0.6 * H)]
    return out


def cross(v, p, r=2.2, col="#000"):
    v.line((p[0] - r, p[1]), (p[0] + r, p[1]), w=0.3, col=col)
    v.line((p[0], p[1] - r), (p[0], p[1] + r), w=0.3, col=col)


def deck_sheet():
    s = Sheet(590, 850)
    s.text((20, 16), "Deck with the floor opening: full size (1:1), millimetres", size=6)
    s.note((20, 25), [
        f"{fmt(PLY)} mm plywood top {fmt(Y1 - Y0)} x {fmt(X1 - X0)} with a {fmt(2 * H)} x {fmt(2 * H)} opening, on "
        f"{fmt(BAT_H)} mm tall battens: {fmt(DECK_H)} mm from the table to the deck's top.",
        "The four battens round the opening are flush with its edges all the way down, so the opening is a square "
        f"pocket {fmt(DECK_H)} mm deep to the table.",
        f"Simulation numbers: every size shown except the batten width ({fmt(BATTEN_W)} mm, a build choice; the "
        "simulation fills the whole space under the plywood)",
        "and the screw positions (suggested). Dashed brown: battens under the plywood. Dashed red: the scoring square "
        "to draw on the deck's top.",
    ], size=3.4)

    # plan: page right = the robot's right (-y), page up = away from the robot (+x)
    v = View(s, 295, 300)

    def pl(x, y):
        return (-y, x)

    def prect(xa, ya, xb, yb, **kw):
        (ua, va), (ub, vb) = pl(xa, ya), pl(xb, yb)
        v.rect(min(ua, ub), min(va, vb), max(ua, ub), max(va, vb), **kw)

    s.text((75, 78), "PLAN (from above)", size=4.5)
    prect(X0, Y0, X1, Y1, w=0.7, fill=C_PLY)
    prect(-H, -H, H, H, w=0.7, fill="#ffffff")
    for name, xa, ya, xb, yb in battens():
        prect(xa, ya, xb, yb, w=0.3, col=BROWN, dash=(1.2, 2.0) if name == "edge filler" else (3, 1.5))
    q = H + LAP
    prect(-q, -q, q, q, w=0.4, col=RED, dash=(4, 2))
    for x, y in deck_screws():
        cross(v, pl(x, y))
    v.centre((-(Y1 + 6), 0), (Y1 + 6, 0))
    v.centre((0, X0 - 6), (0, X1 + 6))
    cross(v, (0, 0), r=4, col=GREY)
    v.text((0, 0), f"opening's middle: ({OPEN_AT[0] / MM:.2f}, {OPEN_AT[1] / MM:.2f}) m", dx=3, dy=-3, size=3.2,
           col=GREY)
    v.text((0, 0), "from the midpoint between the arm bases", dx=3, dy=7, size=3.2, col=GREY)
    v.text((0, -H), f"opening {fmt(2 * H)} x {fmt(2 * H)}, cut square to the line", dy=-4, size=3.4, anchor="middle")
    v.text((0, -q), f"scoring square {fmt(2 * q)} x {fmt(2 * q)} ({fmt(LAP)} outside each edge)", dy=5.5, size=3.2,
           anchor="middle", col=RED)
    v.text((-Y1 + 4, H + BATTEN_W / 2), "long batten", dy=1.2, size=3.2, col=BROWN)
    v.text((-Y1 + 4, -H - BATTEN_W / 2), "long batten", dy=1.2, size=3.2, col=BROWN)
    v.text((-Y1 + 4, X1 - FILLER_W / 2), "edge filler (optional)", dy=1.2, size=3.2, col=BROWN)
    v.text((-Y1 + 4, X0 + FILLER_W / 2), "edge filler (optional)", dy=1.2, size=3.2, col=BROWN)
    for sgn in (-1, 1):
        v.text((sgn * (H + BATTEN_W / 2), -H + 30), "cross", dy=0, size=3.0, anchor="middle", col=BROWN)
        v.text((sgn * (H + BATTEN_W / 2), -H + 30), "batten", dy=4, size=3.0, anchor="middle", col=BROWN)
        v.text((sgn * (Y1 - BATTEN_W / 2), -H + 30), "end", dy=0, size=3.0, anchor="middle", col=BROWN)
        v.text((sgn * (Y1 - BATTEN_W / 2), -H + 30), "batten", dy=4, size=3.0, anchor="middle", col=BROWN)
    # dimensions
    v.dim_h(-Y1, -Y0, X1, 25)
    v.dim_h(-H, H, X1, 12)
    v.dim_h(-Y1, -H, X1, 12)
    v.dim_h(H, -Y0, X1, 12)
    v.dim_v(X0, X1, -Y0, 28)
    v.dim_v(-H, H, -Y0, 13)
    v.dim_v(H, X1, -Y0, 13)
    v.dim_v(X0, -H, -Y0, 13)
    v.dim_v(H, H + BATTEN_W, -Y1, -12)
    v.dim_v(H + BATTEN_W, X1, -Y1, -12)
    v.dim_h(H, H + BATTEN_W, X0, -12)
    v.dim_h(H + BATTEN_W, -Y0 - BATTEN_W, X0, -12)
    v.dim_h(-Y0 - BATTEN_W, -Y0, X0, -12)
    # section marks
    for sgn in (-1, 1):
        v.line((sgn * (Y1 + 2), 0), (sgn * (Y1 + 9), 0), w=0.9)
        v.text((sgn * (Y1 + 5.5), 0), "A", dy=-2, size=4.5, anchor="middle")
        v.line((0, sgn * (X1 + 2)), (0, sgn * (X1 + 9)), w=0.9)
        v.text((0, sgn * (X1 + 5.5)), "B", dx=2.5, dy=1.6, size=4.5)
    v.text((-Y1, X0), "+  countersunk wood screw down into a batten, head flush with the deck's top (suggested "
                      "positions)", dy=24, size=3.2)
    v.text((-Y1, X0), "Robot side. Top of the page is away from the robot (+x); the left of the page is the robot's "
                      "left (+y).", dy=30, size=3.2)

    # section A-A: along y through the opening
    a = View(s, 295, 620)
    s.text((75, 520), "SECTION A-A (along y, through the opening)", size=4.5)
    a.rect(-Y1, 0, -Y0, BAT_H, w=0.25, col=GREY)  # the long batten beyond the cut
    a.line((-H, BAT_H), (-H, DECK_H), w=0.25, col=GREY)
    a.line((H, BAT_H), (H, DECK_H), w=0.25, col=GREY)
    a.line((-H, DECK_H), (H, DECK_H), w=0.25, col=GREY)
    for sgn in (-1, 1):
        lo, hi = sorted((sgn * H, sgn * Y1))
        a.rect(lo, BAT_H, hi, DECK_H, w=0.6, fill=C_PLY)
        lo, hi = sorted((sgn * H, sgn * (H + BATTEN_W)))
        a.rect(lo, 0, hi, BAT_H, w=0.6, fill=C_BAT)
        lo, hi = sorted((sgn * (Y1 - BATTEN_W), sgn * Y1))
        a.rect(lo, 0, hi, BAT_H, w=0.6, fill=C_BAT)
    a.line((-Y1 - 25, 0), (Y1 + 25, 0), w=1.0, col="#444")
    a.text((0, 0), "table (the subfloor)", dy=5, size=3.2, anchor="middle", col="#444")
    a.text((0, BAT_H / 2), "long batten beyond: the pocket's far wall", dy=1, size=3.2, anchor="middle", col=GREY)
    for sgn in (-1, 1):
        a.text((sgn * (H + BATTEN_W / 2), BAT_H / 2), "cross", dy=-1, size=3.0, anchor="middle")
        a.text((sgn * (H + BATTEN_W / 2), BAT_H / 2), "batten", dy=3, size=3.0, anchor="middle")
        a.text((sgn * (Y1 - BATTEN_W / 2), BAT_H / 2), "end", dy=-1, size=3.0, anchor="middle")
        a.text((sgn * (Y1 - BATTEN_W / 2), BAT_H / 2), "batten", dy=3, size=3.0, anchor="middle")
    a.text((-Y1 + 60, DECK_H), f"{fmt(PLY)} mm plywood", dy=-2, size=3.2, anchor="middle")
    a.dim_h(-H, H, DECK_H, 10)
    a.dim_v(0, DECK_H, -Y0, 14)
    a.dim_v(0, BAT_H, -Y1, -12)
    a.dim_v(BAT_H, DECK_H, -Y1, -12)

    # section B-B: along x through the opening
    b = View(s, 295, 765)
    s.text((75, 650), "SECTION B-B (along x, through the opening)", size=4.5)
    b.rect(X0, 0, X1, BAT_H, w=0.25, col=GREY)
    b.line((-H, BAT_H), (-H, DECK_H), w=0.25, col=GREY)
    b.line((H, BAT_H), (H, DECK_H), w=0.25, col=GREY)
    b.line((-H, DECK_H), (H, DECK_H), w=0.25, col=GREY)
    for sgn in (-1, 1):
        lo, hi = sorted((sgn * H, sgn * X1))
        b.rect(lo, BAT_H, hi, DECK_H, w=0.6, fill=C_PLY)
        lo, hi = sorted((sgn * H, sgn * (H + BATTEN_W)))
        b.rect(lo, 0, hi, BAT_H, w=0.6, fill=C_BAT)
        lo, hi = sorted((sgn * (H + BATTEN_W), sgn * X1))
        b.rect(lo, 0, hi, BAT_H, w=0.4, col=BROWN, dash=(1.2, 2.0))
    b.line((X0 - 25, 0), (X1 + 25, 0), w=1.0, col="#444")
    b.text((0, BAT_H / 2), "cross batten beyond", dy=1, size=3.2, anchor="middle", col=GREY)
    b.text((X1, BAT_H / 2), "edge filler (optional)", dx=24, dy=1, size=3.0, col=BROWN)
    b.text((H + BATTEN_W / 2, BAT_H / 2), "long", dy=-1, size=3.0, anchor="middle")
    b.text((H + BATTEN_W / 2, BAT_H / 2), "batten", dy=3, size=3.0, anchor="middle")
    b.dim_h(X0, X1, DECK_H, 22)
    b.dim_h(-H, H, DECK_H, 10)
    b.dim_h(X0, -H, DECK_H, 10)
    b.dim_h(H, H + BATTEN_W, 0, -10)
    b.dim_h(H + BATTEN_W, X1, 0, -10)
    b.dim_v(0, DECK_H, X1, 14)
    b.text((X0, 0), "robot side", dy=8, size=3.2)

    s.check_bar(40, 815)
    return s


def cleat_section(cy):
    return [(cy - CW / 2, 0), (cy + CW / 2, 0), (cy + CW / 2, -CH + BEV), (cy + CW / 2 - BEV, -CH),
            (cy - CW / 2 + BEV, -CH), (cy - CW / 2, -CH + BEV)]


def cover_sheet():
    s = Sheet(560, 730)
    p = PLATE / 2
    hl, hw, hh = HANDLE
    s.text((20, 16), "Hole cover: full size (1:1), millimetres", size=6)
    s.note((20, 25), [
        f"{fmt(PLATE)} x {fmt(PLATE)} plate of {fmt(PLY)} mm plywood, painted yellow and marked HOLE; a "
        f"{fmt(hl)} x {fmt(hw)} x {fmt(hh)} softwood handle on top at the middle;",
        f"two {fmt(CL)} x {fmt(CW)} x {fmt(CH)} plywood cleats underneath, parallel to the handle, outer faces "
        f"{fmt(2 * CY + CW)} apart, all four bottom edges bevelled {fmt(BEV)} at 45 degrees.",
        f"Seated in the {fmt(2 * H)} mm opening the cleats have {fmt(PLAY)} mm of play each side and {fmt(END_PLAY)} "
        f"mm each end. Modelled mass {T.COVER_MASS:.2f} kg (an estimate): weigh the built cover.",
        "Every size shown is a simulation number. Screw positions are suggested.",
    ], size=3.4)

    # plan from above: page right = the cover's x (along the handle)
    v = View(s, 255, 225)
    s.text((75, 68), "PLAN (from above; cleats underneath shown dashed)", size=4.5)
    v.rect(-p, -p, p, p, w=0.7, fill=C_YEL)
    v.rect(-hl / 2, -hw / 2, hl / 2, hw / 2, w=0.6, fill=C_WOOD)
    for sgn in (-1, 1):
        c = sgn * CY
        v.rect(-CL / 2, c - CW / 2, CL / 2, c + CW / 2, w=0.35, col=BROWN, dash=(3, 1.5))
        v.rect(-CL / 2 + BEV, c - CW / 2 + BEV, CL / 2 - BEV, c + CW / 2 - BEV, w=0.2, col=BROWN, dash=(1.2, 1.2))
        for x in (-CL / 3, 0, CL / 3):
            cross(v, (x, c))
    for x in (-hl * 0.3, hl * 0.3):
        v.circle((x, 0), 1.8, w=0.3)
    v.centre((-p - 6, 0), (p + 6, 0))
    v.centre((0, -p - 6), (0, p + 6))
    v.text((0, 0), "handle", dy=-8, size=3.2, anchor="middle")
    v.text((0, CY + CW / 2), "cleat (underneath)", dy=-2, size=3.2, anchor="middle", col=BROWN)
    v.dim_h(-p, p, p, 14)
    v.dim_h(-hl / 2, hl / 2, hw / 2, 12, text_dx=-25)
    v.dim_v(-hw / 2, hw / 2, hl / 2, 10, text_dy=-7)
    v.dim_h(-CL / 2, CL / 2, -CY - CW / 2, -12, text_dx=-25)
    v.dim_h(-p, -CL / 2, -CY - CW / 2, -12)
    v.dim_v(CY - CW / 2, CY + CW / 2, -CL / 2, -14)
    v.dim_v(-p, p, p, 44)
    v.dim_v(-CY - CW / 2, CY + CW / 2, p, 28)
    v.dim_v(-CY, CY, p, 12, text_dy=-12)
    v.dim_v(CY + CW / 2, p, -p, -12)
    v.text((-p, -p), "+  countersunk screw down through the plate into a cleat, head flush with the plate's top "
                     "(suggested)", dy=9, size=3.2)
    v.text((-p, -p), "o  countersunk screw up through the plate into the handle, head flush with the underside "
                     "(suggested)", dy=15, size=3.2)
    v.text((-p, -p), f"{fmt(2 * CY)}: between the cleats' centre lines.  {fmt(2 * CY + CW)}: over the cleats' outer "
                     "faces.", dy=21, size=3.2)

    # end view: looking along the handle
    e = View(s, 255, 470)
    s.text((75, 396), "END VIEW (looking along the handle), seated over the opening", size=4.5)
    e.rect(-p, 0, p, PLY, w=0.7, fill=C_YEL)
    e.rect(-hw / 2, PLY, hw / 2, PLY + hh, w=0.6, fill=C_WOOD)
    for sgn in (-1, 1):
        e.poly(cleat_section(sgn * CY), w=0.6, fill=C_WOOD)
        e.line((sgn * H, 0), (sgn * H, -CH - 14), w=0.3, col=GREY, dash=(3, 1.5))
        e.line((sgn * H, 0), (sgn * (p + 22), 0), w=0.3, col=GREY)
    e.text((-p - 22, 0), "deck's top", dy=-1.5, size=3.0, col=GREY)
    e.text((H, -CH - 14), "opening's edge", dx=2, dy=0, size=3.0, col=GREY)
    e.dim_v(0, PLY, p, 12)
    e.dim_v(PLY, PLY + hh, hw / 2, 12)
    e.dim_h(-hw / 2, hw / 2, PLY + hh, 9)
    e.dim_v(-CH, 0, -CY + CW / 2, 12)
    e.dim_h(-CY - CW / 2, CY + CW / 2, -CH, -12, label=f"{fmt(2 * CY + CW)} over the cleats")
    e.dim_h(-H, H, -CH, -25, label=f"{fmt(2 * H)} opening")
    e.dim_h(-H, -CY - CW / 2, -CH, -12, label=f"{fmt(PLAY)} play", text_dx=-28)
    e.dim_v(-CH, PLY + hh, p, 34)

    # side view: looking across the handle
    d = View(s, 255, 610)
    s.text((75, 536), "SIDE VIEW (looking across the handle)", size=4.5)
    d.rect(-p, 0, p, PLY, w=0.7, fill=C_YEL)
    d.rect(-hl / 2, PLY, hl / 2, PLY + hh, w=0.6, fill=C_WOOD)
    d.poly([(-CL / 2, 0), (CL / 2, 0), (CL / 2, -CH + BEV), (CL / 2 - BEV, -CH), (-CL / 2 + BEV, -CH),
            (-CL / 2, -CH + BEV)], w=0.6, fill=C_WOOD)
    d.dim_h(-hl / 2, hl / 2, PLY + hh, 10)
    d.dim_h(-CL / 2, CL / 2, -CH, -12)
    d.dim_h(-CL / 2 + BEV, CL / 2 - BEV, -CH, -24, label=f"{fmt(CL - 2 * BEV)} foot")
    d.dim_h(-p, p, -CH, -36)

    # the cleat's section at 2:1
    k = View(s, 470, 640, scale=2.0)
    s.text((432, 580), "CLEAT SECTION, 2:1", size=4.5)
    k.poly([(0, 0), (CW, 0), (CW, -CH + BEV), (CW - BEV, -CH), (BEV, -CH), (0, -CH + BEV)], w=0.6, fill=C_WOOD)
    k.line((-6, 0), (CW + 6, 0), w=0.3, col=GREY)
    k.text((-6, 0), "plate's underside", dx=-1.5, dy=1, size=3.0, anchor="end", col=GREY)
    k.dim_h(0, CW, 0, 10)
    k.dim_v(-CH, 0, CW, 16)
    k.dim_h(BEV, CW - BEV, -CH, -10, label=f"{fmt(CW - 2 * BEV)} foot")
    k.dim_v(-CH, -CH + BEV, 0, -10, label=f"{fmt(BEV)} x 45 deg")
    k.text((0, -CH), "(twice full size: do not trace)", dy=22, size=3.0)

    s.check_bar(40, 700)
    return s


def cut_list():
    hl, hw, hh = HANDLE
    ply = f"{fmt(PLY)} mm softwood (pine) plywood sheet"
    bat = (f"planed softwood batten, {fmt(BAT_H)} mm tall x {fmt(BATTEN_W)} mm wide (nominal 38 x 63 mm studwork "
           f"timber planed to {fmt(BAT_H)} mm is one option, an estimate: measure the stock)")
    n_deck = len(deck_screws())
    rows = [("part", "quantity", "length_mm", "width_mm", "thickness_or_height_mm", "stock", "notes")]
    rows.append(("deck top", 1, fmt(Y1 - Y0), fmt(X1 - X0), fmt(PLY), ply,
                 f"{fmt(2 * H)} x {fmt(2 * H)} mm square opening cut through its middle"))
    rows.append(("long batten", 2, fmt(Y1 - Y0), fmt(BATTEN_W), fmt(BAT_H), bat,
                 "inner face flush with the opening's edge; width is a build choice"))
    rows.append(("cross batten", 2, fmt(2 * H), fmt(BATTEN_W), fmt(BAT_H), bat,
                 "between the long battens, inner face flush with the opening's edge"))
    rows.append(("end batten", 2, fmt(2 * H), fmt(BATTEN_W), fmt(BAT_H), bat,
                 "between the long battens, under the deck's 340 mm edges"))
    rows.append(("edge filler (optional)", 2, fmt(Y1 - Y0), fmt(FILLER_W), fmt(BAT_H),
                 f"the same batten ripped to {fmt(FILLER_W)} mm wide",
                 "closes the overhang on the 440 mm sides, as the simulation models them"))
    rows.append(("cover plate", 1, fmt(PLATE), fmt(PLATE), fmt(PLY), ply, "painted yellow, marked HOLE"))
    rows.append(("cover handle", 1, fmt(hl), fmt(hw), fmt(hh),
                 f"planed softwood, {fmt(hw)} x {fmt(hh)} mm section (plane a larger square section down if "
                 f"{fmt(hw)} mm is not stocked)", "centred on top of the plate"))
    rows.append(("cover cleat", 2, fmt(CL), fmt(CW), fmt(CH), f"strip cut from the {fmt(PLY)} mm plywood",
                 f"all four bottom edges bevelled {fmt(BEV)} mm at 45 degrees; foot {fmt(CL - 2 * BEV)} x "
                 f"{fmt(CW - 2 * BEV)} mm"))
    rows.append(("wood screw, deck", n_deck, 40, "", 4, "4 x 40 mm countersunk wood screws (suggested size)",
                 "through the deck top into the battens, heads flush"))
    rows.append(("wood screw, cleats", 6, 30, "", 3.5, "3.5 x 30 mm countersunk wood screws (suggested size)",
                 "down through the plate into the cleats, heads flush; drill pilot holes"))
    rows.append(("wood screw, handle", 2, 40, "", 4, "4 x 40 mm countersunk wood screws (suggested size)",
                 "up through the plate into the handle, heads flush"))
    rows.append(("paint and marker", "", "", "", "", "yellow paint; black marker or paint for the word HOLE", ""))
    rows.append(("wrench (bought)", 1, fmt(mm(T.W_LEN)), "", fmt(mm(T.HEAD_T)),
                 "any 14 mm combination wrench about 180 mm long", "modelled sizes are typical ones (estimates)"))
    rows.append(("tool tote (bought)", 1, fmt(mm(T.TOTE_IN[0])), fmt(mm(T.TOTE_IN[1])), fmt(mm(T.TOTE_H)),
                 "any open tote of about this inside size", "length and width are inside sizes; height overall"))
    with open(HERE / "cut-list.csv", "w", newline="") as f:
        csv.writer(f).writerows(rows)


def assembly():
    hl, hw, hh = HANDLE
    n_deck = len(deck_screws())
    text = f"""# Building the deck and the hole cover

Written by `make_drawing.py` from the simulation's numbers. Drawings: `deck-1to1.svg` and `cover-1to1.svg` (full
size; each has a 100 mm check bar). Pieces: `cut-list.csv`. Tools: a saw (a jigsaw for the opening), a drill with a
countersink, a square, a plane or file, a rule. Nothing in this package has been built yet; sizes are the simulated
ones.

Not simulation numbers, and free to change: the batten width ({fmt(BATTEN_W)} mm), the screw sizes and positions,
and the edge fillers. The simulation fills the whole space under the plywood, so only the deck's outline, its
{fmt(DECK_H)} mm height and the opening matter.

## Deck

1. Cut the deck top, {fmt(Y1 - Y0)} x {fmt(X1 - X0)} mm, from {fmt(PLY)} mm plywood.
2. Mark the opening on it: a {fmt(2 * H)} x {fmt(2 * H)} mm square in the middle, {fmt(Y1 - H)} mm in from each
   {fmt(X1 - X0)} mm edge and {fmt(X1 - H)} mm in from each {fmt(Y1 - Y0)} mm edge. Drill a starter hole inside
   each corner, saw the square out and file the edges straight and square to the face. Check both widths measure
   {fmt(2 * H)} mm and the two diagonals are equal.
3. Cut the battens ({fmt(BAT_H)} mm tall, {fmt(BATTEN_W)} mm wide): two long battens {fmt(Y1 - Y0)} mm, two cross
   battens {fmt(2 * H)} mm, two end battens {fmt(2 * H)} mm. If the stock is not {fmt(BAT_H)} mm tall, plane it
   to {fmt(BAT_H)} mm, or build with it as it is and record the deck's built height with the results.
4. Lay the top face down. Stand the two long battens on it along the {fmt(Y1 - Y0)} mm direction, one each side
   of the opening, inner faces flush with the opening's edges. Fit the two cross battens between them, flush with
   the opening's other two edges. Fit the two end battens between the long battens under the top's
   {fmt(X1 - X0)} mm edges. Check with the square that all four inner faces are flush with the opening all the
   way down: the opening must be a straight-sided pocket with no ledge or gap for the wrench to slide under.
5. Turn it over and screw down through the top into the battens ({n_deck} screws at the marked positions, or as
   suits), countersunk so every head is flush with or below the deck's top: the cover slides on this face.
6. Optional: glue or pin the two edge fillers ({fmt(Y1 - Y0)} x {fmt(FILLER_W)} x {fmt(BAT_H)} mm) under the
   overhang on the two {fmt(Y1 - Y0)} mm sides, so those sides are solid as modelled.
7. On the deck's top draw a {fmt(2 * (H + LAP))} mm square round the opening, {fmt(LAP)} mm outside each edge.
   It is used to score the cover's lap.
8. Stand the deck on the table with its {fmt(Y1 - Y0)} mm sides across the robot's view (along y) and the
   opening's middle {fmt(OPEN_AT[0])} mm in front of the midpoint between the arm bases, on the centre line. Fix
   it to the table (clamps or screws; it must not move when the cover is pushed against the opening's edges).
9. Check: the deck's top is {fmt(DECK_H)} mm above the table, and the opening is {fmt(2 * H)} x {fmt(2 * H)} mm
   and {fmt(DECK_H)} mm deep down to the bare table.

## Hole cover

10. Cut the plate, {fmt(PLATE)} x {fmt(PLATE)} mm, from {fmt(PLY)} mm plywood. Cut the handle,
    {fmt(hl)} x {fmt(hw)} x {fmt(hh)} mm, from softwood. Cut two cleats, {fmt(CL)} x {fmt(CW)} mm, from the
    {fmt(PLY)} mm plywood ({fmt(CH)} mm tall as they hang under the plate).
11. Plane or file a {fmt(BEV)} mm bevel at 45 degrees on all four bottom edges of each cleat (both long edges
    and both ends), leaving a flat foot {fmt(CL - 2 * BEV)} x {fmt(CW - 2 * BEV)} mm.
12. On the plate's underside mark its two centre lines. Fix the cleats parallel to one centre line, their own
    centre lines {fmt(CY)} mm either side of it, so their outer faces are {fmt(2 * CY + CW)} mm apart, and
    centred end to end ({fmt(PLATE / 2 - CL / 2)} mm in from the plate's edges). Screw down through the plate
    into each cleat (three screws each, pilot holes drilled, heads flush with the plate's top), with glue if
    wanted.
13. Fix the handle on top of the plate at its middle, parallel to the cleats: two screws up through the plate
    from the underside, heads flush.
14. Paint the cover yellow and mark it HOLE.
15. Check: outer faces of the cleats {fmt(2 * CY + CW)} mm apart; cleats {fmt(CL)} mm long. Seat the cover on
    the deck: the plate lies flat on the deck all round, and the cover can be pushed {fmt(2 * PLAY)} mm from stop
    to stop both along and across the handle ({fmt(PLAY)} mm of play each way). It seats the same turned a
    quarter turn. With the cover pushed to any stop the drawn square stays hidden.
16. Weigh the finished cover and note the reading with the results. The modelled mass is {T.COVER_MASS:.2f} kg,
    an estimate from typical densities; the arm's payload is about 1 kg, so use a lighter plywood if the built
    cover comes out heavier than that.
"""
    (HERE / "ASSEMBLY.md").write_text(text)


def main():
    for name, sheet in (("deck-1to1", deck_sheet()), ("cover-1to1", cover_sheet())):
        (HERE / f"{name}.svg").write_text(sheet.svg())
        for w in sheet.png(HERE / f"{name}.png"):
            print(f"{name}: {w}")
    cut_list()
    assembly()


if __name__ == "__main__":
    main()
