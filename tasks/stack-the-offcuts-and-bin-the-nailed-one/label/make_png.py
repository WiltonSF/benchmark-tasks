"""Draws denail.png, the printed label on the tote's front face: black DENAIL capitals on safety yellow.

Pure Python (zlib only), no font files: each letter is drawn from straight strokes and ellipse arcs given in
millimetres on the 220 x 70 mm label, at 5 px/mm with 2 x 2 supersampling. Run it from anywhere:

    python label/make_png.py
"""

import pathlib
import struct
import zlib

W_MM, H_MM = 220.0, 70.0  # label size
PX_PER_MM = 5
SS = 2  # supersampling per axis
YELLOW = (255, 205, 0)
BLACK = (22, 22, 22)
CAP, STROKE, GAP = 44.0, 8.5, 7.0  # letter height, stroke width and spacing (mm)
BASE = (H_MM - CAP) / 2
FRAME_IN, FRAME_W = 3.0, 2.5  # a black rule 3 mm in from the label's edge, 2.5 mm wide


def _d(x, y, w, h=CAP, s=STROKE):
    r = 16.0
    def bowl(x, y, x0, rx, ry, cy):
        return x >= x0 and ((x - x0) / rx) ** 2 + ((y - cy) / ry) ** 2 <= 1.0
    outer = (0 <= x <= w - r and 0 <= y <= h) or bowl(x, y, w - r, r, h / 2, h / 2)
    inner = x >= s and ((x <= w - r and s <= y <= h - s) or bowl(x, y, w - r, r - s, h / 2 - s, h / 2))
    return outer and not inner


def _e(x, y, w, h=CAP, s=STROKE):
    return x <= s or y <= s or y >= h - s or (abs(y - h / 2) <= s / 2 and x <= 0.86 * w)


def _n(x, y, w, h=CAP, s=STROKE):
    xc = s / 2 + (w - s) * (1 - y / h)
    return x <= s or x >= w - s or abs(x - xc) <= 0.62 * s


def _a(x, y, w, h=CAP, s=STROKE):
    left = s * 0.55 + (w / 2 - s * 0.55) * (y / h)
    right = w - left
    bar = 0.22 * h <= y <= 0.22 * h + 0.85 * s and left <= x <= right
    return abs(x - left) <= 0.58 * s or abs(x - right) <= 0.58 * s or bar


def _i(x, y, w, h=CAP, s=STROKE):
    return True


def _l(x, y, w, h=CAP, s=STROKE):
    return x <= s or y <= s


LETTERS = [(_d, 31.0), (_e, 25.0), (_n, 31.0), (_a, 33.0), (_i, STROKE), (_l, 24.0)]  # D E N A I L


def ink(x, y):
    """True where the label is black, at (x, y) mm from its lower-left corner."""
    if FRAME_IN <= x <= W_MM - FRAME_IN and FRAME_IN <= y <= H_MM - FRAME_IN:
        if not (FRAME_IN + FRAME_W < x < W_MM - FRAME_IN - FRAME_W and FRAME_IN + FRAME_W < y < H_MM - FRAME_IN - FRAME_W):
            return True
    if not BASE <= y <= BASE + CAP:
        return False
    total = sum(w for _, w in LETTERS) + GAP * (len(LETTERS) - 1)
    x0 = (W_MM - total) / 2
    for fn, w in LETTERS:
        if x0 <= x <= x0 + w:
            return fn(x - x0, y - BASE, w)
        x0 += w + GAP
    return False


def render(path):
    wpx, hpx = int(W_MM * PX_PER_MM), int(H_MM * PX_PER_MM)
    rows = []
    for row in range(hpx):  # PNG rows run top to bottom
        line = bytearray([0])
        for col in range(wpx):
            k = 0
            for i in range(SS):
                for j in range(SS):
                    x = (col + (i + 0.5) / SS) / PX_PER_MM
                    y = H_MM - (row + (j + 0.5) / SS) / PX_PER_MM
                    k += ink(x, y)
            f = k / (SS * SS)
            line += bytes(round(YELLOW[c] * (1 - f) + BLACK[c] * f) for c in range(3))
        rows.append(bytes(line))
    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", wpx, hpx, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(b"".join(rows), 9)) + chunk(b"IEND", b"")
    pathlib.Path(path).write_bytes(png)


if __name__ == "__main__":
    render(pathlib.Path(__file__).resolve().parent / "denail.png")
