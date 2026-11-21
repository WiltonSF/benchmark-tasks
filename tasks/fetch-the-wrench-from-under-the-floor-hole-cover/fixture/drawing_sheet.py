"""A small drawing sheet used by make_drawing.py. Shapes, text and dimensions are collected in millimetres and written
as an SVG (1 unit = 1 mm, so a view at scale 1 prints full size at 100 %) and, when Pillow is installed, as a PNG
preview of the same sheet. No other dependencies.
"""

from __future__ import annotations

import math

BLUE = "#0b5cad"  # dimensions
GREY = "#777777"


def fmt(v: float) -> str:
    """A length in mm with at most one decimal and no trailing zeros."""
    s = f"{v:.1f}"
    return s.rstrip("0").rstrip(".") if "." in s else s


class Sheet:
    def __init__(self, width: float, height: float):
        self.w, self.h = float(width), float(height)
        self.ops: list[tuple] = []

    # -- primitives (page millimetres, y down) --------------------------------------------------------------------
    def line(self, a, b, w=0.35, col="#000", dash=None):
        self.ops.append(("line", tuple(a), tuple(b), w, col, dash))

    def poly(self, pts, w=0.5, col="#000", fill=None, dash=None):
        self.ops.append(("poly", [tuple(p) for p in pts], w, col, fill, dash))

    def rect(self, x0, y0, x1, y1, **kw):
        self.poly([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], **kw)

    def circle(self, c, r, w=0.5, col="#000", fill=None, dash=None):
        self.ops.append(("circle", tuple(c), float(r), w, col, fill, dash))

    def text(self, p, s, size=3.5, anchor="start", col="#000"):
        self.ops.append(("text", tuple(p), str(s), float(size), anchor, col))

    def note(self, p, lines, size=3.5, gap=1.45, col="#000"):
        for i, s in enumerate(lines):
            self.text((p[0], p[1] + i * size * gap), s, size=size, col=col)

    def arrow(self, tip, direction, size=2.2, col=BLUE):
        """A filled arrowhead with its point at ``tip``, pointing along ``direction``."""
        dx, dy = direction
        n = math.hypot(dx, dy)
        dx, dy = dx / n, dy / n
        bx, by = tip[0] - dx * size, tip[1] - dy * size
        self.poly([tip, (bx - dy * size * 0.3, by + dx * size * 0.3), (bx + dy * size * 0.3, by - dx * size * 0.3)],
                  w=0.1, col=col, fill=col)

    def dim_h(self, x0, x1, y_obj, y_dim, label, size=3.2, col=BLUE, text_dx=0.0):
        """A horizontal dimension between page x0 and x1, measured at y_obj, drawn at y_dim."""
        x0, x1 = min(x0, x1), max(x0, x1)
        far = 1.5 if y_dim > y_obj else -1.5
        for x in (x0, x1):
            self.line((x, y_obj + (1.0 if y_dim > y_obj else -1.0)), (x, y_dim + far), w=0.18, col=col)
        if x1 - x0 >= 9.0:
            self.line((x0, y_dim), (x1, y_dim), w=0.18, col=col)
            self.arrow((x0, y_dim), (-1, 0), col=col)
            self.arrow((x1, y_dim), (1, 0), col=col)
            self.text(((x0 + x1) / 2 + text_dx, y_dim - 1.0), label, size=size, anchor="middle", col=col)
        else:  # too short for arrows inside: arrows outside, label past the right-hand end
            self.line((x0 - 5.0, y_dim), (x1 + 5.0, y_dim), w=0.18, col=col)
            self.arrow((x0, y_dim), (1, 0), col=col)
            self.arrow((x1, y_dim), (-1, 0), col=col)
            self.text((x1 + 6.0 + text_dx, y_dim + size * 0.35), label, size=size, col=col)

    def dim_v(self, y0, y1, x_obj, x_dim, label, size=3.2, col=BLUE, text_dy=0.0):
        """A vertical dimension between page y0 and y1, measured at x_obj, drawn at x_dim; the label is written
        level, beside the dimension line on the side away from the object."""
        y0, y1 = min(y0, y1), max(y0, y1)
        right = x_dim > x_obj
        far = 1.5 if right else -1.5
        for y in (y0, y1):
            self.line((x_obj + (1.0 if right else -1.0), y), (x_dim + far, y), w=0.18, col=col)
        if y1 - y0 >= 9.0:
            self.line((x_dim, y0), (x_dim, y1), w=0.18, col=col)
            self.arrow((x_dim, y0), (0, -1), col=col)
            self.arrow((x_dim, y1), (0, 1), col=col)
        else:
            self.line((x_dim, y0 - 5.0), (x_dim, y1 + 5.0), w=0.18, col=col)
            self.arrow((x_dim, y0), (0, 1), col=col)
            self.arrow((x_dim, y1), (0, -1), col=col)
        tx = x_dim + (2.2 if right else -2.2)
        self.text((tx, (y0 + y1) / 2 + size * 0.35 + text_dy), label, size=size, anchor="start" if right else "end",
                  col=col)

    def check_bar(self, x, y, length=100.0):
        """A bar that must measure ``length`` mm on the print."""
        self.line((x, y), (x + length, y), w=0.8)
        for i in range(11):
            xx = x + length * i / 10
            self.line((xx, y), (xx, y - (3.0 if i % 5 == 0 else 1.6)), w=0.3)
        self.text((x, y + 5.0), f"{fmt(length)} mm check bar: must measure {fmt(length)} mm on the print "
                                "(print at 100 %, no fit-to-page)", size=3.2)

    # -- output ---------------------------------------------------------------------------------------------------
    def svg(self) -> str:
        def esc(s):
            return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")

        def style(w, col, fill, dash):
            d = f' stroke-dasharray="{dash[0]:g},{dash[1]:g}"' if dash else ""
            return f'fill="{fill or "none"}" stroke="{col}" stroke-width="{w:g}"{d}'

        out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w:g}mm" height="{self.h:g}mm" '
               f'viewBox="0 0 {self.w:g} {self.h:g}">', f'<rect width="{self.w:g}" height="{self.h:g}" fill="white"/>']
        for op in self.ops:
            if op[0] == "line":
                _, a, b, w, col, dash = op
                out.append(f'<line x1="{a[0]:.2f}" y1="{a[1]:.2f}" x2="{b[0]:.2f}" y2="{b[1]:.2f}" '
                           f'{style(w, col, None, dash)}/>')
            elif op[0] == "poly":
                _, pts, w, col, fill, dash = op
                out.append('<polygon points="' + " ".join(f"{x:.2f},{y:.2f}" for x, y in pts) + '" '
                           + style(w, col, fill, dash) + "/>")
            elif op[0] == "circle":
                _, c, r, w, col, fill, dash = op
                out.append(f'<circle cx="{c[0]:.2f}" cy="{c[1]:.2f}" r="{r:.2f}" {style(w, col, fill, dash)}/>')
            else:
                _, p, s, size, anchor, col = op
                out.append(f'<text x="{p[0]:.2f}" y="{p[1]:.2f}" font-size="{size:g}" font-family="Helvetica, Arial, '
                           f'sans-serif" text-anchor="{anchor}" fill="{col}">{esc(s)}</text>')
        out.append("</svg>")
        return "\n".join(out) + "\n"

    def png(self, path, px_per_mm=3.0) -> list[str]:
        """Writes a PNG preview; returns warnings about labels that overlap each other or leave the sheet.
        Returns ["no Pillow"] and writes nothing when Pillow is not installed."""
        try:
            from PIL import Image, ImageDraw, ImageFont
        except ImportError:
            return ["Pillow is not installed: PNG preview not written"]
        k = px_per_mm
        img = Image.new("RGB", (round(self.w * k), round(self.h * k)), "white")
        d = ImageDraw.Draw(img)
        fonts: dict = {}

        def font(size):
            if size not in fonts:
                fonts[size] = ImageFont.load_default(size=max(6, round(size * k)))
            return fonts[size]

        def seg(a, b, w, col, dash):
            width = max(1, round(w * k))
            if not dash:
                d.line([(a[0] * k, a[1] * k), (b[0] * k, b[1] * k)], fill=col, width=width)
                return
            n = math.hypot(b[0] - a[0], b[1] - a[1])
            if n < 1e-9:
                return
            ux, uy = (b[0] - a[0]) / n, (b[1] - a[1]) / n
            t = 0.0
            while t < n:
                e = min(n, t + dash[0])
                d.line([((a[0] + ux * t) * k, (a[1] + uy * t) * k), ((a[0] + ux * e) * k, (a[1] + uy * e) * k)],
                       fill=col, width=width)
                t += dash[0] + dash[1]

        boxes = []
        for op in self.ops:
            if op[0] == "line":
                _, a, b, w, col, dash = op
                seg(a, b, w, col, dash)
            elif op[0] == "poly":
                _, pts, w, col, fill, dash = op
                if fill:
                    d.polygon([(x * k, y * k) for x, y in pts], fill=fill)
                for i in range(len(pts)):
                    seg(pts[i], pts[(i + 1) % len(pts)], w, col, dash)
            elif op[0] == "circle":
                _, c, r, w, col, fill, dash = op
                if fill:
                    d.ellipse([(c[0] - r) * k, (c[1] - r) * k, (c[0] + r) * k, (c[1] + r) * k], fill=fill)
                n = max(24, int(2 * math.pi * r * k / 4))
                ring = [(c[0] + r * math.cos(2 * math.pi * i / n), c[1] + r * math.sin(2 * math.pi * i / n))
                        for i in range(n)]
                if dash:
                    for i in range(0, n, 2):
                        seg(ring[i], ring[(i + 1) % n], w, col, None)
                else:
                    for i in range(n):
                        seg(ring[i], ring[(i + 1) % n], w, col, None)
            else:
                _, p, s, size, anchor, col = op
                a = {"start": "ls", "middle": "ms", "end": "rs"}[anchor]
                d.text((p[0] * k, p[1] * k), s, font=font(size), fill=col, anchor=a)
                boxes.append((d.textbbox((p[0] * k, p[1] * k), s, font=font(size), anchor=a), s))
        img.save(path)
        warn = []
        for i, (b, s) in enumerate(boxes):
            if b[0] < 0 or b[1] < 0 or b[2] > img.width or b[3] > img.height:
                warn.append(f"label leaves the sheet: {s!r}")
            for c, t in boxes[i + 1:]:
                if b[0] < c[2] - 1 and c[0] < b[2] - 1 and b[1] < c[3] - 1 and c[1] < b[3] - 1:
                    warn.append(f"labels overlap: {s!r} and {t!r}")
        return warn


class View:
    """A view on a sheet: model millimetres (x right, y up) placed with its origin at page (ox, oy) at ``scale``."""

    def __init__(self, sheet: Sheet, ox: float, oy: float, scale: float = 1.0):
        self.s, self.ox, self.oy, self.k = sheet, float(ox), float(oy), float(scale)

    def P(self, x, y):
        return self.ox + x * self.k, self.oy - y * self.k

    def line(self, a, b, **kw):
        self.s.line(self.P(*a), self.P(*b), **kw)

    def rect(self, x0, y0, x1, y1, **kw):
        self.s.poly([self.P(x0, y0), self.P(x1, y0), self.P(x1, y1), self.P(x0, y1)], **kw)

    def poly(self, pts, **kw):
        self.s.poly([self.P(*p) for p in pts], **kw)

    def circle(self, c, r, **kw):
        self.s.circle(self.P(*c), r * self.k, **kw)

    def text(self, p, s, dx=0.0, dy=0.0, **kw):
        x, y = self.P(*p)
        self.s.text((x + dx, y + dy), s, **kw)

    def centre(self, a, b, col=GREY):
        self.line(a, b, w=0.18, col=col, dash=(6, 2))

    def dim_h(self, x0, x1, y, off, label=None, **kw):
        """Between model x0 and x1 at model height y; the dimension line ``off`` page mm above (+) or below (-)."""
        (px0, py), (px1, _) = self.P(x0, y), self.P(x1, y)
        self.s.dim_h(px0, px1, py, py - off, fmt(abs(x1 - x0)) if label is None else label, **kw)

    def dim_v(self, y0, y1, x, off, label=None, **kw):
        """Between model y0 and y1 at model x; the dimension line ``off`` page mm to the right (+) or left (-)."""
        (px, py0), (_, py1) = self.P(x, y0), self.P(x, y1)
        self.s.dim_v(py0, py1, px, px + off, fmt(abs(y1 - y0)) if label is None else label, **kw)
