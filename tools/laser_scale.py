"""Printable wall scale for the laser-pointer heading test (requirement 2 truth).

Usage: python tools/laser_scale.py [--distance-mm 2480] [--out Reports/laser_scale_2480mm.pdf]

A laser on the deck shines on a wall at a perpendicular distance D from the table's rotation axis.
Turning the table by theta moves the dot by x = D tan(theta) from the zero mark (exact when the beam
passes through the axis; a sideways laser offset d adds only d (sec theta - 1), < 0.4 mm at 5 deg for
d = 10 cm). Ticks: every 0.5 deg out to +-5 deg, every 0.1 deg inside +-1 deg. Three landscape
letter pages, overlapping so the centre page holds +-2 deg in one piece; tape them so the shared
ticks coincide. Print at 100 % ("Actual size"); each page has a 100 mm check bar.

Sign: seen from the table looking at the wall, a counter-clockwise turn (positive firmware yaw,
z up) moves the dot to the LEFT, so positive angles are on the left.
"""
import argparse, math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

W, H = 279.4, 215.9  # letter landscape, mm
BASE = 28.0           # baseline height above the page bottom, mm


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--distance-mm', type=float, default=2480.0)
    ap.add_argument('--out', default='Reports/laser_scale_2480mm.pdf')
    ap.add_argument('--png', help='also save page previews as <prefix>_p<n>.png')
    a = ap.parse_args()
    D = a.distance_mm
    x = lambda deg: D * math.tan(math.radians(deg))

    # (title, centre angle, angle range shown) - positive on the left as seen on the wall
    pages = [('LEFT page (+ side)', 3.25, (1.3, 5.2)), ('CENTRE page', 0.0, (-2.0, 2.0)),
             ('RIGHT page (− side)', -3.25, (-5.2, -1.3))]
    with PdfPages(a.out) as pdf:
        for n, (title, c, (lo, hi)) in enumerate(pages, 1):
            fig = plt.figure(figsize=(W / 25.4, H / 25.4))
            ax = fig.add_axes([0, 0, 1, 1])
            ax.set_xlim(0, W); ax.set_ylim(0, H); ax.axis('off')
            u = lambda deg: W / 2 - (x(deg) - x(c))  # page x (mm); mirrored so + is on the left
            # ticks in hundredths of a degree to avoid float drift
            for k in range(-520, 521, 10):
                deg = k / 100.0
                if not (lo - 1e-9 <= deg <= hi + 1e-9):
                    continue
                major = k % 100 == 0
                half = k % 50 == 0
                if not (major or half or abs(deg) <= 1.0 + 1e-9):
                    continue  # 0.1-deg ticks only within +-1 deg
                if k == 0:
                    L, lw, col = 160, 1.6, '#c0392b'
                elif major:
                    L, lw, col = 150, 1.1, 'k'
                elif half:
                    L, lw, col = 118, 0.8, 'k'
                elif k % 20 == 0:
                    L, lw, col = 90, 0.55, 'k'
                else:
                    L, lw, col = 76, 0.45, 'k'
                X = u(deg)
                ax.plot([X, X], [BASE, BASE + L], color=col, lw=lw, solid_capstyle='butt')
                if major or k == 0:
                    ax.text(X, BASE + L + 3, f'{deg:+.0f}°' if k else '0°', ha='center', va='bottom', fontsize=20,
                            fontweight='bold', color=col)
                elif half:
                    ax.text(X, BASE + L + 3, f'{deg:+.1f}', ha='center', va='bottom', fontsize=12)
                elif k % 20 == 0:
                    ax.text(X, BASE + L + 2, f'{deg:+.1f}', ha='center', va='bottom', fontsize=7)
            # baseline across the shown range
            ax.plot([u(hi), u(lo)], [BASE, BASE], color='k', lw=0.6)
            # 100 mm check bar
            ax.plot([12, 112], [12, 12], color='k', lw=1.2, solid_capstyle='butt')
            for e in (12, 112):
                ax.plot([e, e], [9, 15], color='k', lw=0.8)
            ax.text(62, 15.5, 'this bar must measure 100 mm (print at 100 % / Actual size)', ha='center', fontsize=7)
            ax.text(W - 10, 10, f'Laser heading scale, wall {D:.0f} mm from the rotation axis: x = {D:.0f} mm × tan θ\n'
                    f'Ticks: 0.5° to ±5°, 0.1° within ±1°. Positive = table turns counter-clockwise from above '
                    f'(firmware yaw +): dot moves LEFT.\n{title}, page {n} of 3. Tape the pages so the shared ticks overlap exactly.',
                    ha='right', va='bottom', fontsize=7, linespacing=1.4)
            pdf.savefig(fig)
            if a.png:
                fig.savefig(f'{a.png}_p{n}.png', dpi=60)
            plt.close(fig)
    print('wrote', a.out)
    for deg in (0.1, 0.5, 1, 2, 5):
        print(f'  {deg:4.1f} deg -> {x(deg):6.1f} mm from zero')


if __name__ == '__main__':
    main()
