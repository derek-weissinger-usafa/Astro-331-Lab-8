"""Fit the room's magnetic distortion at the table's spot (heading error vs field direction).

Usage: python tools/mag_room_fit.py <LOG.CSV> [--spinning] [--t0 s] [--t1 s] [--hard x,y] [--plot out.png]

Needs slow full turns on the air bearing with the aid OFF (MAG OFF), so the logged yaw is gyro-only:
e.g. 2 turns each way at the same steady pace, 30-40 s per turn. Do them in the wheel state the table
runs in: --spinning takes Manual rows with the wheels held at constant speed (e.g. SPEED A 100,
B -100, C 100, D -100: zero net momentum). With the wheels STOPPED the field also carries a random
offset from where each motor shaft stopped (2026-10-31: ~5 uT, ~+-15 deg at 15 uT), so a stopped-wheel
fit doesn't carry over to spinning wheels. Works on an SD log or on rows captured from the XBee
(same 50 columns, no header needed).

Nearby steel (desk legs, rebar, the stand) bends Earth's field so the magnetometer heading is wrong
by an amount that depends on which way the table faces. With the table hard iron removed (--hard,
default the firmware's kMagHardIronUt), the body-frame direction of the horizontal field, theta, is
fitted (linear least squares) as
    theta = theta0 + k * yaw + c1 cos th + s1 sin th + c2 cos 2th + s2 sin 2th
k ~ -1 (gyro scale), and e(theta) = the Fourier part is what the firmware removes
(mag_aid.h setRoomCorrection). Prints SET MAG_C1/S1/C2/S2 (deg) and, when both turn directions
are present, a cross-check: fitted on one direction, tested on the other.
"""
import argparse, csv, math, os, re, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.join(HERE, '..', 'Astro 331 Lab 8')
# Data-row columns (data_logger.cpp); used when the file has no header (XBee capture)
COLS = {'mcutime_ms': 0, 'mag_x_uT': 7, 'mag_y_uT': 8, 'yaw_deg': 24, 'state': 32,
        'motor_a_measured_rate_rpm': 10, 'motor_b_measured_rate_rpm': 11,
        'motor_c_measured_rate_rpm': 12, 'motor_d_measured_rate_rpm': 13}


def fw_hard_iron():
    src = open(os.path.join(FW, 'include', 'control_params.h'), encoding='utf-8').read()
    m = re.search(r'kMagHardIronUt\[3\]\s*=\s*\{([^}]*)\}', src)
    return [float(v.strip().rstrip('f')) for v in m.group(1).split(',')][:2]


def load(path):
    rows = list(csv.reader(open(path, newline='')))
    if rows and not rows[0][0][:1].isdigit():
        ci = {h: i for i, h in enumerate(rows[0])}
        rows = rows[1:]
    else:
        ci = COLS
    out = []
    for r in rows:
        if len(r) < 33 or not r[0][:1].isdigit():
            continue
        try:
            w = max(abs(float(r[ci[f'motor_{x}_measured_rate_rpm']])) for x in 'abcd')
            out.append((float(r[ci['mcutime_ms']]) * 1e-3, float(r[ci['mag_x_uT']]), float(r[ci['mag_y_uT']]),
                        float(r[ci['yaw_deg']]), r[ci['state']][:1], w))
        except (ValueError, IndexError):
            continue
    return out


def design(theta, yaw):
    return np.column_stack([np.ones_like(theta), yaw, np.cos(theta), np.sin(theta), np.cos(2 * theta),
                            np.sin(2 * theta)])


def binned_pp(theta_wrapped, err_deg):
    b = ((np.degrees(theta_wrapped) + 180.0) // 30).astype(int)
    means = [err_deg[b == i].mean() for i in range(12) if (b == i).sum() > 2]
    return np.ptp(means) if len(means) > 2 else float('nan'), len(means)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log')
    ap.add_argument('--spinning', action='store_true', help='use Manual rows with the wheels spinning')
    ap.add_argument('--t0', type=float, default=0.0, help='start, s from the first row')
    ap.add_argument('--t1', type=float, default=1e9)
    ap.add_argument('--hard', help='table hard iron x,y uT (default: firmware kMagHardIronUt)')
    ap.add_argument('--plot')
    a = ap.parse_args()
    hx, hy = [float(v) for v in a.hard.split(',')] if a.hard else fw_hard_iron()

    d = load(a.log)
    if not d:
        sys.exit('no data rows')
    t = np.array([r[0] for r in d]); t -= t[0]
    if a.spinning:
        ok = [r[4] == 'M' and r[5] > 50.0 for r in d]  # Manual, wheels at speed (rpm)
    else:
        ok = [r[4] == 'I' and r[5] < 1.0 for r in d]
    keep = (t >= a.t0) & (t <= a.t1) & np.array(ok)
    if keep.sum() < 50:
        sys.exit(f'only {keep.sum()} usable rows in the window ({"Manual, spinning" if a.spinning else "Idle, stopped"})')
    idx = np.where(keep)[0]
    yaw = np.unwrap(np.radians([d[i][3] for i in idx]))
    mx = np.array([d[i][1] for i in idx]) - hx
    my = np.array([d[i][2] for i in idx]) - hy
    th_w = np.arctan2(my, mx)
    th = np.unwrap(th_w)
    span = math.degrees(np.ptp(yaw))
    print(f'{len(idx)} rows ({"Manual, wheels spinning" if a.spinning else "Idle, wheels stopped"}), yaw span {span:.0f} deg, hard iron ({hx:+.1f}, {hy:+.1f}) uT, '
          f'horizontal field {np.hypot(mx, my).mean():.1f} uT')
    if span < 300:
        print('WARNING: less than a full turn: the fit is poorly constrained')

    def fit(sel):
        c, *_ = np.linalg.lstsq(design(th[sel], yaw[sel]), th[sel], rcond=None)
        return c

    def resid(c, sel, with_room=True):
        X = design(th[sel], yaw[sel])
        cc = c.copy()
        if not with_room:
            cl, *_ = np.linalg.lstsq(X[:, :2], th[sel], rcond=None)
            return np.degrees(th[sel] - X[:, :2] @ cl)
        return np.degrees(th[sel] - X @ cc)

    allsel = np.ones(len(idx), bool)
    c = fit(allsel)
    pp0, nb = binned_pp(th_w, resid(c, allsel, with_room=False))
    pp1, _ = binned_pp(th_w, resid(c, allsel))
    coef = np.degrees(c[2:])
    print(f'gyro scale seen by the magnetometer: true/gyro = {-c[1]:.4f}')
    print(f'heading error vs field direction (30-deg bins, {nb}/12 filled): {pp0:.2f} deg p-p uncorrected, '
          f'{pp1:.2f} deg p-p after the correction')
    print(f'coefficients (deg): c1 {coef[0]:+.2f} s1 {coef[1]:+.2f} c2 {coef[2]:+.2f} s2 {coef[3]:+.2f}')
    print(f'  firmware: SET MAG_C1 {coef[0]:.2f}   SET MAG_S1 {coef[1]:.2f}   SET MAG_C2 {coef[2]:.2f}   '
          f'SET MAG_S2 {coef[3]:.2f}')

    # Cross-check: fit on one turn direction, test on the other.
    rate = np.gradient(yaw)
    ccw, cw = rate > 0, rate < 0
    if math.degrees(np.ptp(yaw[ccw])) > 300 and math.degrees(np.ptp(yaw[cw])) > 300 if ccw.any() and cw.any() else False:
        for name, tr, te in (('CCW fit -> CW test', ccw, cw), ('CW fit -> CCW test', cw, ccw)):
            ct = fit(tr)
            X = design(th[te], yaw[te])
            cl, *_ = np.linalg.lstsq(X[:, :2], th[te] - X[:, 2:] @ ct[2:], rcond=None)  # own offset/scale
            e_after = np.degrees(th[te] - X[:, :2] @ cl - X[:, 2:] @ ct[2:])
            cl0, *_ = np.linalg.lstsq(X[:, :2], th[te], rcond=None)
            e_before = np.degrees(th[te] - X[:, :2] @ cl0)
            print(f'  {name}: {binned_pp(th_w[te], e_before)[0]:.2f} -> {binned_pp(th_w[te], e_after)[0]:.2f} deg p-p')
    else:
        print('  (no cross-check: needs a full turn in each direction)')

    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(np.degrees(th_w), resid(c, allsel, with_room=False), '.', ms=3, label='uncorrected')
        ax.plot(np.degrees(th_w), resid(c, allsel), '.', ms=3, label='after correction')
        ax.set(xlabel='field direction in body frame (deg)', ylabel='heading error (deg)')
        ax.legend()
        fig.tight_layout()
        fig.savefig(a.plot, dpi=130)
        print(f'wrote {a.plot}')


if __name__ == '__main__':
    main()
