"""Balance assistant: where the table's centre of mass is, and which trim moves fix it.

Usage: python tools/balance.py <LOG.CSV> [--mass kg] [--ixx kg m^2] [--iyy kg m^2] [--period-target s]
                               [--trim g] [--level-offset roll,pitch] [--plot out.png]

Needs a stretch of the log where the table swings FREELY on the air bearing: Idle, wheels stopped (a
DISARM that braked the wheels and set the table swinging is ideal; or nudge the table by hand while
logging with BEGIN STATIC TEST 0). Each such stretch (>= 8 s, swing >= 0.3 deg) is fitted with a damped
sinusoid per axis (roll, pitch):  angle = rest + A e^(-s t) cos(w t + phi).

Physics (the table is a pendulum about the bearing centre when its centre of mass sits below it):
  gravity stiffness  k = m g L = I w^2   =>   L = I w^2 / (m g)        (L: centre-of-mass distance)
  at rest the centre of mass hangs straight below the bearing centre, so in body axes
  r_cm = -L * (accelerometer direction at rest)    (the accelerometer reads "up" when still)
Horizontal parts of r_cm make the rest tilt; the vertical part sets the swing period. A balanced
table for the slews needs the period >= ~55 s (Reports/Slew_and_Fault_Simulation.html: CG within
~0.02 mm). Trim: moving a mass dm by d shifts the centre of mass by dm d / m, so the needed mass
moment is m * (shift) [g mm], shown for a --trim g weight (default 20 g).

Caveats: L scales with I (the table inertia is still a model: 0.0623 kg m^2; DEMO SPIN suggested
~0.08 about z), and m (default 5 kg) is an estimate: weigh the table. The rest tilt is the IMU's: if
the IMU isn't mounted level, give --level-offset with the IMU roll,pitch (deg) read when the table is
physically level (spirit level on the deck), and the horizontal offsets are measured from that.
Body axes (frames.h): +x toward wheel A, +y toward wheel B, +z up.
"""
import argparse, csv, math, sys
import numpy as np

G = 9.81


def load(path):
    with open(path, newline='') as f:
        rd = csv.reader(f)
        hdr = next(rd)
        ci = {h: i for i, h in enumerate(hdr)}
        need = ['mcutime_ms', 'roll_deg', 'pitch_deg', 'accel_x_m_s2', 'accel_y_m_s2', 'accel_z_m_s2', 'state'] + \
               [f'motor_{w}_measured_rate_rpm' for w in 'abcd']
        rows = []
        for r in rd:
            if not r or not r[0][:1].isdigit() or len(r) < len(hdr) - 8:
                continue
            try:
                rows.append([float(r[ci['mcutime_ms']]) * 1e-3, float(r[ci['roll_deg']]), float(r[ci['pitch_deg']]),
                             float(r[ci['accel_x_m_s2']]), float(r[ci['accel_y_m_s2']]), float(r[ci['accel_z_m_s2']]),
                             1.0 if r[ci['state']][:1] == 'I' else 0.0] +
                            [float(r[ci[f'motor_{w}_measured_rate_rpm']]) for w in 'abcd'])
            except (ValueError, IndexError):
                continue
    return np.array(rows)


def segments(d, min_len=8.0, min_pp=0.3):
    """Idle, wheels stopped, swinging stretches: list of index arrays."""
    ok = (d[:, 6] > 0.5) & (np.abs(d[:, 7:11]).max(axis=1) < 3.0)
    out, start = [], None
    for i, v in enumerate(np.append(ok, False)):
        if v and start is None:
            start = i
        elif not v and start is not None:
            k = np.arange(start, i)
            start = None
            if d[k[-1], 0] - d[k[0], 0] < min_len:
                continue
            k = k[d[k, 0] - d[k[0], 0] > 0.5]  # skip the braking moment
            k = k[d[k, 0] - d[k[0], 0] < 20.0]  # first 20 s: later the swing is often disturbed (nudges, ARM)
            pp = max(np.ptp(d[k, 1]), np.ptp(d[k, 2]))
            if pp >= min_pp:
                out.append(k)
    return out


def fit_damped(t, x):
    """Least squares x = c + e^(-s t)(a cos wt + b sin wt) over a (w, s) grid. Returns w, s, c, amp, rms."""
    t = t - t[0]
    dt = np.median(np.diff(t))
    xd = x - x.mean()
    n = 1 << int(math.ceil(math.log2(len(xd) * 8)))
    spec = np.abs(np.fft.rfft(xd * np.hanning(len(xd)), n))
    f = np.fft.rfftfreq(n, dt)
    band = (f > 0.02) & (f < 3.0)
    w0 = 2 * math.pi * f[band][np.argmax(spec[band])]
    best = None
    for w in np.linspace(0.7 * w0, 1.3 * w0, 241):
        c, s_ = np.cos(w * t), np.sin(w * t)
        for s in np.linspace(0.0, 1.0, 41):
            e = np.exp(-s * t)
            A = np.column_stack([np.ones_like(t), e * c, e * s_])
            coef, *_ = np.linalg.lstsq(A, x, rcond=None)
            r = x - A @ coef
            q = float(r @ r)
            if best is None or q < best[0]:
                best = (q, w, s, coef)
    q, w, s, coef = best
    return w, s, float(coef[0]), float(math.hypot(coef[1], coef[2])), math.sqrt(q / len(t))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log')
    ap.add_argument('--mass', type=float, default=5.0, help='table mass, kg (default 5, weigh it)')
    ap.add_argument('--ixx', type=float, default=0.0623)
    ap.add_argument('--iyy', type=float, default=0.0623)
    ap.add_argument('--period-target', type=float, default=55.0)
    ap.add_argument('--trim', type=float, default=20.0, help='trim weight used for the suggestions, g')
    ap.add_argument('--level-offset', default='0,0', help='IMU roll,pitch (deg) when the deck is truly level')
    ap.add_argument('--plot')
    a = ap.parse_args()
    off = [float(v) for v in a.level_offset.split(',')]

    d = load(a.log)
    segs = segments(d)
    if not segs:
        sys.exit('no free-swing stretch found (Idle, wheels stopped, >= 8 s, swing >= 0.3 deg)')
    res = []
    for k in segs:
        t = d[k, 0]
        fr = fit_damped(t, d[k, 1])
        fp = fit_damped(t, d[k, 2])
        acc = d[k, 3:6].mean(axis=0)
        # A free swing fits a damped sinusoid closely; a stretch where the table was handled or yawing
        # hard (after a braking kick) doesn't (DEMO_0003: 1.2-1.9 deg rms with "periods" of 7-15 s, vs
        # 0.07-0.22 deg for real swings). Keep a stretch only if both axes fit within 0.4 deg rms.
        ok = all(f[4] <= 0.4 for f in (fr, fp))
        print(f'stretch {t[0]:8.1f}-{t[-1]:8.1f} s: roll period {2 * math.pi / fr[0]:5.2f} s (amp {fr[3]:.2f} deg, '
              f'rest {fr[2]:+.2f}, fit rms {fr[4]:.3f}), pitch period {2 * math.pi / fp[0]:5.2f} s (amp {fp[3]:.2f} deg, '
              f'rest {fp[2]:+.2f}, fit rms {fp[4]:.3f}){"" if ok else "  -> rejected (poor fit: not a free swing)"}')
        if ok:
            res.append(dict(t0=t[0], t1=t[-1], roll=fr, pitch=fp, acc=acc))
    if not res:
        sys.exit('no stretch fits a free swing well enough')
    segs = [k for k in segs if any(abs(d[k[0], 0] - r['t0']) < 1e-6 for r in res)]

    # Average the stretches, weighted by swing amplitude over fit error (disturbed stretches count less).
    def wavg(key, idx):
        w = np.array([r[key][3] / (r[key][4] + 0.05) for r in res])
        v = np.array([r[key][idx] for r in res])
        return float((w * v).sum() / w.sum())
    wr, wp = wavg('roll', 0), wavg('pitch', 0)
    roll0, pitch0 = wavg('roll', 2) - off[0], wavg('pitch', 2) - off[1]
    Lx = a.ixx * wr ** 2 / (a.mass * G)   # roll swing: about x
    Ly = a.iyy * wp ** 2 / (a.mass * G)
    L = 0.5 * (Lx + Ly)
    # centre of mass direction: straight down at rest = -(accelerometer direction), measured from true level
    ph, th = math.radians(roll0), math.radians(pitch0)
    up = np.array([-math.sin(th), math.sin(ph) * math.cos(th), math.cos(ph) * math.cos(th)])  # level deck: (0,0,1)
    r = -L * up * 1000.0  # mm, body axes
    Lt = (a.ixx + a.iyy) / 2 * (2 * math.pi / a.period_target) ** 2 / (a.mass * G) * 1000.0
    gmm = a.mass * 1000.0  # g per (mm of CG shift) per mm of trim travel

    print()
    print(f'Swing periods: roll {2 * math.pi / wr:.2f} s, pitch {2 * math.pi / wp:.2f} s   '
          f'(I = {a.ixx}/{a.iyy} kg m^2, m = {a.mass} kg)')
    print(f'Rest tilt (from level): roll {roll0:+.2f} deg, pitch {pitch0:+.2f} deg')
    print(f'Centre of mass below the bearing centre: L = {Lx * 1000:.2f} mm (roll swing), {Ly * 1000:.2f} mm (pitch swing)')
    print(f'Centre of mass from the bearing centre, body axes: x {r[0]:+.3f} mm, y {r[1]:+.3f} mm, z {r[2]:+.3f} mm')
    print()
    print(f'To level the deck: shift the centre of mass by x {-r[0]:+.3f} mm, y {-r[1]:+.3f} mm')
    for ax, v, toward in (('x', -r[0], ('+x (toward wheel A)', '-x (toward wheel C)')),
                          ('y', -r[1], ('+y (toward wheel B)', '-y (toward wheel D)'))):
        mom = gmm * abs(v)
        print(f'  {ax}: {mom:7.0f} g mm  = move {a.trim:.0f} g by {mom / a.trim:6.1f} mm toward {toward[0] if v > 0 else toward[1]}')
    rise = (-r[2]) - Lt
    mom = gmm * max(rise, 0.0)
    print(f'For a {a.period_target:.0f} s swing (slews): centre of mass {Lt:.3f} mm below the bearing centre, i.e. raise it '
          f'{max(rise, 0):.3f} mm')
    print(f'  {mom:7.0f} g mm  = raise {a.trim:.0f} g by {mom / a.trim:6.1f} mm (or add mass above the bearing centre; '
          f'a mass dm added at height h above it raises the CG by dm h / m)')
    print('  Every 1% error in I or m is a 1% error in L; the horizontal shifts also depend on the IMU being level '
          '(--level-offset).')

    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(len(res), 1, figsize=(10, 2.6 * len(res)), squeeze=False)
        for ax, rr, k in zip(axs[:, 0], res, segs):
            t = d[k, 0] - d[k[0], 0]
            for col, key, lab, cc in ((1, 'roll', 'roll', '#2f6fb0'), (2, 'pitch', 'pitch', '#c98a12')):
                w, s, c, amp, _ = rr[key]
                ax.plot(t, d[k, col], color=cc, lw=0.9, label=f'{lab} ({2 * math.pi / w:.2f} s)')
                ax.axhline(c, color=cc, lw=0.6, ls='--')
            ax.set(ylabel='deg', xlabel='s since the stretch began')
            ax.legend(fontsize=8, loc='upper right')
        fig.tight_layout()
        fig.savefig(a.plot, dpi=140)
        print(f'wrote {a.plot}')


if __name__ == '__main__':
    main()
