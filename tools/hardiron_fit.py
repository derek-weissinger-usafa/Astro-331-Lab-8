"""Fit the table's hard iron (a magnetic field fixed to the body) from a yaw turn.

Usage: python tools/hardiron_fit.py <LOG.CSV> [--use-stopped] [--plot out.png]
       python tools/hardiron_fit.py --selftest

Needs an SD log where the table turns a long way in yaw on the air bearing (a full turn is best:
DEMO SPIN, or turning it slowly by hand), ideally with MAG OFF so the logged attitude is gyro-only
and can't have been bent by the very offset being fitted. Model, per 0.5 s average:
    m_body - wheel terms = A(q) B + h + c_pattern
  A(q): the logged attitude (qToMatrix, inertial -> body), B: Earth's field (inertial, unknown),
  h: hard iron (body, unknown), c_pattern: a small constant per wheel direction pattern (the same
  model error the aid absorbs with a reference per pattern; the first pattern's is 0).
Wheel terms are the firmware's own kDirStep / kSpeedSlope, read from src/mag_aid.cpp. Only
samples with every wheel spinning (> 6 rad/s) are used, since a stopped shaft leaves a random
offset; --use-stopped adds stopped stretches as one extra pattern per stop.
Prints h for SET MAG_HX/HY/HZ (try it without an upload; then copy into kMagHardIronUt) and
checks it with the replay (tools/replay, --hard). Soft iron (a 3x3 scale) is not fitted; the
residual's twice-per-turn content (printed) says whether it is needed.
"""
import argparse, csv, math, os, re, subprocess, sys, tempfile
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.join(HERE, '..', 'Astro 331 Lab 8')
REPLAY = os.path.join(HERE, 'replay', 'replay.exe' if os.name == 'nt' else 'replay')


def wheel_model():
    src = open(os.path.join(FW, 'src', 'mag_aid.cpp'), encoding='utf-8').read()
    def table(name):
        m = re.search(name + r'\[4\]\[3\] = \{(.*?)\};', src, re.S)
        return np.array([float(v) for v in re.findall(r'-?\d+\.\d+', m.group(1))]).reshape(4, 3)
    ref = float(re.search(r'kMagRefSpeedRadS = ([\d.]+)f', open(os.path.join(FW, 'include', 'control_params.h')).read()).group(1))
    return table('kSpeedSlope'), table('kDirStep'), ref


def qmat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def load(path):
    with open(path, newline='') as f:
        rd = csv.reader(f)
        hdr = next(rd)
        ci = {h: i for i, h in enumerate(hdr)}
        cols = ['mcutime_ms', 'mag_x_uT', 'mag_y_uT', 'mag_z_uT', 'qw', 'qx', 'qy', 'qz'] + \
               [f'motor_{w}_measured_rate_rpm' for w in 'abcd']
        rows, ws = [], []
        for r in rd:
            if not r or not r[0][:1].isdigit():
                continue
            try:
                rows.append([float(r[ci[c]]) for c in cols])
                ws.append(r[ci['wheel_status']] if 'wheel_status' in ci else '....')
            except (ValueError, IndexError):
                continue
    d = np.array(rows)
    return dict(t=d[:, 0] * 1e-3, m=d[:, 1:4], q=d[:, 4:8], w=d[:, 8:12] * 2 * math.pi / 60, ws=np.array(ws))


def bins(d, use_stopped, slope, step, wref):
    """0.5 s averages of the wheel-corrected field, with the attitude at the bin centre and a pattern key."""
    t, w = d['t'], d['w']
    en = np.array([[c != '-' for c in s.ljust(4, '.')[:4]] for s in d['ws']])
    spin = np.all((np.abs(w) > 6.0) | ~en, axis=1)
    still = np.all((np.abs(w) < 0.05) | ~en, axis=1)
    m = d['m'].copy()
    for i in range(4):
        on = en[:, i] & (np.abs(w[:, i]) >= 4.0)
        m[on] -= np.outer(np.abs(w[on, i]) - wref, slope[i]) + np.outer(np.sign(w[on, i]), step[i])
    pat = np.array([''.join('+' if v > 0 else '-' for v in row) for row in w])
    out = []
    k0 = 0
    stop_id = 0
    for k in range(1, len(t) + 1):
        if k < len(t) and t[k] - t[k0] < 0.5:
            continue
        kk = np.arange(k0, k)
        k0 = k
        if spin[kk].all() and len(set(pat[kk])) == 1:
            key = pat[kk[0]]
        elif use_stopped and still[kk].all():
            key = f'stop{stop_id}'
        else:
            if use_stopped and not still[kk].all():
                stop_id += 1
            continue
        out.append((t[kk].mean(), m[kk].mean(axis=0), d['q'][kk[len(kk) // 2]], key))
    return out


def solve(samples):
    keys = sorted(set(s[3] for s in samples), key=lambda k: -sum(1 for s in samples if s[3] == k))
    nk = len(keys)
    A = np.zeros((3 * len(samples), 6 + 3 * (nk - 1)))
    y = np.zeros(3 * len(samples))
    for i, (t, m, q, key) in enumerate(samples):
        R = qmat(q)
        A[3 * i:3 * i + 3, 0:3] = R
        A[3 * i:3 * i + 3, 3:6] = np.eye(3)
        j = keys.index(key)
        if j > 0:
            A[3 * i:3 * i + 3, 6 + 3 * (j - 1):9 + 3 * (j - 1)] = np.eye(3)
        y[3 * i:3 * i + 3] = m
    x, *_ = np.linalg.lstsq(A, y, rcond=None)
    res = (y - A @ x).reshape(-1, 3)
    return x[0:3], x[3:6], keys, res


def yaw_of(q):
    w, x, y, z = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def report(samples, B, h, keys, res):
    yaws = np.unwrap(np.radians([yaw_of(s[2]) for s in samples]))
    span = math.degrees(yaws.max() - yaws.min())
    BH = math.hypot(B[0], B[1])
    print(f'{len(samples)} half-second samples, {len(keys)} wheel pattern(s), yaw span {span:.0f} deg')
    if span < 90:
        print('  WARNING: under 90 deg of yaw - h is poorly determined (turn the table further)')
    print(f'Earth field (inertial): {B[0]:+.2f} {B[1]:+.2f} {B[2]:+.2f} uT, |B| {np.linalg.norm(B):.1f} uT, '
          f'horizontal {BH:.1f} uT, dip {math.degrees(math.atan2(-B[2], BH)):.1f} deg')
    print(f'Hard iron h (body): {h[0]:+.2f} {h[1]:+.2f} {h[2]:+.2f} uT  (|h_horizontal| {math.hypot(h[0], h[1]):.1f} uT)')
    print(f'  firmware: SET MAG_HX {h[0]:.2f}   SET MAG_HY {h[1]:.2f}   SET MAG_HZ {h[2]:.2f}')
    print(f'  (worst heading error a {math.hypot(h[0], h[1]):.1f} uT offset causes, as a fraction of a yaw change: '
          f'~{math.hypot(h[0], h[1]) / max(BH, 1e-6):.2f} x the change, for small turns)')
    r = res - res.mean(axis=0)
    print(f'Residual rms: {np.sqrt((r ** 2).mean(axis=0)).round(3)} uT')
    if span >= 300:
        ph = yaws - yaws[0]
        hx = np.abs(np.fft.rfft(r[:, 0]))
        c2 = np.abs((r[:, 0] + 1j * r[:, 1]) @ np.exp(-2j * ph)) / len(ph)
        print(f'  twice-per-turn residual amplitude {c2:.3f} uT (> ~0.3 uT suggests soft iron is worth fitting)')


def selftest():
    rng = np.random.default_rng(3)
    B = np.array([20.5, 0.0, -47.0]); h = np.array([7.5, -4.2, 9.1])
    samples = []
    for i in range(720):
        psi = math.radians(i * 0.5); ph = math.radians(2 * math.sin(i / 50)); th = math.radians(-1 + math.cos(i / 70))
        cy, sy, cr, sr, cp, sp = math.cos(psi / 2), math.sin(psi / 2), math.cos(ph / 2), math.sin(ph / 2), math.cos(th / 2), math.sin(th / 2)
        # body = A inertial with A = qToMatrix(q): build q for a passive 3-2-1 rotation (frames.h)
        q = np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])
        q = q * np.array([1, -1, -1, -1])  # conjugate: inertial -> body
        key = '+-+-' if i < 400 else '++++'
        c = np.zeros(3) if key == '+-+-' else np.array([0.3, -0.2, 0.1])
        m = qmat(q) @ B + h + c + rng.normal(0, 0.3, 3)
        samples.append((i * 0.5, m, q, key))
    Bf, hf, keys, res = solve(samples)
    report(samples, Bf, hf, keys, res)
    print(f'true h {h}, fitted {hf.round(2)}  -> error {np.linalg.norm(hf - h):.3f} uT')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log', nargs='?')
    ap.add_argument('--use-stopped', action='store_true')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    if a.selftest or not a.log:
        selftest()
        return
    slope, step, wref = wheel_model()
    d = load(a.log)
    samples = bins(d, a.use_stopped, slope, step, wref)
    if len(samples) < 20:
        sys.exit(f'only {len(samples)} usable samples (need the wheels spinning, or --use-stopped)')
    B, h, keys, res = solve(samples)
    report(samples, B, h, keys, res)
    if os.path.exists(REPLAY):
        for label, extra in (('no hard iron', []), ('fitted hard iron', ['--hard', f'{h[0]:.3f},{h[1]:.3f},{h[2]:.3f}'])):
            out = tempfile.mktemp(suffix='.csv')
            p = subprocess.run([REPLAY, a.log, out] + extra, capture_output=True, text=True)
            print(f'replay, {label}: ' + p.stdout.strip().replace('\n', '; '))
            if os.path.exists(out):
                os.remove(out)


if __name__ == '__main__':
    main()
