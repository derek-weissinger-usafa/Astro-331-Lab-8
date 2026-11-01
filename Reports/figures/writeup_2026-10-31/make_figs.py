"""Figures for Reports/Lab8_Writeup_2026-10-31.html. Run from the repo root:
    python Reports/figures/writeup_2026-10-31/make_figs.py <scratch dir with replay outputs>
Reads the SD logs in test_runs/ and the simulation traces next to this script.
"""
import csv, math, os, re, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = os.path.dirname(os.path.abspath(__file__))
RUNS = 'test_runs'
SCR = sys.argv[1] if len(sys.argv) > 1 else '.'
R2D = 180 / math.pi
RPM = 2 * math.pi / 60
COL = {'A': '#2f6fb0', 'B': '#c98a12', 'C': '#3a9a5b', 'D': '#b8433a'}
plt.rcParams.update({'font.size': 9, 'axes.grid': True, 'grid.alpha': 0.3, 'figure.dpi': 110})


def load(path, cols, strcols=()):
    """Selected columns of an SD log as numpy arrays (float) / lists (str); t in s from the first row."""
    with open(path, newline='') as f:
        rd = csv.reader(f)
        h = next(rd)
        ci = {k: i for i, k in enumerate(h)}
        idx = [ci[c] for c in cols]
        sidx = [ci[c] for c in strcols]
        F, S = [], []
        n = len(h)
        for r in rd:
            if len(r) < n or not r[0][:1].isdigit():
                continue
            try:
                F.append([float(r[i]) for i in idx])
                S.append([r[i] for i in sidx])
            except ValueError:
                continue
    F = np.array(F)
    d = {c: F[:, k] for k, c in enumerate(cols)}
    for k, c in enumerate(strcols):
        d[c] = np.array([s[k] for s in S])
    d['t'] = (d['mcutime_ms'] - d['mcutime_ms'][0]) * 1e-3
    return d


def blocks(mask, min_len=1):
    out, s = [], None
    for i, v in enumerate(np.r_[mask, False]):
        if v and s is None:
            s = i
        if not v and s is not None:
            if i - s >= min_len:
                out.append((s, i))
            s = None
    return out


def save(fig, name):
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, name), dpi=130)
    plt.close(fig)
    print('wrote', name)


WCOLS = [f'motor_{w}_measured_rate_rpm' for w in 'abcd']
DCOLS = [f'motor_{w}_duty' for w in 'abcd']


# ---------- F1: MAG TEST (10-30) ----------
def f1():
    d = load(f'{RUNS}/MAGTEST_0001.CSV', ['mcutime_ms', 'yaw_deg', 'test_step'])
    rp = list(csv.DictReader(open(os.path.join(SCR, 'mt1_nomag.csv'))))
    tr = np.array([float(r['t_s']) for r in rp]); tr -= tr[0]
    yr = np.array([float(r['yaw_replay_deg']) for r in rp])
    fig, ax = plt.subplots(2, 1, figsize=(9, 5.2), sharex=True)
    ax[0].plot(d['t'] / 60, d['yaw_deg'], lw=0.6, color='#2f6fb0', label='on board (magnetometer aid on)')
    ax[0].plot(tr / 60, yr, lw=0.8, color='#b8433a', label='same data, gyro only (replay, kicks removed)')
    ax[0].axhspan(-1, 1, color='#3a9a5b', alpha=0.12, label='±1° requirement')
    ax[0].set_ylim(-6, 6)
    ax[0].set_ylabel('yaw error (deg)\n(table clamped: any yaw is error)')
    ax[0].legend(fontsize=7, loc='lower left')
    ax[0].set_title('MAG TEST run 1 (MAGTEST_0001, 10-30): 36/37 steps within ±1° with the aid; gyro alone 5.7 °/h\n(spikes at 1, 20 and 22 min are the scripted ±10° estimate kicks)')
    ax[1].plot(d['t'] / 60, d['test_step'], lw=0.8, color='k')
    ax[1].set_ylabel('test step')
    ax[1].set_xlabel('minutes into the test')
    save(fig, 'f01_magtest.png')


# ---------- F2: balance iterations ----------
def f2():
    lines = open(f'{RUNS}/balance_iterations_2026-10-31.txt', encoding='utf-8').read().splitlines()
    R, P, notes = [], [], []
    pending = []
    for l in lines:
        m = re.search(r'DEMO LEVEL rest tilt: roll ([-+\d.]+) deg, pitch ([-+\d.]+)', l)
        if m:
            R.append(float(m.group(1))); P.append(float(m.group(2)))
            for n in pending:
                notes.append((len(R) - 1, n))
            pending = []
        elif l.startswith('NOTE'):
            pending.append(l[5:].strip())
    fig, ax = plt.subplots(figsize=(10, 4.2))
    i = np.arange(len(R))
    ax.plot(i, R, 'o-', ms=3.5, color='#2f6fb0', label='roll at rest (deg)')
    ax.plot(i, P, 's-', ms=3.5, color='#c98a12', label='pitch at rest (deg)')
    ax.axhspan(-5, 5, color='#3a9a5b', alpha=0.06)
    key = [('glued', 'IMU glued,\nLEVEL ZERO'), ('Heading test', 'heading\ntest'), ('Big jump', 'mass shifted\n(pack check)'),
           ('LEVEL ZERO redone', 'LEVEL ZERO\nredone'), ('Loose heavy weights', 'weights & top\nscrews off'), ('Removed 2 x 8 g', 'washers\nAB/BC/AD off'),
           ('Added 3 x 8 g', 'washers on\nCD & AD')]
    for q, (k, lab) in enumerate(key):
        for j, n in notes:
            if k in n:
                ax.axvline(j - 0.5, color='grey', lw=0.7, ls='--')
                ax.text(j - 0.4, 6.9 if q % 2 == 0 else -11.0, lab, fontsize=6.5, va='top', color='#444')
                break
    ax.set_ylim(-14, 7.5)
    ax.set_xlabel('rest-tilt reading (DEMO LEVEL, in order, 2026-10-31)')
    ax.set_ylabel('deg')
    ax.legend(fontsize=8, loc='lower right')
    ax.set_title('Balance session: rest tilt per reading. Final: roll ≈ −0.9°, pitch ≈ −0.7° (green band: ±5° trim rule)')
    save(fig, 'f02_balance.png')


# ---------- F3: dead-motor false alarm ----------
def f3():
    d = load(f'{RUNS}/DEMO_0005.CSV', ['mcutime_ms'] + WCOLS + DCOLS, ['wheel_status', 'state'])
    ws = d['wheel_status']
    k = next(i for i in range(1, len(ws)) if ws[i][3] == 'D' and ws[i - 1][3] != 'D')
    sl = slice(max(0, k - 300), k + 60)
    t = d['t'][sl] - d['t'][k]
    sp = np.abs(d['motor_d_measured_rate_rpm'][sl] * RPM)
    du = np.abs(d['motor_d_duty'][sl])
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
    ax[0].plot(t, du, color='#b8433a', label='|wheel D duty| (commanded)')
    ax[0].plot(t, du - sp / 160, '--', color='#b8433a', lw=0.8, label='|effort| = |duty − speed/160|')
    ax[0].axhline(0.2, color='k', lw=0.8, ls=':'); ax[0].text(-2.9, 0.205, 'old dead threshold 0.2', fontsize=7)
    ax[0].axhline(0.4, color='k', lw=0.8, ls='-.'); ax[0].text(-2.9, 0.405, 'new standstill threshold 0.4', fontsize=7)
    a2 = ax[0].twinx(); a2.plot(t, sp, color='#2f6fb0', lw=0.9, label='wheel D speed'); a2.set_ylabel('speed (rad/s)', color='#2f6fb0')
    a2.set_ylim(-2, 10); a2.grid(False)
    ax[0].axvline(0, color='grey', lw=0.8)
    ax[0].set_xlabel('s from the "dead motor" flag'); ax[0].set_ylabel('duty / effort')
    ax[0].legend(fontsize=7, loc='upper left'); ax[0].set_ylim(-0.05, 0.5)
    ax[0].set_title('DEMO_0005: wheel D "dead" while stuck at rest')
    # breakaway duty per wheel, both logs
    brk = {w: [] for w in 'ABCD'}
    for fn in ('DEMO_0004.CSV', 'DEMO_0005.CSV'):
        e = load(f'{RUNS}/{fn}', ['mcutime_ms'] + WCOLS + DCOLS, ['state'])
        arm = e['state'] == 'A'
        for j, w in enumerate('abcd'):
            s = e[f'motor_{w}_measured_rate_rpm'] * RPM; du_ = np.abs(e[f'motor_{w}_duty'])
            rest = 0.0; peak = 0.0
            tt = e['t']
            for i in range(1, len(tt)):
                if not arm[i]:
                    rest = 0; peak = 0; continue
                if abs(s[i]) < 0.8:
                    rest += tt[i] - tt[i - 1]; peak = max(peak, du_[i])
                else:
                    if rest > 0.3 and abs(s[i]) > 2 and peak > 0.03:
                        brk[w.upper()].append(du_[i])
                    if abs(s[i]) > 2:
                        rest = 0; peak = 0
    ax[1].hist([brk[w] for w in 'ABCD'], bins=np.arange(0, 0.32, 0.02), label=[f'{w} ({len(brk[w])} starts)' for w in 'ABCD'],
               color=[COL[w] for w in 'ABCD'])
    ax[1].axvline(0.2, color='k', ls=':', lw=0.8)
    ax[1].set_xlabel('duty when the wheel broke free from rest'); ax[1].set_ylabel('starts')
    ax[1].legend(fontsize=7); ax[1].set_title('Breakaway duty, healthy starts (DEMO_0004/0005)')
    save(fig, 'f03_deadmotor.png')


# ---------- F4: vibration and IMU gaps ----------
def f4():
    d = load(f'{RUNS}/STATIC_0008 31 Oct.CSV', ['mcutime_ms', 'accel_x_m_s2', 'accel_y_m_s2', 'accel_z_m_s2'] + WCOLS)
    w = np.mean([d[c] for c in WCOLS], axis=0)
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.4))
    labs, sx, sy, sz = [], [], [], []
    for tgt in (0, 200, 400, 600, 760, -760):
        m = (np.abs(w - tgt) < 8) & (np.max([np.abs(d[c] - tgt) for c in WCOLS], axis=0) < 15) & (d['t'] < 405)
        labs.append(str(tgt)); sx.append(d['accel_x_m_s2'][m].std()); sy.append(d['accel_y_m_s2'][m].std()); sz.append(d['accel_z_m_s2'][m].std())
    x = np.arange(len(labs))
    ax[0].bar(x - 0.25, sx, 0.25, label='x'); ax[0].bar(x, sy, 0.25, label='y'); ax[0].bar(x + 0.25, sz, 0.25, label='z')
    ax[0].set_xticks(x, labs); ax[0].set_xlabel('all wheels, rpm (table clamped)'); ax[0].set_ylabel('accel std dev (m/s²)')
    ax[0].legend(fontsize=7); ax[0].set_title('Vibration up to the new 80 rad/s limit: small')
    # gaps vs file offset
    off, gap = [], []
    pos = 0; prev = None; first = True
    for line in open(f'{RUNS}/STATIC_0008 31 Oct.CSV', 'rb'):
        if first:
            pos += len(line); first = False; continue
        try:
            tt = float(line.split(b',', 1)[0])
        except ValueError:
            pos += len(line); continue
        if prev is not None and tt - prev > 25:
            off.append(pos / 1e6); gap.append(tt - prev)
        prev = tt; pos += len(line)
    ax[1].stem(off, gap)
    ax[1].axhline(100, color='k', ls=':', lw=0.8); ax[1].text(0.2, 104, 'old 100 ms IMU timeout', fontsize=7)
    ax[1].set_xlabel('position in the SD log file (MB)'); ax[1].set_ylabel('gyro gap (ms)')
    ax[1].set_title('IMU gaps come in pairs every ~1.67 MB of log (SD writes)')
    save(fig, 'f04_vibration_gaps.png')


# ---------- F5: heading error explained (DEMO_0007) ----------
def f5():
    d = load(f'{RUNS}/DEMO_0007.CSV', ['mcutime_ms', 'yaw_deg', 'yaw_ref_deg', 'test_step', 'mag_x_uT', 'mag_y_uT'] + WCOLS,
             ['state', 'demo_phase'])
    rp = list(csv.DictReader(open(os.path.join(SCR, 'd7_nomag.csv'))))
    tr = np.array([float(r['t_s']) for r in rp]); tr -= tr[0]
    yn = np.unwrap(np.radians([float(r['yaw_replay_deg']) for r in rp])) * R2D
    # STEP TEST window: first Armed block with test_step 0..4
    arm = (d['state'] == 'A') & (d['test_step'] >= 0)
    bl = [b for b in blocks(arm, 500) if d['test_step'][b[0]:b[1]].max() >= 4]
    s, e = bl[0]
    t0 = d['t'][s]
    yl = np.unwrap(np.radians(d['yaw_deg'][s:e])) * R2D
    ref = d['yaw_ref_deg'][s:e]
    k = (tr >= d['t'][s]) & (tr <= d['t'][e - 1])
    yg = yn[k] - (yn[k][0] - yl[0])
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.8), gridspec_kw={'width_ratios': [1.6, 1]})
    base = ref[0]
    ax[0].plot(d['t'][s:e] - t0, ref - base, 'k--', lw=0.8, label='commanded')
    ax[0].plot(d['t'][s:e] - t0, yl - base, color='#2f6fb0', label='firmware estimate (aid on, no hard-iron correction)')
    ax[0].plot(tr[k] - t0, yg - base, color='#b8433a', label='gyro only (replay) ≈ true heading')
    ax[0].set_xlabel('s'); ax[0].set_ylabel('heading change (deg)')
    ax[0].legend(fontsize=7, loc='lower left')
    ax[0].set_title('STEP TEST 1 (DEMO_0007): the estimate held, the table did not')
    # hard iron circle
    m = (d['t'] > 55) & (d['t'] < 185) & (np.max(np.abs([d[c] for c in WCOLS]), axis=0) < 1)
    mx, my = d['mag_x_uT'][m], d['mag_y_uT'][m]
    A = np.column_stack([mx, my, np.ones(len(mx))]); c, *_ = np.linalg.lstsq(A, mx ** 2 + my ** 2, rcond=None)
    cx, cy = c[0] / 2, c[1] / 2; r = math.sqrt(c[2] + cx * cx + cy * cy)
    ax[1].plot(mx[::20], my[::20], '.', ms=2, color='#2f6fb0', label='measured (376° free turn)')
    th = np.linspace(0, 2 * np.pi, 200)
    ax[1].plot(cx + r * np.cos(th), cy + r * np.sin(th), 'k', lw=0.8, label=f'circle: Earth {r:.1f} µT')
    ax[1].plot([0], [0], 'r+', ms=10); ax[1].plot([cx], [cy], 'kx')
    ax[1].annotate(f'table field ({cx:+.1f}, {cy:+.1f}) µT', (cx, cy), (cx - 35, cy + 25), fontsize=7, arrowprops=dict(arrowstyle='->', lw=0.6))
    ax[1].text(2, 2, 'zero', color='r', fontsize=7)
    ax[1].set_aspect('equal'); ax[1].set_xlabel('mag x (µT)'); ax[1].set_ylabel('mag y (µT)')
    ax[1].legend(fontsize=7, loc='lower left'); ax[1].set_title('Hard iron: circle centre far from zero')
    save(fig, 'f05_heading_hardiron.png')


# ---------- F6: STEP TEST rerun (DEMO_0008) ----------
def f6():
    d = load(f'{RUNS}/DEMO_0008.CSV', ['mcutime_ms', 'yaw_deg', 'yaw_ref_deg', 'test_step'] + WCOLS, ['state', 'wheel_status'])
    arm = (d['state'] == 'A') & (d['test_step'] >= 0)
    bl = [b for b in blocks(arm, 500) if d['test_step'][b[0]:b[1]].max() >= 4]
    s, e = bl[-1]
    t = d['t'][s:e] - d['t'][s]
    y = np.unwrap(np.radians(d['yaw_deg'][s:e])) * R2D; ref = d['yaw_ref_deg'][s:e]
    base = ref[0]
    fig, ax = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
    ax[0].plot(t, ref - base, 'k--', lw=0.8, label='commanded'); ax[0].plot(t, y - base, color='#2f6fb0', lw=0.9, label='heading estimate')
    ax[0].set_ylabel('heading (deg)'); ax[0].legend(fontsize=7)
    ax[0].set_title('STEP TEST rerun (DEMO_0008, hard iron + measured I_zz): rise 1.2–1.7 s, ≤ 4 % overshoot')
    for w in 'abcd':
        ax[1].plot(t, d[f'motor_{w}_measured_rate_rpm'][s:e] * RPM, lw=0.7, color=COL[w.upper()], label=w.upper())
    ax[1].set_ylabel('wheel speed (rad/s)'); ax[1].set_xlabel('s from the step test start'); ax[1].legend(fontsize=7, ncol=4)
    ax[1].text(0.01, 0.05, 'wheel D out of service from the start (flagged dead: see Fig. 8)', transform=ax[1].transAxes, fontsize=7)
    save(fig, 'f06_steptest.png')


# ---------- F7: FAULT TEST (DEMO_0007) ----------
def f7():
    d = load(f'{RUNS}/DEMO_0007.CSV', ['mcutime_ms', 'err_yaw_deg', 'err_roll_deg', 'err_pitch_deg'] + WCOLS, ['fault_phase', 'wheel_status'])
    fp = d['fault_phase']
    k = np.where(fp != '-')[0]
    inj = next(i for i in k if fp[i] == 'F')
    s, e = max(0, inj - 1500), min(len(fp), inj + 4500)
    t = d['t'][s:e] - d['t'][inj]
    det = next(i for i in range(inj, e) if d['wheel_status'][i][1] == 'D')
    fig, ax = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
    for w in 'abcd':
        ax[0].plot(t, d[f'motor_{w}_measured_rate_rpm'][s:e] * RPM, lw=0.8, color=COL[w.upper()], label=w.upper())
    for a in ax:
        a.axvline(0, color='k', lw=0.8, ls='--'); a.axvline(d['t'][det] - d['t'][inj], color='#b8433a', lw=0.8)
    ax[0].text(0.3, ax[0].get_ylim()[1] * 0.85, 'B made dead', fontsize=7)
    ax[0].text(d['t'][det] - d['t'][inj] + 0.3, ax[0].get_ylim()[1] * 0.65, f'detected (+{d["t"][det]-d["t"][inj]:.2f} s), B coasts', fontsize=7, color='#b8433a')
    ax[0].set_ylabel('wheel speed (rad/s)'); ax[0].legend(fontsize=7, ncol=4)
    ax[0].set_title('FAULT TEST B on the air bearing (DEMO_0007): PASS, detected 0.91 s, never outside ±1°')
    ax[1].plot(t, d['err_yaw_deg'][s:e], color='#2f6fb0', lw=0.8, label='yaw')
    ax[1].plot(t, d['err_roll_deg'][s:e], color='#c98a12', lw=0.8, label='roll')
    ax[1].plot(t, d['err_pitch_deg'][s:e], color='#3a9a5b', lw=0.8, label='pitch')
    ax[1].axhspan(-1, 1, color='#3a9a5b', alpha=0.08)
    ax[1].set_ylabel('controller error (deg)'); ax[1].set_xlabel('s from the injected failure'); ax[1].legend(fontsize=7, ncol=3)
    save(fig, 'f07_faulttest.png')


# ---------- F8: wheel D really not turning (DEMO_0008) ----------
def f8():
    d = load(f'{RUNS}/DEMO_0008.CSV', ['mcutime_ms', 'motor_d_duty', 'motor_d_measured_rate_rpm', 'enc_d', 'motor_a_measured_rate_rpm',
                                       'motor_c_measured_rate_rpm'], ['wheel_status'])
    ws = d['wheel_status']
    k = next(i for i in range(1, len(ws)) if ws[i][3] == 'D' and ws[i - 1][3] != 'D')
    sl = slice(max(0, k - 400), k + 100)
    t = d['t'][sl] - d['t'][k]
    fig, ax = plt.subplots(figsize=(9, 3.4))
    ax.plot(t, d['motor_d_duty'][sl], color='#b8433a', label='wheel D duty')
    ax.set_ylabel('duty'); ax.set_xlabel('s from the flag'); ax.axvline(0, color='k', lw=0.8)
    a2 = ax.twinx()
    a2.plot(t, d['enc_d'][sl] - d['enc_d'][sl][0], color='#2f6fb0', lw=0.9, label='wheel D encoder counts')
    a2.plot(t, d['motor_a_measured_rate_rpm'][sl] * RPM, color=COL['A'], lw=0.6, ls='--', label='wheel A speed (rad/s)')
    a2.plot(t, d['motor_c_measured_rate_rpm'][sl] * RPM, color=COL['C'], lw=0.6, ls='--', label='wheel C speed (rad/s)')
    a2.set_ylabel('counts / rad/s'); a2.grid(False)
    h1, l1 = ax.get_legend_handles_labels(); h2, l2 = a2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=7, loc='upper left')
    ax.set_title('DEMO_0008: wheel D driven to the 0.45 limit, encoder does not move; A and C absorb its share (a real failure)')
    save(fig, 'f08_wheelD.png')


# ---------- F9: fixed-tilt SLEW TEST, table vs simulation ----------
def f9():
    d = load(f'{RUNS}/DEMO_0007.CSV', ['mcutime_ms', 'roll_deg', 'pitch_deg', 'yaw_deg'] + WCOLS, ['state', 'demo_phase'])
    m = (d['state'] == 'A') & np.isin(d['demo_phase'], ['S', 'W', 'H'])
    s, e = blocks(m, 200)[-1]
    t = d['t'][s:e] - d['t'][s]
    sim = np.genfromtxt(os.path.join(OUT, 'sim_fixed.csv'), delimiter=',', names=True, dtype=None, encoding=None)
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.6), sharey=True)
    for w in 'abcd':
        ax[0].plot(t, d[f'motor_{w}_measured_rate_rpm'][s:e] * RPM, color=COL[w.upper()], lw=0.9, label=w.upper())
        ax[1].plot(sim['t_s'], sim['w' + w.upper()], color=COL[w.upper()], lw=0.9, label=w.upper())
    for a in ax:
        a.axhline(75, color='k', ls=':', lw=0.8); a.axhline(-75, color='k', ls=':', lw=0.8); a.legend(fontsize=7, ncol=4)
        a.set_xlabel('s from the test start')
    ax[0].set_ylabel('wheel speed (rad/s)')
    ax[0].set_title('Table: SLEW TEST, roll/pitch held at −1.2/−0.2°\nwheel limit after 13.5 s, during slew 1')
    ax[1].set_title('Simulation (tools/slewsim, same targets)\nwheel limit after 88 s, in target 4')
    ax[0].set_xlim(0, 20)
    save(fig, 'f09_fixedtilt_vs_sim.png')


# ---------- F10: SLEW YAW, table vs simulation ----------
def f10():
    d = load(f'{RUNS}/SLEW_0002.CSV', ['mcutime_ms', 'roll_deg', 'pitch_deg', 'yaw_deg', 'yaw_ref_deg', 'test_step'] + WCOLS,
             ['state', 'demo_phase'])
    m = (d['state'] == 'A') & np.isin(d['demo_phase'], ['S', 'W', 'H', 'D'])
    s, e = blocks(m, 2000)[-1]
    t = d['t'][s:e] - d['t'][s]
    y = np.unwrap(np.radians(d['yaw_deg'][s:e])) * R2D
    yr = np.unwrap(np.radians(d['yaw_ref_deg'][s:e])) * R2D
    sim = np.genfromtxt(os.path.join(OUT, 'sim_slewyaw.csv'), delimiter=',', names=True, dtype=None, encoding=None)
    ys = np.unwrap(np.radians(sim['yaw_deg'])) * R2D; yrs = np.unwrap(np.radians(sim['yaw_ref_deg'])) * R2D
    fig, ax = plt.subplots(3, 2, figsize=(11, 7.5), sharex='col')
    ax[0, 0].plot(t, yr - yr[0], 'k--', lw=0.8, label='reference'); ax[0, 0].plot(t, y - yr[0], color='#2f6fb0', lw=0.9, label='estimate')
    ax[0, 1].plot(sim['t_s'], yrs - yrs[0], 'k--', lw=0.8); ax[0, 1].plot(sim['t_s'], ys - yrs[0], color='#2f6fb0', lw=0.9)
    ax[0, 0].legend(fontsize=7); ax[0, 0].set_ylabel('heading from start (deg)')
    ax[0, 0].set_title('Table: SLEW YAW, presentation spot, gyro only (SLEW_0002)\n4/4 PASS, ended on the tape marker')
    ax[0, 1].set_title('Simulation (tools/slewsim, 2 mN·m room torque)\n4/4 PASS')
    err = (y - yr)
    ax[1, 0].plot(t, err, color='#2f6fb0', lw=0.6); ax[1, 1].plot(sim['t_s'], ys - yrs, color='#2f6fb0', lw=0.6)
    for a in ax[1]:
        a.set_ylim(-1.2, 1.2); a.axhspan(-1, 1, color='#3a9a5b', alpha=0.06)
    ax[1, 0].set_ylabel('yaw − reference (deg)')
    for w in 'abcd':
        ax[2, 0].plot(t, d[f'motor_{w}_measured_rate_rpm'][s:e] * RPM, color=COL[w.upper()], lw=0.7, label=w.upper())
        ax[2, 1].plot(sim['t_s'], sim['w' + w.upper()], color=COL[w.upper()], lw=0.7, label=w.upper())
    ax[2, 0].set_ylabel('wheel speed (rad/s)'); ax[2, 0].legend(fontsize=7, ncol=4)
    ax[2, 0].text(0.01, 0.04, '±100 rpm (±10.5 rad/s) heading-hold bias on', transform=ax[2, 0].transAxes, fontsize=7)
    ax[2, 1].text(0.01, 0.04, 'no wheel bias in the simulation', transform=ax[2, 1].transAxes, fontsize=7)
    for a in ax[2]:
        a.set_xlabel('s from the test start')
    save(fig, 'f10_slewyaw_vs_sim.png')
    rr = d['roll_deg'][s:e]; pp = d['pitch_deg'][s:e]
    print('SLEW_0002 run: %.0f s, peak wheel %.1f rad/s, roll %.2f..%.2f pitch %.2f..%.2f, max |yaw err| in slews %.2f' % (
        t[-1], np.max(np.abs([d[c][s:e] for c in WCOLS])) * RPM, rr.min(), rr.max(), pp.min(), pp.max(), np.abs(err).max()))


# ---------- F11: room distortion ----------
def f11():
    def binned(path, t0, t1, spinning, hard=(20.5, -48.9), free_centre=False, smooth=False):
        d = load(path, ['mcutime_ms', 'mag_x_uT', 'mag_y_uT', 'yaw_deg'] + WCOLS, ['state'])
        w = np.abs(np.array([d[c] for c in WCOLS])) * RPM
        if spinning:
            m = (d['state'] == 'M') & (w.min(axis=0) > 8)
        else:
            m = (w.max(axis=0) < 0.05)
        m &= (d['t'] >= t0) & (d['t'] <= t1)
        tt = d['t'][m]; mx = d['mag_x_uT'][m].copy(); my = d['mag_y_uT'][m].copy(); y = np.unwrap(np.radians(d['yaw_deg'][m]))
        if spinning:  # remove the firmware's wheel-magnet model (mag_aid.cpp kSpeedSlope / kDirStep)
            sys.path.insert(0, 'tools')
            from hardiron_fit import wheel_model
            slope, step, wref = wheel_model()
            for i, c in enumerate(WCOLS):
                ws_ = d[c][m] * RPM
                mx -= (np.abs(ws_) - wref) * slope[i][0] + np.sign(ws_) * step[i][0]
                my -= (np.abs(ws_) - wref) * slope[i][1] + np.sign(ws_) * step[i][1]
        if smooth:
            a = np.clip(np.diff(np.r_[tt[0], tt]) / 0.5, 0, 1)
            for arr in (mx, my, y):
                p = arr[0]
                for i in range(len(arr)):
                    p += a[i] * (arr[i] - p); arr[i] = p
        if free_centre:
            A = np.column_stack([mx, my, np.ones(len(mx))]); c, *_ = np.linalg.lstsq(A, mx ** 2 + my ** 2, rcond=None)
            hx, hy = c[0] / 2, c[1] / 2
        else:
            hx, hy = hard
        th = np.unwrap(np.arctan2(my - hy, mx - hx))
        X = np.column_stack([np.ones_like(th), y])
        cl, *_ = np.linalg.lstsq(X, th, rcond=None)
        e0 = np.degrees(th - X @ cl)
        Xf = np.column_stack([np.ones_like(th), y, np.cos(th), np.sin(th), np.cos(2 * th), np.sin(2 * th)])
        cf, *_ = np.linalg.lstsq(Xf, th, rcond=None)
        e1 = np.degrees(th - Xf @ cf)
        thw = np.degrees(np.arctan2(np.sin(th), np.cos(th)))
        b = ((thw + 180) // 30).astype(int)
        ctr = np.arange(12) * 30 - 165
        m0 = np.array([e0[b == i].mean() if (b == i).sum() > 3 else np.nan for i in range(12)])
        m1 = np.array([e1[b == i].mean() if (b == i).sum() > 3 else np.nan for i in range(12)])
        return ctr, m0 - np.nanmean(m0), m1 - np.nanmean(m1)
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.8))
    c, a0, _ = binned(f'{RUNS}/DEMO_0007.CSV', 55, 185, False, free_centre=True)
    ax[0].plot(c, a0, 'o-', color='#3a9a5b', label=f'first spot, wheels stopped ({np.nanmax(a0)-np.nanmin(a0):.1f}° p-p)')
    c, b0, _ = binned(f'{RUNS}/SLEW_0001.CSV', 708, 1342, False, free_centre=True)
    ax[0].plot(c, b0, 's-', color='#b8433a', label=f'presentation spot, wheels stopped ({np.nanmax(b0)-np.nanmin(b0):.1f}° p-p)')
    ax[0].set_xlabel('field direction in the body frame (deg)'); ax[0].set_ylabel('heading error (deg)')
    ax[0].legend(fontsize=7); ax[0].set_title('Magnetometer heading error vs heading\n(circle centre fitted freely: room effect only)')
    c, s0, s1 = binned(f'{RUNS}/SLEW_0001.CSV', 1344, 1564, True, smooth=True)
    ax[1].plot(c, s0, 'o-', color='#b8433a', label=f'uncorrected ({np.nanmax(s0)-np.nanmin(s0):.1f}° p-p)')
    ax[1].plot(c, s1, 's-', color='#2f6fb0', label=f'room correction, in-sample ({np.nanmax(s1)-np.nanmin(s1):.1f}° p-p)')
    ax[1].text(0.02, 0.04, 'cross-check (fit one direction, test the other): 3.5° and 5.3° p-p', transform=ax[1].transAxes, fontsize=7)
    ax[1].set_xlabel('field direction in the body frame (deg)'); ax[1].legend(fontsize=7)
    ax[1].set_title('Presentation spot, wheels spinning ±100 rpm\n(firmware hard iron + wheel model, 0.5 s smoothing)')
    save(fig, 'f11_room_distortion.png')


# ---------- F12: gyro scale ----------
def f12():
    d = load(f'{RUNS}/DEMO_0007.CSV', ['mcutime_ms', 'mag_x_uT', 'mag_y_uT', 'gyro_z_rad_s'] + WCOLS)
    m = (d['t'] > 55) & (d['t'] < 185) & (np.max(np.abs([d[c] for c in WCOLS]), axis=0) < 1)
    tt = d['t'][m]; mx = d['mag_x_uT'][m]; my = d['mag_y_uT'][m]
    A = np.column_stack([mx, my, np.ones(len(mx))]); c, *_ = np.linalg.lstsq(A, mx ** 2 + my ** 2, rcond=None)
    ang = -np.unwrap(np.arctan2(my - c[1] / 2, mx - c[0] / 2)) * R2D
    gz = np.cumsum((d['gyro_z_rad_s'][m] + 0.0493 / R2D) * np.diff(np.r_[tt[0], tt])) * R2D
    ang -= ang[0]
    p = np.polyfit(gz, ang, 1)
    fig, ax = plt.subplots(figsize=(7, 3.4))
    n = len(gz) // 100 * 100
    gzb = gz[:n].reshape(-1, 100).mean(axis=1); dfb = (ang - gz)[:n].reshape(-1, 100).mean(axis=1)
    ax.plot(gzb, dfb, lw=0.9, color='#2f6fb0', label='magnetometer turn − gyro turn (1 s averages)')
    ax.plot(gz, np.polyval(p, gz) - gz, 'k--', lw=0.9, label=f'fit: true/gyro = {p[0]:.4f}')
    ax.set_xlabel('turn integrated from the raw gyro (deg)'); ax.set_ylabel('difference (deg)')
    ax.legend(fontsize=7); ax.set_title('Yaw gyro scale (DEMO_0007, 376° free turn): gyro reads ~0.9 % short')
    save(fig, 'f12_gyroscale.png')


if __name__ == '__main__':
    which = sys.argv[2:] or ['f1', 'f2', 'f3', 'f4', 'f5', 'f6', 'f7', 'f8', 'f9', 'f10', 'f11', 'f12']
    for w in which:
        globals()[w]()
