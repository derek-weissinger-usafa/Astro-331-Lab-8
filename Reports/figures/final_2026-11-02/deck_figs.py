"""Projector-sized figures for the final deck's nine presented data slides (2026-11-02 rebuild).
Wide aspect (fills the slide width at ~3.5 in tall), 13 pt text, no in-figure titles (the slide's takeaway
banner is the title), short legends. Run from the repo root:
    python Reports/figures/final_2026-11-02/deck_figs.py <scratch dir with mt1_nomag.csv>
Laser readings were called out by the operator during the runs (PROGRESS_LOG 2.27-2.34).
"""
import csv, math, os, re, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

OUT = os.path.dirname(os.path.abspath(__file__))
RUNS = 'test_runs'
SCR = sys.argv[1] if len(sys.argv) > 1 else '.'
RPM = 2 * math.pi / 60
COL = {'A': '#2f6fb0', 'B': '#c98a12', 'C': '#3a9a5b', 'D': '#b8433a'}
INK, RED, GREEN, BLUE, GREY = '#1f2933', '#b8433a', '#2e7d4f', '#2f5d8a', '#777777'
BAND = '#e8f3ec'
plt.rcParams.update({'font.size': 13, 'axes.titlesize': 13, 'axes.labelsize': 13, 'legend.fontsize': 11,
                     'xtick.labelsize': 12, 'ytick.labelsize': 12, 'axes.grid': True, 'grid.alpha': 0.3,
                     'figure.dpi': 150, 'axes.edgecolor': '#444', 'legend.framealpha': 0.9})


def save(fig, name):
    fig.tight_layout(pad=0.6)
    fig.savefig(os.path.join(OUT, name))
    plt.close(fig)
    print('wrote', name)


def load(path, cols, strcols=()):
    with open(path, newline='') as f:
        rd = csv.reader(f)
        h = next(rd)
        ci = {k: i for i, k in enumerate(h)}
        idx = [ci[c] for c in cols]
        sidx = [ci[c] for c in strcols]
        F, S = [], []
        for r in rd:
            if len(r) < len(h) or not r[0][:1].isdigit():
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


def band(ax, lo=-1, hi=1, color=BAND):
    ax.axhspan(lo, hi, color=color, zorder=0)


# ---------------------------------------------------------------- d9 laser setup (slide 8)
def d9_laser_setup():
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.9), gridspec_kw={'width_ratios': [1.7, 1]})
    a = ax[0]
    a.set_aspect('equal'); a.axis('off'); a.grid(False)
    a.add_patch(Circle((0, 0), 0.38, fc='#dfe7f1', ec=BLUE, lw=1.5))
    a.text(0, -0.66, 'air bearing table', ha='center', fontsize=12)
    a.add_patch(Rectangle((-0.06, 0.06), 0.12, 0.3, fc=RED, ec='k'))
    a.text(0.14, 0.44, 'laser on the deck', fontsize=12, color=RED)
    L = 4.0
    th = math.radians(5)
    a.plot([0, L], [0.2, 0.2], color=RED, lw=1.4, ls='--')
    a.plot([0, L], [0.2, 0.2 + L * math.tan(th)], color=RED, lw=1.6)
    a.add_patch(Rectangle((L, -0.9), 0.08, 2.0, fc='#cfcfcf', ec='k'))
    a.text(L + 0.16, 0.85, 'wall + printed scale\n(0.1° ticks)', fontsize=12)
    a.annotate('', xy=(L, -0.78), xytext=(0, -0.78), arrowprops=dict(arrowstyle='<->', lw=1.2))
    a.text(L / 2, -1.0, 'R = 2.48 m', ha='center', fontsize=13, fontweight='bold')
    a.annotate('', xy=(L - 0.06, 0.2 + L * math.tan(th)), xytext=(L - 0.06, 0.2), arrowprops=dict(arrowstyle='<->', lw=1.2))
    a.text(L - 0.18, 0.4, 'x', ha='right', fontsize=14, fontweight='bold')
    a.text(0.75, 0.28, 'θ', fontsize=15)
    a.set_xlim(-0.8, 5.5); a.set_ylim(-1.25, 1.25)
    b = ax[1]
    deg = np.linspace(-1, 1, 201)
    b.plot(deg, 2480 * np.tan(np.radians(deg)), color=BLUE, lw=2)
    for d in (0.1, 0.5, 1.0):
        y = 2480 * math.tan(math.radians(d))
        b.plot([d], [y], 'o', color=RED, ms=6)
        b.text(d - 0.06, y + 2.5, f'{d}° = {y:.1f} mm', ha='right', fontsize=11)
    b.set_xlabel('heading change θ (deg)'); b.set_ylabel('dot movement x (mm)')
    b.set_title('x = R tan θ', fontsize=13)
    save(fig, 'd9_laser_setup.png')


# ---------------------------------------------------------------- d1 MAG TEST (slide 9)
def d1_magtest():
    d = load(f'{RUNS}/MAGTEST_0001.CSV', ['mcutime_ms', 'yaw_deg'])
    rp = list(csv.DictReader(open(os.path.join(SCR, 'mt1_nomag.csv'))))
    tr = np.array([float(r['t_s']) for r in rp]); tr -= tr[0]
    yr = np.array([float(r['yaw_replay_deg']) for r in rp])
    fig, ax = plt.subplots(figsize=(11, 3.7))
    band(ax)
    ax.plot(d['t'] / 60, d['yaw_deg'], lw=0.9, color=BLUE, label='on board, magnetometer aid on')
    ax.plot(tr / 60, yr, lw=1.3, color=RED, label='same data replayed, gyro only')
    for m in (1.0, 20.0, 22.5):
        ax.axvline(m, color=GREY, lw=0.8, ls=':')
    ax.text(24.0, 2.3, '±10° estimate kicks (clipped): the aid pulls the heading back', fontsize=10.5, color=GREY)
    ax.set_ylim(-6, 3.2)
    ax.set_xlim(0, 57)
    ax.set_xlabel('minutes into the test (table clamped: any yaw is error)')
    ax.set_ylabel('yaw error (deg)')
    ax.text(3.0, -0.85, '±1° requirement', fontsize=10.5, color=GREEN)
    ax.text(40, -4.55, 'gyro alone: −5.7 °/h', fontsize=12, color=RED, fontweight='bold')
    ax.text(34, 1.35, 'with the aid: 36 / 37 steps within ±1°', fontsize=12, color=BLUE, fontweight='bold')
    ax.legend(loc='lower left')
    save(fig, 'd1_magtest.png')


# ---------------------------------------------------------------- d2 balance (slide 10)
def d2_balance():
    lines = open(f'{RUNS}/balance_iterations_2026-10-31.txt', encoding='utf-8').read().splitlines()
    R, P, notes, pending = [], [], [], []
    for l in lines:
        m = re.search(r'^2026-10-31 .*DEMO LEVEL rest tilt: roll ([-+\d.]+) deg, pitch ([-+\d.]+)', l)
        if m:
            R.append(float(m.group(1))); P.append(float(m.group(2)))
            for n in pending:
                notes.append((len(R) - 1, n))
            pending = []
        elif l.startswith('NOTE'):
            pending.append(l[5:].strip())
    fig, ax = plt.subplots(figsize=(11, 3.7))
    i = np.arange(len(R))
    ax.axhspan(-5, 5, color=BAND, zorder=0)
    ax.plot(i, R, 'o-', ms=5, lw=1.6, color=BLUE, label='roll at rest')
    ax.plot(i, P, 's-', ms=5, lw=1.6, color='#c98a12', label='pitch at rest')
    key = [('glued', 'IMU glued,\nLEVEL ZERO'), ('Big jump', 'mass shifted'), ('Loose heavy weights', 'weights + top\nscrews off'),
           ('Added 3 x 8 g', 'washers on\nCD, AD')]
    for q, (k, lab) in enumerate(key):
        for j, n in notes:
            if k in n:
                ax.axvline(j - 0.5, color='grey', lw=0.8, ls='--')
                ax.text(j - 0.3, 7.0 if q % 2 == 0 else -10.8, lab, fontsize=10.5, va='top', color='#444')
                break
    ax.annotate(f'final: {R[-1]:+.1f} / {P[-1]:+.1f}°', xy=(i[-1], R[-1]), xytext=(i[-1] - 6.5, -8.5), fontsize=12,
                color=INK, arrowprops=dict(arrowstyle='->', color=INK))
    ax.set_ylim(-14, 8)
    ax.set_xlabel('rest-tilt reading, in order (DEMO LEVEL, 10-31 session)')
    ax.set_ylabel('rest tilt (deg)')
    ax.text(0.3, 4.1, '±5° comfortable for DEMO LEVEL', fontsize=10.5, color=GREEN)
    ax.legend(loc='upper center', ncol=2)
    save(fig, 'd2_balance.png')


# ---------------------------------------------------------------- d3 laser holds (slide 11)
def d3_laser_holds():
    fig, ax = plt.subplots(figsize=(11, 3.7))
    runs = [
        ('calibrated 5 min after power-on', [0, 31, 61, 91, 121, 151], [0, -0.25, -0.43, -0.58, -0.65, -0.78], RED, 'o'),
        ('warm calibration, 5 min hold', [0, 61, 91, 121, 151, 182, 212, 242, 272, 302],
         [-0.20, -0.22, -0.30, -0.40, -0.45, -0.38, -0.37, -0.30, -0.30, -0.37], BLUE, 's'),
        ('warm calibration, live run', [30, 60, 120, 150, 180], [-0.05, -0.03, -0.03, -0.08, -0.18], GREEN, 'D'),
    ]
    band(ax)
    for lab, t, v, c, m in runs:
        v = np.array(v) - v[0]
        ax.plot(t, v, marker=m, ms=7, color=c, lw=2, label=lab)
    ax.text(160, -0.92, '≈ 15 °/h drift', color=RED, fontsize=12, fontweight='bold')
    ax.text(215, 0.08, 'within 0.25° for 5 min, no trend', color=BLUE, fontsize=12, fontweight='bold')
    ax.set_ylim(-1.15, 0.55)
    ax.set_xlabel('time in the DEMO LEVEL hold (s)')
    ax.set_ylabel('laser heading change (deg)')
    ax.text(3, 0.38, 'band = ±1° requirement', fontsize=10.5, color=GREEN)
    ax.legend(loc='lower left')
    save(fig, 'd3_laser_holds.png')


# ---------------------------------------------------------------- d4 heading leak (slide 12)
def d4_heading_leak():
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.9), gridspec_kw={'width_ratios': [1.4, 1]})
    a = ax[0]
    runs = [('run 1', -0.10), ('run 2', 1.1), ('run 3', 0.06), ('run 4', 1.7), ('run 5', 2.28), ('mirrored\n(CW)', -1.8),
            ('after\nfix', 0.05)]
    x = np.arange(len(runs)); vals = [v for _, v in runs]
    a.bar(x, vals, color=[RED] * 6 + [GREEN], width=0.62)
    for xi, v in zip(x, vals):
        a.text(xi, v + (0.12 if v >= 0 else -0.33), f'{v:+.2f}°', ha='center', fontsize=11, fontweight='bold')
    a.set_xticks(x); a.set_xticklabels([r for r, _ in runs], fontsize=11)
    band(a); a.axhline(0, color='k', lw=0.6)
    a.set_ylim(-2.5, 3.0)
    a.set_ylabel('laser error after 360° of slews (deg)')
    a.set_title('Six full-turn slews on 11-01, in order', fontsize=12.5)
    b = ax[1]
    names = ['run 4\nCCW', 'run 5\nCCW', 'mirrored\nCW', 'after fix\nCCW']
    gyro = [3.4, 4.1, 3.1, 0.01]; ekf = [0.0, 0.0, 0.0, 0.0]; laser = [1.7, 2.28, 1.8, 0.05]
    xx = np.arange(4); w = 0.27
    b.bar(xx - w, gyro, w, color=GREY, label='∫ gyro')
    b.bar(xx, ekf, w, color=RED, label='estimator')
    b.bar(xx + w, laser, w, color=BLUE, label='laser')
    for xi, (g, l) in enumerate(zip(gyro, laser)):
        b.text(xi - w, g + 0.08, f'{g:.1f}', ha='center', fontsize=10)
        b.text(xi + w, l + 0.08, f'{l:.1f}', ha='center', fontsize=10)
    b.set_xticks(xx); b.set_xticklabels(names, fontsize=11)
    b.set_ylabel('|turn| − 360° (deg)')
    b.set_title('Estimator vs its own gyro input', fontsize=12.5)
    b.legend(loc='upper right', fontsize=10)
    b.set_ylim(-0.3, 5.2)
    save(fig, 'd4_heading_leak.png')


# ---------------------------------------------------------------- d5 SD stall (slide 13)
def d5_sd_stall():
    d = load(f'{RUNS}/DEMO_0016.CSV', ['mcutime_ms'] + [f'motor_{w}_measured_rate_rpm' for w in 'abcd'] +
             [f'motor_{w}_duty' for w in 'abcd'] + [f'enc_{w}' for w in 'abcd'], ['state'])
    a0 = next(i for i, s in enumerate(d['state']) if s == 'A')
    t = d['t'] - d['t'][a0]
    m = (t >= -0.15) & (t <= 1.9)
    fig, ax = plt.subplots(3, 1, figsize=(11, 4.6), sharex=True)
    for w in 'ABCD':
        k = w.lower()
        ax[0].plot(t[m], d[f'enc_{k}'][m] - d[f'enc_{k}'][a0], color=COL[w], lw=1.8, label=f'wheel {w}')
        ax[1].plot(t[m], d[f'motor_{k}_measured_rate_rpm'][m] * RPM, color=COL[w], lw=1.8)
        ax[2].plot(t[m], d[f'motor_{k}_duty'][m], color=COL[w], lw=1.8)
    for a in ax:
        a.axvspan(0.2, 1.0, color='#fbe3e0', zorder=0)
    ax[0].set_ylabel('encoder\ncounts'); ax[1].set_ylabel('measured\nspeed (rad/s)'); ax[2].set_ylabel('motor\nduty')
    ax[0].legend(ncol=4, loc='lower left', fontsize=10)
    ax[0].text(0.6, 1700, 'loop() blocked ≈ 0.8 s by the SD card: wheels turning, speed and duty frozen',
               fontsize=11.5, color='#9c2f25', ha='center')
    ax[2].text(1.05, 0.1, 'then all four flagged "dead", disarm', fontsize=11.5, color='#9c2f25')
    ax[2].set_xlabel('seconds after arming (DEMO_0016, first arm after a boot)')
    for a in ax:
        a.tick_params(labelsize=11)
    save(fig, 'd5_sd_stall.png')


# ---------------------------------------------------------------- DEMO ALL (slides 14-16)
D20_F = ['mcutime_ms', 'yaw_deg', 'roll_deg', 'pitch_deg', 'yaw_ref_deg', 'err_roll_deg', 'err_pitch_deg', 'err_yaw_deg',
         'test_step'] + [f'motor_{w}_measured_rate_rpm' for w in 'abcd']
D20_S = ['state', 'demo_phase', 'fault_phase', 'wheel_status']


def demo_all():
    d = load(f'{RUNS}/DEMO_0020.CSV', D20_F, D20_S)
    armed = (d['state'] == 'A').astype(int)
    e = np.flatnonzero(np.diff(np.r_[0, armed, 0]))
    a, b = max(zip(e[::2], e[1::2]), key=lambda s: s[1] - s[0])
    s = {k: v[a:b] for k, v in d.items()}
    t = s['t'] - s['t'][0]
    yrel = np.degrees(np.unwrap(np.radians(s['yaw_deg'] - s['yaw_deg'][0])))
    ph, fp = s['demo_phase'], s['fault_phase']
    first = lambda mask, st=0: (st + np.flatnonzero(mask[st:])[0]) if np.any(mask[st:]) else None
    i_settle = first(ph == 'K'); i_slew = first(np.isin(ph, ['S', 'W', 'H', 'D'])); i_done = first(ph == 'D')
    i_inj = first(fp == 'F'); i_det = first(np.char.find(s['wheel_status'].astype(str), 'D') == 1)
    i_end = first(np.isin(fp, ['P', 'X']))
    T = dict(settle=t[i_settle], slew=t[i_slew], done=t[i_done], inj=t[i_inj], end=t[i_end])

    # d6 overview
    fig, ax = plt.subplots(3, 1, figsize=(11, 5.3), sharex=True, gridspec_kw={'height_ratios': [2.0, 1.1, 1.3]})
    def shade(axx):
        axx.axvspan(T['settle'], T['slew'], color='#e8f0fa', zorder=0)
        axx.axvspan(T['slew'], T['done'], color='#eef7ee', zorder=0)
        axx.axvspan(T['done'], T['end'], color='#fbf0e6', zorder=0)
    shade(ax[0])
    ax[0].plot(t, yrel, color=INK, lw=2)
    ax[0].set_ylabel('heading from\nstart (deg)')
    for x, txt in [((T['settle'] + T['slew']) / 2, '1  heading hold, 60 s'), ((T['slew'] + T['done']) / 2, '2  four slews, 360°'),
                   ((T['done'] + T['end']) / 2, '3  wheel B fails')]:
        ax[0].text(x, 300, txt, ha='center', fontsize=12.5, fontweight='bold')
    shade(ax[1])
    ax[1].plot(t, s['roll_deg'], color=BLUE, lw=1.3, label='roll')
    ax[1].plot(t, s['pitch_deg'], color=RED, lw=1.3, label='pitch')
    ax[1].set_ylabel('tilt (deg)'); ax[1].legend(loc='upper left', ncol=2, fontsize=10)
    shade(ax[2])
    for w in 'ABCD':
        ax[2].plot(t, s[f'motor_{w.lower()}_measured_rate_rpm'] * RPM, color=COL[w], lw=1.3, label=w)
    ax[2].axvline(T['inj'], color='k', lw=1, ls=':')
    ax[2].set_ylabel('wheels\n(rad/s)'); ax[2].set_xlabel('time since DEMO ALL (s)')
    ax[2].legend(loc='lower left', ncol=4, fontsize=10)
    for axx in ax:
        axx.tick_params(labelsize=11)
    save(fig, 'd6_demo_all.png')

    # d7 requirement 2
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.8), gridspec_kw={'width_ratios': [1.15, 1]})
    before, lo, hi = -0.05, -0.03, 0.02
    after = 0.5 * (lo + hi)
    vals = [2.1 - (-0.18), -1.8, after - before]
    labels = ['CCW turn\nbefore fix', 'CW turn\nbefore fix', 'CCW turn\nafter fix (this run)']
    ax[0].bar(labels, vals, color=[RED, RED, GREEN], width=0.6)
    ax[0].errorbar([2], [vals[2]], yerr=[[after - lo], [hi - after]], color='k', capsize=5)
    band(ax[0]); ax[0].axhline(0, color='k', lw=0.6)
    for x, v in enumerate(vals):
        ax[0].text(x, v + (0.12 if v >= 0 else -0.3), f'{v:+.2f}°', ha='center', fontsize=12, fontweight='bold')
    ax[0].set_ylim(-2.5, 2.9)
    ax[0].set_ylabel('laser error after 360° (deg)')
    ax[0].set_title('Truth by laser; band = ±1°', fontsize=12.5)
    ax[0].tick_params(axis='x', labelsize=11)
    step = s['test_step'].astype(int)
    errs = [np.max(np.abs(s['err_yaw_deg'][(ph == 'H') & (step == k)])) for k in range(4)]
    ax[1].bar([f'{k + 1}\n{n}' for k, n in enumerate(['+45°', '+135°', '−135°', '0°'])], errs, color=GREEN, width=0.6)
    for x, v in enumerate(errs):
        ax[1].text(x, v + 0.03, f'{v:.2f}°', ha='center', fontsize=11)
    ax[1].axhline(1.0, color=RED, lw=1.2, ls='--')
    ax[1].text(3.3, 1.03, 'limit 1°', color=RED, fontsize=11, ha='right')
    ax[1].set_ylim(0, 1.15)
    ax[1].set_ylabel('max |yaw error| in 5 s hold (deg)')
    ax[1].set_title('Each target reached and held', fontsize=12.5)
    ax[1].tick_params(axis='x', labelsize=11)
    save(fig, 'd7_req2.png')

    # d8 requirement 3
    w0, w1 = T['inj'] - 5, min(T['end'] + 2, t[-1])
    m = (t >= w0) & (t <= w1)
    tt = t[m] - T['inj']
    fig, ax = plt.subplots(2, 1, figsize=(11, 4.6), sharex=True)
    band(ax[0], color='#fbf0e6')
    for c, lab, colr in [('err_roll_deg', 'roll', BLUE), ('err_pitch_deg', 'pitch', RED), ('err_yaw_deg', 'yaw', INK)]:
        ax[0].plot(tt, s[c][m], color=colr, lw=1.6, label=lab)
    ax[0].set_ylim(-1.25, 1.25); ax[0].set_ylabel('attitude error\n(deg)')
    ax[0].legend(loc='lower right', ncol=3, fontsize=10)
    for w in 'ABCD':
        ax[1].plot(tt, s[f'motor_{w.lower()}_measured_rate_rpm'][m] * RPM, color=COL[w], lw=1.6, label=f'wheel {w}')
    ax[1].set_ylabel('wheel speed\n(rad/s)'); ax[1].set_xlabel('seconds since wheel B\'s motor was cut')
    ax[1].legend(loc='upper right', ncol=4, fontsize=10)
    det = t[i_det] - T['inj']
    for axx in ax:
        axx.axvline(0, color='k', lw=1, ls=':')
        axx.axvline(det, color='#c98a12', lw=1.2, ls='--')
        axx.tick_params(labelsize=11)
    ax[0].text(0.3, 1.02, 'motor cut', fontsize=11, color='k')
    ax[0].text(det + 0.3, -1.12, f'detected {det:.2f} s later', fontsize=11, color='#c98a12')
    ax[0].text(12, 0.6, 'B coasts; A, C, D hold within 0.36° for 30 s', fontsize=12, fontweight='bold')
    save(fig, 'd8_req3.png')


if __name__ == '__main__':
    d9_laser_setup()
    d2_balance()
    d3_laser_holds()
    d4_heading_leak()
    d5_sd_stall()
    demo_all()
    d1_magtest()
