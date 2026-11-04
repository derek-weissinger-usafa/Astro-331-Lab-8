"""Projector-sized figures for the final deck's presented data slides (2026-11-02 rebuild; 2026-11-03: saturated
projector palette, thicker lines, and each figure drawn at the exact size of its slide's plot area, SLOT below, so
it fills the space between the takeaway banner and the text with no white margins). 13 pt text at 11 in wide
(~11 pt on the slide), a suptitle with the test conditions. Run from the repo root:
    python Reports/figures/final_2026-11-02/deck_figs.py <scratch dir with mt1_nomag.csv>
Then Reports/deck_builder/projector.py places them on the slides.
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
# Projector palette: saturated hues that survive a washed-out projector (2026-11-03)
COL = {'A': '#0b5ed7', 'B': '#f08a00', 'C': '#0a9a3c', 'D': '#d7191c'}
INK, RED, GREEN, BLUE, GREY, ORANGE = '#111111', '#d7191c', '#0a9a3c', '#0b5ed7', '#555555', '#f08a00'
DRED, DGREEN, DBLUE, DORANGE = '#b30000', '#007a2c', '#0047b3', '#b35900'   # text on light fills
BAND = '#c6efd2'                                           # +-1 deg requirement band
PART = ('#cfe2ff', '#c9f0d2', '#ffdcb5')                   # DEMO ALL parts 1-3
plt.rcParams.update({'font.size': 13, 'axes.titlesize': 13, 'axes.labelsize': 13, 'legend.fontsize': 11.5,
                     'xtick.labelsize': 12, 'ytick.labelsize': 12, 'axes.grid': True, 'grid.alpha': 0.35,
                     'figure.dpi': 150, 'axes.edgecolor': '#222', 'axes.linewidth': 1.0, 'legend.framealpha': 0.95,
                     'lines.linewidth': 2.2})

# Plot area of each figure's slide (inches, w x h), from Reports/deck_builder/projector.py's layout: the picture
# spans 0.3-9.7 in and runs from just under the takeaway banner to just above the slide's text block.
SLOT = {'d10_imu': (9.4, 3.71), 'd1_magtest': (9.4, 3.49), 'd2_balance': (9.4, 3.27), 'd3_laser_holds': (9.4, 3.71),
        'd11_tilt': (9.4, 3.93), 'd6_demo_all': (9.4, 4.25), 'd12_req1': (9.4, 4.16), 'd7_req2': (9.4, 3.71),
        'd8_req3': (9.4, 4.22)}


def figsize(name, width=11.0):
    w, h = SLOT[name]
    return width, width * h / w


def save(fig, name, title=None):
    # title: the test and its conditions (course guidance: titles, units, legends, test conditions, limits)
    if title:
        h = fig.get_figheight()   # fixed gap in inches under the title, whatever the figure height
        fig.suptitle(title, fontsize=13, fontweight='bold', y=1 - 0.04 / h, va='top')
        fig.tight_layout(pad=0.5, rect=(0, 0, 1, 1 - 0.26 / h))
    else:
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
    fig, ax = plt.subplots(figsize=figsize('d1_magtest'))
    band(ax)
    ax.plot(d['t'] / 60, d['yaw_deg'], lw=1.6, color=BLUE, label='on board, magnetometer aid on')
    ax.plot(tr / 60, yr, lw=2.4, color=RED, label='same data replayed, gyro only')
    for m in (1.0, 20.0, 22.5):
        ax.axvline(m, color=GREY, lw=1.0, ls=':')
    ax.text(24.0, 2.3, '±10° estimate kicks (clipped): the aid pulls the heading back', fontsize=11, color=GREY)
    ax.set_ylim(-6, 3.2)
    ax.set_xlim(0, 57)
    ax.set_xlabel('minutes into the test (table clamped: any yaw is error)')
    ax.set_ylabel('yaw error (deg)')
    ax.text(3.0, -0.85, '±1° requirement', fontsize=11, color=DGREEN, fontweight='bold')
    ax.text(39, -3.55, 'gyro alone: −5.7 °/h', fontsize=12.5, color=DRED, fontweight='bold')
    ax.text(34, 1.35, 'with the aid: 36 / 37 steps within ±1°', fontsize=12.5, color=DBLUE, fontweight='bold')
    ax.legend(loc='lower left')
    save(fig, 'd1_magtest.png',
         'MAG TEST run 1 (MAGTEST_0001, 10-30): table clamped, wheels stepped through 14 direction patterns')


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
    fig, ax = plt.subplots(figsize=figsize('d2_balance'))
    i = np.arange(len(R))
    ax.axhspan(-5, 5, color=BAND, zorder=0)
    ax.plot(i, R, 'o-', ms=6.5, lw=2.4, color=BLUE, label='roll at rest')
    ax.plot(i, P, 's-', ms=6.5, lw=2.4, color=ORANGE, label='pitch at rest')
    key = [('glued', 'IMU glued,\nLEVEL ZERO'), ('Big jump', 'mass shifted'), ('Loose heavy weights', 'weights + top\nscrews off'),
           ('Added 3 x 8 g', 'washers on\nCD, AD')]
    for q, (k, lab) in enumerate(key):
        for j, n in notes:
            if k in n:
                ax.axvline(j - 0.5, color='#333', lw=1.0, ls='--')
                ax.text(j - 0.3, 7.0 if q % 2 == 0 else -10.8, lab, fontsize=11, va='top', color='#222')
                break
    ax.annotate(f'final: {R[-1]:+.1f} / {P[-1]:+.1f}°', xy=(i[-1], R[-1]), xytext=(i[-1] - 6.5, -8.5), fontsize=12.5,
                color=INK, fontweight='bold', arrowprops=dict(arrowstyle='->', color=INK, lw=1.4))
    ax.set_ylim(-14, 8)
    ax.set_xlabel('rest-tilt reading, in order (DEMO LEVEL, 10-31 session)')
    ax.set_ylabel('rest tilt (deg)')
    ax.text(10.6, 4.1, '±5° comfortable for DEMO LEVEL', fontsize=11, color=DGREEN, fontweight='bold')
    ax.legend(loc='lower center', ncol=2)
    save(fig, 'd2_balance.png',
         'Rest tilt after each trim move: DEMO LEVEL on the air bearing, 10-31 balance session')


# ---------------------------------------------------------------- d3 laser holds (slide 11)
def d3_laser_holds():
    fig, ax = plt.subplots(figsize=figsize('d3_laser_holds'))
    # legend names the calibration each line had (team's wording, 2026-11-03): red 5 min, blue and green 10 min
    runs = [
        ('red: 5 min calibration', [0, 31, 61, 91, 121, 151], [0, -0.25, -0.43, -0.58, -0.65, -0.78], RED, 'o'),
        ('blue: 10 min calibration (5 min hold)', [0, 61, 91, 121, 151, 182, 212, 242, 272, 302],
         [-0.20, -0.22, -0.30, -0.40, -0.45, -0.38, -0.37, -0.30, -0.30, -0.37], BLUE, 's'),
        ('green: 10 min calibration (live run)', [30, 60, 120, 150, 180], [-0.05, -0.03, -0.03, -0.08, -0.18], GREEN,
         'D'),
    ]
    band(ax)
    for lab, t, v, c, m in runs:
        v = np.array(v) - v[0]
        ax.plot(t, v, marker=m, ms=8.5, color=c, lw=2.8, label=lab)
    ax.text(160, -0.92, '≈ 15 °/h drift', color=DRED, fontsize=12.5, fontweight='bold')
    ax.text(215, 0.08, 'within 0.25° for 5 min, no trend', color=DBLUE, fontsize=12.5, fontweight='bold')
    ax.set_ylim(-1.15, 0.55)
    ax.set_xlabel('time in the DEMO LEVEL hold (s)')
    ax.set_ylabel('laser heading change (deg)')
    ax.text(3, 0.38, 'band = ±1° requirement', fontsize=11, color=DGREEN, fontweight='bold')
    ax.legend(loc='lower left')
    save(fig, 'd3_laser_holds.png',
         'DEMO LEVEL heading holds on the air bearing, 11-01: laser on a wall scale at R = 2.48 m')


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
    a.text(-0.45, -0.85, '±1° requirement', fontsize=10.5, color=GREEN)
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
    save(fig, 'd4_heading_leak.png',
         'Full-turn SLEW YAW runs on the air bearing, 11-01: laser (R = 2.48 m) vs logged gyro and estimate')


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
        a.axvspan(0.2, 1.0, color='#ffc9c4', zorder=0)
    ax[0].set_ylabel('encoder\ncounts'); ax[1].set_ylabel('measured\nspeed (rad/s)'); ax[2].set_ylabel('motor\nduty')
    ax[0].legend(ncol=4, loc='lower left', fontsize=10)
    ax[0].text(0.6, 1700, 'loop() blocked ≈ 0.8 s by the SD card: wheels turning, speed and duty frozen',
               fontsize=11.5, color=DRED, ha='center')
    ax[2].text(1.05, 0.1, 'then all four flagged "dead", disarm', fontsize=11.5, color=DRED)
    ax[2].set_xlabel('seconds after arming (DEMO_0016, first arm after a boot)')
    for a in ax:
        a.tick_params(labelsize=11)
    save(fig, 'd5_sd_stall.png',
         'First arm after a boot (DEMO_0016, 11-01): wheel data from the 100 Hz SD log')


# ---------------------------------------------------------------- d10 IMU integrity (slide 11)
def d10_imu():
    """Time between gyro samples before / after the IMU read fixes (was Reports/figures/fig3_imu_integrity.png)."""
    fig, axs = plt.subplots(1, 2, figsize=figsize('d10_imu'), sharey=True)
    for ax, name, title, colr in ((axs[0], 'STATIC_0001.CSV', 'Before: polled I²C, Adafruit driver', RED),
                                  (axs[1], 'STATIC_0009.CSV', 'After: INT-driven reads + fixes (10-20)', BLUE)):
        t = load(f'{RUNS}/{name}', ['mcutime_ms'])['mcutime_ms']
        dt = np.diff(t)
        tm = (t[1:] - t[0]) / 60e3
        ax.scatter(tm, dt, s=5, color=colr, alpha=0.75, lw=0, rasterized=True)
        ax.axhline(100, color=INK, ls='--', lw=1.8)
        ax.axhline(10, color=DGREEN, ls=':', lw=2.0)
        ax.set(yscale='log', ylim=(5, 1000), xlabel='time (min)')
        ax.set_title(title, fontsize=12.5, fontweight='bold')
        eff = len(t) / ((t[-1] - t[0]) / 1e3)
        ax.text(0.98, 0.95, f'{int((dt > 100).sum())} gaps > 100 ms\nmax gap {dt.max():.0f} ms\n{eff:.1f} of 100 samples/s',
                transform=ax.transAxes, ha='right', va='top', fontsize=12, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='#888888'))
    axs[0].set_ylabel('time between gyro\nsamples (ms)')
    axs[0].text(0.01, 100 * 1.15, 'IMU timeout (disarms wheels)', transform=axs[0].get_yaxis_transform(), fontsize=11,
                fontweight='bold')
    axs[1].text(0.01, 14, 'nominal 10 ms (dotted)', transform=axs[1].get_yaxis_transform(), fontsize=11,
                color=DGREEN, fontweight='bold')
    save(fig, 'd10_imu.png',
         'Every gyro sample logged to SD, IMU at rest, motors off: STATIC_0001 (37 min) vs STATIC_0009 (4.9 min)')


# ---------------------------------------------------------------- d11 tilted slews vs simulation (slide 15)
def d11_tilt():
    """SLEW TEST with a fixed tilt on the table vs the simulation (was writeup_2026-10-31/f09_fixedtilt_vs_sim.png)."""
    wc = [f'motor_{w}_measured_rate_rpm' for w in 'abcd']
    d = load(f'{RUNS}/DEMO_0007.CSV', ['mcutime_ms'] + wc, ['state', 'demo_phase'])
    m = np.r_[(d['state'] == 'A') & np.isin(d['demo_phase'], ['S', 'W', 'H']), False]
    e = np.flatnonzero(np.diff(np.r_[0, m.astype(int)]))
    s, e = [(a, b) for a, b in zip(e[::2], e[1::2]) if b - a >= 200][-1]
    t = d['t'][s:e] - d['t'][s]
    sim = np.genfromtxt('Reports/figures/writeup_2026-10-31/sim_fixed.csv', delimiter=',', names=True, dtype=None,
                        encoding=None)
    fig, ax = plt.subplots(1, 2, figsize=figsize('d11_tilt'), sharey=True)
    for w in 'ABCD':
        ax[0].plot(t, d[f'motor_{w.lower()}_measured_rate_rpm'][s:e] * RPM, color=COL[w], lw=2.2, label=f'wheel {w}')
        ax[1].plot(sim['t_s'], sim['w' + w], color=COL[w], lw=2.2, label=f'wheel {w}')
    for a in ax:
        a.axhspan(75, 90, color='#ffc9c4', zorder=0); a.axhspan(-90, -75, color='#ffc9c4', zorder=0)
        a.axhline(75, color=DRED, ls='--', lw=1.6); a.axhline(-75, color=DRED, ls='--', lw=1.6)
        a.set_xlabel('s from the test start'); a.set_ylim(-90, 90)
    ax[0].text(0.3, 79, 'wheel limit 75 rad/s', fontsize=11, color=DRED, fontweight='bold')
    ax[1].legend(loc='lower left', ncol=2, fontsize=11)
    ax[0].set_ylabel('wheel speed (rad/s)')
    ax[0].set_title('Table: roll/pitch held at −1.2 / −0.2°\nwheel limit after 13.5 s, during slew 1', fontsize=12.5)
    ax[1].set_title('Simulation, same targets\nwheel limit after 88 s, in target 4', fontsize=12.5)
    ax[0].set_xlim(0, 20)
    save(fig, 'd11_tilt.png', 'SLEW TEST with a fixed tilt (DEMO_0007, 10-31) vs the simulation (tools/slewsim)')


# ---------------------------------------------------------------- DEMO ALL (slides 16-19)
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
    fig, ax = plt.subplots(3, 1, figsize=figsize('d6_demo_all'), sharex=True, gridspec_kw={'height_ratios': [2.0, 1.1, 1.3]})
    def shade(axx):
        axx.axvspan(T['settle'], T['slew'], color=PART[0], zorder=0)
        axx.axvspan(T['slew'], T['done'], color=PART[1], zorder=0)
        axx.axvspan(T['done'], T['end'], color=PART[2], zorder=0)
    shade(ax[0])
    ax[0].plot(t, yrel, color=INK, lw=2.6)
    ax[0].set_ylabel('heading from\nstart (deg)')
    for x, txt in [((T['settle'] + T['slew']) / 2, '1  heading hold, 60 s'), ((T['slew'] + T['done']) / 2, '2  four slews, 360°'),
                   ((T['done'] + T['end']) / 2, '3  wheel B fails')]:
        ax[0].text(x, 300, txt, ha='center', fontsize=12.5, fontweight='bold')
    shade(ax[1])
    ax[1].plot(t, s['roll_deg'], color=BLUE, lw=1.8, label='roll')
    ax[1].plot(t, s['pitch_deg'], color=RED, lw=1.8, label='pitch')
    ax[1].set_ylabel('tilt (deg)'); ax[1].legend(loc='upper left', ncol=2, fontsize=10)
    shade(ax[2])
    for w in 'ABCD':
        ax[2].plot(t, s[f'motor_{w.lower()}_measured_rate_rpm'] * RPM, color=COL[w], lw=1.8, label=w)
    ax[2].axvline(T['inj'], color='k', lw=1.2, ls=':')
    ax[2].set_ylabel('wheels\n(rad/s)'); ax[2].set_xlabel('time since DEMO ALL (s)')
    ax[2].legend(loc='lower left', ncol=4, fontsize=10)
    for axx in ax:
        axx.tick_params(labelsize=11)
    save(fig, 'd6_demo_all.png',
         'DEMO ALL (DEMO_0020, 11-01 23:13): one armed run on the air bearing, 100 Hz log')

    # d12 requirement 1: the graded 60 s heading hold (part 1)
    hold = slice(i_settle, i_slew)
    herr = yrel[hold]
    th = t[hold] - T['settle']
    fig, ax = plt.subplots(figsize=figsize('d12_req1'))
    band(ax)
    ax.plot(th, herr, color=INK, lw=2.0, label='heading error (estimate)')
    ax.set_ylim(-1.2, 1.2); ax.set_xlim(th[0], th[-1])
    ax.set_xlabel('time in the graded hold (s)')
    ax.set_ylabel('heading error (deg)')
    ax.text(1, 0.85, 'band = ±1° requirement', fontsize=11.5, color=DGREEN, fontweight='bold')
    ax.text(th[-1] - 1, 0.35, f'max error {np.max(np.abs(herr)):.2f}°, jitter {np.std(herr):.3f}° rms\n'
            f'laser steady at −0.05° (dot did not move)', fontsize=12.5, color=DBLUE, fontweight='bold', ha='right')
    save(fig, 'd12_req1.png', 'DEMO ALL part 1 (DEMO_0020): 60 s graded heading hold on the air bearing')

    # d7 requirement 2
    fig, ax = plt.subplots(1, 2, figsize=figsize('d7_req2'), gridspec_kw={'width_ratios': [1.15, 1]})
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
    ax[1].axhline(1.0, color=RED, lw=1.8, ls='--')
    ax[1].text(3.3, 1.03, 'limit 1°', color=DRED, fontsize=11.5, ha='right', fontweight='bold')
    ax[1].set_ylim(0, 1.15)
    ax[1].set_ylabel('max |yaw error| in 5 s hold (deg)')
    ax[1].set_title('Each target reached and held', fontsize=12.5)
    ax[1].tick_params(axis='x', labelsize=11)
    save(fig, 'd7_req2.png',
         'DEMO ALL part 2 (DEMO_0020): four slews through 360°, laser at R = 2.48 m')

    # d8 requirement 3
    w0, w1 = T['inj'] - 5, min(T['end'] + 2, t[-1])
    m = (t >= w0) & (t <= w1)
    tt = t[m] - T['inj']
    fig, ax = plt.subplots(2, 1, figsize=figsize('d8_req3'), sharex=True)
    band(ax[0], color=PART[2])
    for c, lab, colr in [('err_roll_deg', 'roll', BLUE), ('err_pitch_deg', 'pitch', RED), ('err_yaw_deg', 'yaw', INK)]:
        ax[0].plot(tt, s[c][m], color=colr, lw=2.0, label=lab)
    ax[0].set_ylim(-1.25, 1.25); ax[0].set_ylabel('attitude error\n(deg)')
    ax[0].text(12, -0.95, '±1° requirement', fontsize=11, color=DORANGE, fontweight='bold')
    ax[0].legend(loc='lower right', ncol=3, fontsize=10)
    for w in 'ABCD':
        ax[1].plot(tt, s[f'motor_{w.lower()}_measured_rate_rpm'][m] * RPM, color=COL[w], lw=2.0, label=f'wheel {w}')
    ax[1].set_ylabel('wheel speed\n(rad/s)'); ax[1].set_xlabel('seconds since wheel B\'s motor was cut')
    ax[1].legend(loc='upper right', ncol=4, fontsize=10)
    det = t[i_det] - T['inj']
    for axx in ax:
        axx.axvline(0, color='k', lw=1.3, ls=':')
        axx.axvline(det, color=DORANGE, lw=1.8, ls='--')
        axx.tick_params(labelsize=11)
    ax[0].text(0.3, 1.02, 'motor cut', fontsize=11, color='k')
    ax[0].text(det + 0.3, -1.12, f'detected {det:.2f} s later', fontsize=11.5, color=DORANGE, fontweight='bold')
    ax[0].text(12, 0.6, 'B coasts; A, C, D hold within 0.36° for 30 s', fontsize=12, fontweight='bold')
    save(fig, 'd8_req3.png',
         "DEMO ALL part 3 (DEMO_0020): wheel B's motor cut at t = 0 without telling the controller")


if __name__ == '__main__':
    d9_laser_setup()
    d10_imu()
    d11_tilt()
    d2_balance()
    d3_laser_holds()
    d4_heading_leak()
    d5_sd_stall()
    demo_all()
    d1_magtest()
