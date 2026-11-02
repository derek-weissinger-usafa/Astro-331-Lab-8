"""Figures for the final presentation (2026-11-02) not covered by writeup_2026-10-31/ or demo_all_2026-11-01/.
Run from the repo root:  python Reports/figures/final_2026-11-02/make_figs.py
Laser readings were called out by the operator during the runs (PROGRESS_LOG 2.27-2.34).
"""
import csv, math, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch

OUT = os.path.dirname(os.path.abspath(__file__))
RUNS = 'test_runs'
COL = {'A': '#2f6fb0', 'B': '#c98a12', 'C': '#3a9a5b', 'D': '#b8433a'}
plt.rcParams.update({'font.size': 10, 'axes.grid': True, 'grid.alpha': 0.3, 'figure.dpi': 130})


def save(fig, name):
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, name))
    plt.close(fig)


# ---------------------------------------------------------------- N1 laser truth setup
def fig_laser_setup():
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.0), gridspec_kw={'width_ratios': [1.6, 1]})
    a = ax[0]
    a.set_aspect('equal'); a.axis('off'); a.grid(False)
    a.add_patch(Circle((0, 0), 0.35, fc='#dfe7f1', ec='#2f5d8a', lw=1.5))
    a.text(0, -0.62, 'air bearing table\n(rotates freely)', ha='center', fontsize=9)
    a.add_patch(Rectangle((-0.06, 0.05), 0.12, 0.3, fc='#b8433a', ec='k'))
    a.text(0.12, 0.42, 'laser on the deck', fontsize=9, color='#b8433a')
    L = 4.0
    a.plot([0, L], [0.2, 0.2], color='#b8433a', lw=1.4, ls='--')
    th = math.radians(5)
    a.plot([0, L], [0.2, 0.2 + L * math.tan(th)], color='#b8433a', lw=1.4)
    a.add_patch(Rectangle((L, -0.9), 0.08, 2.1, fc='#cfcfcf', ec='k'))
    a.text(L + 0.15, 0.95, 'wall with printed\nscale (0.1° ticks)', fontsize=9)
    a.annotate('', xy=(L, -0.75), xytext=(0, -0.75), arrowprops=dict(arrowstyle='<->'))
    a.text(L / 2, -0.95, 'R = 2.48 m', ha='center', fontsize=10, fontweight='bold')
    a.annotate('', xy=(L - 0.05, 0.2 + L * math.tan(th)), xytext=(L - 0.05, 0.2), arrowprops=dict(arrowstyle='<->'))
    a.text(L - 0.15, 0.4, 'x', ha='right', fontsize=11, fontweight='bold')
    a.text(0.7, 0.27, 'θ', fontsize=12)
    a.set_xlim(-0.8, 5.3); a.set_ylim(-1.2, 1.3)
    a.set_title('Heading truth: a laser on the table, read on a wall scale', fontsize=10)
    b = ax[1]
    deg = np.linspace(-1, 1, 201)
    b.plot(deg, 2480 * np.tan(np.radians(deg)), color='#2f5d8a')
    for d in (0.1, 0.5, 1.0):
        b.plot([d], [2480 * math.tan(math.radians(d))], 'o', color='#b8433a')
        b.text(d - 0.05, 2480 * math.tan(math.radians(d)) + 3, f'{d}° = {2480 * math.tan(math.radians(d)):.1f} mm',
               ha='right', fontsize=9)
    b.set_xlabel('heading change θ (deg)'); b.set_ylabel('dot movement x (mm)')
    b.set_title('x = R tan θ: 0.1° ≈ 4.3 mm, readable by eye', fontsize=10)
    save(fig, 'n1_laser_setup.png')


# ---------------------------------------------------------------- N2 laser heading holds
def fig_laser_holds():
    fig, ax = plt.subplots(figsize=(10, 4.2))
    runs = [
        ('11-01 ~16:30, calibrated 5 min after power-on', [0, 31, 61, 91, 121, 151], [0, -0.25, -0.43, -0.58, -0.65, -0.78],
         '#b8433a', 'o'),
        ('11-01 18:34, warm calibration (5 min hold)', [0, 61, 91, 121, 151, 182, 212, 242, 272, 302],
         [-0.20, -0.22, -0.30, -0.40, -0.45, -0.38, -0.37, -0.30, -0.30, -0.37], '#2f6fb0', 's'),
        ('11-01 21:35, warm calibration (live run)', [30, 60, 120, 150, 180], [-0.05, -0.03, -0.03, -0.08, -0.18], '#3a9a5b', 'D'),
    ]
    for lab, t, v, c, m in runs:
        v = np.array(v) - v[0]
        ax.plot(t, v, marker=m, color=c, lw=1.6, label=lab)
    ax.axhspan(-1, 1, color='#eef7ee', zorder=0)
    ax.set_ylim(-1.2, 0.6)
    ax.set_xlabel('time in the DEMO LEVEL hold (s)')
    ax.set_ylabel('laser heading change (deg)')
    ax.set_title('Requirement 1 by laser: holds after a warm calibration stay within 0.25°; an early calibration drifts '
                 '~15 °/h\n(green band = ±1°; each run plotted from its first reading)', fontsize=10)
    ax.legend(loc='lower left', fontsize=9)
    save(fig, 'n2_laser_holds.png')


# ---------------------------------------------------------------- N3 the EKF heading leak
def fig_heading_leak():
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2), gridspec_kw={'width_ratios': [1.35, 1]})
    a = ax[0]
    runs = [('SLEW_0003\n4 wheels\nCCW', -0.10), ('18:49\n3 wheels\nCCW', 1.1), ('18:53\n3 wheels\nCCW', 0.06),
            ('21:41\n4 wheels\nCCW', 1.7), ('22:35\nDEMO ALL\nCCW', 2.28), ('22:41\nmirrored\nCW', -1.8),
            ('23:13\nafter fix\nCCW', 0.05)]
    x = np.arange(len(runs))
    vals = [v for _, v in runs]
    cols = ['#b8433a'] * 6 + ['#3a9a5b']
    a.bar(x, vals, color=cols, width=0.6)
    for xi, v in zip(x, vals):
        a.text(xi, v + (0.1 if v >= 0 else -0.28), f'{v:+.2f}°', ha='center', fontsize=9, fontweight='bold')
    a.set_xticks(x); a.set_xticklabels([r for r, _ in runs], fontsize=8)
    a.axhspan(-1, 1, color='#eef7ee', zorder=0); a.axhline(0, color='k', lw=0.6)
    a.set_ylim(-2.4, 2.8)
    a.set_ylabel('laser heading error after 360° of slews (deg)')
    a.set_title('Full-turn SLEW YAW error on 11-01: scattered, and it flips with\nthe turn direction (red: old estimator; green: fixed)',
                fontsize=10)
    b = ax[1]
    names = ['21:41 CCW', '22:35 CCW', '22:41 CW', '23:13 CCW\n(fixed)']
    gyro = [363.4, 364.1, 363.1, 360.01]
    ekf = [360.0, 360.0, 360.0, 360.0]
    laser = [361.7, 362.28, 361.8, 360.05]
    xx = np.arange(len(names)); w = 0.26
    b.bar(xx - w, np.array(gyro) - 360, w, color='#888', label='∫ gyro z (logged rate − bias)')
    b.bar(xx, np.array(ekf) - 360, w, color='#b8433a', label='estimator yaw')
    b.bar(xx + w, np.array(laser) - 360, w, color='#2f6fb0', label='laser (truth)')
    for xi, (g, e, l) in enumerate(zip(gyro, ekf, laser)):
        for dx, v in ((-w, g - 360), (0, e - 360), (w, l - 360)):
            b.text(xi + dx, v + 0.08, f'{v:.1f}', ha='center', fontsize=7.5)
    b.set_xticks(xx); b.set_xticklabels(names, fontsize=8.5)
    b.set_ylabel('rotation − 360° (deg, |turn|)')
    b.set_title('The estimator turned less than its own gyro input\n(accelerometer updates were taking ~1% off each turn)', fontsize=10)
    b.legend(fontsize=8, loc='upper right')
    b.set_ylim(-0.5, 5)
    save(fig, 'n3_heading_leak.png')


# ---------------------------------------------------------------- N4 first-arm SD stall
def fig_sd_stall():
    path = os.path.join(RUNS, 'DEMO_0016.CSV')
    T, S, R, E, D = [], [], [], [], []
    with open(path, newline='') as f:
        rd = csv.reader(f); h = next(rd); ci = {k: i for i, k in enumerate(h)}
        for r in rd:
            if len(r) < len(h) or not r[0][:1].isdigit():
                continue
            try:
                T.append(float(r[ci['mcutime_ms']]) / 1000); S.append(r[ci['state']])
                R.append([float(r[ci[f'motor_{w}_measured_rate_rpm']]) * 2 * math.pi / 60 for w in 'abcd'])
                E.append([int(r[ci[f'enc_{w}']]) for w in 'abcd'])
                D.append([float(r[ci[f'motor_{w}_duty']]) for w in 'abcd'])
            except ValueError:
                continue
    T, R, E, D = np.array(T), np.array(R), np.array(E), np.array(D)
    a0 = next(i for i, s in enumerate(S) if s == 'A')
    t = T - T[a0]
    m = (t >= -0.2) & (t <= 2.0)
    fig, ax = plt.subplots(3, 1, figsize=(10, 5.6), sharex=True)
    for k, w in enumerate('ABCD'):
        ax[0].plot(t[m], E[m, k] - E[a0, k], color=COL[w], lw=1.4, label=f'wheel {w}')
        ax[1].plot(t[m], R[m, k], color=COL[w], lw=1.4)
        ax[2].plot(t[m], D[m, k], color=COL[w], lw=1.4)
    ax[0].set_ylabel('encoder counts\nsince arm')
    ax[1].set_ylabel('measured speed\n(rad/s)')
    ax[2].set_ylabel('motor duty')
    ax[2].set_xlabel('s after DEMO ALL armed (DEMO_0016, 21:59, first arm after a boot)')
    for a in ax:
        a.axvspan(0.2, 1.0, color='#fbe3e0', zorder=0)
    ax[0].legend(ncol=4, fontsize=8, loc='lower left')
    ax[1].text(0.25, 12, 'loop() blocked ~0.8 s by the SD card\'s first write to the new log file:\n'
               'speed estimate and duty frozen; these rows were logged in a burst afterwards', fontsize=9, color='#9c2f25')
    ax[2].text(1.05, 0.08, 'one huge wheel-loop step: all four\nflagged "dead motor", then disarm', fontsize=9, color='#9c2f25')
    fig.suptitle('Why every first run after a boot failed: a wheel-loop stall, not the wheels', fontsize=11, fontweight='bold')
    save(fig, 'n4_sd_stall.png')


if __name__ == '__main__':
    fig_laser_setup()
    fig_laser_holds()
    fig_heading_leak()
    fig_sd_stall()
    print('done')
