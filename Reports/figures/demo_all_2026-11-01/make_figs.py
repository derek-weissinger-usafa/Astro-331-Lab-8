"""Figures for the final presentation: the DEMO ALL run of 2026-11-01 23:13 (DEMO_0020.CSV).
Run from the repo root:
    python Reports/figures/demo_all_2026-11-01/make_figs.py [test_runs/DEMO_0020.CSV]
Uses the armed stretch that contains the fault test (DEMO ALL: hold -> SLEW YAW -> wheel B failure).
Laser readings (deg, + = CCW) were called out during the run and are entered below.
Writes fig*.png and summary.txt next to this script.
"""
import csv, math, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = os.path.dirname(os.path.abspath(__file__))
LOG = sys.argv[1] if len(sys.argv) > 1 else 'test_runs/DEMO_0020.CSV'
RPM = 2 * math.pi / 60
COL = {'A': '#2f6fb0', 'B': '#c98a12', 'C': '#3a9a5b', 'D': '#b8433a'}
PART = {1: '#e8f0fa', 2: '#eef7ee', 3: '#fbf0e6'}
plt.rcParams.update({'font.size': 9, 'axes.grid': True, 'grid.alpha': 0.3, 'figure.dpi': 120})

# Laser (truth), from the operator during the 23:13 run. Times are events in the log (see below).
LASER = {
    'hold': -0.05,            # steady through part 1 (reference)
    'after_slews': (0.02, -0.03),  # oscillating range when the slews ended
    'fault_peak': -0.31,      # when wheel B was cut
    'fault_steady': -0.18,    # held through the 30 s graded hold
}

FCOLS = ['mcutime_ms', 'yaw_deg', 'roll_deg', 'pitch_deg', 'yaw_ref_deg', 'err_roll_deg', 'err_pitch_deg',
         'err_yaw_deg', 'motor_a_measured_rate_rpm', 'motor_b_measured_rate_rpm', 'motor_c_measured_rate_rpm',
         'motor_d_measured_rate_rpm', 'test_step']
SCOLS = ['state', 'demo_phase', 'fault_phase', 'wheel_status']


def load(path):
    F, S = [], []
    with open(path, newline='') as f:
        rd = csv.reader(f)
        h = next(rd)
        ci = {k: i for i, k in enumerate(h)}
        fi = [ci[c] for c in FCOLS]
        si = [ci[c] for c in SCOLS]
        for r in rd:
            if len(r) < len(h) or not r[0][:1].isdigit():
                continue
            try:
                F.append([float(r[i]) for i in fi])
                S.append([r[i] for i in si])
            except ValueError:
                continue
    F = np.array(F)
    d = {c: F[:, k] for k, c in enumerate(FCOLS)}
    for k, c in enumerate(SCOLS):
        d[c] = np.array([s[k] for s in S])
    d['t'] = (d['mcutime_ms'] - d['mcutime_ms'][0]) / 1000
    return d


def segment(d):
    """Armed stretch containing the fault test (else the longest armed stretch)."""
    armed = (d['state'] == 'A').astype(int)
    e = np.flatnonzero(np.diff(np.r_[0, armed, 0]))
    segs = list(zip(e[::2], e[1::2]))
    for a, b in segs:
        if np.any(np.isin(d['fault_phase'][a:b], ['P', 'X'])):
            return a, b
    return max(segs, key=lambda s: s[1] - s[0])


def first(mask, start=0):
    i = np.flatnonzero(mask[start:])
    return start + i[0] if len(i) else None


def main():
    d = load(LOG)
    a, b = segment(d)
    s = {k: v[a:b] for k, v in d.items()}
    t = s['t'] - s['t'][0]
    yaw0 = s['yaw_deg'][0]
    yrel = np.degrees(np.unwrap(np.radians(s['yaw_deg'] - yaw0)))
    ph, fp = s['demo_phase'], s['fault_phase']

    # Part boundaries: part 1 = DEMO LEVEL (L/K), part 2 = slews until the fault test starts, part 3 = fault test.
    i_settle = first(ph == 'K')
    i_slew = first(np.isin(ph, ['S', 'W', 'H', 'D']))
    i_done = first(ph == 'D')                                  # slews complete, holding the start heading
    i_inject = first(fp == 'F')
    i_detect = first(np.char.find(s['wheel_status'].astype(str), 'D') == 1) if i_inject is not None else None
    i_end = first(np.isin(fp, ['P', 'X']))
    t_settle, t_slew = t[i_settle], t[i_slew]
    t_done = t[i_done] if i_done is not None else t[-1]
    t_inj = t[i_inject] if i_inject is not None else None
    t_end = t[i_end] if i_end is not None else t[-1]

    hold = slice(i_settle, i_slew)
    herr = yrel[hold]
    summary = []
    summary.append(f'log {LOG}, armed rows {a}-{b} ({t[-1]:.0f} s)')
    summary.append(f'part 1 hold: {t_slew - t_settle:.1f} s graded, heading error max {np.max(np.abs(herr)):.3f} deg, '
                   f'rms about mean {np.std(herr):.4f} deg, mean {np.mean(herr):+.4f} deg; laser {LASER["hold"]:+.2f} deg')

    # Per-target results from the slew holds (phase H, test_step = target index)
    step = s['test_step'].astype(int)
    targets = []
    for k in range(4):
        m = (ph == 'H') & (step == k)
        if not np.any(m):
            continue
        ey = np.max(np.abs(s['err_yaw_deg'][m]))
        targets.append((k + 1, ey))
    summary.append('part 2 slews: ' + ', '.join(f'target {k} max |yaw err| {e:.2f} deg (controller)' for k, e in targets))
    lo, hi = min(LASER['after_slews']), max(LASER['after_slews'])
    summary.append(f'part 2 laser: {LASER["hold"]:+.2f} before, {hi:+.2f}..{lo:+.2f} after the 360 deg turn '
                   f'=> {0.5 * (lo + hi) - LASER["hold"]:+.3f} deg')
    if t_inj is not None:
        f = slice(i_inject, i_end)
        summary.append(f'part 3 fault: injected at {t_inj:.1f} s, detected {t[i_detect] - t_inj:.2f} s later'
                       if i_detect is not None else f'part 3 fault: injected at {t_inj:.1f} s')
        summary.append(f'part 3 peak |err| roll {np.max(np.abs(s["err_roll_deg"][f])):.2f} pitch '
                       f'{np.max(np.abs(s["err_pitch_deg"][f])):.2f} yaw {np.max(np.abs(s["err_yaw_deg"][f])):.2f} deg; '
                       f'laser peak {LASER["fault_peak"]:+.2f}, steady {LASER["fault_steady"]:+.2f}')

    def shade(ax):
        ax.axvspan(t_settle, t_slew, color=PART[1], zorder=0)
        ax.axvspan(t_slew, t_done, color=PART[2], zorder=0)
        if t_inj is not None:
            ax.axvspan(t_done, t_end, color=PART[3], zorder=0)

    # Fig 1: overview
    fig, ax = plt.subplots(3, 1, figsize=(10, 7.2), sharex=True, gridspec_kw={'height_ratios': [2.2, 1.2, 1.4]})
    shade(ax[0])
    ax[0].plot(t, yrel, color='#333', lw=1.2, label='heading (estimate)')
    ax[0].plot(t, np.degrees(np.unwrap(np.radians(s['yaw_ref_deg'] - yaw0))) if np.any(s['yaw_ref_deg'] != 0) else yrel,
               color='#d06a00', lw=0.8, ls='--', label='slew reference')
    ax[0].set_ylabel('heading from start (deg)')
    ax[0].legend(loc='center right', fontsize=8)
    for x, txt in [((t_settle + t_slew) / 2, '1  heading hold'), ((t_slew + t_done) / 2, '2  SLEW YAW (360°)'),
                   (((t_done + t_end) / 2) if t_inj else t_done, '3  wheel B fails')]:
        ax[0].text(x, ax[0].get_ylim()[1] * 0.80, txt, ha='center', fontsize=9, fontweight='bold')
    shade(ax[1])
    ax[1].plot(t, s['roll_deg'], color='#2f6fb0', lw=0.9, label='roll')
    ax[1].plot(t, s['pitch_deg'], color='#b8433a', lw=0.9, label='pitch')
    ax[1].set_ylabel('tilt (deg)')
    ax[1].legend(loc='upper left', fontsize=8)
    shade(ax[2])
    for i, w in enumerate('ABCD'):
        ax[2].plot(t, s[f'motor_{w.lower()}_measured_rate_rpm'] * RPM, color=COL[w], lw=0.9, label=f'wheel {w}')
    if t_inj is not None:
        ax[2].axvline(t_inj, color='k', lw=0.8, ls=':')
    ax[2].set_ylabel('wheel speed (rad/s)')
    ax[2].set_xlabel('time since DEMO ALL (s)')
    ax[2].legend(loc='upper left', ncol=4, fontsize=8)
    fig.suptitle('DEMO ALL, 2026-11-01 23:13: all three requirements in one continuous run', fontweight='bold')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fig1_overview.png'))
    plt.close(fig)

    # Fig 2: requirement 1 - heading hold
    fig, ax = plt.subplots(figsize=(8, 3.4))
    th = t[hold] - t_settle
    ax.plot(th, herr, color='#333', lw=1)
    ax.axhspan(-1, 1, color='#e8f0fa', zorder=0)
    ax.set_ylim(-1.2, 1.2)
    ax.set_xlabel('time in graded hold (s)')
    ax.set_ylabel('heading error (deg)')
    ax.set_title(f'Requirement 1: 60 s heading hold, max error {np.max(np.abs(herr)):.2f}°, '
                 f'jitter {np.std(herr):.3f}° rms\n(laser steady at {LASER["hold"]:+.2f}°; shaded band = ±1°)',
                 fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fig2_req1_hold.png'))
    plt.close(fig)

    # Fig 3: requirement 2 - laser before / after the full turn, and per-target controller errors
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.6), gridspec_kw={'width_ratios': [1.2, 1]})
    before, after = LASER['hold'], 0.5 * (lo + hi)
    # Laser error after one full 360 deg turn: tonight's runs before the heading fix (DEMO_0018, laser called
    # out during the runs: CCW 22:35 -0.18 -> +2.1, CW 22:41 0 -> -1.8) and this run after it.
    labels = ['CCW turn\nbefore fix\n(22:35)', 'CW turn\nbefore fix\n(22:41)', 'CCW turn\nafter fix\n(this run)']
    vals = [2.1 - (-0.18), -1.8, after - before]
    cols = ['#b8433a', '#b8433a', '#3a9a5b']
    ax[0].bar(labels, vals, color=cols, width=0.55)
    ax[0].errorbar([2], [vals[2]], yerr=[[after - lo], [hi - after]], color='k', capsize=4)
    ax[0].axhspan(-1, 1, color='#eef7ee', zorder=0)
    ax[0].axhline(0, color='k', lw=0.6)
    for x, v in enumerate(vals):
        ax[0].text(x, v + (0.12 if v >= 0 else -0.25), f'{v:+.2f}°', ha='center', fontsize=9, fontweight='bold')
    ax[0].set_ylim(-2.6, 2.9)
    ax[0].set_ylabel('laser heading error after 360° (deg)')
    ax[0].set_title('Truth (laser at 2.48 m) after a full slew turn; band = ±1°', fontsize=9)
    if targets:
        ax[1].bar([f'{k}\n({["+45", "+135", "−135", "0"][k - 1]}°)' for k, _ in targets], [e for _, e in targets],
                  color='#3a9a5b', width=0.55)
        ax[1].axhline(1.0, color='#b8433a', lw=1, ls='--')
        ax[1].set_ylim(0, 1.1)
        ax[1].set_ylabel('max |yaw error| in 5 s hold (deg)')
        ax[1].set_title('Each target reached and held (limit 1°)', fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fig3_req2_slews.png'))
    plt.close(fig)

    # Fig 4: requirement 3 - wheel B failure
    if t_inj is not None:
        w0, w1 = t_inj - 5, min(t_end + 2, t[-1])
        m = (t >= w0) & (t <= w1)
        fig, ax = plt.subplots(2, 1, figsize=(9, 5.2), sharex=True)
        tt = t[m] - t_inj
        for c, lab, colr in [('err_roll_deg', 'roll', '#2f6fb0'), ('err_pitch_deg', 'pitch', '#b8433a'),
                             ('err_yaw_deg', 'yaw', '#333')]:
            ax[0].plot(tt, s[c][m], color=colr, lw=1, label=lab)
        ax[0].axhspan(-1, 1, color='#fbf0e6', zorder=0)
        pk = max(np.max(np.abs(s[c][m])) for c in ('err_roll_deg', 'err_pitch_deg', 'err_yaw_deg'))
        ax[0].set_ylim(-max(1.3, 1.1 * pk), max(1.3, 1.1 * pk))
        ax[0].set_ylabel('attitude error (deg)')
        ax[0].legend(loc='lower right', fontsize=8)
        for i, w in enumerate('ABCD'):
            ax[1].plot(tt, s[f'motor_{w.lower()}_measured_rate_rpm'][m] * RPM, color=COL[w], lw=1, label=f'wheel {w}')
        ax[1].set_ylabel('wheel speed (rad/s)')
        ax[1].set_xlabel('time since wheel B motor cut (s)')
        ax[1].legend(loc='upper right', ncol=4, fontsize=8)
        for axx in ax:
            axx.axvline(0, color='k', lw=0.8, ls=':')
            if i_detect is not None:
                axx.axvline(t[i_detect] - t_inj, color='#c98a12', lw=0.8, ls='--')
        det = f'{t[i_detect] - t_inj:.2f} s' if i_detect is not None else 'n/a'
        ax[0].set_title(f'Requirement 3: wheel B motor cut (dotted), detected {det} later (dashed), '
                        f'held within ±1° on A, C, D (laser {LASER["fault_peak"]:+.2f}° peak)', fontsize=9)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, 'fig4_req3_fault.png'))
        plt.close(fig)

    with open(os.path.join(OUT, 'summary.txt'), 'w') as f:
        f.write('\n'.join(summary) + '\n')
    print('\n'.join(summary))


if __name__ == '__main__':
    main()
