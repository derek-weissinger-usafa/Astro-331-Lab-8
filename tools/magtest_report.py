"""Write-up page for a MAG TEST run (fixed-table validation of the magnetometer yaw aid).

Usage: python tools/magtest_report.py <MAGTEST_####.CSV> [xbee_runner_log.txt] [-o out.html]

Re-grades every step from the SD log on its own (same rules as the firmware, mag_test.h: the
table is fixed, so |yaw| <= 1 deg; KICK steps back within 1 deg in <= 90 s and never out again),
and, if tools/replay/replay(.exe) is built (tools/replay/build.sh), replays the log through the
firmware's estimator and aid three ways: firmware settings (checks the replay), gyro only (what
the aid prevented) and predicted references (MagAid::setPredictAlways, how good a predicted
reference is). Writes a self-contained HTML page (figures embedded) next to the log.
"""
import base64, csv, io, math, os, re, subprocess, sys, tempfile
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

TOL, RECOVER = 1.0, 90.0
HERE = os.path.dirname(os.path.abspath(__file__))
REPLAY = os.path.join(HERE, 'replay', 'replay.exe' if os.name == 'nt' else 'replay')
INK, GREY, BLUE, RED, GREEN, GOLD = '#1f2933', '#7b8794', '#2f6fb0', '#c0392b', '#2e8b57', '#c98a12'


def load(path):
    with open(path, newline='') as f:
        rd = csv.reader(f)
        hdr = next(rd)
        ci = {h: i for i, h in enumerate(hdr)}
        cols = {k: [] for k in ('t', 'yaw', 'step', 'mode', 'resid', 'state', 'ws', 'rpm')}
        for r in rd:
            if not r or not r[0][:1].isdigit() or len(r) < len(hdr) - 4:
                continue
            cols['t'].append(float(r[ci['mcutime_ms']]) * 1e-3)
            cols['yaw'].append(float(r[ci['yaw_deg']]))
            cols['step'].append(int(r[ci['test_step']]))
            cols['mode'].append(r[ci['mag_mode']][:1])
            cols['resid'].append(float(r[ci['mag_resid_deg']]))
            cols['state'].append(r[ci['state']][:1])
            cols['ws'].append(r[ci['wheel_status']] if 'wheel_status' in ci else '....')
            cols['rpm'].append([float(r[ci[f'motor_{w}_measured_rate_rpm']]) for w in 'abcd'])
    d = {k: np.array(v) for k, v in cols.items()}
    d['t_abs0'] = float(d['t'][0])
    d['t'] = d['t'] - d['t'][0]
    return d


def wrap(x):
    return (x + 180.0) % 360.0 - 180.0


def grade(d):
    steps = []
    yaw, t = d['yaw'], d['t']
    jumps = np.where(np.abs(wrap(np.diff(yaw))) > 3.0)[0] + 1
    for s in sorted(set(d['step'][d['step'] >= 0])):
        k = np.where(d['step'] == s)[0]
        tt, yy = t[k], yaw[k]
        a = np.abs(yy)
        kick_idx = [j for j in jumps if k[0] <= j <= k[-1]]
        kick = float(wrap(yaw[kick_idx[0]] - yaw[kick_idx[0] - 1])) if kick_idx else 0.0
        med = np.median(d['rpm'][k], axis=0)
        pat = ''.join('+' if m > 20 else '-' if m < -20 else '0' for m in med)
        inuse = np.isin(d['mode'][k], ['s', 'S'])
        res = d['resid'][k][inuse]
        e = dict(step=int(s) + 1, t0=float(tt[0]), t1=float(tt[-1]), pat=pat, rpm=[int(round(m)) for m in med],
                 kick=kick, end=float(yy[-1]), peak=float(a.max()), aid=float(inuse.mean()),
                 resid=float(np.sqrt((res ** 2).mean())) if res.size else float('nan'),
                 ws=''.join(sorted(set(d['ws'][k]))))
        if kick:
            rel = tt - tt[0]
            out = rel[a > TOL]
            e['recover'] = float(out.max()) if out.size else 0.0
            late = a[rel > RECOVER]
            e['max'] = float(late.max()) if late.size else 0.0
            e['pass'] = e['recover'] <= RECOVER
        else:
            e['max'] = e['peak']
            e['pass'] = e['peak'] <= TOL
        steps.append(e)
    return steps


def replay(log, *opts):
    if not os.path.exists(REPLAY):
        return None, 'replay tool not built (sh tools/replay/build.sh)'
    out = tempfile.mktemp(suffix='.csv')
    p = subprocess.run([REPLAY, log, out, *opts], capture_output=True, text=True)
    if p.returncode != 0 or not os.path.exists(out):
        return None, p.stderr.strip() or p.stdout.strip()
    with open(out, newline='') as f:
        r = list(csv.DictReader(f))
    os.remove(out)
    rep = dict(t=np.array([float(x['t_s']) for x in r]), yaw=np.array([float(x['yaw_replay_deg']) for x in r]),
               ylog=np.array([float(x['yaw_log_deg']) for x in r]), mode=np.array([x['mode_replay'] for x in r]),
               step=np.array([int(x['test_step']) for x in r]))
    return rep, p.stdout.strip()


def fig_png(fig):
    b = io.BytesIO()
    fig.savefig(b, format='png', dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    return base64.b64encode(b.getvalue()).decode()


def main():
    args = [a for a in sys.argv[1:] if a != '-o']
    outp = sys.argv[sys.argv.index('-o') + 1] if '-o' in sys.argv else None
    if outp in args:
        args.remove(outp)
    log = args[0]
    xlog = args[1] if len(args) > 1 else None
    outp = outp or os.path.splitext(log)[0] + '_report.html'
    name = os.path.basename(log)

    d = load(log)
    steps = grade(d)
    t0 = d['t'][d['step'] >= 0][0] if (d['step'] >= 0).any() else 0.0
    d['t'] = d['t'] - t0
    for e in steps:
        e['t0'] -= t0
        e['t1'] -= t0
    npass = sum(e['pass'] for e in steps)

    # drift run = steps after the main schedule (2-min steps after the last KICK step block)
    main_n = 22  # mag_test::kNumMain
    drift = [e for e in steps if e['step'] > main_n]
    trend = dmax = None
    if drift:
        k = (d['step'] >= main_n)
        tt, yy = d['t'][k], d['yaw'][k]
        trend = float(np.polyfit(tt, yy, 1)[0] * 3600.0)
        dmax = float(np.abs(yy).max())

    # Two ways to set a new wheel pattern's reference (mag_aid.h): replay both, take the one that
    # reproduces the log as "as run" (the replay check) and show the other as the alternative.
    rep_l, msg_l = replay(log, '--learn')
    rep_p, msg_p = replay(log, '--predict')
    rep_gyro, msg_gyro = replay(log, '--nomag', '--nokick')
    MODES = {'learn': 'new patterns learned in place', 'predict': 'new patterns predicted from a known one'}
    def _rms(r):
        return float(np.sqrt(np.mean(wrap(r['yaw'] - r['ylog']) ** 2))) if r is not None else 1e9
    if _rms(rep_l) <= _rms(rep_p):
        asrun, other, rep_pred, msg_pred, rep_fw, msg_fw = 'learn', 'predict', rep_l, msg_l, rep_p, msg_p
    else:
        asrun, other, rep_pred, msg_pred, rep_fw, msg_fw = 'predict', 'learn', rep_p, msg_p, rep_l, msg_l
    for r in (rep_fw, rep_gyro, rep_pred):  # replay times are the log's own clock
        if r is not None:
            keep = r['step'] >= 0
            for key in ('t', 'yaw', 'ylog', 'mode', 'step'):
                r[key] = r[key][keep]
            r['t'] = r['t'] - d['t_abs0'] - t0

    # ---- figure 1: yaw over the run
    figs = []
    fig, ax = plt.subplots(figsize=(11, 4.4))
    for e in steps:
        ax.axvspan(e['t0'] / 60, e['t1'] / 60, color=('#e8f3ec' if e['pass'] else '#f8e1de'), lw=0, zorder=0)
    ax.axhspan(-TOL, TOL, color=GREEN, alpha=0.08, lw=0)
    ax.axhline(TOL, color=GREEN, lw=0.6, ls='--'); ax.axhline(-TOL, color=GREEN, lw=0.6, ls='--')
    k = d['step'] >= 0
    if rep_gyro is not None:
        tg = rep_gyro['t']
        ax.plot(tg / 60, np.clip(rep_gyro['yaw'], -2.45, 2.45), color=GREY, lw=0.9, label='replayed, gyro only (aid off)')
    ax.plot(d['t'][k] / 60, np.clip(d['yaw'][k], -2.45, 2.45), color=INK, lw=1.0, label='EKF yaw (logged, aid on)')
    for e in steps:
        if e['kick']:
            y0 = 2.3 if e['kick'] > 0 else -2.3
            ax.annotate(f"kick {e['kick']:+.0f}° (off scale)\nback within 1° in {e['recover']:.0f} s", xy=(e['t0'] / 60, y0),
                        xytext=(e['t0'] / 60 + (-0.8 if e['kick'] > 0 else 0.8), y0 * 0.72), fontsize=8.5, color=RED,
                        ha='right' if e['kick'] > 0 else 'left', arrowprops=dict(arrowstyle='->', color=RED, lw=0.9))
    ax.set_ylim(-2.5, 2.5)
    ax.set(xlabel='Time since MAG TEST start (min)', ylabel='Yaw (deg), clipped ±2.5  [true yaw = 0]')
    ax.legend(loc='upper right', fontsize=8.5)
    figs.append(('Yaw through the test', fig_png(fig),
                 'Green bands: steps that passed; red: failed. The table is clamped, so every degree shown is '
                 'estimation error. Grey: the same run replayed with the aid off.'))

    # ---- figure 2: aid in use and residual per step
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(11, 4.6), sharex=True)
    x = [e['step'] for e in steps]
    a1.bar(x, [100 * e['aid'] for e in steps], color=[BLUE if e['pat'] != '0000' else INK for e in steps])
    a1.set_ylabel('Aid in use (%)'); a1.set_ylim(0, 100)
    a2.bar(x, [e['resid'] for e in steps], color=[BLUE if e['pat'] != '0000' else INK for e in steps])
    a2.set(ylabel='Field residual\nrms while in use (deg)', xlabel='Step (black = wheels stopped)')
    figs.append(('How much the aid worked, per step', fig_png(fig),
                 'Top: share of samples with a heading update applied (mode s/S). Bottom: field heading minus '
                 'its reference while in use.'))

    # ---- figure 3: replay checks
    # Per-step grade of the alternative-mode replay (same rules), for the table.
    fw_max = {}
    if rep_fw is not None:
        for e in steps:
            k = rep_fw['step'] == e['step'] - 1
            if not k.any():
                continue
            tt = rep_fw['t'][k] - rep_fw['t'][k][0]
            a = np.abs(rep_fw['yaw'][k])
            fw_max[e['step']] = float(a[tt > RECOVER].max()) if e['kick'] and (tt > RECOVER).any() else float(a.max())
    if rep_pred is not None and rep_fw is not None:
        fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 3.6), gridspec_kw=dict(width_ratios=[1, 2]))
        a1.plot(rep_pred['t'] / 60, wrap(rep_pred['yaw'] - rep_pred['ylog']), color=INK, lw=0.8)
        a1.set(title=f'Replay check ({asrun}):\nreplay minus logged yaw', xlabel='min', ylabel='deg')
        a2.axhspan(-TOL, TOL, color=GREEN, alpha=0.08, lw=0)
        a2.plot(rep_fw['t'] / 60, np.clip(rep_fw['ylog'], -2.5, 2.5), color=INK, lw=0.8, label=f'logged on the table ({MODES[asrun]})')
        a2.plot(rep_fw['t'] / 60, np.clip(rep_fw['yaw'], -2.5, 2.5), color=GOLD, lw=0.9, label=f'replayed, {MODES[other]}')
        a2.set(title='Same run, the other way', xlabel='min', ylabel='yaw (deg), clipped ±2.5')
        a2.legend(fontsize=8, loc='lower left')
        figs.append(('Offline replay', fig_png(fig),
                     f'Left: the replay reproduces what ran on the table ({MODES[asrun]}), so it can be trusted to test '
                     f'changes. Right: the same log with {MODES[other]} (mag_aid.h, kMagPredictNewPatterns).'))

    # ---- table
    rows = []
    for e in steps:
        kick = f"{e['kick']:+.0f}° / back in {e['recover']:.0f} s" if e['kick'] else ''
        rows.append(f"<tr class='{'p' if e['pass'] else 'f'}'><td>{e['step']}</td><td><code>{e['pat']}</code></td>"
                    f"<td>{' '.join(str(v) for v in e['rpm'])}</td><td>{(e['t1'] - e['t0']):.0f}</td><td>{kick}</td>"
                    f"<td>{e['max']:.2f}</td><td>{e['end']:+.2f}</td><td>{100 * e['aid']:.0f}%</td>"
                    f"<td>{e['resid']:.2f}</td><td><code>{e['ws']}</code></td><td>{'PASS' if e['pass'] else 'FAIL'}</td>"
                    f"<td>{fw_max[e['step']]:.2f}</td></tr>" if e['step'] in fw_max else
                    f"<td>{e['resid']:.2f}</td><td><code>{e['ws']}</code></td><td>{'PASS' if e['pass'] else 'FAIL'}</td><td></td></tr>")
    onboard = ''
    if xlog and os.path.exists(xlog):
        lines = [l.rstrip() for l in open(xlog, encoding='utf-8', errors='replace') if 'MAGTEST' in l or 'WHEEL FAULT' in l]
        onboard = '<h2>On-board messages (XBee)</h2><pre>' + '\n'.join(
            re.sub(r'^\S+\s+\S+\s+', '', l).replace('<', '&lt;') for l in lines) + '</pre>'

    def stat(label, val):
        return f"<div class='stat'><div class='v'>{val}</div><div class='l'>{label}</div></div>"
    stats = stat('steps passed', f'{npass}/{len(steps)}')
    stats += stat('max |yaw| outside kicks', f"{max([e['max'] for e in steps] or [0]):.2f}°")
    if fw_max:
        nfw = sum(v <= TOL for k_, v in fw_max.items())
        stats += stat(f'replayed, {MODES[other]}', f'{nfw}/{len(fw_max)} · worst {max(fw_max.values()):.2f}°')
    if trend is not None:
        stats += stat('drift run trend', f'{trend:+.2f}°/h')
        stats += stat('drift run max |yaw|', f'{dmax:.2f}°')
    stats += stat('aid in use (all)', f"{100 * np.isin(d['mode'][d['step'] >= 0], ['s', 'S']).mean():.0f}%")

    notes = '<ul>' + ''.join(f'<li><b>{lab}:</b> {m.replace(chr(10), "; ")}</li>' for lab, m in
                             ((f'Replay as run ({MODES[asrun]})', msg_pred), ('Replay, gyro only', msg_gyro),
                              (f'Replay, {MODES[other]}', msg_fw))) + '</ul>'
    html = f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>MAG TEST {name}</title>
<style>
:root{{--ink:#1f2933;--muted:#616e7c;--bg:#fbfaf7;--card:#fff;--line:#e4e2dc;--pass:#e8f3ec;--fail:#f8e1de}}
body{{margin:0;background:var(--bg);color:var(--ink);font:15px/1.5 system-ui,-apple-system,Segoe UI,sans-serif}}
main{{max-width:1100px;margin:0 auto;padding:24px 16px}} h1{{font-size:24px;margin:0 0 4px}} h2{{font-size:18px;margin:28px 0 8px}}
.sub{{color:var(--muted)}} .stats{{display:flex;flex-wrap:wrap;gap:10px;margin:16px 0}}
.stat{{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:10px 14px;min-width:140px}}
.stat .v{{font-size:22px;font-weight:600}} .stat .l{{color:var(--muted);font-size:13px}}
figure{{margin:0 0 18px;background:#fff;border:1px solid var(--line);border-radius:8px;padding:10px}}
figure img{{width:100%;height:auto}} figcaption{{color:var(--muted);font-size:13px}}
.tw{{overflow-x:auto}} table{{border-collapse:collapse;font-size:13px;width:100%;background:#fff}}
td,th{{border:1px solid var(--line);padding:4px 6px;text-align:right;white-space:nowrap}} th{{background:#f2f0ea}}
tr.p td:last-child{{background:var(--pass)}} tr.f td{{background:var(--fail)}}
pre{{background:#fff;border:1px solid var(--line);padding:10px;font-size:12px;overflow-x:auto}}
</style></head><body><main>
<h1>MAG TEST — magnetometer yaw aid on the fixed table</h1>
<div class="sub">{name} · {len(steps)} steps · {d['t'][-1] / 60:.0f} min · pass rule: |yaw| ≤ {TOL:.0f}° (true yaw is 0);
KICK steps back within {TOL:.0f}° in ≤ {RECOVER:.0f} s</div>
<div class="stats">{stats}</div>
{''.join(f'<h2>{h}</h2><figure><img src="data:image/png;base64,{b}" alt="{h}"><figcaption>{c}</figcaption></figure>' for h, b, c in figs)}
<h2>Per step</h2><div class="tw"><table><tr><th>#</th><th>pattern A B C D</th><th>rpm</th><th>s</th><th>kick</th>
<th>max |yaw|</th><th>end yaw</th><th>aid in use</th><th>resid rms</th><th>wheels</th><th>result</th><th>max |yaw|, other mode (replay)</th></tr>
{''.join(rows)}</table></div>
<p class="sub">KICK steps: "max |yaw|" counts only after the {RECOVER:.0f} s recovery allowance. Re-graded here from the SD
log; the on-board grades (below, if the XBee log was given) use the same rules.</p>
<h2>Replay</h2>{notes}
{onboard}
</main></body></html>"""
    open(outp, 'w', encoding='utf-8').write(html)
    print(f'wrote {outp}: {npass}/{len(steps)} steps pass' + (f', drift trend {trend:+.2f} deg/h' if trend is not None else ''))


if __name__ == '__main__':
    main()
