"""Live balance check over the XBee: fit the table's free swing from the ~2 Hz telemetry rows.

Usage: python tools/balance_live.py [--port COM5] [--seconds 60] [--mass kg] [--ixx ..] [--iyy ..] [--trim g]
                                    [--level-offset roll,pitch] [--log file.txt]

Float the table with the wheels stopped (Idle, motors may be off), nudge it so it swings a few degrees, then run
this. It records roll/pitch from the XBee data rows for --seconds, fits a damped sinusoid per axis (same fit as
tools/balance.py) and prints the swing period, the rest tilt, where the centre of mass is relative to the bearing
centre, and the trim moves. Record at least 3 swing periods: as the centre of mass rises the swing slows (about
4 s at 3 mm below the bearing centre, 7 s at 1 mm, 50 s at 0.02 mm), so lengthen --seconds as you go.

If the table leans over instead of swinging back, the centre of mass is ABOVE the bearing centre: take weight off.
2 Hz is plenty for a swing of a few seconds or longer; for the final 0.02 mm use tools/balance.py on an SD log.
"""
import argparse, math, os, sys, time
import numpy as np
import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from balance import fit_damped, G  # noqa: E402

# Data-row columns (data_logger.cpp)
C_T, C_ROLL, C_PITCH, C_STATE = 0, 22, 23, 32
C_RPM = slice(10, 14)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='COM5')
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--mass', type=float, default=5.0)
    ap.add_argument('--ixx', type=float, default=0.0623)
    ap.add_argument('--iyy', type=float, default=0.0623)
    ap.add_argument('--trim', type=float, default=20.0, help='trim weight used for the suggestions, g')
    ap.add_argument('--level-offset', default='0,0', help='IMU roll,pitch (deg) when the deck is truly level')
    ap.add_argument('--log', help='append the result here')
    a = ap.parse_args()
    off = [float(v) for v in a.level_offset.split(',')]

    s = serial.Serial(a.port, 9600, timeout=0.3)
    s.reset_input_buffer()
    T, R, P = [], [], []
    buf, t0 = b'', time.time()
    print(f'recording {a.seconds:.0f} s of free swing from {a.port} ...', flush=True)
    while time.time() - t0 < a.seconds:
        buf += s.read(4096)
        while b'\n' in buf:
            ln, buf = buf.split(b'\n', 1)
            f = ln.decode(errors='replace').strip().split(',')
            if len(f) < 40 or not f[0][:1].isdigit():
                continue
            try:
                if f[C_STATE][:1] != 'I' or max(abs(float(v)) for v in f[C_RPM]) > 5:
                    continue  # only Idle with the wheels stopped is a free swing
                T.append(float(f[C_T]) * 1e-3); R.append(float(f[C_ROLL])); P.append(float(f[C_PITCH]))
            except ValueError:
                continue
    s.close()
    T, R, P = np.array(T), np.array(R), np.array(P)
    if len(T) < 20:
        sys.exit(f'only {len(T)} usable rows (Idle, wheels stopped): is the table powered and Idle?')
    fr, fp = fit_damped(T, R), fit_damped(T, P)
    span = T[-1] - T[0]
    lines = []
    for name, f in (('roll', fr), ('pitch', fp)):
        per = 2 * math.pi / f[0]
        warn = '' if span >= 3 * per else f'  (only {span / per:.1f} periods recorded: use --seconds {math.ceil(3.5 * per)})'
        lines.append(f'{name}: period {per:5.2f} s, swing {f[3]:.2f} deg, rest {f[2]:+.2f} deg, fit rms {f[4]:.2f}{warn}')
    Lx = a.ixx * fr[0] ** 2 / (a.mass * G)
    Ly = a.iyy * fp[0] ** 2 / (a.mass * G)
    L = 0.5 * (Lx + Ly)
    roll0, pitch0 = fr[2] - off[0], fp[2] - off[1]
    ph, th = math.radians(roll0), math.radians(pitch0)
    up = np.array([-math.sin(th), math.sin(ph) * math.cos(th), math.cos(ph) * math.cos(th)])
    r = -L * up * 1000.0
    gmm = a.mass * 1000.0
    lines.append(f'centre of mass {L * 1000:.2f} mm below the bearing centre ({Lx * 1000:.2f} roll / {Ly * 1000:.2f} pitch), '
                 f'off-centre x {r[0]:+.3f} mm, y {r[1]:+.3f} mm')
    for ax, v, toward in (('x', -r[0], ('+x (toward wheel A)', '-x (toward wheel C)')),
                          ('y', -r[1], ('+y (toward wheel B)', '-y (toward wheel D)'))):
        mom = gmm * abs(v)
        lines.append(f'  level {ax}: {mom:6.0f} g mm = {a.trim:.0f} g by {mom / a.trim:5.1f} mm toward {toward[0] if v > 0 else toward[1]}')
    lines.append(f'  raise: {gmm * L * 1000:6.0f} g mm to reach the bearing centre (a mass dm added h above it raises the CG by ~dm h / m)')
    if fr[4] > 0.4 or fp[4] > 0.4:
        lines.append('  WARNING: poor fit (> 0.4 deg rms): the table was probably touched or turning; repeat the swing.')
    out = '\n'.join(lines)
    print(out)
    if a.log:
        with open(a.log, 'a', encoding='utf-8') as f:
            f.write(time.strftime('%Y-%m-%d %H:%M:%S') + f'  ({len(T)} rows, {span:.0f} s)\n' + out + '\n\n')


if __name__ == '__main__':
    main()
