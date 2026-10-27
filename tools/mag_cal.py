"""Magnetometer wheel-disturbance calibration (fixed table): per-wheel speed sweep + fit.

Spins each listed wheel ALONE through +100/+200/+300/+100/-100/-200/-300/-100 rpm (12 s each),
streaming raw rows over USB (RAW ON), then fits, per wheel and body axis,
    field = const + dirStep * sign(speed) + slope * (|speed| - 10.47 rad/s)
and prints kDirStep / kSpeedSlope rows to paste into src/mag_aid.cpp. ~2 min per wheel.
Table must be FIXED (single wheels push on it). Refuses to start unless the Teensy is Idle;
always sends KILL on exit. pyserial is in PlatformIO's Python:
  ~/.platformio/penv/Scripts/python.exe tools/mag_cal.py COM5 ABC [raw_out.txt]
Method and results: test_runs/MAGCAL_2026-10-27_analysis.html.
"""
import math, serial, sys, threading, time

PORT, WHEELS = sys.argv[1], sys.argv[2].upper()
OUT = sys.argv[3] if len(sys.argv) > 3 else None
RPMS = [100, 200, 300, 100, -100, -200, -300, -100]
STEP_S, SETTLE_S, REF = 12.0, 4.0, 10.47

s = serial.Serial(PORT, 115200, timeout=0.05)
rows, lines, stop = [], [], threading.Event()


def reader():
    buf = b""
    while not stop.is_set():
        buf += s.read(65536)
        while b"\n" in buf:
            ln, buf = buf.split(b"\n", 1)
            l = ln.decode(errors="replace").strip()
            if l.startswith("R,"):
                f = l.split(",")
                if len(f) >= 20:
                    try:
                        rows.append((time.time(), [float(x) for x in f[2:5]], [float(x) for x in f[9:13]]))
                    except ValueError:
                        pass
            else:
                lines.append(l)
                if l.startswith("#"):
                    print("   ", l)


def solve3(X, y):
    """Least squares for 3 unknowns via normal equations (no numpy)."""
    A = [[sum(r[i] * r[j] for r in X) for j in range(3)] for i in range(3)]
    b = [sum(r[i] * v for r, v in zip(X, y)) for i in range(3)]
    for i in range(3):  # Gauss-Jordan
        p = A[i][i]
        A[i] = [v / p for v in A[i]]; b[i] /= p
        for k in range(3):
            if k != i:
                f = A[k][i]
                A[k] = [a - f * c for a, c in zip(A[k], A[i])]; b[k] -= f * b[i]
    return b


threading.Thread(target=reader, daemon=True).start()
try:
    s.write(b"STATUS\n"); time.sleep(0.5)
    st = [l for l in lines if l.startswith("S,")]
    if not st or st[-1].split(",")[1] != "I":
        raise SystemExit("Teensy not Idle - nothing spun")
    s.write(b"RAW ON\n"); time.sleep(0.5)
    results = {}
    for nm in WHEELS:
        i = "ABCD".index(nm)
        print(f"wheel {nm}:")
        steps = []
        for r in RPMS:
            print(f"  SPEED {nm} {r}")
            s.write(f"SPEED {nm} {r}\n".encode())
            t0 = time.time(); time.sleep(STEP_S)
            seg = [(m, w) for (t, m, w) in rows if t0 + SETTLE_S < t < t0 + STEP_S - 0.5]
            if len(seg) < 200:
                raise SystemExit("too few raw rows - is RAW streaming?")
            w = sum(x[1][i] for x in seg) / len(seg)
            m = [sum(x[0][k] for x in seg) / len(seg) for k in range(3)]
            steps.append((w, m))
            print(f"    speed {w:+.1f} rad/s  field {m[0]:.2f} {m[1]:.2f} {m[2]:.2f} uT")
        s.write(f"SPEED {nm} 0\n".encode()); time.sleep(4); s.write(b"DISARM\n"); time.sleep(3)
        X = [[1.0, math.copysign(1.0, w), abs(w) - REF] for w, _ in steps]
        fit = [solve3(X, [m[k] for _, m in steps]) for k in range(3)]
        results[nm] = fit
    print("\nPaste into src/mag_aid.cpp (rows for the wheels you ran):")
    for nm, fit in results.items():
        print(f"  {nm}: kDirStep {{{fit[0][1]:.3f}f, {fit[1][1]:.3f}f, {fit[2][1]:.3f}f}}  "
              f"kSpeedSlope {{{fit[0][2]:.4f}f, {fit[1][2]:.4f}f, {fit[2][2]:.4f}f}}")
finally:
    s.write(b"KILL\nRAW OFF\n"); time.sleep(0.3); stop.set()
    if OUT:
        with open(OUT, "w") as f:
            for t, m, w in rows:
                f.write(f"{t:.3f},{m[0]},{m[1]},{m[2]},{w[0]},{w[1]},{w[2]},{w[3]}\n")
    s.close()
