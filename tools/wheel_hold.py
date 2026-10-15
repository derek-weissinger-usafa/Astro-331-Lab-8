"""Spin one wheel (or ALL) at a fixed SPEED, print speed and duty every 0.5 s, then KILL.

Refuses to start unless the Teensy reports Idle; always sends KILL on exit.
Usage (pyserial is in PlatformIO's Python):
  ~/.platformio/penv/Scripts/python.exe tools/wheel_hold.py <A|B|C|D|ALL> <rpm> <seconds> <out.csv>
"""
import csv, math, serial, sys, threading, time

WHEEL, RPM, DUR, OUT = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
COL = {"A": 15, "B": 16, "C": 17, "D": 18, "ALL": 17}[WHEEL]  # ALL: highlight C
K = 60 / (2 * math.pi)

s = serial.Serial("COM3", 115200, timeout=0.05)
lines, stop = [], threading.Event()
def reader():
    buf = b""
    while not stop.is_set():
        buf += s.read(4096)
        while b"\n" in buf:
            ln, buf = buf.split(b"\n", 1)
            lines.append((time.time(), ln.decode(errors="replace").strip()))
threading.Thread(target=reader, daemon=True).start()

try:
    s.write(b"STATUS\n"); time.sleep(0.5)
    st = [l for _, l in lines if l.startswith("S,")]
    print("status:", st[-1] if st else "(no reply)")
    if not st or st[-1].split(",")[1] != "I":
        raise SystemExit("Teensy not in Idle - not spinning anything")
    t0 = time.time(); seen = len(lines)
    s.write(f"SPEED {WHEEL} {RPM:.0f}\n".encode())
    nxt = 0.0
    print(f"  t(s)   {WHEEL} rpm    (A, B, C, D rpm)   state")
    while (t := time.time() - t0) < DUR:
        new = lines[seen:]
        if any(l.startswith("# DISARM") or "REFUSED" in l for _, l in new):
            print("STOPPED BY FIRMWARE:", [l for _, l in new if l.startswith("#")]); break
        if t >= nxt:
            tel = [l.split(",") for _, l in reversed(lines) if l[:1].isdigit() and l.count(",") >= 19]
            if tel:
                f = tel[0]; w = [float(x) * K for x in f[15:19]]
                d = " duty (" + ",".join(f"{float(x):5.2f}" for x in f[20:24]) + ")" if len(f) >= 24 else ""
                print(f"  {t:4.1f}  {float(f[COL]) * K:7.1f}    ({w[0]:5.0f},{w[1]:5.0f},{w[2]:5.0f},{w[3]:5.0f})   {f[19]}{d}")
            nxt += 0.5
        time.sleep(0.02)
finally:
    for _ in range(3):
        s.write(b"KILL\n"); time.sleep(0.05)
    time.sleep(0.4); stop.set(); s.close()
    with open(OUT, "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(["host_time_s", "line"]); w.writerows(lines)
    print("killed; raw log saved")
