"""Smooth SPEED ALL sweep on the Teensy; records commands and USB telemetry.

Profile: 0 rpm for 2 s, cosine ramp to PEAK over 10 s, hold 3 s, ramp down over 10 s, 0 for 3 s.
Refuses to start unless the Teensy reports Idle; always sends KILL on exit.
Usage (pyserial is in PlatformIO's Python):
  ~/.platformio/penv/Scripts/python.exe tools/speed_sweep.py test_runs/<name>
Writes <name>_commands.csv and <name>_telemetry.csv (raw USB lines with host timestamps).
"""
import csv, math, serial, sys, threading, time

PORT, PEAK = "COM3", 300.0
OUT = sys.argv[1]
T_PRE, T_UP, T_HOLD, T_DOWN, T_POST = 2.0, 10.0, 3.0, 10.0, 3.0

def profile(t):
    if t < T_PRE: return 0.0
    t -= T_PRE
    if t < T_UP: return PEAK * (1 - math.cos(math.pi * t / T_UP)) / 2
    t -= T_UP
    if t < T_HOLD: return PEAK
    t -= T_HOLD
    if t < T_DOWN: return PEAK * (1 + math.cos(math.pi * t / T_DOWN)) / 2
    return 0.0

s = serial.Serial(PORT, 115200, timeout=0.05)
lines, stop = [], threading.Event()
def reader():
    buf = b""
    while not stop.is_set():
        buf += s.read(4096)
        while b"\n" in buf:
            ln, buf = buf.split(b"\n", 1)
            lines.append((time.time(), ln.decode(errors="replace").strip()))
th = threading.Thread(target=reader, daemon=True); th.start()

cmds, aborted = [], None
try:
    s.write(b"STATUS\n"); time.sleep(0.5)
    st = [l for _, l in lines if l.startswith("S,")]
    print("status:", st[-1] if st else "(no reply)")
    if not st or st[-1].split(",")[1] != "I":
        raise SystemExit("Teensy not in Idle - aborting before spinning anything")
    t0 = time.time(); seen = len(lines)
    total = T_PRE + T_UP + T_HOLD + T_DOWN + T_POST
    while (t := time.time() - t0) < total:
        rpm = profile(t)
        s.write(f"SPEED ALL {rpm:.0f}\n".encode())
        cmds.append((time.time(), rpm))
        bad = [l for _, l in lines[seen:] if l.startswith("# DISARM") or "REFUSED" in l]
        if bad: aborted = bad[0]; break
        time.sleep(0.1)
finally:
    for _ in range(3):
        s.write(b"KILL\n"); time.sleep(0.05)
    time.sleep(0.5); stop.set(); th.join(1); s.close()

with open(OUT + "_telemetry.csv", "w", newline="") as f:
    w = csv.writer(f); w.writerow(["host_time_s", "line"])
    w.writerows(lines)
with open(OUT + "_commands.csv", "w", newline="") as f:
    w = csv.writer(f); w.writerow(["host_time_s", "cmd_rpm"])
    w.writerows(cmds)
print(f"commands sent: {len(cmds)}, lines received: {len(lines)}")
print("ABORTED:", aborted if aborted else "no")
for _, l in lines:
    if l.startswith("#") and "MANUAL" not in l: print(l)
