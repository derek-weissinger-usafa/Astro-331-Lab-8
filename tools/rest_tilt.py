"""Measure the table's rest tilt over the XBee with DEMO LEVEL (no swing fitting).

Usage: python tools/rest_tilt.py [--port COM5] [--hold 5] [--timeout 60] [--log file.txt]

Floating table, Idle, wheels stopped. Sends BIAS 0 (wheels stay at rest, so STOP's brake barely kicks the table),
then DEMO LEVEL: heading is held (which also stops the air bearing's slow yaw spin-up) and roll/pitch are only
damped, so the table settles where gravity puts it. Waits for "settled ... at rest tilt roll R pitch P", holds
--hold s more, sends STOP and prints both reports, plus the mean heading (yaw) over the hold: a torque fixed in
the room (air from the bearing cup, a draft, a cable) makes the rest tilt depend on heading. The wheel-bias setting stays 0 until BIAS 100 or a reboot.
"""
import argparse, re, sys, time
import serial


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='COM5')
    ap.add_argument('--hold', type=float, default=5.0)
    ap.add_argument('--timeout', type=float, default=60.0)
    ap.add_argument('--log')
    a = ap.parse_args()
    s = serial.Serial(a.port, 9600, timeout=0.2)
    s.reset_input_buffer()
    buf = b''
    msgs = []
    rows = []  # (pc time, roll, pitch, yaw) from the ~2 Hz data rows

    def pump(seconds, until=None):
        nonlocal buf
        end = time.time() + seconds
        while time.time() < end:
            buf += s.read(4096)
            while b'\n' in buf:
                ln, buf = buf.split(b'\n', 1)
                l = ln.decode(errors='replace').strip()
                f = l.split(',')
                if len(f) > 40 and f[0][:1].isdigit():
                    try:
                        rows.append((time.time(), float(f[22]), float(f[23]), float(f[24])))
                    except ValueError:
                        pass
                    continue
                if l.startswith('#'):
                    msgs.append(l); print(l, flush=True)
                    if until and re.search(until, l):
                        return l
        return None

    def send(c):
        s.write((c + '\n').encode()); print('>> ' + c, flush=True)

    send('BIAS 0'); pump(1.5)
    send('DEMO LEVEL')
    hit = pump(a.timeout, r'settled after|REFUSED|DISARM')
    if not hit or 'settled' not in hit:
        send('STOP'); pump(3)
        sys.exit(f'did not settle: {hit}')
    t_hold = time.time()
    pump(a.hold)
    t_end = time.time()
    send('STOP')
    pump(4, r'DISARM')
    s.close()
    m = re.search(r'rest tilt roll ([-\d.]+) pitch ([-\d.]+)', hit)
    held = next((x for x in msgs if 'STOPPED' in x), '')
    hold = [r for r in rows if t_hold <= r[0] <= t_end]
    yaw = f', yaw {sum(r[3] for r in hold) / len(hold):+7.1f} deg' if hold else ''
    summary = f'rest tilt: roll {float(m.group(1)):+.2f} deg, pitch {float(m.group(2)):+.2f} deg{yaw}   ({held})'
    print(summary)
    if a.log:
        with open(a.log, 'a', encoding='utf-8') as f:
            f.write(time.strftime('%Y-%m-%d %H:%M:%S') + '  DEMO LEVEL ' + summary + '\n\n')


if __name__ == '__main__':
    main()
