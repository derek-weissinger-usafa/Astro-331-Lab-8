"""Final presentation v2 (2026-11-02): everything since the prelim, ending in the DEMO ALL run.
Starts from the deck as committed (Reports/...Final Pres.pptx, built 10-31 by build_deck.py) and reuses its helpers."""
import copy, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = r'C:\Users\C27Derek.Weissinger\PlatformIO\Projects\Astro 331 Lab 8'
DECK = os.path.join(ROOT, 'Reports', 'Astro 331 Lab 8_2_3 Body Final Pres.pptx')
SRC = os.path.join(HERE, 'final_v1.pptx')  # copy of the committed deck (made below if missing)
OUT = sys.argv[1] if len(sys.argv) > 1 else DECK
if not os.path.exists(SRC):
    import shutil
    shutil.copy(DECK, SRC)

# Helpers from build_deck.py (everything before its first slide edit), pointed at the v1 deck.
helpers = open(os.path.join(HERE, 'build_deck.py'), encoding='utf-8').read().split('# ---------------------------------------------------------------- 3 NEW')[0]
helpers = re.sub(r"SRC = .*\n", "", helpers, count=1)
helpers = re.sub(r"OUT = .*\n", "", helpers, count=1)
helpers = helpers.replace("NUM_XML = next(sh._element for sh in orig[2].shapes if sh.is_placeholder and sh.placeholder_format.idx == 12)",
                          "NUM_XML = next(sh._element for s in orig for sh in s.shapes if sh.is_placeholder and sh.placeholder_format.idx == 12)")
exec(helpers)

WU = os.path.join(FIG, 'writeup_2026-10-31')
DA = os.path.join(FIG, 'demo_all_2026-11-01')
NF = os.path.join(FIG, 'final_2026-11-02')
OK = ('✔', dict(b=True, color=GREEN, size=17))
PART = ('◐', dict(b=True, color=AMBER, size=17))


def para_set(shape, start, new, keep_label=True):
    """Paragraph whose text starts with `start`: keep its first run (the bold label) and replace the rest with `new`
    (or, keep_label=False, replace the whole paragraph text)."""
    for p in shape.text_frame.paragraphs:
        if p.text.startswith(start):
            rs = p.runs
            if keep_label and len(rs) > 1:
                rs[1].text = new
                for r in rs[2:]:
                    r._r.getparent().remove(r._r)
            else:
                rs[0].text = new
                for r in rs[1:]:
                    r._r.getparent().remove(r._r)
            return
    raise KeyError(start)


def rewrite(shape, items, size=14):
    """Replace the paragraphs' text; item k reuses the formatting of original paragraph k (or the last one)."""
    tf = shape.text_frame
    tmpl = [copy.deepcopy(p._p) for p in tf.paragraphs if p.runs]
    for p in list(tf.paragraphs):
        p._p.getparent().remove(p._p)
    txBody = tf._txBody
    for k, it in enumerate(items):
        p = copy.deepcopy(tmpl[min(k, len(tmpl) - 1)])
        rs = p.findall(qn('a:r'))
        for r in rs[1:]:
            p.remove(r)
        rs[0].find(qn('a:t')).text = it
        rPr = rs[0].find(qn('a:rPr'))
        if rPr is not None:
            rPr.set('sz', str(int(size * 100)))
        txBody.append(p)


def fig_slide(title, path, msg, m, c, e, n, note, pic_h=3.35, size=14):
    s = new_slide(title, note)
    takeaway(s, msg)
    pic = picture(s, path, 0.3, PIC_Y, w=9.4, h=pic_h, center_w=9.4)
    y = PIC_Y + pic.height / 914400 + 0.08
    mcen(s, 0.3, y, 9.4, 6.95 - y, m, c, e, n, size=size)
    return s


# ================================================================ edits to existing slides
S = {k: orig[k - 1] for k in range(1, len(orig) + 1)}  # v1 slide numbers

# ---- 4: requirements, all verified on hardware
req = S[4]
remove(shape_named(req, 'Table 91'))
rows = [
    ['Requirement', 'Pass criterion', 'Status (DEMO ALL, 11-01, laser truth)'],
    [[('(1) Hold a heading, no visible jitter or drift', dict(b=True, size=15))],
     'Free table holds the commanded heading 1 min, |error| < 1°',
     [OK, ('  60 s hold: max 0.04°, jitter 0.01° rms; laser steady. 5 min hold within 0.25° (laser)', {})]],
    [[('(2) Know attitude to ±1° in all 3 axes', dict(b=True, size=15))],
     'Estimate vs truth over the run, error < 1°',
     [OK, ('  Laser after 360° of slews: 0.05° off. Roll/pitch σ 0.02°', {})]],
    [[('(3) Detect a wheel failure and compensate', dict(b=True, size=15))],
     'Kill a motor mid-maneuver: flagged < 1 s, then (1) and (2) still met',
     [OK, ('  Wheel B cut mid-hold: detected 0.50 s, back within ±1° in 0.5 s, held ≤ 0.36° on 3 wheels', {})]],
    [[('Stretch: 4-attitude optimal slew', dict(b=True, size=15))],
     'Each attitude reached on the shortest path and held 5 s within ±1°',
     [OK, ('  SLEW YAW 4/4 PASS (yaw targets at the rest tilt). ', {}), PART, (' Tilted targets: needs CG ≲ 1 mm', {})]],
]
table(req, 0.3, 1.05, 9.4, [2.9, 2.9, 3.6], rows, header_size=15, body_size=14, row_h=1.1, header_h=0.45)
notes(req, 'All three requirements are now verified on the floating table, in one run, against an independent truth: a laser '
           'on the deck read on a wall 2.48 m away. The stretch goal is met for yaw targets at the table\'s rest tilt; tilted '
           'targets need the centre of mass within about a millimetre of the bearing, which we could not hold reliably.')

# ---- 5: control loop: estimator text
loop = S[5]
para_set(shape_named(loop, 'TextBox 108'), 'Estimator:',
         '6-state MEKF. Gyro predicts at 100 Hz; accelerometer corrects roll/pitch only (its gain is kept off the vertical); '
         'heading from the gyro, checked against a laser. Magnetometer aid built, off at the demo site.')
notes(loop, 'Input is the commanded attitude and rate from whichever mode is running; DEMO ALL chains three of them. The '
            'estimator feeds the controller, the controller asks the wheels for torque, and the table\'s attitude is the output. '
            'Gravity from the centre-of-mass offset and air drag are the disturbances. Since the prelim we changed the estimator '
            'in one important way: the accelerometer may only correct roll and pitch (slide 9).')

# ---- 6: control law: measured numbers
law = S[6]
tb = shape_named(law, 'TextBox 92')
para_set(tb, 'Wheel J', 'Wheel J = 3.06×10⁻⁴ kg·m² (CAD); table I measured: I_zz = 0.077 (STEP TEST), I_xx, I_yy ≈ 0.15 kg·m²',
         keep_label=False)
para_set(tb, 'Speed limit', 'Speed limit 80 rad/s (was 60) → ≈ 32 mN·m·s per tilt axis', keep_label=False)

# ---- 8: magnetometer code: the prediction bullet is no longer true
mag = S[7]
for p in shape_named(mag, 'TextBox 94').text_frame.paragraphs:
    if p.text.startswith('Prediction:'):
        p.runs[0].text = 'References: '
para_set(shape_named(mag, 'TextBox 94'), 'References:',
         'learning in place again (predicting new patterns failed MAG TEST 2); the aid is off at the demo site (slide 26)')
notes(mag, 'This is the magnetometer update we added after the prelim. The key design choice is the gain edit: the filter may '
           'only rotate the estimate about vertical, so a bad magnetic sample can never tip roll or pitch. The aid decides when '
           'the field can be trusted at all. It worked on the clamped table, but the presentation spot bends the field, so the '
           'demo runs with the aid off and the heading from the gyro.')

# ---- 9 (v1): wheel-failure code: table numbers
fc_code = S[9]
shape_named(fc_code, 'TextBox 96').text_frame.paragraphs[0].runs[0].text = '0.49–0.91 s'
shape_named(fc_code, 'TextBox 97').text_frame.paragraphs[0].runs[0].text = 'to detect on the air bearing (4 table tests)'
shape_named(fc_code, 'TextBox 100').text_frame.paragraphs[0].runs[0].text = 'false alarms in 4 healthy simulated runs'

# ---- data slide NEXT lines that are out of date
para_set(shape_named(S[17], 'MCEN'), 'NEXT:',
         'Done: 10 min warm-up + 180 s calibration is now the standard; laser holds confirm it (slide 42).')
para_set(shape_named(S[25], 'MCEN'), 'NEXT:',
         'Done: DEMO LEVEL holds the natural rest tilt; balance sessions on 10-31 and 11-01 (slide 33).')
para_set(shape_named(S[26], 'MCEN'), 'NEXT:',
         'Done: the spare runs everything. Its VIN–VUSB pads are still joined: upload on USB with the batteries off, run on the batteries with USB out.')

# ---- 27: where we stand
ws = S[27]
ws.shapes.title.text = 'Where We Stand'
rewrite(shape_named(ws, 'TextBox 129'), ['Req. 1: heading hold (laser)', 'Req. 2: 0.05° after 360° (laser)',
                                        'Req. 3: wheel failure caught and held (4 table tests)', 'Stretch: SLEW YAW 4/4',
                                        'DEMO ALL: one command, graded on board', 'Estimator heading leak found + fixed',
                                        'First-run start failures fixed'], size=13.5)
rewrite(shape_named(ws, 'TextBox 131'), ['Wheel loop pauses ~0.1 s during SD writes', 'Wheel D encoder lead (reseated, not soldered)',
                                        'Motor-switch leads loose', 'Hold time: yaw drag fills the wheels in ~5 min',
                                        'Magnetometer aid unusable at the demo spot'], size=13.5)
rewrite(shape_named(ws, 'TextBox 133'), ['Tilted stretch targets (CG ≲ 1 mm)', 'Wheel loop on a timer interrupt',
                                        'Momentum dumping (no thrusters or magnetorquers)'], size=13.5)
notes(ws, 'Every requirement is verified on the table. What needs improvement is mostly hardware: connectors that cost us '
          'table time, and the wheel loop sharing a processor loop with the SD card. The firmware is frozen until after this '
          'presentation, so the fixes are listed, not made.')

# ---- 28: lessons learned & future work
pf = S[28]
pf.shapes.title.text = 'Lessons Learned & Future Work'
remove(shape_named(pf, 'Table 94'))
rows = [['#', 'Lesson → next step', 'Evidence'],
        ['1', 'Measure truth independently (laser), never with the estimator', 'Estimator said 0.1°, table was 2.3° off'],
        ['2', 'Warm up 10 min, calibrate 180 s before heading work', 'Early calibration drifted ~15 °/h'],
        ['3', 'Keep the SD card out of the control path (timer-interrupt wheel loop)', 'SD stalls froze the wheel loop'],
        ['4', 'Solder and strain-relieve every connector', 'Wheel D lead, motor switch: most lost table time'],
        ['5', 'Balance with a rigid, repeatable mass layout, then tilted slews', 'Loose mass made trims unpredictable'],
        ]
table(pf, 0.3, 1.05, 9.4, [0.4, 5.3, 3.7], rows, header_size=13.5, body_size=13, row_h=0.5, header_h=0.42)
rewrite(shape_named(pf, 'TextBox 92'), ['Demo checklist', 'Motor switch + pack leads checked before arming',
                                       'Power on, 10 min warm-up, CALIBRATE 180 clamped',
                                       'DEMO ALL; laser read after part 1, part 2 and the failure'], size=13)
rewrite(shape_named(pf, 'TextBox 93'), ['Improvements beyond KestrelSAT', 'Use of the air bearing table',
                                       'Automatic wheel-failure detection, tested for real', 'One-command, self-grading demo',
                                       'Laser heading truth; log replay on the PC'], size=13)
for name in ('TextBox 92', 'TextBox 93'):
    sh = shape_named(pf, name)
    sh.top = Inches(4.4)
    for k, p in enumerate(sh.text_frame.paragraphs):
        for r in p.runs:
            r.font.bold = k == 0
notes(pf, 'The most important lesson: the estimator graded its own slews 4 of 4 while the table ended 2.3 degrees off. Only '
          'the laser showed it. Everything else on this list cost us table time this week.')

# ---- 29: GenAI
ga = S[29]
body = shape_named(ga, 'Text Placeholder 2')
paras = body.text_frame.paragraphs
upd = {
    1: 'Firmware: drafted the EKF and magnetometer aid, controller, allocation, fault detector, IMU driver, logger, command set, '
       'the self-grading tests and DEMO ALL, from the team’s requirements and design decisions; the team reviewed, uploaded and '
       'tested every build',
    2: 'Simulation and analysis: wrote the truth models, the log-replay and balance tools and the plots, and traced the '
       'estimator heading leak and the start-up stall from the logs; the team ran the tests, read the laser and checked the conclusions',
    3: 'Documentation: CLAUDE.md, TODO.md, PROGRESS_LOG.md, test reports and drafts of these slides',
}
for k, t in upd.items():
    rs = paras[k].runs
    rs[0].text = t
    for r in rs[1:]:
        r._r.getparent().remove(r._r)

# ================================================================ new slides
N = {}

# ---- bottom line up front
bl = new_slide('Bottom Line: One Command, All Requirements',
               'This is the result the rest of the briefing builds to. One command, DEMO ALL, runs a heading hold, four slews '
               'through a full turn, and then cuts power to wheel B without telling the controller. Nothing is reset between '
               'the parts. The Teensy grades each part; the laser on the wall is the independent truth.')
takeaway(bl, 'One command passed every requirement, confirmed by a laser 2.48 m away')
for k, (big, lab, col) in enumerate([('0.04°', 'max heading error, 60 s hold (Req. 1); laser steady', GREEN),
                                     ('0.05°', 'laser heading error after a full 360° of slews (Req. 2)', GREEN),
                                     ('0.50 s', 'to detect wheel B’s dead motor; held ≤ 0.36° on 3 wheels (Req. 3)', GREEN),
                                     ('4 / 4', 'slew targets reached and held within ±1° (stretch)', GREEN)]):
    stat(bl, 0.3 + k * 2.37, 1.6, 2.22, 1.75, big, lab, color=col, big_size=30)
for k, (head, body, fill) in enumerate([('1  Heading hold', 'DEMO LEVEL: heading held, roll/pitch damped at the rest tilt; graded for 60 s', TINT_B),
                                        ('2  SLEW YAW', 'four targets, one full turn, from the heading the hold kept', TINT_G),
                                        ('3  Wheel failure', 'wheel B’s motor cut 10 s later; detected, then 30 s on A, C, D', TINT_A)]):
    x = 0.3 + k * 3.18
    rect(bl, x, 3.6, 3.0, 1.55, fill, radius=0.06)
    text(bl, x + 0.1, 3.65, 2.8, 0.45, [dict(runs=[(head, dict(b=True, size=16, color=BLUE))])])
    text(bl, x + 0.1, 4.08, 2.8, 1.05, [dict(runs=[(body, dict(size=13))])])
text(bl, 0.3, 5.4, 9.4, 1.4, [
    dict(runs=[('Getting there: ', dict(b=True)), ('two problems that the estimator itself could not see. ', {}),
               ('The heading lost ~1% of every turn', dict(b=True)), (' (found with the laser, slide 43), and ', {}),
               ('every first run after a boot failed', dict(b=True)), (' (an SD-card stall, slide 44). Both fixed and verified in this run.', {})])],
     size=14, fill=TINT_GREY, margin=0.12, anchor=MSO_ANCHOR.MIDDLE)
N['bl'] = bl

# ---- code: heading-free accelerometer update
ca = new_slide('Code: Gravity Update Fix',
               'Gravity tells the filter which way is up, never which way it is facing. But while the table turns, the filter\'s '
               'yaw error becomes correlated with its tilt error, and the standard gain then also rotates the heading. That took '
               'about 1% off every turn. The fix is one projection: remove the gain\'s component along vertical. With it, the '
               'gyro scale was re-fitted from the laser runs.')
code_box(ca, 0.3, 1.0, 5.6, 4.55, [
    '// attitude_ekf.cpp: gravity update',
    'Vec3 y = accel / |accel|;     // measured "up"',
    'Vec3 yh = A * ez;             // predicted "up"',
    'H = [-[yh]x, 0];  r = y - yh;',
    'K = P H^T (H P H^T + R)^-1;',
    'K[5][*] = 0;                  // z bias locked',
    '// NEW: gravity carries no heading',
    'for (j = 0; j < 3; j++) {',
    '  along = yh . K[0..2][j];',
    '  K[0..2][j] -= along * yh;   // off vertical',
    '}',
    'q = exp(K[0..2] r) (x) q;     // tilt only',
    '',
    '// main.cpp: yaw gyro scale (laser fit)',
    'gyro.z *= 1.0045;             // was 1.009',
], caption='Excerpts, simplified from attitude_ekf.cpp and main.cpp')
bullets(ca, 6.1, 1.0, 3.6, 5.9, [
    [('Symptom: ', dict(b=True)), ('full-turn slews ended 1–2.3° off by laser while the estimate said 0', {})],
    [('Clue: ', dict(b=True)), ('mirrored targets flipped the error (+2.3° CCW, −1.8° CW)', {})],
    [('Cause: ', dict(b=True)), ('∫ gyro = 364°, estimator = 360°: the accelerometer update was turning the heading back', {})],
    [('Fix: ', dict(b=True)), ('project K off the measured vertical; refit the yaw scale by replaying the logs', {})],
    [('Result: ', dict(b=True)), ('0.05° after 360° (laser, DEMO ALL)', dict(b=True, color=GREEN))],
], size=13.5, sb=6)
stat(ca, 0.3, 5.75, 2.7, 1.15, '+2.3° / −1.8°', 'laser error after 360° before (CCW / CW)', color=RED, big_size=22)
stat(ca, 3.2, 5.75, 2.7, 1.15, '0.05°', 'after the fix (DEMO ALL)', color=GREEN, big_size=22)
N['ca'] = ca

# ---- code: DEMO ALL sequencer
cd = new_slide('Code: DEMO ALL Sequencer',
               'DEMO ALL chains the three existing test modes inside one armed run. The important design choice is the '
               'hand-over: the slews start from the heading the hold kept, and nothing is reset or braked in between. An earlier '
               'attempt stopped one test and started the next, and braking the wheels kicked the table off its zero. The '
               'Teensy grades each part and prints one summary line.')
code_box(cd, 0.3, 1.0, 5.6, 4.6, [
    '// main.cpp: one armed run, no resets',
    'startDemo(levelOnly);         // 1: hold heading',
    'if (settled)                  // graded 60 s',
    '  done = fullDemo.holdStep(dt, yawErr);',
    'if (done) {                   // 2: hand over',
    '  demo.abort();               // no disarm/brake',
    '  slew.start(q, YawOnly, 0,',
    '             demo.startYaw());// held heading',
    '}',
    'if (slew complete)            // 3: wheel fails',
    '  fault.start(B, 10 s);       // ctrl not told',
    'if (fault graded)',
    '  say("DEMO ALL COMPLETE: ...");',
    '',
    '// data_logger.cpp: open + flush the log',
    '// file before arming (slide 44)',
], caption='Excerpts, simplified from main.cpp, full_demo.cpp, data_logger.cpp')
bullets(cd, 6.1, 1.0, 3.6, 5.9, [
    [('One command: ', dict(b=True)), ('Req. 1, Req. 2 + stretch, Req. 3, in order', {})],
    [('No wrong zero: ', dict(b=True)), ('slew targets are relative to the heading the hold kept', {})],
    [('No kick: ', dict(b=True)), ('no disarm between parts; wheels keep their momentum', {})],
    [('Self-grading: ', dict(b=True)), ('max error, jitter, drift, per-target errors, detection time; one summary line', {})],
    [('Any stop: ', dict(b=True)), ('reports the part it stopped in', {})],
], size=13.5, sb=6)
stat(cd, 0.3, 5.85, 2.7, 1.05, '≈ 5 min', 'one command, one armed run', color=BLUE, big_size=22)
stat(cd, 3.2, 5.85, 2.7, 1.05, '3 / 3', 'requirements graded on board, one summary line', color=GREEN, big_size=22)
N['cd'] = cd

# ---- hardware: laser truth
lz = new_slide('Hardware: Laser Heading Truth',
               'The estimator cannot grade itself: it uses the same gyro it is trying to check. So we mounted an 11 g laser on the '
               'deck and printed a scale for a wall 2.48 m from the bearing centre. A tenth of a degree moves the dot 4.3 mm, '
               'easy to read by eye. This is the measurement behind every heading number from here on.')
takeaway(lz, 'A deck laser on a wall scale: heading truth to ≈ 0.05°, independent of the IMU')
picture(lz, os.path.join(NF, 'n1_laser_setup.png'), 0.3, PIC_Y, w=9.4, h=3.6, center_w=9.4)
bullets(lz, 0.3, 5.3, 9.4, 1.6, [
    [('Scale: ', dict(b=True)), ('tools/laser_scale.py prints x = R tan θ for R = 2480 mm: 0.5° ticks to ±5°, 0.1° within ±1°, positive = CCW', {})],
    [('Use: ', dict(b=True)), ('line the dot up at 0 before a run; read it at each checkpoint (end of hold, end of slews, through the failure)', {})],
    [('Limits: ', dict(b=True)), ('heading only (roll/pitch from the accelerometer, σ 0.02°); reading resolution ≈ 2 mm ≈ 0.05°', {})],
], size=13.5, sb=4)
N['lz'] = lz

# ---- software build & tests (cont.)
sw2 = new_slide('Progress: Software Build & Tests (Cont.)',
                'Ten days since the prelim; these are the last two. Each row is a logged test with its result; PROGRESS_LOG.md '
                'has the details.')
rows = [['Date', 'Test', 'Key result'],
        ['10-31', 'Balance session; 80 rad/s wheel limit', 'Rest tilt to −0.9 / −0.7°; ~35% more momentum'],
        ['10-31', 'STEP TEST; hard-iron fit', 'Rise 1.2–1.7 s, ≤ 4% overshoot; I_zz = 0.077 kg·m²'],
        ['10-31', 'FAULT TEST B on the air bearing', 'PASS: detected 0.91 s, never outside ±1°'],
        ['10-31', 'MAG TEST 2; presentation spot', 'Aid not reliable there → off by default'],
        ['10-31', 'SLEW TEST vs SLEW YAW', 'Tilted: wheel limit at 13.5 s. SLEW YAW 4/4'],
        ['11-01', 'Laser truth: SLEW YAW, holds', '−0.1° after 360°; 5 min hold within 0.25°'],
        ['11-01', 'Raise CG; SLEW TILT', 'Scan filled the wheels; needs CG ≲ 1 mm'],
        ['11-01', 'Wheel D lead; first-run failures', 'Encoder lead intermittent; SD stall found + fixed'],
        ['11-01', 'Mirrored slews (laser)', 'Estimator heading leak found + fixed'],
        ['11-01', 'DEMO ALL', 'All requirements PASS, laser-verified'],
        ]
table(sw2, 0.3, 1.0, 9.4, [0.75, 3.75, 4.9], rows, header_size=13.5, body_size=12.5, row_h=0.42, header_h=0.4)
text(sw2, 0.3, 5.9, 9.4, 0.95, [dict(runs=[('New on the Teensy since 10-30: ', dict(b=True)),
                                           ('STEP TEST, SLEW YAW / SLEW TILT, DEMO ALL, wheel-loop stall handling, heading-free '
                                            'accelerometer gain. On the PC: log replay through the firmware’s own estimator, balance '
                                            'and hard-iron fits, laser scale.', {})])], size=13.5, fill=TINT_GREY, margin=0.1)
N['sw2'] = sw2

# ---- MAG TEST, both runs (replaces the v1 slide)
N['mt'] = fig_slide('Data: Magnetometer Yaw Aid (MAG TEST)', os.path.join(WU, 'f01_magtest.png'),
    'The aid held yaw on a clamped table (36/37), but not reliably enough for the demo',
    'Two MAG TESTs, table clamped (true heading 0): stopped, wheel direction patterns, ±10° kicks, drift run; graded on the Teensy.',
    'Requirement ±1°. Gyro alone (replay, aid off): 5.7 °/h even after warm-up.',
    'Run 1 (learning references): 36/37, worst 1.02°. Run 2 (predicting new patterns): stopped at 10/15, the motor-magnet model moved ~1° between sessions. Each new pattern costs 0.3–1.2°.',
    'Done: learning restored; the presentation spot bends the field (next slide), so the demo runs gyro-only.',
    'MAG TEST grades itself on the Teensy. The table is clamped, so any yaw is estimation error. The first run passed 36 of 37 '
    'steps; the one miss was 1.02 degrees. We then tried predicting each new wheel pattern\'s reference from a known one: the '
    'second run failed, because the motor-magnet model moved about a degree between sessions. Grey is the gyro alone: almost '
    '6 degrees an hour. The aid works, but every new wheel pattern costs up to a degree, so we did not rely on it for the demo.')

N['room'] = fig_slide('Data: Magnetometer at the Presentation Spot', os.path.join(WU, 'f11_room_distortion.png'),
    'The presentation spot bends the field up to 14°: there, the gyro alone is better',
    'Magnetometer heading error vs heading, table turned by hand, hard iron removed: first test spot vs the presentation spot.',
    'The aid needs a field error well under 1°; the first spot: 1.8° p-p with the wheels stopped.',
    'Presentation spot: 4.1° p-p stopped, 14.3° spinning. A fitted room correction reached 1.7° in-sample but 3.5–5.3° cross-checked.',
    'Done: aid off by default (MAG ON enables it); heading from the gyro, verified by laser.',
    'Steel near the presentation spot bends the Earth\'s field. Turning the table by hand shows a heading error that depends on '
    'which way the table faces: 14 degrees peak to peak with the wheels spinning. A correction fitted in place did not survive '
    'a cross-check. So at the demo site the heading comes from the gyro, and the laser checks it.')

N['bal'] = fig_slide('Data: Balancing the Table', os.path.join(WU, 'f02_balance.png'),
    'Rest tilt trimmed from 12° to under 1°; raising the CG made the table unpredictable',
    'DEMO LEVEL rest-tilt readings after each trim move (10-31); free-swing fits for the CG height.',
    'Rest tilt within ±5° keeps DEMO LEVEL comfortable; tilted slews need the CG within ~1 mm of the pivot.',
    '10-31: −0.9 / −0.7° at CG ≈ 13 mm. 11-01: raising the CG to ~3 mm made trims land 2–4× off and the table tip; final stiff setup rests at −2.5 / +0.1°.',
    'Tilted targets need a rigid, repeatable mass layout and CG ≲ 1 mm (future work).',
    'Each dot is one rest-tilt reading after a trim move. On 10-31 we brought the table to within a degree of level. On 11-01 we '
    'tried to raise the centre of mass to about a millimetre below the pivot for tilted slews. At that height the table became '
    'very sensitive: trims landed two to four times further than predicted and it tipped twice. We went back to a stiff setup '
    'for the demo.')

N['dead'] = fig_slide('Data: False Dead-Motor Flags (Unexpected)', os.path.join(WU, 'f03_deadmotor.png'),
    'Healthy wheels need up to 0.27 effort to break free; the detector now judges a stopped wheel only above 0.4',
    'Detector flags on the air bearing (DEMO_0004/0005, 230 wheel starts from rest): duty, effort, speed.',
    'A dead motor gains no speed under effort ≥ 0.2; a healthy one breaks free at 0.10–0.15.',
    'Static friction: 14 healthy starts reached 0.23–0.30 effort before moving (13 on wheel D, which sticks).',
    'Done: standstill threshold 0.4 (just under the current limit): replaying both logs gives 0 false flags.',
    'Our first air-bearing runs flagged healthy wheels as dead. The log shows why: a stopped wheel has to overcome static '
    'friction, and some starts needed more effort than our threshold before they moved. Wheel D sticks the most. We raised the '
    'threshold only for a wheel at rest; a spinning wheel is still judged at 0.2.')

N['st1'] = fig_slide('Data: STEP TEST 1 (Unexpected)', os.path.join(WU, 'f05_heading_hardiron.png'),
    'Uncorrected hard iron: the table turned 8° while the estimate looked perfect',
    'STEP TEST (DEMO_0007): ±5° heading steps on the air bearing; gyro-only replay = the true heading.',
    'The estimate (blue) followed the steps exactly; the table (red) should have too.',
    'The table carries its own magnetic field, (+20.5, −48.9) µT, about twice Earth\'s: the circle is far from zero, so the aid "corrected" toward a wrong reference.',
    'Done: hard iron fitted from a full turn (tools/hardiron_fit.py); STEP TEST rerun (next).',
    'A lesson in why the estimator cannot be its own judge. The blue line is what the firmware believed: perfect steps. The red '
    'line is the gyro alone, which is the truth over a minute: the table drifted 8 degrees. The magnetometer readings circle a '
    'centre far from zero: steel on the table itself. Once fitted and removed, the steps tracked.')

N['st2'] = fig_slide('Data: STEP TEST (Heading Steps)', os.path.join(WU, 'f06_steptest.png'),
    'Steps rise in 1.2–1.7 s with ≤ 4% overshoot; measured I_zz = 0.077 kg·m²',
    'STEP TEST rerun (DEMO_0008, hard iron removed): ±5° steps, 20 s each, graded on board; I_zz from wheel momentum exchange.',
    'Design: ωn = 2 rad/s, ζ = 0.9 → ≈ 1.5 s rise, no overshoot.',
    'Rise 1.2–1.7 s, overshoot ≤ 4%, no steady error. Gravity gives no yaw torque, so I_zz (ψ − ψ0) + ∫(h_z − h_z0) dt fits the inertia.',
    'Done: I_zz = 0.077 set as the default; gains unchanged.',
    'STEP TEST checks requirement 1\'s controller directly: the heading is stepped 5 degrees and back. The response matches the '
    'design. The same data measures the table\'s yaw inertia: when the wheels speed up, the table turns the other way, and the '
    'ratio gives I_zz.')

N['ft'] = fig_slide('Data: FAULT TEST on the Air Bearing (Req. 3)', os.path.join(WU, 'f07_faulttest.png'),
    'First wheel-failure test on the table: wheel B detected in 0.91 s, attitude never left ±1°',
    'FAULT TEST B during a heading hold (DEMO_0007): B’s motor cut without telling the controller.',
    'Pass: detected ≤ 1 s, back within ±1° ≤ 10 s, then ±1° for 30 s (graded on the Teensy).',
    'PASS: detected 0.91 s; peak 0.8° yaw; B coasts, A, C and D take its share; 30 s held.',
    'Done: repeated inside DEMO ALL (0.50 s, slide 48); a real failure follows.',
    'This is requirement 3 on the real table. The firmware cuts wheel B\'s motor but tells the controller nothing. The '
    'detector sees B driven but not accelerating, takes it out of the allocation, and the other three wheels hold the heading.')

N['wd'] = fig_slide('Data: Wheel D Encoder Lead (Real Failure)', os.path.join(WU, 'f08_wheelD.png'),
    'A real failure: wheel D’s encoder lead opened, and the detector took D out of service',
    'DEMO_0008: wheel D driven to the 0.45 duty limit while its encoder count stood still.',
    'A healthy wheel at 0.45 duty accelerates at hundreds of rad/s².',
    'The lead conducts at rest (10 hand turns ≈ 4000 counts) but opens under vibration (0 under power); A and C absorbed D’s share.',
    'Reseated and strain-relieved; replace with soldered wires after the presentation.',
    'Requirement 3 again, but not a test: wheel D\'s encoder wire opened mid-run. The controller drove D harder and harder, '
    'the detector caught it, and the other wheels carried on. We traced it to an intermittent lead: it reads fine when turned by '
    'hand, and drops out when the motor vibrates it.')

N['tilt'] = fig_slide('Data: Tilted Slews vs. Gravity (Stretch)', os.path.join(WU, 'f09_fixedtilt_vs_sim.png'),
    'A tilt held off rest costs steady torque: tilted targets fill a wheel in seconds',
    'SLEW TEST with roll/pitch held at −1.2 / −0.2° (table) vs the same targets in simulation.',
    'Simulation reached target 4 before a wheel limit (88 s).',
    'Table: wheel limit after 13.5 s. The rest tilt moves ~0.3° with heading; every degree off rest costs ≈ 5–11 mN·m.',
    'SLEW YAW: yaw-only targets at the rest tilt (next slide).',
    'On a pendulous table, gravity pulls back toward the rest tilt with a torque proportional to the angle. The wheels can '
    'supply torque only by storing momentum, so any tilt held for long fills them. Even holding a fixed tilt while turning in '
    'yaw failed in 13 seconds, because the rest tilt itself changes slightly with heading.')

N['sy'] = fig_slide('Data: SLEW YAW on the Air Bearing (Stretch)', os.path.join(WU, 'f10_slewyaw_vs_sim.png'),
    'Four yaw targets on the shortest path, 4/4 within ±1°; table and simulation agree',
    'SLEW YAW (SLEW_0002): +45, +135, −135, 0° relative to start, roll/pitch damped at the rest tilt; same targets in simulation.',
    'Each target reached and held 5 s within ±1° (graded on board).',
    '4/4 PASS, yaw error ≤ 0.6° in the holds; ended on the tape marker. Laser on 11-01 (SLEW_0003): −0.1° after 360°.',
    'Later laser runs disagreed (up to 2.3°): the estimator heading leak (slide 43).',
    'SLEW YAW is the stretch requirement as this table can do it: the slews turn about the vertical, the shortest path between '
    'headings, and roll and pitch stay at the rest tilt. Table and simulation look alike. The first laser check after a full turn '
    'was 0.1 degrees. Later runs were worse, which led us to the estimator bug on slide 43.')

st = new_slide('Data: SLEW TILT (Tilted Targets)',
               'SLEW TILT was our attempt at tilted targets: scan the rest tilt over a full turn, aim for small offsets from it, '
               'and use gravity to drain the wheels between targets. It works in simulation at about a millimetre of CG height. '
               'On the table the scan filled the wheels in 40 seconds: a tiny gyro bias times the damping gain is a steady torque. '
               'We added a momentum leak, but could not hold the balance at a millimetre, so tilted targets remain future work.')
takeaway(st, 'Tilted targets work in simulation near neutral balance; the table could not be held there')
for k, (big, lab, col) in enumerate([('±2°', 'tilt targets pass 4/4 in simulation at CG 1 mm (±3° at 0.5 mm)', GREEN),
                                     ('2 mm', 'CG height where even ±1° fails in simulation', RED),
                                     ('40 s', 'until the table’s rest-tilt scan filled a wheel (DEMO_0012)', AMBER),
                                     ('~3 mm', 'lowest CG we could hold repeatably; trims landed 2–4× off', AMBER)]):
    stat(st, 0.3 + (k % 2) * 4.75, 1.6 + (k // 2) * 1.55, 4.6, 1.4, big, lab, color=col, big_size=28)
bullets(st, 0.3, 4.75, 9.4, 2.15, [
    [('Method: ', dict(b=True)), ('360° rest-tilt scan → targets as offsets from rest → gravity feed-forward k(θ_ref − θ_rest) → unload the '
                                    'wheels by tilting against gravity between targets', {})],
    [('Found: ', dict(b=True)), ('x/y gyro bias × damping gain = a steady 0.4–0.7 mN·m the wheels absorb; fixed with a momentum leak '
                                   '(reference rate += h_xy / (Kd·10 s))', {})],
    [('Next: ', dict(b=True)), ('rigid trim masses, CG ≲ 1 mm, then SLEW TILT (firmware ready)', {})],
], size=13.5, sb=4)
N['st'] = st

N['lh'] = fig_slide('Data: Heading Holds by Laser (Req. 1)', os.path.join(NF, 'n2_laser_holds.png'),
    'Warm calibration: within 0.25° for 5 min by laser; calibrating too early drifts ≈ 15 °/h',
    'DEMO LEVEL heading holds on the floating table; laser read every 30–60 s.',
    'Requirement: no visible jitter or drift, |error| < 1°.',
    'Calibrated 5 min after power-on: −0.78° in 2.5 min (gyro z bias still warming). After 10+ min warm-up: 0.25° band for 5 min, no trend; 0.15° for 3 min.',
    'Done: power on, 10 min warm-up, CALIBRATE 180 is now the demo procedure.',
    'The red line is the same hold, calibrated too soon after power-on: the gyro\'s yaw bias was still changing, and the table '
    'drifted almost a degree in two and a half minutes. Blue and green are after a proper warm-up: a quarter-degree band, no '
    'trend. That is requirement 1 measured against truth.')

N['leak'] = fig_slide('Data: Estimator Heading Leak (Unexpected)', os.path.join(NF, 'n3_heading_leak.png'),
    'The estimator lost ~1% of every turn; only the laser could see it. Fixed: 0.05° after 360°',
    'Laser error after each full-turn SLEW YAW on 11-01; logged gyro integrated vs estimator vs laser.',
    'Every run passed 4/4 by the Teensy\'s own grading, so the estimate said 0° every time.',
    'Errors scattered −0.1 to +2.3° and flipped with the turn direction; the estimator turned 360° while its gyro said 363–364°. Cause and fix: slide 9.',
    'Done: heading-free accelerometer gain + yaw scale 1.0045 (replay fit); DEMO ALL 0.05°.',
    'This is the most important result we found this week. The slews passed every time by the firmware\'s own measure, while '
    'the laser said the table was up to two and a quarter degrees off. Running the turn the other way flipped the sign, which '
    'pointed at a scale error. Integrating the logged gyro showed the gyro was right and the estimator was taking about one '
    'percent off each turn. The cause was the accelerometer update. After the fix, the laser shows 0.05 degrees.')

N['sd'] = fig_slide('Data: First-Run Start Failures (Unexpected)', os.path.join(NF, 'n4_sd_stall.png'),
    'Every first run after a boot failed: the SD card blocked the wheel loop, not a wheel fault',
    'Wheel encoder counts, speed and duty in the first 2 s after arming (DEMO_0016, 100 Hz log).',
    'Expected: smooth spin-up to the ±100 rpm bias. Instead, 7 of 7 first arms after a boot flagged all wheels dead; later arms all worked.',
    'The first write to a new log file searches the card for space: loop() froze ~0.8 s, the wheel loop then saw one huge step.',
    'Done: the log file is written and flushed before arming; a wheel-loop step > 50 ms skips the fault checks. Next: run the wheel loop from a timer interrupt.',
    'We spent hours on wiring because two wheels ran away at startup. The pattern was in the logbook: it only ever happened on '
    'the first run after a boot. The 100 Hz log shows the wheel loop simply stopped for most of a second while the encoders '
    'kept counting. The first run of DEMO ALL after the fix was the clean run on the next slides.')

N['d1'] = fig_slide('DEMO ALL: The Whole Run', os.path.join(DA, 'fig1_overview.png'),
    'One command, one armed run: hold → full-turn slews → wheel failure, graded on board',
    'DEMO ALL (DEMO_0020, 11-01 23:13), first arm after a boot; heading, tilt and wheel speeds.',
    'Each part against its requirement; laser read at the end of each part.',
    'Hold 60 s; four slews through 360°; wheel B cut 10 s later and the table held on A, C and D.',
    'Next three slides: one per requirement.',
    'The whole demonstration in one plot. Blue: the graded heading hold. Green: four slews, one full turn, each held five '
    'seconds. Orange: wheel B fails; its speed drops to zero and A, C and D take over. Nothing was reset between the parts.',
    pic_h=4.3, size=12.5)

N['d2'] = fig_slide('DEMO ALL: Requirement 1 (Heading Hold)', os.path.join(DA, 'fig2_req1_hold.png'),
    'Heading held for 60 s: max 0.04°, jitter 0.01° rms, no drift; the laser dot did not move',
    'Heading error during the graded 60 s of the hold (estimate); laser read through the hold.',
    'Requirement: |error| < 1°, no visible jitter or drift.',
    'Max 0.04°, 0.011° rms, drift 0.000°; laser steady at −0.05° (its reference for part 2).',
    'Met. Longer holds are limited by yaw drag filling the wheels (~5 min).',
    'Requirement 1 with a 25 times margin. The laser reading at the end of this hold is the reference for the slews.')

N['d3'] = fig_slide('DEMO ALL: Requirement 2 (Attitude Truth)', os.path.join(DA, 'fig3_req2_slews.png'),
    'Laser: 0.05° off after a full 360° of slews; every target held within ±1°',
    'Laser before and after the four slews (one full turn); per-target yaw error in each 5 s hold.',
    'Requirement ±1°. Before the estimator fix the same test ended +2.3° (CCW) and −1.8° (CW) off.',
    'Laser −0.05° → +0.02…−0.03°: ≈ 0.05° after 360°. Targets 0.03 / 0.02 / 0.13 / 0.05°. Roll/pitch within 0.23° of rest.',
    'Met. The same data replayed through the old estimator ends 0.46° off on this run (1.7–2.3° on earlier runs).',
    'This is the number that matters most: after a full turn, the table came back to within five hundredths of a degree of '
    'where it started, measured by the laser, not by the estimator. Note the honest comparison: on this particular run the old '
    'estimator would have been half a degree off; on earlier runs it was up to two and a quarter.')

N['d4'] = fig_slide('DEMO ALL: Requirement 3 (Wheel Failure)', os.path.join(DA, 'fig4_req3_fault.png'),
    'Wheel B cut without warning: caught in 0.50 s, held ≤ 0.36° on three wheels',
    'Attitude error and wheel speeds around the moment wheel B’s motor was cut (simulated failure, controller not told).',
    'Pass: detected ≤ 1 s, back within ±1° ≤ 10 s, then ±1° for 30 s.',
    'Detected 0.50 s; peak 0.90° (estimate), laser peak 0.3°; 30 s hold max 0.36° yaw, 0.01° roll/pitch.',
    'Met, on the first arm after a boot (the case that always failed before).',
    'The last part of the demo. B\'s motor is cut and the controller is not told. Half a second later the detector takes B out '
    'of the allocation; A, C and D hold the heading. On the wall the dot moved three tenths of a degree and stopped.')

# ---- '~' looks like a minus sign in Quattrocento Sans: write '≈' (code text in Courier is left alone)
def _fix_tilde(shapes):
    for sh in shapes:
        if sh.shape_type == 6:  # group
            _fix_tilde(sh.shapes)
        if sh.has_text_frame:
            for p in sh.text_frame.paragraphs:
                for r in p.runs:
                    if '~' in r.text and (r.font.name or '') != CODEFONT:
                        r.text = r.text.replace('~ ', '≈ ').replace('~', '≈ ')
        if getattr(sh, 'has_table', False) and sh.has_table:
            for row in sh.table.rows:
                for cell in row.cells:
                    for p in cell.text_frame.paragraphs:
                        for r in p.runs:
                            r.text = r.text.replace('~ ', '≈ ').replace('~', '≈ ')
for _s in prs.slides:
    _fix_tilde(_s.shapes)

# ================================================================ order
sld = prs.slides._sldIdLst
ids = list(sld)
O = {i + 1: ids[i] for i in range(len(orig))}
NEW = list(N.keys())
NI = {k: ids[len(orig) + i] for i, k in enumerate(NEW)}
order = [O[1], O[2], O[3], O[4], NI['bl'], O[5], O[6], O[7], NI['ca'], O[8], O[9], NI['cd'], O[10], O[11], NI['lz'],
         O[12], NI['sw2'], O[13], O[14], O[15], O[16], O[17], O[18], O[19], NI['mt'], NI['room'],
         O[21], O[22], O[23], O[24], O[25], O[26], NI['bal'], NI['dead'], NI['st1'], NI['st2'], NI['ft'], NI['wd'],
         NI['tilt'], NI['sy'], NI['st'], NI['lh'], NI['leak'], NI['sd'], NI['d1'], NI['d2'], NI['d3'], NI['d4'],
         O[27], O[28], O[29]]
dropped = [O[20]]  # v1 MAG TEST slide (claimed 37/37 with prediction)
for el in ids:
    sld.remove(el)
for el in order:
    sld.append(el)
# drop the replaced slide's part so it doesn't linger in the package
for el in dropped:
    rid = el.get(qn('r:id'))
    prs.part.drop_rel(rid)
prs.save(OUT)
print('saved', OUT, len(order), 'slides')
