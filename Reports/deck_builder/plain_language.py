"""Plain-language pass (2026-11-02): slide text rewritten as short full sentences for a general engineering audience,
in the third person and the active voice
(fewer symbols and fragments, numbers kept where they matter). Formatting is kept: every rewrite reuses the existing
paragraph and run properties. Code panels, equations, dated test tables and the GenAI slide are not touched.
Afterwards: edit_safe.py, then fragility.ps1 + fit_overflow.py.
    python Reports/deck_builder/plain_language.py <in.pptx> <out.pptx>"""
import copy, sys
from pptx import Presentation
from pptx.oxml.ns import qn

SRC, OUT = sys.argv[1], sys.argv[2]
prs = Presentation(SRC)


def slide(start):
    for s in prs.slides:
        if s.shapes.title is not None and s.shapes.title.text.startswith(start):
            return s
    raise KeyError(start)


def all_shapes(s):
    for sh in s.shapes:
        if sh.shape_type == 6:
            yield from sh.shapes
        else:
            yield sh


def shape(s, start):
    """First text shape whose text starts with start (a string or a tuple of alternatives)."""
    for sh in all_shapes(s):
        if sh.has_text_frame and sh.text_frame.text.strip().startswith(start):
            return sh
    raise KeyError((s.shapes.title.text, start))


def set_runs(sh, text):
    p = sh.text_frame.paragraphs[0]
    p.runs[0].text = text
    for r in p.runs[1:]:
        r._r.getparent().remove(r._r)


def takeaway(s, text):
    rs = shape(s, 'Takeaway').text_frame.paragraphs[0].runs
    rs[0].text = 'Takeaway: '
    rs[1].text = text
    for r in rs[2:]:
        r._r.getparent().remove(r._r)


def mcen(s, **parts):
    box = shape(s, 'MEASURE')
    done = set()
    for p in box.text_frame.paragraphs:
        lab = p.text.split(':')[0].strip()
        if lab in parts:
            rs = p.runs
            rs[0].text = lab + ': '
            rs[1].text = parts[lab]
            for r in rs[2:]:
                r._r.getparent().remove(r._r)
            done.add(lab)
    assert done == set(parts), (s.shapes.title.text, set(parts) - done)


def labelled(sh, items):
    """Replace a bulleted box of 'Label: text' paragraphs; items = [(label, text), ...]."""
    tb = sh.text_frame._txBody
    ps = tb.findall(qn('a:p'))
    tp = copy.deepcopy(next(p for p in ps if len(p.findall(qn('a:r'))) >= 2))
    lab_r, txt_r = [copy.deepcopy(r) for r in tp.findall(qn('a:r'))[:2]]
    for r in tp.findall(qn('a:r')):
        tp.remove(r)
    for p in ps:
        tb.remove(p)
    end = tp.find(qn('a:endParaRPr'))
    for lab, txt in items:
        p = copy.deepcopy(tp)
        e = p.find(qn('a:endParaRPr'))
        for tmpl, t in ((lab_r, lab), (txt_r, txt)):
            r = copy.deepcopy(tmpl)
            r.find(qn('a:t')).text = t
            if e is not None:
                e.addprevious(r)
            else:
                p.append(r)
        tb.append(p)


def lines(sh, texts):
    """Replace a box of single-run paragraphs (lists, card bodies) with one paragraph per text."""
    tb = sh.text_frame._txBody
    ps = [p for p in tb.findall(qn('a:p')) if p.findall(qn('a:r'))]
    tp = copy.deepcopy(ps[0])
    r0 = copy.deepcopy(tp.findall(qn('a:r'))[0])
    for r in tp.findall(qn('a:r')):
        tp.remove(r)
    for p in tb.findall(qn('a:p')):
        tb.remove(p)
    for t in texts:
        p = copy.deepcopy(tp)
        r = copy.deepcopy(r0)
        r.find(qn('a:t')).text = t
        e = p.find(qn('a:endParaRPr'))
        if e is not None:
            e.addprevious(r)
        else:
            p.append(r)
        tb.append(p)


def para(sh, i, text):
    """Replace paragraph i's text with one run (first run's format)."""
    p = sh.text_frame.paragraphs[i]
    p.runs[0].text = text
    for r in p.runs[1:]:
        r._r.getparent().remove(r._r)


def cell(tbl, r, c, text):
    tb = tbl.cell(r, c).text_frame._txBody
    ps = tb.findall(qn('a:p'))
    for p in ps[1:]:
        tb.remove(p)
    rs = ps[0].findall(qn('a:r'))
    rs[0].find(qn('a:t')).text = text
    for x in rs[1:]:
        ps[0].remove(x)


def callout(s, old, new):
    set_runs(shape(s, old), new)


# ================================================================ presented slides
# Third person, active voice: the team, the table, the firmware or the laser does each thing.
s = slide('Why Attitude Control Matters')
set_runs(shape(s, 'Wheels wear out'), 'Wheels wear out: Kepler lost 2 of its 4 wheels by 2013, and that ended its '
                                      'primary mission. A wheel failure should not end a mission.')
labelled(shape(s, 'Objective'), [
    ('Objective: ', 'control the table\'s orientation in all three axes with reaction wheels, and know that orientation '
                    'at all times by combining gyroscope, accelerometer and magnetometer data.'),
    ('New KestrelSAT capability: ', 'the table holds and turns its attitude on its own, detects a failed wheel and '
                                    'keeps pointing with the other three; a laser on the wall checks the result.'),
    ('Why an air bearing: ', 'the table floats on a thin layer of air and turns almost without friction, like a '
                             'satellite in orbit, so the team can test the whole loop on the ground.'),
])

s = slide('Requirements: Verification')
t = next(sh for sh in s.shapes if sh.has_table).table
rows = [
    ('(1) Hold a heading with no visible jitter or drift',
     'The floating table held its heading for 60 s in the full demo while the team read the laser. Pass: error '
     'under 1°.',
     'The heading stayed within 0.04° and the laser dot did not move. A 5-minute hold stayed within 0.25°.'),
    ('(2) Know the attitude to within 1° in all 3 axes',
     'The team compared the estimated heading with the laser after a full turn, and measured roll and pitch noise at '
     'rest. Pass: error under 1°.',
     'After a full turn the heading was 0.05° off. Roll and pitch vary by only 0.02° at rest, but have no '
     'independent check.'),
    ('(3) Detect a wheel failure and compensate',
     "The full demo cut wheel B's motor without telling the controller. Pass: detect within 1 s, return within 1° in "
     "10 s, hold 30 s.",
     'The firmware caught the failure in half a second, and the other three wheels held the attitude within 0.36°.'),
    ('Stretch: slew through 4 attitudes on the shortest path',
     'The table turned to four headings and held each for 5 s. Goal: within 1° without filling the wheels.',
     'All four headings passed. Tilted attitudes did not: the table is bottom-heavy, so holding a tilt fills the '
     'wheels in seconds.'),
]
for i, (a, b, c) in enumerate(rows, start=1):
    cell(t, i, 0, a)
    cell(t, i, 1, b)
    cell(t, i, 2, c)

s = slide('Theory: Estimation & Control Loop')
labelled(shape(s, 'Input'), [
    ('Input and output: ', 'the active test mode commands an attitude and turning rate; the output is the table\'s '
                           'actual attitude.'),
    ('Estimator: ', 'a Kalman filter predicts attitude from the gyro 100 times a second, and gravity corrects roll and '
                    'pitch. The gyro provides heading, and the laser checks it.'),
    ('Disturbances: ', 'gravity, because the table is slightly bottom-heavy, and air drag. The wheels can\'t get rid of '
                       'momentum, so these limit how long a hold can last.'),
])

s = slide('Theory: Control Law & Wheel Allocation')
ph = shape(s, 'Quaternion PD')
texts = [p.text for p in ph.text_frame.paragraphs]
para(ph, texts.index(next(x for x in texts if x.startswith(('4 wheels', 'Four wheels')))),
     'Four wheels sit in a 45° pyramid, so any three can still turn the table about all three axes: this makes a '
     'wheel failure survivable.')
para(ph, texts.index(next(x for x in texts if x.startswith(('Minimum-norm', 'The requested torque', 'The controller splits')))),
     'The controller splits the torque over the working wheels with the least total effort (u = −D⁺τ); with all four '
     'wheels, the spare one keeps every wheel spinning away from zero.')
para(ph, texts.index(next(x for x in texts if x.startswith(('Inner loop', 'Each wheel has')))),
     'Each wheel has its own speed controller, running 100 times a second, with a software current limit in place '
     'of a current sensor.')
para(ph, texts.index(next(x for x in texts if x.startswith('Physics limit'))),
     'Physics limit: wheels can only store momentum, never get rid of it, so any steady disturbance eventually fills '
     'them.')
lines(shape(s, ('Wheel J', 'Wheel inertia')), [
    'Wheel inertia 3.06×10⁻⁴ kg·m² (from CAD). The team measured the table inertia: 0.077 kg·m² in yaw, about '
    '0.15 kg·m² in roll and pitch.',
    'Raising the wheel speed limit from 60 to 80 rad/s added about a third more momentum storage.'])

s = slide('Code: Attitude Estimation')
labelled(shape(s, 'Predict'), [
    ('Predict: ', 'every gyro reading, minus its estimated bias, moves the attitude forward 100 times a second; the '
                  'uncertainty grows.'),
    ('Update: ', 'the direction of gravity corrects roll and pitch, except while the table accelerates.'),
    ('Multiplicative: ', 'the filter tracks small angle errors and folds them into the attitude, so the quaternion '
                         'stays valid.'),
    ('Heading: ', 'gravity says nothing about heading, so the 3-minute calibration fixes the gyro\'s heading bias.'),
    ('The fix: ', 'while the table turned, gravity updates nudged the heading, about 1% per turn. Removing the '
                  'vertical part of the correction stopped it.'),
    ('Result: ', 'a full turn now ends 0.05° from where it started, by laser.'),
])
callout(s, 'error after one full turn before', 'error after one full turn before the fix (either direction)')
callout(s, 'error after one full turn with', 'error after one full turn with the fix')

s = slide('Code: Control Law & Wheel Allocation')
labelled(shape(s, 'Law'), [
    ('Law: ', 'each term of the equation on the theory slide is one line of code, which runs 100 times a second.'),
    ('Gains: ', 'the team chose them for a 1.5 s response with no overshoot (ωₙ = 2 rad/s, ζ = 0.9).'),
    ('Allocation: ', 'the code splits the requested torque over the wheels with the least total wheel torque.'),
    ('Failure: ', 'the detector drops a dead wheel from the list; the same code then uses the other three.'),
    ('Spare wheel: ', 'the wheels can also spin against each other without turning the table; this keeps every wheel '
                      'away from zero speed.'),
    ('Inner loop: ', 'the torque becomes a wheel speed target that each motor\'s speed controller follows.'),
])
callout(s, 'wheels after a failure', 'wheels after a failure, with no change to the control code')
callout(s, 'largest heading error on three', 'largest heading error on three wheels over 30 s')

s = slide('Code: Wheel-Failure Detection')
labelled(shape(s, 'No current sensor'), [
    ('No current sensor: ', 'the firmware estimates motor current from the drive level and the wheel\'s speed.'),
    ('Healthy or dead: ', 'a motor that the firmware drives hard must speed up; a dead one does not.'),
    ('Coast, not brake: ', 'the firmware lets a failed wheel coast, because braking it would jolt the table.'),
    ('FAULT TEST: ', 'cuts a motor without telling the controller; the Teensy grades the response.'),
])
callout(s, 'to detect a failure', 'to detect a failure on the floating table (4 tests)')

s = slide('Hardware: System Block Diagram')
labelled(shape(s, 'Power'), [
    ('Power: ', 'the computer and each wheel have separate batteries, so motor surges can\'t reset it.'),
    ('Sensing: ', 'the IMU reports rotation, gravity and magnetic field 100 times a second.'),
    ('Data: ', 'the SD card logs every sample; commands and live data travel over the radio.'),
    ('Heading truth: ', 'a laser on the deck reads heading to ≈ 0.05° on a wall scale 2.48 m away.'),
])

s = slide('Data: IMU Data Integrity')
takeaway(s, 'Better IMU reading cut lost samples from 25% to under 0.1%')
mcen(s, MEASURE='The team logged the time between gyro samples with the IMU at rest, before and after the fixes.',
     COMPARE='A sample should arrive every 10 ms. Before the fixes a quarter went missing, and 38 gaps were long '
             'enough to shut the wheels down.',
     EXPLAIN='The sensor library kept only the last reading of each batch, and SD card writes stalled the main loop.',
     IMPROVE='The team wired the IMU\'s data-ready and reset lines, and the firmware now reads it during SD writes. '
             'Short pauses inside the sensor remain but do no harm.')

s = slide('Data: Magnetometer Yaw Aid')
takeaway(s, 'The magnetometer held heading within 1° on a clamped table, but not reliably')
mcen(s, MEASURE='With the table clamped, any heading change is an error. The team ran the wheels through 14 spin '
                'patterns for almost an hour.',
     COMPARE='The requirement is 1°. The gyro alone drifts almost 6° per hour on the same data.',
     EXPLAIN='The first run kept heading within 1° on 36 of 37 steps. A second run failed, because the wheels\' '
             'magnetic fields had shifted between sessions.',
     IMPROVE='The presentation room bends the magnetic field by up to 14°, so the demo uses the gyro alone, and the '
             'laser checks it.')

s = slide('Data: Balancing the Table')
takeaway(s, 'Trimming cut the rest tilt from 12° to under 1°; a higher balance was unstable')
mcen(s, MEASURE='After each trim move the team measured the table\'s rest angle and its swing period, which gives '
                'the height of the centre of mass.',
     COMPARE='Holding heading only needs the table to rest within 5° of level. Tilted slews need the centre of mass '
             'within about 1 mm of the pivot.',
     EXPLAIN='Moving washers brought the rest tilt under 1°. Raising the centre of mass to 3 mm below the pivot made '
             'trims overshoot, and the table tipped over twice.',
     IMPROVE='The team returned to a stable, bottom-heavy setup for the demo. Tilted targets need a finer balance: '
             'future work.')

s = slide('Data: Heading Holds by Laser')
takeaway(s, 'After warm-up the heading stayed within 0.25° for 5 minutes, by laser')
mcen(s, MEASURE='The floating table held its heading while the team read the laser every 30 to 60 seconds.',
     COMPARE='The requirement is no visible jitter or drift. The gyro\'s tested drift predicts about 0.25° in 5 '
             'minutes.',
     EXPLAIN='After warm-up the heading stayed within 0.25° with no trend: a pass. A calibration too soon after '
             'power-on drifted almost 1° in 2.5 minutes.',
     IMPROVE='The team now warms up for 10 minutes and calibrates for 3 minutes before every run. The demo hold then '
             'stayed within 0.04°.')

s = slide('Data: Estimator Heading Leak')
takeaway(s, 'The estimator lost about 1% of every turn; only the laser saw it. Fixed: 0.05° per turn')
mcen(s, MEASURE='After each full-turn slew the team compared the laser with the gyro and the estimate.',
     COMPARE='Every run passed by the computer\'s own measure: the estimate always said the table ended where it '
             'started.',
     EXPLAIN='The laser showed errors up to 2.3°, flipping sign with the turn direction. The gyro measured the turn '
             'correctly; the filter\'s gravity correction took some of it back.',
     IMPROVE='The team stopped gravity from changing the heading and refit the gyro\'s scale from the logs. A full '
             'turn now ends 0.05° off.')

s = slide('Data: First-Run Start Failures')
takeaway(s, 'Every first run after power-up failed: the SD card froze the wheel loop, not a wheel')
mcen(s, MEASURE='The team examined encoder, speed and drive data for the first 2 s after arming.',
     COMPARE='The wheels should spin up smoothly; instead, all seven first runs marked every wheel dead.',
     EXPLAIN='Creating the log file froze the program for almost a second, so the checks saw one huge jump.',
     IMPROVE='The firmware now creates the log file before arming and skips such freezes; the next try passed.')

s = slide('DEMO ALL: The Whole Run')
takeaway(s, 'One command ran the whole demonstration, and the table graded each part itself')
mcen(s, MEASURE='This plot shows the full demo run on 11-01, started right after a power-up.',
     COMPARE='The computer graded each part against its requirement; the team read the laser after each part.',
     EXPLAIN='All three parts passed: a 60 s hold, four slews through a full turn, then a wheel failure.',
     IMPROVE='One continuous run avoids stopping between tests, which had jolted the table before.')

s = slide('DEMO ALL: Requirement 2')
takeaway(s, 'A full turn of slews ended 0.05° off by laser; the table held every target within 1°')
mcen(s, MEASURE='The team read the laser before and after the four slews; the firmware recorded the worst heading '
                'error at each target.',
     COMPARE='The requirement is 1°. Before the estimator fix this test ended up to 2.3° off; the simulation '
             'predicted almost no error.',
     EXPLAIN='This is a pass: 0.05° after a full turn, and the table held each target within 0.13°.',
     IMPROVE='The estimator fix made the difference: the old estimator, run on the same data, ends 0.46° off.')

s = slide('DEMO ALL: Requirement 3')
takeaway(s, 'The demo cut wheel B without warning: caught in 0.5 s, three wheels held on')
mcen(s, MEASURE='The demo cut wheel B\'s motor without telling the controller; the log shows attitude and wheel '
                'speeds.',
     COMPARE='To pass: detect within 1 s and keep the attitude within 1°. The simulation predicted 0.6–0.8 s.',
     EXPLAIN='A pass: the firmware caught it in 0.5 s; the error peaked at 0.9°, then stayed within 0.36°.',
     IMPROVE='This pass came on the first run after power-up, which always failed before the start-up fix.')

s = slide('Final Results & Limitations')
lines(shape(s, 'Req. 1'), ['Req. 1: heading held (laser)', 'Req. 2: 0.05° after a full turn',
                           'Req. 3: failure caught in 4 tests', 'Stretch: four headings reached',
                           'One command runs the whole demo', 'The team fixed the heading leak',
                           'The team fixed the start-up failures'])
lines(shape(s, 'The wheel loop pauses'), ['The wheel loop pauses briefly during SD card writes',
                                          'Wheel D\'s encoder wire needs soldering', 'The motor switch wires are loose',
                                          'Air drag fills the wheels in about 5 minutes',
                                          'The room distorts the magnetic field too much to use'])
lines(shape(s, 'Tilted targets need'), ['Tilted targets need a finer balance',
                                        'The wheel loop belongs on a timer interrupt',
                                        'Dumping momentum needs thrusters or magnetic torquers'])

s = slide('Lessons Learned')
t = next(sh for sh in s.shapes if sh.has_table).table
for i, (a, b) in enumerate([
        ('Check results against the laser, and extend it beyond heading, rather than trusting the estimator',
         'The estimator said 0.1° while the table was 2.3° off'),
        ('Warm up for 10 minutes and calibrate for 3 minutes before heading work',
         'An early calibration drifted about 15° per hour'),
        ('Keep the SD card out of the control loop', 'SD card writes froze the wheel loop'),
        ('Balance with a rigid, repeatable weight layout before trying tilted slews',
         'Loose weights made trims unpredictable')], start=1):
    cell(t, i, 1, a)
    cell(t, i, 2, b)
box = shape(s, 'Demo checklist')
for i, txt in enumerate(['Check the motor switch and battery wires before arming',
                         'Power on, warm up 10 minutes, then calibrate for 3 minutes with the table clamped',
                         'Run the full demo and read the laser after each part'], start=1):
    para(box, i, txt)
box = shape(s, 'Improvements beyond')
for i, txt in enumerate(['The team used the air bearing table', 'The firmware detects wheel failures on the table',
                         'One command runs and grades the whole demo', 'A laser gives heading truth; logs replay on the PC'],
                        start=1):
    para(box, i, txt)

s = slide('Future Work')
labelled(shape(s, 'Embedded MPC'), [
    ('Embedded MPC: ', 'replace the PD controller with a model-predictive controller (TinyMPC) on the Teensy, 100 Hz.'),
    ('Constraints built in: ', 'it would respect wheel speed and torque limits, and re-plan on three wheels after a '
                               'failure.'),
    ('Robustness: ', 'compare it with today\'s controller under inertia errors, disturbances and a live wheel failure.'),
    ('Response time: ', 'compare with today\'s 1.2–1.7 s rise, and check it solves within each 10 ms step.'),
])
labelled(shape(s, 'Moving masses'), [
    ('Moving masses: ', 'motorized weights move the centre of mass to the pivot while the table floats.'),
    ('Closed loop: ', 'the gravity torque the wheels absorb shows how far off the balance is; the masses move until it '
                      'is zero.'),
    ('Removes the pendulum: ', 'today the centre of mass sits about 10 mm below the pivot, so only heading slews work.'),
    ('Unlocks: ', 'tilted targets and full rotation in all three axes.'),
])
labelled(shape(s, 'Camera + AprilTags'), [
    ('Camera + AprilTags: ', 'printed markers at known spots in the room act like stars for a camera on the table.'),
    ('Absolute attitude: ', 'a full 3-axis fix that stops gyro drift and replaces the magnetometer, which the room '
                            'disturbs.'),
    ('Filter update: ', 'the estimator would use each marker fix alongside the gyro and accelerometer.'),
    ('Independent truth: ', 'it checks roll and pitch too; the laser only checks heading.'),
])

# ================================================================ backup slides
s = slide('Code: Magnetometer Heading Update')
labelled(shape(s, 'Measurement'), [
    ('Measurement: ', 'the horizontal field\'s direction, compared with a reference the aid learns in place, gives the '
                      'heading error.'),
    ('Heading only: ', 'only a turn about the vertical changes the measured heading.'),
    ('Slow correction: ', 'the filter corrects heading over about 25 s, so the gyro still handles fast motion.'),
    ('Gain edit: ', 'a disturbed field can never tilt roll or pitch.'),
    ('Bias: ', 'heading lets the filter estimate the gyro\'s heading bias, which gravity cannot.'),
    ('Gating: ', 'the aid only uses the field when the wheels are in a known state: stopped where learned, or spinning.'),
    ('References: ', 'the aid learns them in place again, since predicting them failed; it is off at the presentation '
                     'spot.'),
])
s = slide('Code: DEMO ALL Sequencer')
labelled(shape(s, 'One command'), [
    ('One command: ', 'requirement 1, then requirement 2 and the stretch, then requirement 3, in order.'),
    ('No wrong zero: ', 'the slews measure their targets from the heading the hold actually kept.'),
    ('No jolt: ', 'nothing stops between parts, so the wheels keep their momentum.'),
    ('Self-grading: ', 'the Teensy grades every part and prints one summary line.'),
    ('Any stop: ', 'it reports which part it stopped in.'),
])
s = slide('Hardware: Laser Heading Truth')
labelled(shape(s, 'Scale'), [
    ('Scale: ', 'the team printed it for a wall 2.48 m away, with 0.5° marks out to ±5° and 0.1° marks within ±1°.'),
    ('Use: ', 'line the dot up at zero before a run, and read it at the end of each part.'),
    ('Limits: ', 'it measures heading only, to about 0.05° (a 2 mm reading); the accelerometer provides roll and '
                 'pitch.'),
])
takeaway(slide('Simulation: Methods'), 'In simulation the estimate stayed within 0.1° in roll and pitch')
takeaway(slide('Simulation: Wheel Failure'), 'In simulation the firmware caught a dead motor within 1 s; three wheels '
                                             'held 1°')
takeaway(slide('Simulation: 4-Attitude Slew'), 'The slew logic meets 1° on a balanced table; today\'s balance blocks '
                                               'tilted targets')
callout(slide('Data: SLEW TILT'), 'lowest CG we could hold', 'lowest centre of mass the team could hold repeatably; '
                                                             'trims landed 2–4× off')

BACKUP = {
    'Data: Wheel Speed Control': (
        'Wheels B and D hold speed within 1 rpm; hardware problems hurt A and C',
        dict(MEASURE='The team commanded all four wheels from 0 up to 480 rpm and back, and compared commanded and '
                     'measured speed.',
             COMPARE='With the same controller settings on every wheel, all four should track closely.',
             EXPLAIN='Wheel A rubs and wheel C oscillates by 10–16 rpm. Static friction stops every wheel below about '
                     '46 rpm.',
             IMPROVE='Next: fix wheel A\'s rub, add a friction correction, and repeat the test.')),
    'Data: Attitude Determination at Rest': (
        'Roll and pitch beat the requirement 50 times over; a better calibration fixed yaw drift',
        dict(MEASURE='For 37 minutes, with the table fixed and the motors off, the team logged the estimated attitude.',
             COMPARE='The requirement is 1°. A 3-second gyro calibration predicts a yaw drift of about 26° per hour.',
             EXPLAIN='Roll and pitch varied by only 0.02°. Yaw drifted 27° per hour, as predicted; a longer '
                     'calibration after warm-up gives about 3° per hour.',
             IMPROVE='The team now always warms up for 10 minutes and calibrates for 3; the laser holds confirm it.')),
    'Data: Magnetometer vs. Wheel Motors': (
        'Spinning wheels add noise a filter removes; stopped wheels leave random offsets',
        dict(MEASURE='With the table fixed, the team recorded the magnetometer heading while stepping the wheels '
                     'through different speeds.',
             COMPARE='A heading aid must stay steady to well under 1°. With the motors off the noise was 0.7°, with no '
                     'drift.',
             EXPLAIN='Spinning wheels add 5–8° of fast noise that a filter removes. A stopped wheel leaves an offset '
                     'of up to 10°, depending on where its shaft stops.',
             IMPROVE='The team built an aid that only trusts the field when the wheels are in a known state (MAG '
                     'TEST).')),
    'Data: Magnetometer at the Presentation Spot': (
        'The demo room bends the magnetic field up to 14°; the gyro alone does better',
        dict(MEASURE='The team turned the table by hand at the test spot and at the presentation spot, and compared '
                     'the magnetic heading with the true heading.',
             COMPARE='The aid needs field errors well under 1°. The test spot showed errors under 2° with the wheels '
                     'stopped.',
             EXPLAIN='At the presentation spot the errors reached 4° with the wheels stopped and 14° spinning. A room '
                     'correction failed the cross-check.',
             IMPROVE='The firmware now starts with the aid off; the gyro provides heading, and the laser checks it.')),
    'Data: First DEMO SPIN': (
        'The table behaves like a pendulum: gravity overwhelmed the wheels within 1–2 seconds',
        dict(MEASURE='The team tried the spin demo three times on the floating table and logged tilt and wheel speeds.',
             COMPARE='In simulation, with the centre of mass at the pivot, the table levelled in about 11 s. On the '
                     'table the wheels maxed out in 1–2 s.',
             EXPLAIN='The table rested 3° off level and swung back every 1.8 s: gravity pushed harder than the wheels '
                     'could absorb.',
             IMPROVE='The team added weights low on the table and trimmed it side to side (next slide).')),
    'Data: DEMO SPIN Leveling': (
        'Weights made gravity\'s pull three times weaker and fixed pitch, but roll still rested 3.3° off',
        dict(MEASURE='The team made six more spin attempts after adding weights, logging roll, wheel speeds and the '
                     'free swing.',
             COMPARE='The table should level within 1° and then spin. Every attempt maxed out a wheel after 2–4 s.',
             EXPLAIN='The swing slowed from 1.8 to 3.3 s, but one side was still heavy, so holding level filled wheel '
                     'A in about 3 s.',
             IMPROVE='The team trimmed the roll balance and wrote DEMO LEVEL, which holds the natural rest tilt.')),
    'Data: DEMO LEVEL Hold': (
        'After the roll trim the table levels in 5–6 s and holds steady until gravity fills a wheel',
        dict(MEASURE='The team ran DEMO LEVEL five times: level the table, then hold until a wheel reaches its limit.',
             COMPARE='The table should hold with no visible jitter; how long depends on wheel storage against '
                     'leftover gravity.',
             EXPLAIN='Roll and pitch held within 0.07° and heading within 0.5° for up to 16 s, until leftover gravity '
                     'filled wheel B.',
             IMPROVE='DEMO LEVEL now holds the table at its natural rest tilt; later balance sessions improved it.')),
    'Data: Flight Computer Failure': (
        'The Teensy\'s 3.3 V supply shorted; the team had a spare running within a day',
        dict(MEASURE='The board died after a test (no USB, no 3.3 V). The team installed the spare and re-tested every '
                     'wheel.',
             COMPARE='Each wheel should reach about 45 rad/s at 30% drive and start turning at 8–10% drive.',
             EXPLAIN='A short on the old board itself killed its 3.3 V supply; the team found no wiring cause. All '
                     'wheels passed on the spare; wheel D\'s encoder wire was loose.',
             IMPROVE='The spare runs everything. Its USB and battery inputs are still joined, so the team uploads with '
                     'the batteries off.')),
    'Data: False Dead-Motor Flags': (
        'Friction made healthy stopped wheels look dead; a higher threshold fixed it',
        dict(MEASURE='The team examined all 230 wheel starts from rest on the floating table, and the false failure '
                     'flags.',
             COMPARE='A dead motor never speeds up under hard drive; a healthy one normally breaks free at 10–15% '
                     'drive.',
             EXPLAIN='Static friction held 14 healthy starts until 23–30% drive, past the old 20% threshold. Most were '
                     'wheel D, which sticks.',
             IMPROVE='The firmware now judges a stopped wheel only above 40% drive; replaying both logs gives no false '
                     'flags.')),
    'Data: STEP TEST 1': (
        'The table\'s own magnetic field fooled the aid: the table turned 8° unnoticed',
        dict(MEASURE='The team stepped the heading by ±5° on the floating table; a gyro-only replay gives the true '
                     'heading.',
             COMPARE='The estimate (blue) followed the steps exactly, and the table (red) should have too.',
             EXPLAIN='The table carries its own magnetic field, about twice Earth\'s, so the magnetometer pulled the '
                     'heading toward a wrong reference.',
             IMPROVE='The team measured that field from a full turn, subtracted it, and repeated the test (next '
                     'slide).')),
    'Data: STEP TEST (Heading Steps)': (
        'Heading steps settle in 1.2–1.7 s with at most 4% overshoot',
        dict(MEASURE='The team repeated the ±5° heading steps without the table\'s magnetic field, 20 s per step; the '
                     'Teensy graded them.',
             COMPARE='The team designed the controller for a 1.5 s rise with no overshoot.',
             EXPLAIN='Rise times were 1.2–1.7 s with up to 4% overshoot and no steady error. The momentum the wheels '
                     'exchange gives the yaw inertia: 0.077 kg·m².',
             IMPROVE='The firmware now uses the measured inertia; the controller settings did not need to change.')),
    'Data: FAULT TEST on the Air Bearing': (
        'First failure test on the table: the firmware caught wheel B in 0.91 s',
        dict(MEASURE='During a heading hold the test cut wheel B\'s motor without telling the controller.',
             COMPARE='To pass: detect within 1 s, return within 1° in 10 s, and stay there for 30 s.',
             EXPLAIN='It passed: the firmware caught it in 0.91 s, the error peaked at 0.8°, and wheels A, C and D '
                     'took over B\'s share.',
             IMPROVE='The team repeated it inside the full demo (caught in 0.5 s); a real failure follows.')),
    'Data: Wheel D Encoder Lead': (
        'A real failure: wheel D\'s encoder wire lost contact, and the detector switched D off',
        dict(MEASURE='During a test the firmware drove wheel D to its limit while its encoder showed no movement.',
             COMPARE='A healthy wheel under that much drive speeds up within a fraction of a second.',
             EXPLAIN='The wire worked when turned by hand but opened under vibration. Wheels A and C took over D\'s '
                     'share.',
             IMPROVE='The team reseated and strain-relieved the wire, and will replace it with soldered wires.')),
    'Data: Tilted Slews vs. Gravity': (
        'Holding a tilt away from rest takes steady torque, so tilted targets fill a wheel in seconds',
        dict(MEASURE='The team ran the four-attitude slew with a fixed roll and pitch on the table, and the same '
                     'targets in simulation.',
             COMPARE='The simulation reached the fourth target before any wheel hit its limit.',
             EXPLAIN='On the table a wheel hit its limit after 13.5 s: every degree away from the rest tilt costs '
                     'steady torque.',
             IMPROVE='The team wrote SLEW YAW, which changes only heading and lets the table rest at its natural '
                     'tilt.')),
    'Data: SLEW YAW on the Air Bearing': (
        'Four heading targets on the shortest path, all within 1°; table and simulation agree',
        dict(MEASURE='The table turned to +45°, +135°, −135° and 0° from its start, resting at its natural tilt; the '
                     'simulation ran the same targets.',
             COMPARE='The table must reach each target and hold it for 5 s within 1°; the Teensy grades it.',
             EXPLAIN='All four passed, with heading errors under 0.6°. A laser check on 11-01 found 0.1° after a full '
                     'turn.',
             IMPROVE='Later laser runs disagreed by up to 2.3°, which led the team to the estimator heading leak.')),
    'DEMO ALL: Requirement 1': (
        'The table held its heading for 60 s within 0.04°, and the laser dot did not move',
        dict(MEASURE='The firmware recorded the heading error during the graded 60 s hold; the team read the laser.',
             COMPARE='The requirement is an error under 1° with no visible jitter or drift.',
             EXPLAIN='This is a pass: the largest error was 0.04°, jitter 0.01°, and no measurable drift.',
             IMPROVE='Air drag slowly fills the wheels, which limits holds to about 5 minutes.')),
}
for title, (tk, parts) in BACKUP.items():
    s = slide(title)
    takeaway(s, tk)
    mcen(s, **parts)

s = slide('Data: SLEW TILT')
takeaway(s, 'Tilted targets work in simulation, but only near a balance the team could not hold')
labelled(shape(s, 'Method'), [
    ('Method: ', 'scan the rest tilt over a full turn, set targets relative to it, add gravity compensation, and tilt '
                 'against gravity between targets to empty the wheels.'),
    ('Found: ', 'a small gyro bias made the controller push steadily on the wheels; a slow momentum leak fixed it.'),
    ('Next step: ', 'rigid trim weights and a centre of mass within 1 mm of the pivot, then SLEW TILT (firmware ready).'),
])

prs.save(OUT)
print('saved', OUT)
