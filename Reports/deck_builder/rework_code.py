"""Code section rework (2026-11-02): the presented code slides become
    Code: Attitude Estimation (MEKF)        (was Code: Gravity Update Fix; now the whole filter, fix included)
    Code: Control Law & Wheel Allocation    (moved up from backup, with the torque -> speed hand-off)
    Code: Wheel-Failure Detection           (unchanged)
and the backup magnetometer code slide is renamed so the two estimation slides don't share a name.
Applied to the team's current file:
    python Reports/deck_builder/rework_code.py <in.pptx> <out.pptx>"""
import copy, sys
from pptx import Presentation
from pptx.util import Inches
from pptx.oxml.ns import qn

SRC, OUT = sys.argv[1], sys.argv[2]
prs = Presentation(SRC)
CODE_INK, CODE_COMMENT, GREEN = '1F2933', '2E7D4F', '2E7D4F'
LINE_IN_13PT = 0.213  # measured on the rendered slides: Courier New 13 pt, 100% spacing


def slide_by_title(start):
    for s in prs.slides:
        if s.shapes.title is not None and s.shapes.title.text.startswith(start):
            return s
    raise KeyError(start)


def box(slide, start):
    return next(sh for sh in slide.shapes if sh.has_text_frame and sh.text_frame.text.startswith(start))


def set_title(slide, text):
    p = slide.shapes.title.text_frame.paragraphs[0]
    p.runs[0].text = text
    for r in p.runs[1:]:
        r._r.getparent().remove(r._r)


def set_runs(shape, text):
    """Replace a one-paragraph text box's text, keeping its first run's format."""
    p = shape.text_frame.paragraphs[0]
    p.runs[0].text = text
    for r in p.runs[1:]:
        r._r.getparent().remove(r._r)


def _run(template_r, text, color, size=None):
    r = copy.deepcopy(template_r)
    r.find(qn('a:t')).text = text
    rpr = r.find(qn('a:rPr'))
    rpr.find(qn('a:solidFill')).find(qn('a:srgbClr')).set('val', color)
    if size:
        rpr.set('sz', str(size))
    return r


def set_code(shape, bg, lines, size=1300):
    """lines: (code, comment) pairs; the comment starts at column 30 (or after one space)."""
    body = shape.text_frame._txBody
    ps = body.findall(qn('a:p'))
    tp = copy.deepcopy(ps[0])
    tr = copy.deepcopy(tp.find(qn('a:r')))
    for p in tp.findall(qn('a:r')):
        tp.remove(p)
    for p in ps:
        body.remove(p)
    for code, comment in lines:
        p = copy.deepcopy(tp)
        if comment and code:
            code = code.ljust(29) + ' '
        if code or not comment:
            p.append(_run(tr, code or ' ', CODE_INK, size))
        if comment:
            p.append(_run(tr, comment, CODE_COMMENT, size))
        body.append(p)
    h = 0.2 + len(lines) * LINE_IN_13PT * size / 1300
    shape.height = Inches(h)
    bg.height = Inches(h + 0.1)
    return shape.top + shape.height


def set_bullets(shape, items):
    """items: (label, text[, color]) per bullet; the template is the first paragraph's two runs."""
    body = shape.text_frame._txBody
    ps = body.findall(qn('a:p'))
    tp = copy.deepcopy(next(p for p in ps if len(p.findall(qn('a:r'))) >= 2))  # label + text runs
    rs = tp.findall(qn('a:r'))
    lab_r, txt_r = copy.deepcopy(rs[0]), copy.deepcopy(rs[1])
    for r in rs:
        tp.remove(r)
    for p in ps:
        body.remove(p)
    for it in items:
        label, text = it[0], it[1]
        color = it[2] if len(it) > 2 else CODE_INK
        p = copy.deepcopy(tp)
        p.append(_run(lab_r, label + ': ', CODE_INK))
        p.append(_run(txt_r, text, color))
        body.append(p)


def set_notes(slide, text):
    slide.notes_slide.notes_text_frame.text = text


def code_parts(slide):
    code = next(sh for sh in slide.shapes if sh.has_text_frame and sh.text_frame.text.startswith('//'))
    bg = next(sh for sh in slide.shapes if sh.name.startswith('Rounded Rectangle') and abs(sh.top - Inches(1.0)) < Inches(0.05))
    cap = box(slide, 'Excerpt')
    return code, bg, cap


# ------------------------------------------------------------------ Code: Attitude Estimation (MEKF)
est = slide_by_title('Code: Gravity Update Fix')
set_title(est, 'Code: Attitude Estimation (MEKF)')
code, bg, cap = code_parts(est)
bottom = set_code(code, bg, [
    ('', '// attitude_ekf.cpp: predict, each gyro sample'),
    ('w = gyro - b;', '// bias removed'),
    ('q = exp(-w dt) (x) q;', '// 100 Hz'),
    ('P = Phi P Phi^T + Q;', '// P grows'),
    ('', ''),
    ('', '// update, each accelerometer sample'),
    ('if (||a| - g| > gate) return;', '// accelerating'),
    ('y = a / |a|;', '// measured "up"'),
    ('yh = A * ez;', '// predicted "up"'),
    ('H = [-[yh]x, 0];  r = y - yh;', ''),
    ('K = P H^T (H P H^T + R)^-1;', ''),
    ('K[5][*] = 0;', '// z bias locked'),
    ('for (j = 0; j < 3; j++)', '// NEW: off vertical'),
    ('  K[0..2][j] -= (yh.K[0..2][j]) yh;', ''),
    ('q = exp(K[0..2] r) (x) q;', '// tilt only'),
    ('b += K[3..5] r;', '// x/y gyro bias'),
    ('', ''),
    ('', '// main.cpp: yaw gyro scale (laser fit)'),
    ('gyro.z *= 1.0045;', '// was 1.009'),
], size=1250)
cap.top = bottom + Inches(0.03)
set_runs(cap, 'Excerpts, simplified from attitude_ekf.cpp and main.cpp')
set_bullets(box(est, 'Symptom'), [
    ('Predict', 'the gyro, minus its bias, turns the quaternion at 100 Hz; the uncertainty P grows'),
    ('Update', 'gravity\'s direction corrects roll and pitch; skipped while the table accelerates'),
    ('Multiplicative', '6 error states (3 angles, 3 gyro biases) folded into q, so q stays a unit quaternion'),
    ('Heading', 'gravity can\'t see it: the z bias is locked at the 180 s calibration'),
    ('Fix (11-01)', 'turning couples yaw and tilt errors in P, so gravity updates were turning the heading '
                    '≈ 1% per turn. Gain projected off vertical; yaw scale refit by log replay'),
    ('Result', '0.05° after 360° (laser, DEMO ALL)', GREEN),
])
set_notes(est, 'This is the estimator the whole demo runs on: a multiplicative EKF with six error states, three '
               'attitude angles and three gyro biases. The gyro predicts at 100 Hz; the accelerometer measures which way '
               'is up and corrects roll and pitch. Gravity carries no heading, so heading comes from the gyro and its z '
               'bias is fixed by the calibration. The one line marked NEW is the fix from the heading-leak data slide: while '
               'the table turns, the filter\'s yaw and tilt errors become correlated, and the standard gain then lets a '
               'gravity update rotate the heading. Removing the gain\'s component along vertical stops that; with it the '
               'gyro scale was refit against the laser.')

# ------------------------------------------------------------------ Code: Control Law & Wheel Allocation
ctl = slide_by_title('Code: Control Law & Wheel Allocation')
code, bg, cap = code_parts(ctl)
bottom = set_code(code, bg, [
    ('', '// attitude_ctrl.cpp: quaternion PD + FF'),
    ('Quat qe = q (x) conj(q_ref);', '// attitude error'),
    ('Vec3 th = -2 * vec(qe);', '// error angle'),
    ('tau = -Kp*th - Kd*(w - w_ref)', '// PD'),
    ('      + I*a_ref', '// feed-forward'),
    ('      + w x (I*w + h_wheels);', '// gyroscopic'),
    ('', ''),
    ('', '// wheel_geometry.h: min-norm allocation'),
    ('M = sum(a_i a_i^T);', '// enabled only'),
    ('y = inverse(M) * tau;', '// any 3 span xyz'),
    ('u_i = -(a_i . y);', '// wheel torques'),
    ('', '// null mode n = [+1,-1,+1,-1]: no torque'),
    ('if (all 4) u_i += J k (bias - s_null) n_i;', ''),
    ('', ''),
    ('', '// main.cpp: torque -> wheel speed setpoint'),
    ('wheelCmd_i += u_i / J * dt;', '// then 100 Hz PI'),
])
cap.top = bottom + Inches(0.03)
set_runs(cap, 'Excerpts, simplified from attitude_ctrl.cpp, wheel_geometry.h, main.cpp')
set_bullets(box(ctl, 'τ ='), [
    ('Law', 'one line of code per term of the theory slide\'s equation, every gyro sample (100 Hz)'),
    ('Gains', 'Kp = Iωₙ², Kd = 2ζIωₙ; ωₙ = 2 rad/s, ζ = 0.9: ≈ 1.5 s rise, no overshoot'),
    ('Allocation', 'u = −D⁺τ: the least total wheel torque that makes τ'),
    ('Failure', 'a dead wheel is only removed from the enabled set; the same code then allocates over 3 '
                '(any 3 pyramid wheels span all axes, Req. 3)'),
    ('Null mode', 'wheels spin against each other with zero body torque; keeps each ≥ 100 rpm from zero during holds'),
    ('Inner loop', 'torque integrates into a speed setpoint; 100 Hz PI with a back-EMF current limit'),
])
# Stat callouts, copied from the wheel-failure slide so the code slides match.
src = slide_by_title('Code: Wheel-Failure Detection')
tree = ctl.shapes._spTree
next_id = max(int(e.get('id')) for e in tree.iter(qn('p:cNvPr'))) + 1
added = []
for sh in src.shapes:
    if sh.is_placeholder or sh.top < Inches(5.0):
        continue
    el = copy.deepcopy(sh._element)
    el.find('.//' + qn('p:cNvPr')).set('id', str(next_id))
    next_id += 1
    tree.append(el)
    added.append(el)
calls = [sh for sh in ctl.shapes if sh._element in added and sh.has_text_frame and sh.text_frame.text]
calls.sort(key=lambda s: (s.left, s.top))
set_runs(calls[0], '4 → 3')
set_runs(calls[1], 'wheels after a failure: same allocation code, one flag')
set_runs(calls[2], '0.36°')
set_runs(calls[3], 'max yaw error on 3 wheels for 30 s (DEMO ALL)')
set_notes(ctl, 'The control law is one line of code per term of the equation on the theory slide: PD on the quaternion '
               'error, feed-forward of the reference acceleration, and the gyroscopic term. Allocation is a pseudo-inverse '
               'over whichever wheels are enabled, which is why losing a wheel needs no new code path: the detector clears '
               'one flag and the next step allocates over three. With all four wheels the extra degree of freedom, the null '
               'mode, spins the wheels against each other with zero net torque; we use it to keep every wheel away from zero '
               'speed during holds. The torque becomes a wheel-speed setpoint, which a 100 Hz PI loop on each motor follows.')

# Move it into the presented section, right after the estimation slide.
ids = prs.slides._sldIdLst
order = list(ids)
el_ctl = order[[s.slide_id for s in prs.slides].index(ctl.slide_id)]
el_est = order[[s.slide_id for s in prs.slides].index(est.slide_id)]
ids.remove(el_ctl)
el_est.addnext(el_ctl)

# ------------------------------------------------------------------ backup magnetometer slide + cross-references
set_title(slide_by_title('Code: Heading Update (Estimation)'), 'Code: Magnetometer Heading Update')
for s in prs.slides:
    for sh in s.shapes:
        if sh.has_text_frame:
            for p in sh.text_frame.paragraphs:
                for r in p.runs:
                    if 'Code: Gravity Update Fix' in r.text:
                        r.text = r.text.replace('Code: Gravity Update Fix', 'Code: Attitude Estimation')
    if s.has_notes_slide:
        for p in s.notes_slide.notes_text_frame.paragraphs:
            for r in p.runs:
                if 'Code: Gravity Update Fix' in r.text:
                    r.text = r.text.replace('Code: Gravity Update Fix', 'Code: Attitude Estimation')

prs.save(OUT)
print('saved', OUT)
