"""Final presentation: apply the prelim feedback and the new progress to the team's deck (in place)."""
import copy, json, os, sys
from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE
from pptx.oxml.ns import qn
from lxml import etree

ROOT = r'C:\Users\C27Derek.Weissinger\PlatformIO\Projects\Astro 331 Lab 8'
SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'current.pptx')  # the team's file as saved 2026-10-30 23:55
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, 'Reports', 'Astro 331 Lab 8_2_3 Body Final Pres.pptx')
FIG = os.path.join(ROOT, 'Reports', 'figures')
MT = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'magtest_summary.json')))

FONT = 'Quattrocento Sans'
CODEFONT = 'Courier New'
INK = RGBColor(0x1F, 0x29, 0x33)
DARK = RGBColor(0x40, 0x40, 0x40)
MUTED = RGBColor(0x61, 0x6E, 0x7C)
GREEN = RGBColor(0x2E, 0x7D, 0x4F)
AMBER = RGBColor(0xB7, 0x79, 0x0F)
RED = RGBColor(0x9C, 0x2F, 0x25)
BLUE = RGBColor(0x2F, 0x5D, 0x8A)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)
TINT_G = RGBColor(0xE8, 0xF3, 0xEC)
TINT_B = RGBColor(0xE9, 0xEF, 0xF6)
TINT_GREY = RGBColor(0xF3, 0xF4, 0xF6)
TINT_A = RGBColor(0xFB, 0xF1, 0xDE)

prs = Presentation(SRC)
orig = list(prs.slides)
LAYOUT = prs.slide_layouts[1]  # OBJECT: title + body + slide number
NUM_XML = next(sh._element for sh in orig[2].shapes if sh.is_placeholder and sh.placeholder_format.idx == 12)


# ---------------------------------------------------------------- helpers
def new_slide(title, notes=None):
    s = prs.slides.add_slide(LAYOUT)
    for ph in list(s.placeholders):
        if ph.placeholder_format.idx != 0:
            ph._element.getparent().remove(ph._element)
    s.shapes.title.text = title
    s.shapes._spTree.append(copy.deepcopy(NUM_XML))
    if notes:
        s.notes_slide.notes_text_frame.text = notes
    return s


def _para_fmt(p, bullet=False, level_indent=300000, space_before=4, line=95, align=None):
    pPr = p._p.get_or_add_pPr()
    for tag in ('a:buChar', 'a:buNone', 'a:buAutoNum', 'a:lnSpc', 'a:spcBef'):
        for el in pPr.findall(qn(tag)):
            pPr.remove(el)
    ln = etree.SubElement(pPr, qn('a:lnSpc')); etree.SubElement(ln, qn('a:spcPct')).set('val', str(line * 1000))
    sb = etree.SubElement(pPr, qn('a:spcBef')); etree.SubElement(sb, qn('a:spcPts')).set('val', str(int(space_before * 100)))
    if bullet:
        pPr.set('marL', str(level_indent)); pPr.set('indent', '-228600')
        etree.SubElement(pPr, qn('a:buChar')).set('char', '•')
    else:
        pPr.set('marL', '0'); pPr.set('indent', '0')
        etree.SubElement(pPr, qn('a:buNone'))
    if align is not None:
        p.alignment = align


def _run(p, text, size=15, bold=False, color=INK, font=FONT, italic=False):
    r = p.add_run()
    r.text = text
    f = r.font
    f.size = Pt(size); f.bold = bold; f.italic = italic; f.name = font
    f.color.rgb = color
    return r


def text(slide, x, y, w, h, paras, size=15, anchor=MSO_ANCHOR.TOP, fill=None, margin=0.08, name=None, autofit=False):
    """paras: list of items; each item is a str, or dict(runs=[(text, {opts})], bullet, align, sb)."""
    box = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    if name:
        box.name = name
    tf = box.text_frame
    tf.word_wrap = True
    tf.vertical_anchor = anchor
    for side in ('left', 'right', 'top', 'bottom'):
        setattr(tf, f'margin_{side}', Inches(margin))
    if fill is not None:
        box.fill.solid(); box.fill.fore_color.rgb = fill
    for i, item in enumerate(paras):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        if isinstance(item, str):
            item = dict(runs=[(item, {})])
        _para_fmt(p, bullet=item.get('bullet', False), space_before=item.get('sb', 4), align=item.get('align'))
        for t, o in item['runs']:
            _run(p, t, size=o.get('size', item.get('size', size)), bold=o.get('b', False), color=o.get('color', INK),
                 font=o.get('font', FONT), italic=o.get('i', False))
    return box


def bullets(slide, x, y, w, h, items, size=15, sb=6, **kw):
    paras = []
    for it in items:
        runs = it if isinstance(it, list) else [(it, {})]
        paras.append(dict(runs=runs, bullet=True, sb=sb))
    return text(slide, x, y, w, h, paras, size=size, **kw)


def rect(slide, x, y, w, h, fill, line=None, shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.06):
    s = slide.shapes.add_shape(shape, Inches(x), Inches(y), Inches(w), Inches(h))
    s.fill.solid(); s.fill.fore_color.rgb = fill
    if line is None:
        s.line.fill.background()
    else:
        s.line.color.rgb = line; s.line.width = Pt(1.25)
    s.shadow.inherit = False
    if shape == MSO_SHAPE.ROUNDED_RECTANGLE:
        s.adjustments[0] = radius
    s.text_frame.text = ''
    return s


def picture(slide, path, x, y, w=None, h=None, center_w=None):
    """Add a picture fitted inside w x h (keeps aspect); center_w centres it in that width from x."""
    from PIL import Image
    iw, ih = Image.open(path).size
    if w and h:
        sc = min(w / iw, h / ih)
    elif w:
        sc = w / iw
    else:
        sc = h / ih
    pw, ph = iw * sc, ih * sc
    px = x + ((center_w or pw) - pw) / 2
    return slide.shapes.add_picture(path, Inches(px), Inches(y), Inches(pw), Inches(ph))


def remove(shape):
    shape._element.getparent().remove(shape._element)


def shape_named(slide, name):
    return next(sh for sh in slide.shapes if sh.name == name)


def takeaway(slide, msg, y=1.0, x=0.3, w=9.4):
    return text(slide, x, y, w, 0.44, [dict(runs=[('Takeaway: ', dict(b=True, color=GREEN, size=15)), (msg, dict(b=True, size=15))])],
                fill=TINT_G, anchor=MSO_ANCHOR.MIDDLE, margin=0.1, name='Takeaway')


def mcen(slide, x, y, w, h, m, c, e, n, size=14):
    paras = []
    for lab, t in (('MEASURE: ', m), ('COMPARE: ', c), ('EXPLAIN: ', e), ('NEXT: ', n)):
        paras.append(dict(runs=[(lab, dict(b=True)), (t, {})], sb=4))
    return text(slide, x, y, w, h, paras, size=size, name='MCEN')


PIC_Y = 1.55


def data_slide_rework(slide, pic_name, box_name, msg, m, c, e, n, pic_h=3.4):
    """Takeaway banner, picture moved under it (same aspect), shorter Measure/Compare/Explain/Next."""
    pic = shape_named(slide, pic_name)
    asp = pic.width / pic.height
    ph = pic_h
    pw = min(9.4, ph * asp)
    ph = pw / asp
    pic.left, pic.top, pic.width, pic.height = Inches(0.3 + (9.4 - pw) / 2), Inches(PIC_Y), Inches(pw), Inches(ph)
    remove(shape_named(slide, box_name))
    takeaway(slide, msg)
    y = PIC_Y + ph + 0.08
    mcen(slide, 0.3, y, 9.4, 6.95 - y, m, c, e, n)


def figure_slide(slide, path, msg, m, c, e, n, pic_h=3.35):
    """New data slide: takeaway, figure, Measure/Compare/Explain/Next."""
    takeaway(slide, msg)
    pic = picture(slide, path, 0.3, PIC_Y, w=9.4, h=pic_h, center_w=9.4)
    y = PIC_Y + pic.height / 914400 + 0.08
    mcen(slide, 0.3, y, 9.4, 6.95 - y, m, c, e, n)


def code_box(slide, x, y, w, h, lines, size=13, caption=None):
    h = 0.3 + len(lines) * size * 1.18 / 72  # fit the box to the code
    rect(slide, x, y, w, h, TINT_GREY, line=RGBColor(0xC9, 0xCE, 0xD6), radius=0.03)
    paras = []
    for ln in lines:
        if ln.lstrip().startswith('//'):
            runs = [(ln, dict(font=CODEFONT, color=GREEN, size=size))]
        elif '//' in ln:
            a, b = ln.split('//', 1)
            runs = [(a, dict(font=CODEFONT, color=INK, size=size)), ('//' + b, dict(font=CODEFONT, color=GREEN, size=size))]
        else:
            runs = [(ln if ln else ' ', dict(font=CODEFONT, color=INK, size=size))]
        paras.append(dict(runs=runs, sb=0))
    t = text(slide, x + 0.05, y + 0.05, w - 0.1, h - 0.1, paras, size=size, margin=0.1)
    for p in t.text_frame.paragraphs:
        p.line_spacing = 1.0
    if caption:
        text(slide, x, y + h + 0.03, w, 0.3, [dict(runs=[(caption, dict(i=True, color=MUTED, size=11))])], margin=0.02)
    return y + h


def stat(slide, x, y, w, h, big, label, color=GREEN, fill=TINT_GREY, big_size=30):
    rect(slide, x, y, w, h, fill, radius=0.08)
    text(slide, x, y + 0.05, w, h * 0.55, [dict(runs=[(big, dict(b=True, size=big_size, color=color))], align=PP_ALIGN.CENTER)],
         anchor=MSO_ANCHOR.MIDDLE, margin=0.05)
    text(slide, x + 0.08, y + h * 0.55, w - 0.16, h * 0.43, [dict(runs=[(label, dict(size=12.5, color=INK))], align=PP_ALIGN.CENTER)],
         anchor=MSO_ANCHOR.TOP, margin=0.03)


def table(slide, x, y, w, col_w, rows, header_size=14, body_size=13, row_h=0.4, header_h=0.42, first_bold=False,
          fills=None):
    nr, nc = len(rows), len(rows[0])
    gf = slide.shapes.add_table(nr, nc, Inches(x), Inches(y), Inches(w), Inches(header_h + row_h * (nr - 1)))
    tb = gf.table
    for j, cw in enumerate(col_w):
        tb.columns[j].width = Inches(cw)
    for i in range(nr):
        tb.rows[i].height = Inches(header_h if i == 0 else row_h)
        for j in range(nc):
            cell = tb.cell(i, j)
            cell.margin_left = cell.margin_right = Inches(0.07)
            cell.margin_top = cell.margin_bottom = Inches(0.03)
            cell.vertical_anchor = MSO_ANCHOR.MIDDLE
            val = rows[i][j]
            runs = val if isinstance(val, list) else [(str(val), {})]
            tf = cell.text_frame
            tf.word_wrap = True
            p = tf.paragraphs[0]
            for r in list(p.runs):
                r._r.getparent().remove(r._r)
            for t, o in runs:
                if i == 0:
                    _run(p, t, size=header_size, bold=True, color=WHITE)
                else:
                    _run(p, t, size=o.get('size', body_size), bold=o.get('b', first_bold and j == 0), color=o.get('color', INK),
                         font=o.get('font', FONT))
            cell.fill.solid()
            if i == 0:
                cell.fill.fore_color.rgb = DARK
            else:
                f = fills[i][j] if fills and fills[i][j] is not None else (WHITE if i % 2 else RGBColor(0xF4, 0xF5, 0xF7))
                cell.fill.fore_color.rgb = f
    return gf


def notes(slide, t):
    slide.notes_slide.notes_text_frame.text = t


# ---------------------------------------------------------------- 3 NEW: why ADCS matters
why = new_slide('Why Attitude Control Matters',
                'Every spacecraft job depends on pointing: comms antennas at the ground, solar arrays at the sun, payloads at '
                'the target. If we cannot measure attitude, we cannot point; and wheels are the parts that wear out (Kepler). '
                'Our three requirements map one-to-one onto these jobs. The air bearing lets us test the whole loop on the '
                'ground: it removes friction and lets the table rotate freely in all three axes.')
cards = [
    ('1', 'POINT', 'Antennas, solar arrays and cameras only work when aimed. Reaction wheels give fine, continuous '
                   'torque with no propellant.', 'Req. 1: hold a heading'),
    ('2', 'KNOW', 'You can only point what you can measure. Sensor fusion turns noisy gyros, accelerometers and '
                  'magnetometers into one attitude estimate.', 'Req. 2: attitude to ±1°'),
    ('3', 'SURVIVE', 'Wheels wear out: Kepler lost 2 of its 4 wheels by 2013 and its primary mission ended. '
                     'A wheel failure must not end ours.', 'Req. 3: detect + compensate'),
]
for k, (num, head, body, req) in enumerate(cards):
    x = 0.3 + k * 3.18
    rect(why, x, 1.1, 3.0, 4.15, TINT_B, radius=0.05)
    circ = why.shapes.add_shape(MSO_SHAPE.OVAL, Inches(x + 1.12), Inches(1.3), Inches(0.76), Inches(0.76))
    circ.fill.solid(); circ.fill.fore_color.rgb = BLUE; circ.line.fill.background(); circ.shadow.inherit = False
    tfc = circ.text_frame; tfc.paragraphs[0].alignment = PP_ALIGN.CENTER
    _run(tfc.paragraphs[0], num, size=24, bold=True, color=WHITE)
    text(why, x + 0.1, 2.15, 2.8, 0.5, [dict(runs=[(head, dict(b=True, size=22, color=BLUE))], align=PP_ALIGN.CENTER)])
    text(why, x + 0.15, 2.7, 2.7, 1.9, [dict(runs=[(body, dict(size=14.5))])])
    text(why, x + 0.15, 4.62, 2.7, 0.5, [dict(runs=[(req, dict(b=True, size=14.5, color=GREEN))], align=PP_ALIGN.CENTER)],
         fill=TINT_G, anchor=MSO_ANCHOR.MIDDLE)
text(why, 0.3, 5.5, 9.4, 1.35, [
    dict(runs=[('Why an air bearing: ', dict(b=True)), ('the table floats on air, so it can rotate in roll, pitch and yaw '
               'with almost no friction. The full sense → estimate → control → actuate loop can be tested on the ground, '
               'as it would run in orbit.', {})]),
], size=15, fill=TINT_GREY, margin=0.12, anchor=MSO_ANCHOR.MIDDLE)

# ---------------------------------------------------------------- 4 requirements (enlarged, status symbols)
req = orig[2]
remove(shape_named(req, 'Table 91'))
OK, PART = ('✔', dict(b=True, color=GREEN, size=17)), ('◐', dict(b=True, color=AMBER, size=17))
rows = [
    ['Requirement', 'Pass criterion', 'Status'],
    [[('(1) Hold a heading, no visible jitter or drift', dict(b=True, size=15))],
     'Free table holds the commanded heading 1 min, |error| < 1°',
     [PART, (f'  Partial: DEMO LEVEL held heading ±0.5°; hold time limited by balance', {})]],
    [[('(2) Know attitude to ±1° in all 3 axes', dict(b=True, size=15))],
     'Estimate vs truth over the run, error < 1°',
     [PART if MT['fails'] else OK,
      (f'  Roll/pitch σ 0.02°. Yaw with the magnetometer aid: {MT["yaw_claim"]}, {MT["minutes"]:.0f} min, fixed table', {})]],
    [[('(3) Detect a wheel failure and compensate', dict(b=True, size=15))],
     'Kill a motor mid-maneuver: flagged < 1 s, then (1) and (2) still met',
     [PART, ('  Built + simulated 9/9 PASS (detected in 0.6–0.8 s); table test next', {})]],
    [[('Stretch: 4-attitude optimal slew', dict(b=True, size=15))],
     'Each attitude reached on the shortest path and held 5 s within ±1°',
     [PART, ('  Simulated 4/4 PASS on a balanced table; table needs CG raised ≈ 4.4 mm', {})]],
]
table(req, 0.3, 1.05, 9.4, [3.0, 3.0, 3.4], rows, header_size=15, body_size=14, row_h=1.1, header_h=0.45)
text(req, 0.3, 5.95, 9.4, 0.45, [dict(runs=[('✔', dict(b=True, color=GREEN, size=15)), ('  verified on hardware      ', {}),
                                              ('◐', dict(b=True, color=AMBER, size=15)), ('  built / simulated, or met in part      ', {}),
                                              ('○', dict(b=True, color=MUTED, size=15)), ('  not started', {})])],
     size=14, anchor=MSO_ANCHOR.MIDDLE)
notes(req, 'Requirement 2: roll and pitch are far inside the limit. Yaw with the magnetometer aid passed 36 of 37 graded '
           'steps of MAG TEST on the fixed table; the one miss was 1.02°, 0.02° over. Replaying the same log showed why '
           '(each new wheel pattern was learned at the current yaw error) and that predicting those references instead '
           'passes 37/37, worst 0.77°: that firmware is built, not yet run on the table. Requirements 1 and 3 have working '
           'firmware and simulations; the remaining risk is table time and balance.' if MT['fails'] else
           'Requirement 2 is met on the fixed table: MAG TEST graded every step on the Teensy. Requirements 1 and 3 '
           'have working firmware and simulations; the remaining risk is table time and balance.')

# ---------------------------------------------------------------- 5 control loop with explicit input / output
loop = orig[3]
remove(shape_named(loop, 'Picture 107'))
remove(shape_named(loop, 'TextBox 108'))
picture(loop, os.path.join(FIG, 'diag_control_loop_io.png'), 0.3, 0.98, w=9.4, h=4.35, center_w=9.4)
bullets(loop, 0.3, 5.35, 9.4, 1.65, [
    [('Input: ', dict(b=True, color=GREEN)), ('commanded attitude and rate from the active mode (ARM, DEMO LEVEL, STEP / SLEW TEST). ', {}),
     ('Output: ', dict(b=True, color=RED)), ('the table’s attitude.', {})],
    [('Estimator: ', dict(b=True)), ('6-state MEKF. Gyro predicts at 100 Hz; accelerometer corrects roll/pitch; magnetometer '
                                       'corrects heading when the wheels are in a known state.', {})],
    [('Disturbances: ', dict(b=True)), ('gravity from the CG offset and air drag; wheels can’t dump momentum, so these set the hold time.', {})],
], size=14, sb=4)

# ---------------------------------------------------------------- 6 control law: lighter
law = orig[4]

# ---------------------------------------------------------------- 7-9 NEW: code walkthrough
c1 = new_slide('Code: Heading Update (Estimation)',
               'This is the magnetometer update we added after the prelim. The key design choice is the gain edit: the '
               'filter is only allowed to rotate the estimate about vertical, so a bad magnetic sample can never tip roll '
               'or pitch. The aid decides when the field can be trusted at all: the wheels must be in a known state.')
code_box(c1, 0.3, 1.0, 5.6, 4.55, [
    '// attitude_ekf.cpp: heading-only mag update',
    'Vec3 mI = A^T * magBody;      // field, inertial',
    'float alpha = atan2(ref x mI, ref . mI);',
    '                              // heading error',
    'Vec3 h = A * ez;              // H = [h^T, 0]',
    'float S = h^T P h + sigma^2;  // innovation var',
    'K = P H^T / S;                // Kalman gain',
    'K[0..2] = (K . h) h;          // rotate about',
    '                              // vertical only',
    'K[3] = K[4] = 0;              // x/y bias: accel',
    'q = exp(K[0..2]*alpha) (x) q; // multiplicative',
    'b.z += K[5] * alpha;          // z gyro bias',
    'P = (I-KH) P (I-KH)^T + K R K^T; // Joseph',
    '',
    '// mag_aid.cpp: only trust known wheel states',
    'if (slow or reversing wheel) return GATED;',
    'ref = stopped ? stopped_ : spin_[pattern];',
], caption='Excerpts, simplified from attitude_ekf.cpp and mag_aid.cpp')
bullets(c1, 6.1, 1.0, 3.6, 5.9, [
    [('Measurement: ', dict(b=True)), ('direction of the horizontal field vs a reference learned in place → heading error α', {})],
    [('H = [(Aẑ)ᵀ 0]: ', dict(b=True)), ('only a turn about vertical changes the heading', {})],
    [('K = PHᵀ/(HPHᵀ+R): ', dict(b=True)), ('σ sets a ≈ 25 s correction, so the gyro still carries fast motion', {})],
    [('Gain edit: ', dict(b=True)), ('a disturbed field can never tilt roll/pitch', {})],
    [('Bias: ', dict(b=True)), ('heading makes the z-gyro bias observable; gravity can’t', {})],
    [('Gating: ', dict(b=True)), ('only known wheel states (stopped where learned, or spinning)', {})],
    [('Prediction: ', dict(b=True)), ('a new wheel pattern’s reference comes from a known one, so yaw errors can’t pile up (MAG TEST)', {})],
], size=13.5, sb=5)

c2 = new_slide('Code: Control Law & Wheel Allocation',
               'The control law is one line of code per term of the equation on the theory slide. Allocation is a pseudo-'
               'inverse over whichever wheels are enabled, which is why losing one wheel needs no new code path. The null '
               'mode spins the wheels against each other with zero net torque: we use it to keep every wheel away from '
               'zero speed for the magnetometer.')
code_box(c2, 0.3, 1.0, 5.6, 4.55, [
    '// attitude_ctrl.cpp: quaternion PD + FF',
    'Quat qe = q (x) conj(q_ref);  // attitude error',
    'Vec3 th = -2 * vec(qe);       // error angle',
    'tau = -Kp*th - Kd*(w - w_ref) // PD',
    '      + I*a_ref               // feed-forward',
    '      + w x (I*w + h_wheels); // gyroscopic',
    '',
    '// wheel_geometry.h: min-norm allocation',
    'M = sum(a_i a_i^T)            // enabled only',
    'y = inverse(M) * tau;',
    'u_i = -(a_i . y);             // wheel torques',
    '',
    '// null mode n = [+1,-1,+1,-1]: no body torque',
    'u_i += J * k * (bias - s_null) * n_i;',
], caption='Excerpts, simplified from attitude_ctrl.cpp, wheel_geometry.h, null_bias.cpp')
bullets(c2, 6.1, 1.0, 3.6, 5.9, [
    [('τ = −Kp θe − Kd(ω − ω_ref) + Iα_ref + ω × (Iω + h_w)', dict(b=True))],
    [('Kp = Iωn², Kd = 2ζIωn: ', dict(b=True)), ('ωn = 2 rad/s, ζ = 0.9 → about 1.5 s rise, no overshoot', {})],
    [('u = −D⁺τ: ', dict(b=True)), ('least total wheel torque; any 3 of the 4 pyramid wheels still span all 3 axes (Req. 3)', {})],
    [('Null mode: ', dict(b=True)), ('keeps every wheel ≥ 100 rpm from zero, so the magnetometer aid stays usable during holds', {})],
    [('Tunable live: ', dict(b=True)), ('SET ATT_BW / ATT_ZETA / IZZ over the radio', {})],
], size=13.5, sb=6)

c3 = new_slide('Code: Wheel-Failure Detection',
               'There is no current sensor, so we infer motor current from the duty cycle and the back-EMF. A healthy '
               'motor that is pushed hard must speed up; a dead one does not. Two bad 0.1 s blocks in a row and the wheel is '
               'taken out of the allocation. It coasts instead of braking, because braking a spinning wheel would dump its '
               'momentum into the table.')
code_box(c3, 0.3, 1.0, 5.6, 4.3, [
    '// wheel_fault.cpp: every 0.1 s, per wheel',
    'effort = duty - speed / 160;  // ~ I / I_stall',
    'if (|mean(effort)| >= 0.2) {  // driven hard',
    '  predicted = 800 * sum(effort * dt);',
    '                              // rad/s to gain',
    '  gained = (speed - speed0) * sign(effort);',
    '  bad = gained < 0.25 * |predicted|;',
    '}',
    'if (bad for 0.2 s) {          // 2 blocks',
    '  wheelEnabled[i] = false;    // use 3 wheels',
    '  wheels[i].coast();          // zero current',
    '}',
], caption='Excerpt, simplified from wheel_fault.cpp')
bullets(c3, 6.1, 1.0, 3.6, 3.9, [
    [('DC motor: ', dict(b=True)), ('current ∝ V − kₑω, so effort = duty − ω/ω_free estimates it without a current sensor', {})],
    [('Healthy: ', dict(b=True)), ('J ω̇ ∝ effort → a predicted speed gain; a dead motor gains ~0', {})],
    [('Coast, not brake: ', dict(b=True)), ('braking would dump the wheel’s momentum into the table', {})],
    [('FAULT TEST: ', dict(b=True)), ('cuts a motor without telling the controller; graded on the Teensy', {})],
], size=13.5, sb=6)
stat(c3, 0.3, 5.3, 2.7, 1.3, '0.6–0.8 s', 'to detect a dead motor (simulation)', color=BLUE, big_size=24)
stat(c3, 3.2, 5.3, 2.7, 1.3, '0', 'false alarms in 4 healthy simulated runs', color=GREEN, big_size=24)

# ---------------------------------------------------------------- 12 software build & tests (updated table)
sw = orig[7]
remove(shape_named(sw, 'Table 91'))
remove(shape_named(sw, 'Text Placeholder 2'))
rows = [['Date', 'Test', 'Key result'],
        ['10-08', 'Wheel spin direction', 'Wiring corrected, signs set'],
        ['10-15', 'Bench speed sweeps, hand-turn counts', '6.3:1 gearing; wheel A rubs'],
        ['10-15', 'Static IMU test (37 min)', 'Roll/pitch 0.02°; yaw drift 27 °/h'],
        ['10-16', 'MAG MAP; first DEMO SPIN', 'Wheels disturb the magnetometer; table is a pendulum'],
        ['10-20', 'IMU INT/RST + SD fixes', 'Sample loss 25% → 0.085%'],
        ['10-21', 'Balance weights; DEMO LEVEL', 'Swing 1.8 → 3.3 s; level held up to 15.8 s'],
        ['10-27', 'Magnetometer calibration + aid', 'Wheel field model; aid works with wheels stopped'],
        ['10-29', 'Aid spinning test; SLEW / FAULT sims', '±0.94° spinning (2 wheels); sims 4/4 and 9/9 PASS'],
        ['10-29', 'Flight computer failure', '3.3 V rail shorted inside the Teensy → spare'],
        ['10-30', 'Spare bring-up; wheel re-test', 'All 4 wheels 44–47 rad/s at 0.3 duty'],
        ['10-30', 'MAG MAP (4 wheels); MAG TEST', f'New wheel model; yaw within ±1° on {MT["passed"]:.0f}/{MT["steps"]} steps (worst {MT["worst"]:.2f}°)'],
        ]
table(sw, 0.3, 1.0, 9.4, [0.75, 3.75, 4.9], rows, header_size=13.5, body_size=12.5, row_h=0.4, header_h=0.4)
text(sw, 0.3, 5.9, 9.4, 0.95, [dict(runs=[('Teensy software (C++, PlatformIO): ', dict(b=True)),
                                          ('MEKF + magnetometer aid, attitude controller, allocation, wheel loops, fault detector, safety '
                                           'supervisor, 100 Hz SD log + XBee link, scripted tests run and graded on board (DEMO LEVEL, '
                                           'MAG TEST, STEP / SLEW / FAULT TEST), live tuning (SET / GET).', {})])], size=13.5,
     fill=TINT_GREY, margin=0.1)

# ---------------------------------------------------------------- 13-15 NEW: simulation
sm = new_slide('Simulation: Methods',
               'We simulate before every table session: table time is scarce, and a bad controller can throw the satellite '
               'off the bearing. The truth model is written separately from the firmware, so the two can disagree. That is '
               'how we caught real bugs. The estimator check below is the first thing we ran.')
cards = [
    ('Truth model (independent)', ['Rigid table + 4 wheels, gyroscopic coupling', 'Motors: stall torque, back-EMF, friction, pack differences',
                                   'Encoder counts, gyro noise', 'Gravity pendulum + yaw drag fitted to the table']),
    ('Firmware logic (ported)', ['MEKF and magnetometer gating', 'Quaternion PD, allocation, null mode', '100 Hz wheel PI with current limit',
                                         'Fault detector and the on-board test graders']),
    ('Test cases', ['Balanced vs today’s pendulous table', 'Each wheel failing; mid-slew and mid-hold',
                    'Healthy runs to count false alarms', 'Pass/fail by the same rules as on the table']),
]
for k, (head, items) in enumerate(cards):
    x = 0.3 + k * 3.18
    rect(sm, x, 1.0, 3.0, 2.85, TINT_B, radius=0.05)
    text(sm, x + 0.1, 1.05, 2.8, 0.42, [dict(runs=[(head, dict(b=True, size=14.5, color=BLUE))])])
    bullets(sm, x + 0.05, 1.45, 2.9, 2.35, items, size=12.5, sb=3)
takeaway(sm, 'Estimator vs truth (3-axis motion): roll/pitch error < 0.1°, yaw inside its 1σ band', y=4.0)
picture(sm, os.path.join(FIG, 'sim1_ekf_verification.png'), 0.3, 4.5, w=9.4, h=2.45, center_w=9.4)

sf = new_slide('Simulation: Wheel Failure (Req. 3)',
               'This is FAULT TEST in simulation. Wheel A dies 10 s into a level hold on today’s pendulous table. The '
               'detector flags it 0.6 s later, the wheel coasts down, and B, C and D take over. The peak error stays inside '
               '±1°. We ran every wheel, failures mid-slew and mid-hold, and an unplugged motor: 9 of 9 pass.')
takeaway(sf, 'A motor dying mid-hold is caught in < 1 s and the other 3 wheels hold ±1°')
picture(sf, os.path.join(FIG, 'sim9_fault_level.png'), 0.3, PIC_Y, w=6.1, h=5.3)
for k, (big, lab, col) in enumerate([('9 / 9', 'graded runs PASS', GREEN), ('0.6–0.8 s', 'to detect the dead motor', BLUE),
                                     ('≤ 1.0°', 'peak error while recovering', BLUE), ('0', 'false alarms in 4 healthy runs', GREEN)]):
    stat(sf, 6.6, 1.6 + k * 1.33, 3.1, 1.2, big, lab, color=col, big_size=26)

ss = new_slide('Simulation: 4-Attitude Slew (Stretch)',
               'SLEW TEST: four targets, shortest-rotation slews on a smooth profile, then a 5 s hold graded at ±1°. On a '
               'balanced table all four pass with under a tenth of a degree of error. On today’s table, gravity torque '
               'saturates a wheel 4.9 s into the first tilted slew. So the stretch goal is now a balance problem, not a '
               'software one.')
takeaway(ss, 'The slew logic meets ±1° on a balanced table; today’s balance is the blocker')
picture(ss, os.path.join(FIG, 'sim7_slew_test.png'), 0.3, PIC_Y, w=6.1, h=5.3)
for k, (big, lab, col) in enumerate([('4 / 4', 'targets PASS, balanced table (max err 0.08°)', GREEN),
                                     ('4.9 s', 'until a wheel saturates on today’s table', RED),
                                     ('≥ 55 s', 'swing period needed (today 3.3 s)', AMBER),
                                     ('4 / 4', 'yaw-only targets pass at today’s rest tilt', GREEN)]):
    stat(ss, 6.6, 1.6 + k * 1.33, 3.1, 1.2, big, lab, color=col, big_size=26)

# ---------------------------------------------------------------- 16-21 data slides: takeaways, shorter text
data_slide_rework(orig[8], 'Picture 121', 'TextBox 122',
                  'B and D track within ±1 rpm; A and C errors come from hardware, not the control loop',
                  'Commanded vs measured speed, all 4 wheels, 0 → 480 → 0 rpm (12 V packs).',
                  'Tight tracking expected from the 100 Hz PI + feed-forward; same gains on every wheel.',
                  'A rubs and C oscillates (±10–16 rpm); nothing turns below ≈ 46 rpm (static friction).',
                  'Fix A’s rub; add static-friction compensation; repeat.')
data_slide_rework(orig[9], 'Picture 91', 'TextBox 92',
                  'Roll/pitch are 50× inside ±1°; the yaw drift was a calibration error, now fixed',
                  '37 min, IMU fixed, motors off: EKF roll/pitch/yaw and the magnetometer heading.',
                  'Requirement ±1°. A 3 s gyro-bias average predicts ≈ 26 °/h of yaw drift.',
                  'Roll/pitch σ = 0.02°. Yaw drifted 27 °/h, as predicted. A 180 s calibration after a 10 min warm-up gives ≈ 3 °/h.',
                  'Done: the magnetometer aid now holds yaw (MAG TEST).')
data_slide_rework(orig[10], 'Picture 91', 'TextBox 92',
                  'Lost samples cut from 25% to 0.085%: no more false IMU-timeout shutdowns',
                  'Time between gyro samples logged to SD, IMU at rest, before and after the fixes.',
                  'Expected 10 ms. Before: 25% lost, 38 gaps > 100 ms (each would have disarmed the wheels).',
                  'Library kept only the last report per read; SD writes blocked the loop. Fixed: INT/RST wires, IMU read during SD waits, SD DMA.',
                  'Remaining 60–90 ms stalls are inside the BNO085, under the 100 ms timeout.')
data_slide_rework(orig[11], 'Picture 91', 'TextBox 92',
                  'Spinning wheels add fast, filterable noise; stopped wheels leave random offsets',
                  'Raw magnetometer heading, fixed table, wheels stepped through ±50/100/200 rpm (MAG MAP).',
                  'A heading aid must be steady to well under 1°; motors off: 0.7° noise, no drift.',
                  'Spinning: 4.6–7.5° of fast noise (low-passed out). Stopped: offsets up to ±10°, set by where each shaft stops.',
                  'Done: an aid that trusts only known wheel states (next slide).')
data_slide_rework(orig[12], 'Picture 91', 'TextBox 92',
                  'The table is a pendulum: gravity overwhelms the wheels within 1–2 s',
                  'Roll/pitch and wheel speeds, table floating; three DEMO SPIN attempts.',
                  'Simulation (CG at the bearing centre) levelled from 30° in ≈ 11 s; the table hit the wheel limit in 1–2 s.',
                  '1.8 s swing, resting ≈ 3° off level: gravity ≈ 16 mN·m per degree vs ≈ 24 mN·m·s of wheel storage.',
                  'Done: bottom weights and lateral trim (next slide).')
data_slide_rework(orig[13], 'Picture 91', 'TextBox 92',
                  'Weights made gravity’s pull 3× weaker and fixed pitch; roll still rested 3.3° off',
                  'Six DEMO SPIN attempts after adding weights: roll, wheel speeds, free swing.',
                  'Expected: level (< 1° for 3 s), then spin. Every attempt hit the wheel limit after 2.2–3.7 s.',
                  'Swing period 1.8 → 3.3 s; the −y side is heavy, so holding level fills wheel A in ≈ 3 s.',
                  'Done: roll trim and DEMO LEVEL (results after the video).', pic_h=3.0)

# ---------------------------------------------------------------- 20 NEW: MAG TEST
mt = new_slide('Data: Magnetometer Yaw Aid (MAG TEST)',
               'MAG TEST runs on the Teensy and grades itself. The table is clamped, so the true heading is zero and any '
               'yaw shown is estimation error. The wheels step through stopped, 14 direction patterns and three speeds; '
               'three times we deliberately rotate the estimate 10° to prove the aid pulls it back. Then 30 minutes of drift '
               'run. Grey is the same data replayed on the PC with the aid off: the gyro alone drifted almost 6 °/h even '
               'after warm-up. Black is what ran: one step, all wheels at 300 rpm, reached 1.02°. Gold is the same data '
               'through the current firmware, which predicts each new wheel pattern\'s reference instead of learning it '
               'at the current error: every step passes.')
figure_slide(mt, os.path.join(FIG, 'fig9_magtest.png'), MT['takeaway'],
             f'{MT["steps"]} graded steps, {MT["minutes"]:.0f} min, fixed table: stopped, 14 wheel direction patterns, ±10° kicks, '
             '30 min drift run; graded on the Teensy.',
             f'Requirement ±1° (the true heading is 0). Gyro alone (replay, aid off): {abs(MT["gyro_rate"]):.1f} °/h even after warm-up.',
             MT['explain'],
             MT['next'])

# ---------------------------------------------------------------- 25 NEW: DEMO LEVEL
dl = new_slide('Data: DEMO LEVEL Hold',
               'After the roll trim, DEMO LEVEL levels the table and holds it. Each run ends when gravity has filled a wheel. '
               'The right panel is the proof that the leftover tilt is the controller’s expected steady-state error, '
               'torque over Kp, and not a sensor problem.')
figure_slide(dl, os.path.join(FIG, 'fig7_demo_level.png'),
     'After the roll trim the table levels in 5–6 s and holds steady until gravity fills a wheel',
     'Five DEMO LEVEL runs after the roll trim (DEMO_0003): level, then hold until a wheel limit.',
     'Hold level without visible jitter; the limit is wheel momentum vs leftover gravity torque.',
     'Held roll/pitch σ ≤ 0.07°, heading within 0.5°, for 2–15.8 s; leftover 1.3–3.3 mN·m fills wheel B. Offset = torque / Kp (right).',
     'DEMO LEVEL now holds the natural rest tilt (no steady torque); raise the CG ≈ 4.4 mm for the slews.')

# ---------------------------------------------------------------- 26 NEW: flight computer failure (unexpected)
fc = new_slide('Data: Flight Computer Failure (Unexpected)',
               'After the first spinning magnetometer test the Teensy went dark. We traced it to a short on its own 3.3 V '
               'rail, then brought a spare up one stage at a time, checking voltage and heat at each step. Then we re-ran the '
               'motor checks: every wheel back at about 45 rad/s, and wheel D’s missing data turned out to be a loose '
               'encoder wire.')
figure_slide(fc, os.path.join(FIG, 'fig8_motor_bringup.png'),
     'Teensy’s 3.3 V rail shorted; spare brought up, all 4 wheels re-verified in a day',
     'Board dead after test 5 (no USB, no 3.3 V); then the spare, wheels re-tested at ±0.3 duty and from rest.',
     'Expected ≈ 45 rad/s at 0.3 duty and breakaway at 0.08–0.10 duty on every wheel.',
     'Old board’s 3.3 V rail: 0.8 Ω and hot, so it was shorted on the board; no wiring cause found. Wheels 44–47 rad/s; D’s encoder wire was loose.',
     'Solder wheel D’s lead; separate the spare’s VIN–VUSB pads (until then, never USB and battery together).')

# ---------------------------------------------------------------- 27 where we stand
ws = orig[16]


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


rewrite(shape_named(ws, 'TextBox 129'), ['Attitude estimator: roll/pitch 0.02°', 'Magnetometer yaw aid, 55 min graded test',
                                        'Reliable 100 Hz IMU data', 'Wheel loops, all 4 wheels', 'Fault detection + 3-wheel control (simulated)',
                                        'SLEW / FAULT / STEP tests on board', 'Spare flight computer up'], size=13.5)
rewrite(shape_named(ws, 'TextBox 131'), ['Balance: CG ≈ 4.4 mm below the bearing centre', f'Yaw aid: {MT["passed"]:.0f}/{MT["steps"]} steps as run; fix replays {MT["passed_fw"]:.0f}/{MT["steps"]}', 'Hold time: gravity + yaw drag fill the wheels',
                                        'Wheel A rub; wheel D encoder lead', 'Real inertia (STEP TEST ready)'], size=13.5)
rewrite(shape_named(ws, 'TextBox 133'), ['Heading hold on the free table (Req. 1)', 'Yaw accuracy on the floating table (Req. 2)',
                                        'Wheel-failure test on the table (Req. 3)', 'Stretch: 4-attitude slew (needs balance)'], size=13.5)

# ---------------------------------------------------------------- 28 path forward
pf = orig[17]
pf.shapes.title.text = 'Future Plans: Path to the Final Demo'
remove(shape_named(pf, 'Table 91'))
rows = [['#', 'Next task', 'Verifies', 'Needs'],
        ['1', 'Upload; LEVEL ZERO (IMU mount) and STEP TEST (inertia, gains)', 'Req. 1, 2', 'Table time'],
        ['2', 'DEMO LEVEL heading hold with the aid on vs off', 'Req. 1, 2', '—'],
        ['3', 'FAULT TEST on the floating table', 'Req. 3', 'Solder wheel D lead'],
        ['4', 'Hard-iron fit from a full yaw turn', 'Req. 2', '—'],
        ['5', 'Raise CG ≈ 4.4 mm (balance tool), then SLEW TEST', 'Stretch', 'Trim mass'],
        ]
table(pf, 0.3, 1.05, 9.4, [0.4, 5.6, 1.5, 1.9], rows, header_size=13.5, body_size=13, row_h=0.5, header_h=0.42)
rewrite(shape_named(pf, 'TextBox 92'), ['Key risks', 'Balance: tilted targets saturate the wheels until the CG is raised',
                                       'Yaw drag fills the wheels in ≈ 6 min (no momentum dumping)',
                                       'Wheel D encoder lead; spare board’s USB/battery limit'], size=13)
rewrite(shape_named(pf, 'TextBox 93'), ['Improvements beyond KestrelSAT', 'Use of the air bearing table',
                                       'Magnetometer aid that works with spinning wheels', 'Automatic wheel-failure detection',
                                       'Tests that run and grade themselves on board'], size=13)
for name in ('TextBox 92', 'TextBox 93'):  # keep the headers bold, the rest as list items
    sh = shape_named(pf, name)
    sh.top = Inches(4.4)
    for k, p in enumerate(sh.text_frame.paragraphs):
        for r in p.runs:
            r.font.bold = k == 0

# ---------------------------------------------------------------- 29 GenAI
ga = orig[18]
body = shape_named(ga, 'Text Placeholder 2')
paras = body.text_frame.paragraphs
upd = {
    1: 'Firmware: drafted the EKF and magnetometer aid, controller, allocation, fault detector, IMU driver, logger, command set and '
       'the self-grading tests, from the team’s requirements and design decisions; the team reviewed, uploaded and tested every build',
    2: 'Simulation and analysis: wrote the truth models, the log-replay and balance tools and the plots; the team ran the tests and checked the conclusions',
    3: 'Documentation: CLAUDE.md, TODO.md, PROGRESS_LOG.md, test reports and drafts of these slides',
}
for k, t in upd.items():
    rs = paras[k].runs
    rs[0].text = t
    for r in rs[1:]:
        r._r.getparent().remove(r._r)

# ---------------------------------------------------------------- order
sld = prs.slides._sldIdLst
ids = list(sld)
O = {i + 1: ids[i] for i in range(len(orig))}
N = {k: ids[len(orig) + i] for i, k in enumerate(['why', 'c1', 'c2', 'c3', 'sm', 'sf', 'ss', 'mt', 'dl', 'fc'])}
order = [O[1], O[2], N['why'], O[3], O[4], O[5], N['c1'], N['c2'], N['c3'], O[6], O[7], O[8], N['sm'], N['sf'], N['ss'],
         O[9], O[10], O[11], O[12], N['mt'], O[13], O[14], O[15], O[16], N['dl'], N['fc'], O[17], O[18], O[19]]
for el in ids:
    sld.remove(el)
for el in order:
    sld.append(el)
prs.save(OUT)
print('saved', OUT, len(order), 'slides')
