"""Align the final deck with Reports/Astro_331_Final_Presentation_Guidance.pptx (2026-11-02):
  - requirement table: test method, measured outcome and an explicit final status (PASS / achieved in part)
  - every result block is MEASURE / COMPARE / EXPLAIN / IMPROVE ("what changed after testing? did it work?");
    COMPARE adds theory/simulation where we have it; EXPLAIN states pass/fail
  - data figures re-rendered with titles and test conditions (deck_figs.py), swapped in place
  - new presented slide Hardware: Final Build (photos + CAD); final results and limitations moved into the talk
  - objective slide states the new KestrelSAT capability; GenAI slide carries the GitHub repository link
Run Reports/figures/final_2026-11-02/deck_figs.py first; afterwards run edit_safe.py on the result.
    python Reports/deck_builder/align_guidance.py <in.pptx> <out.pptx>"""
import copy, io, os, sys
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.oxml.ns import qn
from pptx.util import Inches, Pt

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
FIG = os.path.join(ROOT, 'Reports', 'figures', 'final_2026-11-02')
REPO = 'github.com/derek-weissinger-usafa/Astro-331-Lab-8'
SRC, OUT = sys.argv[1], sys.argv[2]
prs = Presentation(SRC)
GREEN, AMBER, INK = '2E7D4F', 'B7790F', '1F2933'


def slide_by_title(start):
    for s in prs.slides:
        if s.shapes.title is not None and s.shapes.title.text.startswith(start):
            return s
    raise KeyError(start)


def text_shape(slide, start):
    return next(sh for sh in slide.shapes if sh.has_text_frame and sh.text_frame.text.startswith(start))


def set_title(slide, text):
    p = slide.shapes.title.text_frame.paragraphs[0]
    p.runs[0].text = text
    for r in p.runs[1:]:
        r._r.getparent().remove(r._r)


def move_after(slide, anchor):
    ids = prs.slides._sldIdLst
    sid = [s.slide_id for s in prs.slides]
    el, at = list(ids)[sid.index(slide.slide_id)], list(ids)[sid.index(anchor.slide_id)]
    ids.remove(el)
    at.addnext(el)


def mcen(slide, **parts):
    box = text_shape(slide, 'MEASURE')
    for p in box.text_frame.paragraphs:
        lab = p.text.split(':')[0]
        if lab in parts:
            rs = p.runs
            rs[1].text = ' ' + parts[lab]
            for r in rs[2:]:
                r._r.getparent().remove(r._r)


def takeaway(slide, text):
    rs = text_shape(slide, 'Takeaway').text_frame.paragraphs[0].runs
    rs[1].text = text
    for r in rs[2:]:
        r._r.getparent().remove(r._r)


def replace_text(old, new, expect):
    n = 0
    for s in prs.slides:
        for sh in s.shapes:
            if sh.has_text_frame:
                for p in sh.text_frame.paragraphs:
                    for r in p.runs:
                        if old in r.text:
                            r.text = r.text.replace(old, new)
                            n += 1
        if s.has_notes_slide and old in s.notes_slide.notes_text_frame.text:
            tf = s.notes_slide.notes_text_frame
            tf.text = tf.text.replace(old, new)
            n += 1
    assert n == expect, (old, n)


def swap_picture(slide, fig):
    """Replace the slide's figure with a re-rendered one of the same aspect (geometry unchanged)."""
    pic = next(sh for sh in slide.shapes if sh.shape_type == 13)
    part = slide.part.related_part(pic._element.blipFill.find(qn('a:blip')).get(qn('r:embed')))
    from PIL import Image
    old = Image.open(io.BytesIO(part.blob)).size
    blob = open(os.path.join(FIG, fig), 'rb').read()
    new = Image.open(io.BytesIO(blob)).size
    assert abs(old[0] / old[1] - new[0] / new[1]) < 0.01, (fig, old, new)
    part._blob = blob


# ================================================================== figures (titles + test conditions)
for title, fig in [('Data: Magnetometer Yaw Aid', 'd1_magtest.png'), ('Data: Balancing the Table', 'd2_balance.png'),
                   ('Data: Heading Holds by Laser', 'd3_laser_holds.png'), ('Data: Estimator Heading Leak', 'd4_heading_leak.png'),
                   ('Data: First-Run Start Failures', 'd5_sd_stall.png'), ('DEMO ALL: The Whole Run', 'd6_demo_all.png'),
                   ('DEMO ALL: Requirement 2', 'd7_req2.png'), ('DEMO ALL: Requirement 3', 'd8_req3.png')]:
    swap_picture(slide_by_title(title), fig)

# ================================================================== NEXT -> IMPROVE everywhere
n = 0
for s in prs.slides:
    for sh in s.shapes:
        if sh.has_text_frame:
            for p in sh.text_frame.paragraphs:
                if p.text.startswith('NEXT:') and p.runs[0].text.startswith('NEXT'):
                    p.runs[0].text = p.runs[0].text.replace('NEXT', 'IMPROVE', 1)
                    n += 1
assert n >= 25, n

# ================================================================== result blocks on the presented data slides
mcen(slide_by_title('Data: Magnetometer Yaw Aid'),
     EXPLAIN='Run 1 (references learned in place): 36/37 steps, worst 1.02°: FAIL by 0.02°. Run 2 (predicting new '
             'patterns): 10/15, FAIL; the motor-magnet model moved ≈ 1° between sessions.',
     IMPROVE='Went back to learning in place. At the demo spot the field bends up to 14° (backup), so the demo runs '
             'gyro-only, checked by laser (next slides).')
s = slide_by_title('Data: Magnetometer Yaw Aid')
tf = s.notes_slide.notes_text_frame
tf.text = tf.text.replace('Grey is the gyro alone', 'Red is the same data replayed with the gyro alone')

mcen(slide_by_title('Data: Balancing the Table'),
     IMPROVE='Back to a stiff layout (CG ≈ 10 mm low): DEMO LEVEL and SLEW YAW then worked. Tilted targets still need '
             'CG ≲ 1 mm: not achieved (future work).')

mcen(slide_by_title('Data: Heading Holds by Laser'),
     COMPARE='Requirement: no visible jitter or drift, |error| < 1°. Predicted from static gyro tests after warm-up: '
             '≈ 3 °/h, 0.25° in 5 min.',
     EXPLAIN='Warm calibration: PASS, a 0.25° band for 5 min, no trend; 0.15° for 3 min. Calibrated 5 min after '
             'power-on: −0.78° in 2.5 min (gyro z bias still warming).',
     IMPROVE='Changed the procedure: power on, 10 min warm-up, CALIBRATE 180 clamped. Retest in DEMO ALL: max 0.04° '
             'for 60 s.')

mcen(slide_by_title('Data: Estimator Heading Leak'),
     EXPLAIN='FAIL by laser (up to +2.3°), flipping sign with turn direction: the estimator turned 360° while its gyro '
             'said 363–364°. The accelerometer update was moving heading (Code: Attitude Estimation).',
     IMPROVE='Accelerometer gain now carries no heading; yaw scale refit to 1.0045 by log replay. Retest (DEMO ALL): '
             '0.05° after 360°.')

mcen(slide_by_title('Data: First-Run Start Failures'),
     EXPLAIN='FAIL, not a wheel fault: the first write to a new log file searches the card for space; loop() '
             'froze ≈ 0.8 s.',
     COMPARE='Expected a smooth spin-up; instead 7 of 7 first arms after a boot flagged every wheel dead.',
     IMPROVE='Log file written before arming; a wheel-loop step > 50 ms skips the fault checks. Retest: DEMO ALL '
             'passed on its first arm.')

mcen(slide_by_title('DEMO ALL: The Whole Run'),
     MEASURE='DEMO ALL (DEMO_0020, 11-01 23:13), first arm after a boot.',
     EXPLAIN='All three parts PASS: hold 60 s; four slews through 360°; wheel B cut 10 s later, table held on A, C, D.',
     IMPROVE='One armed run replaced separate tests: no disarm or brake between parts, so no wrong zero or kick.')

mcen(slide_by_title('DEMO ALL: Requirement 2'),
     COMPARE='Requirement ±1°. Before the estimator fix: +2.3° (CCW) / −1.8° (CW). Simulation predicted ≤ 0.011° '
             'slew error; measured ≤ 0.13°.',
     EXPLAIN='PASS. Laser −0.05° → +0.02…−0.03°: ≈ 0.05° after 360°. Targets 0.03 / 0.02 / 0.13 / 0.05°; roll and '
             'pitch within 0.23° of rest.',
     IMPROVE='The estimator fix did it: this run replayed through the old estimator ends 0.46° off (1.7–2.3° on '
             'earlier runs).')

mcen(slide_by_title('DEMO ALL: Requirement 3'),
     COMPARE='Pass: detected ≤ 1 s, back within ±1° ≤ 10 s, then ±1° for 30 s. Simulation predicted detection in '
             '0.6–0.8 s, peak ≤ 1.0°.',
     EXPLAIN='PASS. Detected 0.50 s; peak 0.90° (estimate), laser 0.3°; 30 s hold max 0.36° yaw, 0.01° roll/pitch.',
     IMPROVE='Stall handling added after the start failures: this pass came on a first arm after a boot (7 of 7 '
             'failed before).')

# ================================================================== requirement table
req = slide_by_title('Requirements: Verification')
set_title(req, 'Requirements: Verification & Final Status')
gf = next(sh for sh in req.shapes if sh.has_table)
tbl = gf.table
grid = tbl._tbl.find(qn('a:tblGrid'))
cols = grid.findall(qn('a:gridCol'))
newcol = copy.deepcopy(cols[-1])
for cid in newcol.iter('{http://schemas.microsoft.com/office/drawing/2014/main}colId'):  # must be unique
    cid.set('val', str(max(int(c.get('val')) for c in grid.iter(cid.tag)) + 1))
grid.append(newcol)
for tr in tbl._tbl.findall(qn('a:tr')):
    last = tr.findall(qn('a:tc'))[-1]
    last.addnext(copy.deepcopy(last))  # cells must come before the row's extLst
for c, w in zip(tbl.columns, (2.0, 2.75, 3.15, 1.5)):
    c.width = Inches(w)


def set_cell(cell, runs):
    """runs: (text, pt, bold, color) on one paragraph, formatted from the cell's first run."""
    tb = cell.text_frame._txBody
    ps = tb.findall(qn('a:p'))
    tmpl = copy.deepcopy(ps[0].findall(qn('a:r'))[0])
    for p in ps[1:]:
        tb.remove(p)
    p = ps[0]
    for r in p.findall(qn('a:r')):
        p.remove(r)
    end = p.find(qn('a:endParaRPr'))
    for text, pt, bold, color in runs:
        r = copy.deepcopy(tmpl)
        r.find(qn('a:t')).text = text
        rpr = r.find(qn('a:rPr'))
        rpr.set('sz', str(int(pt * 100)))
        rpr.set('b', '1' if bold else '0')
        rpr.find(qn('a:solidFill')).find(qn('a:srgbClr')).set('val', color)
        if end is not None:
            end.addprevious(r)
        else:
            p.append(r)


head = ['Requirement', 'Test method & pass criterion', 'Measured outcome (DEMO ALL 11-01, laser)', 'Final status']
for c, t in enumerate(head):
    set_cell(tbl.cell(0, c), [(t, 14, True, 'FFFFFF')])
rows = [
    ('(1) Hold a heading, no visible jitter or drift',
     'DEMO ALL part 1: DEMO LEVEL hold on the floating table, graded 60 s on board; laser read. Pass: |error| < 1°',
     '60 s: max 0.04°, jitter 0.01° rms; laser steady. 5 min laser hold within 0.25°', 'PASS'),
    ('(2) Know attitude to ±1° in all 3 axes',
     'Laser vs estimate before and after four slews through 360°. Pass: error < 1°',
     'Heading: laser 0.05° off after 360°. Roll/pitch: σ 0.02° at rest (accelerometer, no independent truth)', 'PASS'),
    ('(3) Detect a wheel failure and compensate',
     "DEMO ALL part 3: wheel B's motor cut, controller not told. Pass: flagged < 1 s, back within ±1° ≤ 10 s, then ±1° 30 s",
     'Flagged in 0.50 s; back within ±1° in 0.5 s; ≤ 0.36° for 30 s on 3 wheels', 'PASS'),
    ('Stretch: 4-attitude optimal slew',
     'SLEW YAW: 4 targets on the shortest path, each held 5 s. Objective: ±1° in all 3 axes',
     'Yaw targets 4/4 at the rest tilt (≤ 0.13°). Tilted targets: wheels fill in seconds (CG ≈ 10 mm low)', 'PART'),
]
for i, (r0, r1, r2, st) in enumerate(rows, start=1):
    set_cell(tbl.cell(i, 0), [(r0, 14, True, INK)])
    set_cell(tbl.cell(i, 1), [(r1, 12.5, False, INK)])
    set_cell(tbl.cell(i, 2), [(r2, 12.5, False, INK)])
    if st == 'PASS':
        set_cell(tbl.cell(i, 3), [('✔', 17, True, GREEN), ('  PASS', 14, True, GREEN)])
    else:
        set_cell(tbl.cell(i, 3), [('◐', 17, True, AMBER), ('  Achieved in part: yaw targets only', 13, True, AMBER)])
for i, row in enumerate(tbl.rows):
    row.height = Inches(0.5 if i == 0 else 1.2)
# The status column now says PASS / achieved in part, so the symbol legend under the table is redundant; removing it
# also leaves the table room to grow when edited.
legend = text_shape(req, '✔')
legend._element.getparent().remove(legend._element)
req.notes_slide.notes_text_frame.text = (
    'Each row gives the test, how it was graded, what we measured and the verdict. All three threshold requirements '
    'pass on the floating table, in one run, against an independent truth: a laser on the deck read on a wall 2.48 m '
    'away. Roll and pitch come from the accelerometer, which is very quiet at rest, but we had no independent truth '
    'for them. The stretch goal is achieved in part: yaw targets at the rest tilt pass 4 of 4; tilted targets need the '
    'centre of mass within about a millimetre of the bearing, which we could not hold.')

# ================================================================== objective: new KestrelSAT capability
obj = text_shape(slide_by_title('Why Attitude Control Matters'), 'Objective')
ps = obj.text_frame._txBody.findall(qn('a:p'))
newp = copy.deepcopy(ps[0])
rs = newp.findall(qn('a:r'))
rs[0].find(qn('a:t')).text = 'New KestrelSAT capability: '
rs[1].find(qn('a:t')).text = ('closed-loop 3-axis reaction-wheel attitude control that detects a failed wheel and keeps '
                              'pointing on the other three, verified against an independent laser.')
for r in rs[2:]:
    newp.remove(r)
ps[0].addnext(newp)
obj.height = Inches(6.95) - obj.top

# ================================================================== GenAI slide: repository link
replace_text('Full record of work: the project log and the git history in the team repository',
             f'Full record of work (code, logs, reports, git history): {REPO}', 1)

# ================================================================== Final results and limitations into the talk
res = slide_by_title('Results and Open Items')
set_title(res, 'Final Results & Limitations')
move_after(res, slide_by_title('DEMO ALL: Requirement 3'))

# ================================================================== Hardware: Final Build (photos + CAD)
spin, cad, diag = slide_by_title('DEMO SPIN'), slide_by_title('Hardware: Battery & Washer Holders'), \
    slide_by_title('Hardware: System Block Diagram')
pics = {sh.name: sh for sh in list(spin.shapes) + list(cad.shapes) if sh.shape_type == 13}
cap_tmpl = next(sh for sh in spin.shapes if sh.name.startswith('Caption'))._element
num_tmpl = next(sh for sh in cad.shapes if sh.is_placeholder and 'Slide Number' in sh.name)._element
fb = prs.slides.add_slide(cad.slide_layout)
for ph in list(fb.placeholders):
    if ph.placeholder_format.type != 1 and ph.placeholder_format.idx != 0:  # keep the title only
        ph._element.getparent().remove(ph._element)
fb.shapes.title.text = 'Hardware: Final Build'
src_title = cad.shapes.title.text_frame.paragraphs[0].runs[0]._r.find(qn('a:rPr'))
if src_title is not None:
    fb.shapes.title.text_frame.paragraphs[0].runs[0]._r.insert(0, copy.deepcopy(src_title))
fb.shapes._spTree.append(copy.deepcopy(num_tmpl))


def group(slide, shapes, name):
    """Group shapes (z-order kept) so a photo and its caption move together."""
    tree = slide.shapes._spTree
    shapes = sorted(shapes, key=lambda s: list(tree).index(s._element))
    x0 = min(s.left for s in shapes); y0 = min(s.top for s in shapes)
    x1 = max(s.left + s.width for s in shapes); y1 = max(s.top + s.height for s in shapes)
    g = tree.makeelement(qn('p:grpSp'), {})
    nv = g.makeelement(qn('p:nvGrpSpPr'), {}); g.append(nv)
    nid = max(int(e.get('id')) for e in tree.iter(qn('p:cNvPr'))) + 1
    nv.append(nv.makeelement(qn('p:cNvPr'), {'id': str(nid), 'name': name}))
    nv.append(nv.makeelement(qn('p:cNvGrpSpPr'), {})); nv.append(nv.makeelement(qn('p:nvPr'), {}))
    gp = g.makeelement(qn('p:grpSpPr'), {}); g.append(gp)
    xf = gp.makeelement(qn('a:xfrm'), {}); gp.append(xf)
    for tag, a, b in (('a:off', x0, y0), ('a:ext', x1 - x0, y1 - y0), ('a:chOff', x0, y0), ('a:chExt', x1 - x0, y1 - y0)):
        xf.append(xf.makeelement(qn(tag), {('cx' if 'ext' in tag or 'Ext' in tag else 'x'): str(a),
                                            ('cy' if 'ext' in tag or 'Ext' in tag else 'y'): str(b)}))
    shapes[0]._element.addprevious(g)
    for s in shapes:
        g.append(s._element)


def place(name, x, y, h=None, w=None, caption=None, cap_w=None):
    src = pics[name]
    ar = src.width / src.height
    if h is None:
        h = w / ar
    if w is None:
        w = h * ar
    pic = fb.shapes.add_picture(io.BytesIO(src.image.blob), Inches(x), Inches(y), Inches(w), Inches(h))
    if caption:
        el = copy.deepcopy(cap_tmpl)
        fb.shapes._spTree.append(el)
        cap = fb.shapes[-1]
        cw = cap_w or max(w, 1.6)
        cap.left, cap.top, cap.width, cap.height = Inches(x + w / 2 - cw / 2), Inches(y + h + 0.02), Inches(cw), Inches(0.32)
        p = cap.text_frame.paragraphs[0]
        p.runs[0].text = caption
        p.runs[0].font.size = Pt(12)
        for r in p.runs[1:]:
            r._r.getparent().remove(r._r)
        group(fb, [pic, cap], 'Photo ' + caption)
    return w, h


place('Picture 5', 0.3, 1.15, h=5.1, caption='Integrated table on the air bearing (10-21)', cap_w=3.9)
x = 4.4
for name, cap in (('Picture 8', 'Under the deck'), ('Picture 11', 'Printed battery holder'),
                  ('Picture 13', 'Battery holder CAD')):
    w, _ = place(name, x, 1.15, h=2.1, caption=cap, cap_w=1.75)
    x += w + 0.12
place('Picture 4', 4.4, 3.75, w=2.7, caption='Trim-washer holder CAD', cap_w=2.7)
# key facts beside the washer CAD, formatted like the block-diagram bullets
facts = copy.deepcopy(text_shape(diag, 'Power')._element)
fb.shapes._spTree.append(facts)
fs = fb.shapes[-1]
fs.left, fs.top, fs.width, fs.height = Inches(7.2), Inches(3.7), Inches(2.55), Inches(3.2)
tb = fs.text_frame._txBody
tp = tb.findall(qn('a:p'))
for p in tp[1:]:
    tb.remove(p)
items = [('Frame: ', '3D-printed deck on a 5 kg air-bearing sphere'), ('Wheels: ', '4 in a 45° pyramid, 12 V gearmotors'),
         ('Avionics: ', 'Teensy 4.1, BNO085 IMU, XBee radio, SD log'), ('Trim: ', 'washer stacks set the balance'),
         ('Truth: ', 'deck laser on a wall scale')]
for lab, txt in items:
    p = copy.deepcopy(tp[0])
    r0, r1 = p.findall(qn('a:r'))[:2]
    r0.find(qn('a:t')).text, r1.find(qn('a:t')).text = lab, txt
    for r in (r0, r1):
        r.find(qn('a:rPr')).set('sz', '1300')
    tb.append(p)
tb.remove(tp[0])
fb.notes_slide.notes_text_frame.text = (
    'This is what we built. The table floats on the air-bearing sphere; the deck carries four wheels in a pyramid, the '
    'Teensy, the IMU, the radio and five battery packs in printed holders. Balance is set with stacks of washers in '
    'printed holders; that is how we moved the centre of mass during the balance sessions. The laser that gives our '
    'heading truth sits on the deck and points at a printed scale on the wall.')
move_after(fb, diag)

# DEMO ALL overview: 0.2 in less plot height so the four result lines fit at their size
s = slide_by_title('DEMO ALL: The Whole Run')
pic = next(sh for sh in s.shapes if sh.shape_type == 13)
cx = pic.left + pic.width / 2
ar = pic.width / pic.height
pic.height = pic.height - Inches(0.2)
pic.width = int(pic.height * ar)
pic.left = int(cx - pic.width / 2)
box = text_shape(s, 'MEASURE')
box.top, box.height = box.top - Inches(0.2), box.height + Inches(0.2)

# backup banner that wrapped to two lines
takeaway(slide_by_title('Data: False Dead-Motor Flags'),
         'Static friction needs up to 0.27 effort; stopped-wheel threshold raised to 0.4')

prs.save(OUT)
print('saved', OUT)
