"""Consolidate the final deck to the prelim's 19 presented slides; the rest go behind a Backup divider (nothing deleted).
Works on the team's current file (keeps their edits, e.g. the Future Work slide)."""
import copy, re, sys
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.oxml.ns import qn

SRC, OUT = sys.argv[1], sys.argv[2]
prs = Presentation(SRC)
S = list(prs.slides)
T = {i + 1: (s.shapes.title.text if s.shapes.title else '') for i, s in enumerate(S)}
assert T[51].startswith('Future Work') and T[52].startswith('GenAI') and len(S) == 52, T

def shape(slide, name):
    return next(sh for sh in slide.shapes if sh.name == name)

def replace_in(slide, old, new, notes=False):
    hit = False
    if notes:
        tf = slide.notes_slide.notes_text_frame
        for p in tf.paragraphs:
            for r in p.runs:
                if old in r.text:
                    r.text = r.text.replace(old, new); hit = True
    else:
        for sh in slide.shapes:
            if sh.has_text_frame:
                for p in sh.text_frame.paragraphs:
                    for r in p.runs:
                        if old in r.text:
                            r.text = r.text.replace(old, new); hit = True
    assert hit, (T[S.index(slide) + 1], old)

# ---- cross-references: titles instead of slide numbers (numbers change with the reorder)
replace_in(S[5], '(slide 9)', '(Code: Gravity Update Fix)', notes=True)
replace_in(S[42], 'Cause and fix: slide 9.', 'Cause and fix: Code: Gravity Update Fix.')
replace_in(S[44], 'Next three slides: one per requirement.', 'Requirements 2 and 3 next; Requirement 1 by laser: Heading Holds by Laser.')
replace_in(S[24], 'the presentation spot bends the field (next slide), so the demo runs gyro-only.',
           'the presentation spot bends the field up to 14° (backup), so the demo runs gyro-only.')
# backup slides
replace_in(S[4], 'slide 43', 'Estimator Heading Leak')
replace_in(S[4], 'slide 44', 'First-Run Start Failures')
replace_in(S[7], '(slide 26)', '(Magnetometer at the Presentation Spot)')
replace_in(S[11], '(slide 44)', '(start failures)')
replace_in(S[21], '(slide 42)', '(Heading Holds by Laser)')
replace_in(S[23], '(next slide)', '(MAG TEST)')
replace_in(S[30], '(slide 33)', '(Balancing the Table)')
replace_in(S[36], 'slide 48', 'DEMO ALL: Requirement 3')
replace_in(S[39], '(slide 43)', '(Estimator Heading Leak)')
replace_in(S[39], 'on slide 43', 'on the heading-leak slide', notes=True)

# ---- Why slide: fold the objective in (the Overview/Objectives slide goes to backup)
box = shape(S[2], 'TextBox 106')
tf = box.text_frame
p0 = tf.paragraphs[0]
newp = copy.deepcopy(p0._p)
p0._p.addprevious(newp)
rs = newp.findall(qn('a:r'))
rs[0].find(qn('a:t')).text = 'Objective: '
rs[1].find(qn('a:t')).text = ('3-DOF reaction-wheel attitude control on the air bearing table, with gyro, accelerometer and '
                               'magnetometer fusion to know the attitude in any state.')
for r in rs[2:]:
    newp.remove(r)
for p in tf.paragraphs:
    for r in p.runs:
        r.font.size = Pt(13.5)
box.top = Inches(5.38); box.height = Inches(1.55)

S[49].shapes.title.text = 'Lessons Learned'  # the team's Future Work slide follows it

# ---- Backup divider
lay = S[2].slide_layout
bk = prs.slides.add_slide(lay)
for ph in list(bk.placeholders):
    if ph.placeholder_format.idx != 0:
        ph._element.getparent().remove(ph._element)
bk.shapes.title.text = 'Backup Slides'
num = next(sh._element for sh in S[2].shapes if sh.is_placeholder and sh.placeholder_format.idx == 12)
bk.shapes._spTree.append(copy.deepcopy(num))
tb = bk.shapes.add_textbox(Inches(0.5), Inches(2.6), Inches(9.0), Inches(2.0))
tf = tb.text_frame; tf.word_wrap = True
for k, (t, sz, b) in enumerate([('Not presented', 32, True),
                                ('Earlier data, extra detail and alternate views, in briefing order. Available for questions.', 18, False)]):
    p = tf.paragraphs[0] if k == 0 else tf.add_paragraph()
    r = p.add_run(); r.text = t; r.font.size = Pt(sz); r.font.bold = b
    r.font.name = 'Quattrocento Sans'; r.font.color.rgb = RGBColor(0x1F, 0x29, 0x33)
bk.notes_slide.notes_text_frame.text = 'Everything after this slide is backup: not presented, kept for questions.'

# ---- order: 19 presented, divider, backup (original order)
MAIN = [1, 3, 4, 6, 7, 9, 11, 15, 25, 33, 42, 43, 44, 45, 47, 48, 50, 51, 52]
BACK = [i for i in range(1, 53) if i not in MAIN]
sld = prs.slides._sldIdLst
ids = list(sld)
O = {i + 1: ids[i] for i in range(52)}
order = [O[i] for i in MAIN] + [ids[52]] + [O[i] for i in BACK]
for el in ids:
    sld.remove(el)
for el in order:
    sld.append(el)
prs.save(OUT)
print('main:', [T[i][:28] for i in MAIN])
print('backup:', len(BACK), 'slides; total', len(order))
