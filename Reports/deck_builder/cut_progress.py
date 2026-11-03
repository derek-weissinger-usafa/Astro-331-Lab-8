"""Deck edit (2026-11-02): the presented hardware slide becomes the system block diagram (the laser-truth slide moves
to backup in its place), and every progress-report reference is removed from slide text, titles and notes.
    python Reports/deck_builder/cut_progress.py <in.pptx> <out.pptx>"""
import copy, sys
from pptx import Presentation
from pptx.oxml.ns import qn

SRC, OUT = sys.argv[1], sys.argv[2]
prs = Presentation(SRC)


def slides_by_title(start):
    return [s for s in prs.slides if s.shapes.title is not None and s.shapes.title.text.startswith(start)]


def set_title(slide, text):
    p = slide.shapes.title.text_frame.paragraphs[0]
    p.runs[0].text = text
    for r in p.runs[1:]:
        r._r.getparent().remove(r._r)


def replace_text(old, new, expect):
    """Replace in slide text (run by run) and in notes; expect = number of places it must be found."""
    n = 0
    for s in prs.slides:
        for sh in s.shapes:
            frames = [sh.text_frame] if sh.has_text_frame else []
            if sh.has_table:
                frames += [c.text_frame for row in sh.table.rows for c in row.cells]
            for tf in frames:
                for p in tf.paragraphs:
                    for r in p.runs:
                        if old in r.text:
                            r.text = r.text.replace(old, new)
                            n += 1
        if s.has_notes_slide:
            tf = s.notes_slide.notes_text_frame
            if old in tf.text:
                tf.text = tf.text.replace(old, new)
                n += 1
    assert n == expect, (old, n)


# ---- block diagram replaces the laser slide in the presented section; the laser slide goes to backup
laser = slides_by_title('Hardware: Laser Heading Truth')[0]
diag, cad = slides_by_title('Build Progress: Hardware')
ids = list(prs.slides._sldIdLst)
sid = [s.slide_id for s in prs.slides]
el_laser, el_diag = ids[sid.index(laser.slide_id)], ids[sid.index(diag.slide_id)]
mark = el_diag.getnext()
el_laser.addprevious(el_diag)  # diagram into the laser slide's place
mark.addprevious(el_laser)     # laser slide into the diagram's old backup place

set_title(diag, 'Hardware: System Block Diagram')
body = next(sh for sh in diag.shapes if sh.has_text_frame and sh.text_frame.text.startswith('Power system'))
items = [
    ('Power: ', 'separate 5 V logic rail; one 12 V pack per wheel; low-side motor switch, star ground'),
    ('Sensing: ', 'BNO085 gyro, accel, mag at 100 Hz (I²C + data-ready/reset); 400 encoder counts per wheel turn'),
    ('Data: ', 'every sample to the SD card (100 Hz); commands and 2 Hz telemetry over the XBee'),
    ('Heading truth: ', 'deck laser on a wall scale 2.48 m away, ≈ 0.05°, independent of the IMU (backup)'),
]
# One paragraph per item, cloned from the first (keeps the bullet, spacing and the deck font).
tmpl = copy.deepcopy(body.text_frame.paragraphs[0]._p)
for r in tmpl.findall(qn('a:r'))[1:]:
    tmpl.remove(r)
txBody = body.text_frame._txBody
for p in txBody.findall(qn('a:p')):
    txBody.remove(p)
for lab, txt in items:
    p = copy.deepcopy(tmpl)
    r0 = p.find(qn('a:r'))
    r1 = copy.deepcopy(r0)
    p.append(r1)
    for r, text, bold in ((r0, lab, '1'), (r1, txt, '0')):
        r.find(qn('a:t')).text = text
        rpr = r.get_or_add_rPr()
        rpr.set('b', bold)
        rpr.set('sz', '1500')
    txBody.append(p)
diag.notes_slide.notes_text_frame.text = (
    'Everything on the table runs on the Teensy 4.1: it reads the IMU at 100 Hz, runs the estimator and the controller '
    'on every gyro sample, drives four motor drivers, reads four encoders, and logs every sample to its SD card. The '
    'ground station only sends commands and watches a 2 Hz copy of the data over the XBee radio. Logic and motors have '
    'separate batteries so motor surges can\'t reset the computer. Heading truth comes from outside the system: a laser '
    'on the deck pointing at a printed scale on the wall, which is how every heading number in the data slides was '
    'checked.')

# ---- progress-report titles
set_title(cad, 'Hardware: Battery & Washer Holders')
set_title(slides_by_title('Progress: Software Build & Tests (Cont.)')[0], 'Test Record: Software & Hardware (Cont.)')
set_title(slides_by_title('Progress: Software Build & Tests')[0], 'Test Record: Software & Hardware')
set_title(slides_by_title('Where We Stand')[0], 'Results and Open Items')

# ---- progress-report wording in slide text and notes
replace_text('New on the Teensy since 10-30: ', 'Also on the Teensy: ', 1)
replace_text('CLAUDE.md, TODO.md, PROGRESS_LOG.md, test reports', 'CLAUDE.md, TODO.md, the project log, test reports', 1)
replace_text('Full record of work: PROGRESS_LOG.md and the git history', 'Full record of work: the project log and the git history', 1)
replace_text('Since the prelim we changed the estimator in one important way:', 'One estimator design choice matters most:', 1)
replace_text('This is the magnetometer update we added after the prelim.', 'This is the magnetometer heading update.', 1)
replace_text('the most important result we found this week', 'the most important result of our testing', 1)
replace_text('cost us table time this week.', 'cost us table time.', 1)
replace_text('Ten days since the prelim; these are the last two. Each row is a logged test with its result; '
             'PROGRESS_LOG.md has the details.', 'Each row is a logged test with its result; the project log has the details.', 1)

prs.save(OUT)
print('saved', OUT)
