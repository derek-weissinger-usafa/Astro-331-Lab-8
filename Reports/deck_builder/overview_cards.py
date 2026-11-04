"""Slide 2 (2026-11-03): the three cards become Overview, Objectives and Why it matters (the objective box under them
was removed by the team), so the talk states the overview, the objectives and the new KestrelSAT capability up front.
Third person, active voice, same card formatting.
    python Reports/deck_builder/overview_cards.py <in.pptx> <out.pptx>"""
import sys
from pptx import Presentation

SRC, OUT = sys.argv[1], sys.argv[2]
prs = Presentation(SRC)
s = next(x for x in prs.slides if x.shapes.title is not None and
         x.shapes.title.text.startswith(('Why Attitude Control', 'Overview & Objectives')))


def card_text(start):
    for sh in s.shapes:
        for g in (sh.shapes if sh.shape_type == 6 else [sh]):
            if g.has_text_frame and g.text_frame.text.strip().startswith(start):
                return g
    raise KeyError(start)


def set_text(sh, text):
    p = sh.text_frame.paragraphs[0]
    p.runs[0].text = text
    for r in p.runs[1:]:
        r._r.getparent().remove(r._r)
    for extra in sh.text_frame.paragraphs[1:]:
        extra._p.getparent().remove(extra._p)


p = s.shapes.title.text_frame.paragraphs[0]
p.runs[0].text = 'Overview & Objectives'
for r in p.runs[1:]:
    r._r.getparent().remove(r._r)

CARDS = [
    (('POINT', 'OVERVIEW'), 'OVERVIEW', ('Antennas', 'The team built'),
     'The team built a 3-axis reaction-wheel attitude control system on the small air bearing table, which turns '
     'almost without friction, like a satellite in orbit.',
     ('Req. 1: hold a heading', 'Air bearing'), 'Air bearing + 4 wheels'),
    (('KNOW', 'OBJECTIVES'), 'OBJECTIVES', ('You can only point', 'Hold a commanded'),
     'Hold a commanded attitude in all three axes with reaction wheels, and fuse gyroscope, accelerometer and '
     'magnetometer data to know the attitude in any state.',
     ('Req. 2: attitude', 'Req. 1–2'), 'Req. 1–2: hold, know to ±1°'),
    (('SURVIVE', 'NEW CAPABILITY'), 'WHY IT MATTERS', ('Wheels wear out', 'The table detects'),
     'Antennas, solar arrays and cameras only work when aimed. Wheels wear out: Kepler lost 2 of its 4 and its '
     'primary mission. This table keeps pointing when a wheel fails.',
     'Req. 3: detect', 'Req. 3: detect + compensate'),
]
def alts(old, new):
    """Earlier texts of a box plus its target text, so the script also runs on an already-updated deck."""
    return (old if isinstance(old, tuple) else (old,)) + (new[:15],)


for old_head, head, old_body, body, old_chip, chip in CARDS:
    set_text(card_text(alts(old_head, head)), head)
    set_text(card_text(alts(old_body, body)), body)
    # the green chips at the bottom of the cards are removed (team's choice, 2026-11-03)
    try:
        el = card_text(alts(old_chip, chip))._element
        el.getparent().remove(el)
    except KeyError:
        pass  # already removed

s.notes_slide.notes_text_frame.text = (
    'This is the project in one slide. Overview: we built a three-axis reaction-wheel attitude control system on the '
    'small air bearing table. The table floats on air, so it turns almost without friction, like a satellite in orbit. '
    'Objectives: hold a commanded attitude with the wheels, and fuse the gyro, accelerometer and magnetometer to know '
    'the attitude in any state; those are requirements 1 and 2. Why it matters: antennas, solar arrays and cameras only '
    'work when aimed, and reaction wheels are the parts that wear out. Kepler lost two of its four wheels and, with '
    'them, its primary mission. Our table keeps pointing when a wheel fails, which is requirement 3 and the new '
    'capability beyond the earlier KestrelSAT labs.')

prs.save(OUT)
print('saved', OUT)
