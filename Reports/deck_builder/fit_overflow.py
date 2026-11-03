"""Fix text that is taller than its box, using PowerPoint's own measurements (fragility.ps1 prints
'OVERFLOW slide N 'name': text X pt > box Y pt'). Such a box looks fine until someone edits it, and then PowerPoint
suddenly shrinks it. Each box is grown into free space below it (never over another shape, never below 6.95 in, or
7.0 in for placeholders); if there isn't room, its font sizes are scaled down to fit (in 0.5 pt steps).
    python Reports/deck_builder/fit_overflow.py <in.pptx> <overflow report.txt> <out.pptx>"""
import math, re, sys
from pptx import Presentation
from pptx.oxml.ns import qn

SRC, REPORT, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
E, PT = 914400, 12700
prs = Presentation(SRC)


def find(slide, name):
    for sh in slide.shapes:
        if sh.shape_type == 6:
            for g in sh.shapes:
                if g.name == name:
                    return g
        elif sh.name == name:
            return sh


def has_fill(sh):
    sp = sh._element.find(qn('p:spPr'))
    return sp is not None and sp.find(qn('a:solidFill')) is not None


for line in open(REPORT, encoding='utf-8-sig'):
    m = re.match(r"OVERFLOW slide (\d+) '(.+?)': text ([\d,.]+) pt > box ([\d,.]+) pt", line.strip())
    if not m:
        continue
    n, name = int(m.group(1)), m.group(2)
    need = float(m.group(3).replace(',', '')) * PT + int(0.03 * E)
    slide = prs.slides[n - 1]
    sh = find(slide, name)
    limit = int((7.0 if sh.is_placeholder else 6.95) * E)
    for o in slide.shapes:
        if o.shape_id == sh.shape_id:
            continue
        if min(sh.left + sh.width, o.left + o.width) - max(sh.left, o.left) > 0.1 * E and o.top >= sh.top + sh.height - 0.02 * E:
            limit = min(limit, o.top - int(0.06 * E))
    if sh.top + need <= limit and (not has_fill(sh) or sh.top + need <= sh.top + sh.height + 0.2 * E):
        sh.height = int(need)
        print(f'slide {n} {name}: grown to {need / E:.2f} in')
        continue
    avail = max(sh.height, limit - sh.top)
    k = avail / need
    for e in sh._element.iter(qn('a:rPr'), qn('a:endParaRPr')):
        if e.get('sz'):
            e.set('sz', str(int(math.floor(int(e.get('sz')) * k / 50) * 50)))
    sh.height = int(avail)
    print(f'slide {n} {name}: fonts x{k:.2f}')

prs.save(OUT)
print('saved', OUT)
