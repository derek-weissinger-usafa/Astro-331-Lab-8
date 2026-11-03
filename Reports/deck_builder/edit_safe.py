"""Make the deck safe to edit in PowerPoint (2026-11-02) without changing how it looks:
  1. A panel made of a background shape with one text box on it (code panels) becomes one filled shape, so the
     background grows and moves with its text.
  2. A card made of a background shape and several parts (stat cards, numbered cards) becomes a group, so it moves as
     one piece; its text boxes shrink text on overflow instead of spilling out of the card.
  3. Every other text box is stretched over the free space below it (never over another shape, never below 6.95 in)
     and set to shrink text on overflow, instead of growing over the figure or off the slide.
  4. Every paragraph gets an end-of-paragraph format copied from its last run, so an emptied line keeps its font.
    python Reports/deck_builder/edit_safe.py <in.pptx> <out.pptx>"""
import copy, sys
from lxml import etree
from pptx import Presentation
from pptx.oxml.ns import qn

SRC, OUT = sys.argv[1], sys.argv[2]
E = 914400
BOTTOM = int(6.95 * E)
prs = Presentation(SRC)
stats = dict(merged=0, grouped=0, stretched=0, autofit=0, endpara=0)


def set_autofit(bodyPr, kind='normAutofit'):
    for tag in ('a:spAutoFit', 'a:normAutofit', 'a:noAutofit'):
        for e in bodyPr.findall(qn(tag)):
            bodyPr.remove(e)
    warp = bodyPr.find(qn('a:prstTxWarp'))
    idx = list(bodyPr).index(warp) + 1 if warp is not None else 0
    bodyPr.insert(idx, etree.SubElement(bodyPr, qn('a:' + kind)))
    stats['autofit'] += 1


def has_fill(sh):
    sp = sh._element.find(qn('p:spPr'))
    return sp is not None and (sp.find(qn('a:solidFill')) is not None or sp.find(qn('a:gradFill')) is not None or
                               sp.find(qn('a:blipFill')) is not None)


def top_anchored(sh):
    return sh.text_frame._txBody.find(qn('a:bodyPr')).get('anchor', 't') == 't'


def is_text(sh):
    return sh.has_text_frame and sh.text_frame.text.strip() and not sh.is_placeholder


def inside(t, r):
    return (t.left >= r.left - 0.08 * E and t.top >= r.top - 0.08 * E and t.left + t.width <= r.left + r.width + 0.08 * E
            and t.top + t.height <= r.top + r.height + 0.15 * E)


def box(sh):
    return sh.left, sh.top, sh.left + sh.width, sh.top + sh.height


def ins(bodyPr, name, default):
    v = bodyPr.get(name)
    return int(v) if v is not None else default


def next_id(slide):
    return max(int(e.get('id')) for e in slide.shapes._spTree.iter(qn('p:cNvPr'))) + 1


def timing_ids(slide):
    t = slide._element.find(qn('p:timing'))
    return {e.get('spid') for e in t.iter() if e.get('spid')} if t is not None else set()


for slide in prs.slides:
    tree = slide.shapes._spTree
    animated = timing_ids(slide)
    shapes = list(slide.shapes)
    used = set()
    # ---- 1, 2: background shapes with text on them
    for r in shapes:
        if r.shape_type != 1 or (r.has_text_frame and r.text_frame.text.strip()) or not has_fill(r):
            continue
        parts = [s for s in shapes if s is not r and not s.is_placeholder and id(s) not in used and inside(s, r)]
        if not parts or any(str(s.shape_id) in animated for s in parts + [r]):
            continue
        texts = [s for s in parts if is_text(s)]
        if len(parts) == 1 and len(texts) == 1 and parts[0].shape_type == 17:
            t = parts[0]
            tb = copy.deepcopy(t._element.find(qn('p:txBody')))
            bp = tb.find(qn('a:bodyPr'))
            l0, t0, r0 = ins(bp, 'lIns', 91440), ins(bp, 'tIns', 45720), ins(bp, 'rIns', 91440)
            bp.set('lIns', str(max(0, t.left - r.left + l0)))
            bp.set('tIns', str(max(0, t.top - r.top + t0)))
            bp.set('rIns', str(max(0, (r.left + r.width) - (t.left + t.width) + r0)))
            bp.set('bIns', str(int(0.05 * E)))
            bp.set('anchor', 't')
            bp.set('wrap', 'square')
            set_autofit(bp)
            old = r._element.find(qn('p:txBody'))
            if old is not None:
                r._element.remove(old)
            r._element.append(tb)
            r._element.find('.//' + qn('p:cNvPr')).set('name', 'Panel ' + t.name)
            tree.remove(t._element)
            used.add(id(t))
            stats['merged'] += 1
            continue
        # group: background first, then the parts in their z-order
        members = sorted([r] + parts, key=lambda s: list(tree).index(s._element))
        x0 = min(s.left for s in members); y0 = min(s.top for s in members)
        x1 = max(s.left + s.width for s in members); y1 = max(s.top + s.height for s in members)
        for s in texts:  # shrink on overflow; the lowest text box may use the card down to its bottom edge
            set_autofit(s.text_frame._txBody.find(qn('a:bodyPr')))
        low = max(texts, key=lambda s: s.top) if texts else None
        if (low is not None and top_anchored(low) and not has_fill(low)
                and low.top + low.height < r.top + r.height - int(0.03 * E)):
            low.height = r.top + r.height - int(0.03 * E) - low.top
        g = etree.SubElement(tree, qn('p:grpSp'))
        tree.remove(g)
        nv = etree.SubElement(g, qn('p:nvGrpSpPr'))
        c = etree.SubElement(nv, qn('p:cNvPr')); c.set('id', str(next_id(slide))); c.set('name', 'Card ' + r.name)
        etree.SubElement(nv, qn('p:cNvGrpSpPr')); etree.SubElement(nv, qn('p:nvPr'))
        gp = etree.SubElement(g, qn('p:grpSpPr'))
        xf = etree.SubElement(gp, qn('a:xfrm'))
        for tag, a, b in (('a:off', x0, y0), ('a:ext', x1 - x0, y1 - y0), ('a:chOff', x0, y0), ('a:chExt', x1 - x0, y1 - y0)):
            e = etree.SubElement(xf, qn(tag))
            e.set('x' if 'ff' in tag or 'Off' in tag else 'cx', str(a)); e.set('y' if 'ff' in tag or 'Off' in tag else 'cy', str(b))
        members[0]._element.addprevious(g)
        for s in members:
            g.append(s._element)
            used.add(id(s))
        stats['grouped'] += 1

    # ---- 3: free text boxes (not in a card)
    shapes = list(slide.shapes)  # top level only: groups appear as one shape
    for t in shapes:
        if not is_text(t) or t.shape_type == 6:
            continue
        bp = t.text_frame._txBody.find(qn('a:bodyPr'))
        if bp.get('wrap') == 'none':
            continue  # single-line labels: leave as they are
        if not has_fill(t) and top_anchored(t):  # stretching a filled or centred box would show
            l, top, rgt, bot = box(t)
            limit = BOTTOM
            for o in slide.shapes:
                if o is t or o.shape_id == t.shape_id:
                    continue
                ol, ot, orr, ob = box(o)
                if min(rgt, orr) - max(l, ol) > 0.1 * E and ot >= bot - int(0.02 * E):
                    limit = min(limit, ot - int(0.06 * E))
            if limit > bot:
                t.height = limit - top
                stats['stretched'] += 1
        set_autofit(bp)

# ---- 4: end-of-paragraph formats (text boxes, shapes, table cells; not placeholders, which inherit from the layout)
for slide in prs.slides:
    for sp in slide.shapes._spTree.iter(qn('p:sp'), qn('p:graphicFrame')):
        if sp.find('.//' + qn('p:ph')) is not None:
            continue
        for p in sp.iter(qn('a:p')):
            if p.find(qn('a:endParaRPr')) is not None:
                continue
            rs = p.findall(qn('a:r'))
            if not rs or rs[-1].find(qn('a:rPr')) is None:
                continue
            e = copy.deepcopy(rs[-1].find(qn('a:rPr')))
            e.tag = qn('a:endParaRPr')
            p.append(e)
            stats['endpara'] += 1

prs.save(OUT)
print('saved', OUT, stats)
