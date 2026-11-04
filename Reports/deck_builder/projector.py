"""Projector pass on the final deck (2026-11-03):
  1. every data slide's plot fills the space between the takeaway banner and the MEASURE/COMPARE text: the text block
     is fitted to its measured height and sits at the bottom, and the plot takes the rest of the height and the full
     content width (presented plots are redrawn at that exact size by Reports/figures/final_2026-11-02/deck_figs.py;
     backup plots are trimmed of their white margins and scaled to fit);
  2. stronger colour for a projector: slide colours (takeaway banners, labels, accents) are remapped to more saturated
     versions, and plot / diagram images that were not redrawn get their saturation raised (photos are left alone);
  3. "firmware" -> "software" in all slide text, tables and speaker notes.
Works on the team's current file, so their edits survive. Run from the repo root, after deck_figs.py:
    python Reports/deck_builder/projector.py <in.pptx> <out.pptx>"""
import io, sys
from pptx import Presentation
from pptx.oxml.ns import qn
from pptx.util import Emu, Inches
from PIL import Image, ImageEnhance, ImageOps

SRC, OUT = sys.argv[1], sys.argv[2]
FIG = 'Reports/figures/final_2026-11-02/'
WR = 'Reports/figures/writeup_2026-10-31/'
MARK = ' (projector)'          # suffix on picture names already saturated, so a re-run doesn't saturate twice
SAT = 1.9                      # saturation factor for plots / diagrams that were not redrawn

# slide title prefix -> figure redrawn at its plot-area size (deck_figs.py SLOT)
REDRAWN = {
    'Data: IMU Data Integrity': 'd10_imu.png', 'Data: Magnetometer Yaw Aid': 'd1_magtest.png',
    'Data: Balancing the Table': 'd2_balance.png', 'Data: Heading Holds by Laser': 'd3_laser_holds.png',
    'Data: Tilted Slews vs. Gravity': 'd11_tilt.png', 'DEMO ALL: The Whole Run': 'd6_demo_all.png',
    'DEMO ALL: Requirement 1': 'd12_req1.png', 'DEMO ALL: Requirement 2': 'd7_req2.png',
    'DEMO ALL: Requirement 3': 'd8_req3.png', 'Hardware: Laser Heading Truth': 'd9_laser_setup.png',
}
# backup plots whose image text said "firmware": redrawn with "software" (writeup make_figs.py), then saturated
SOURCE = {'Data: Magnetometer at the Presentation Spot': WR + 'f11_room_distortion.png',
          'Data: STEP TEST 1': WR + 'f05_heading_hardiron.png'}
# height (in) of each slide's MEASURE/COMPARE text, measured in PowerPoint (TextFrame2.TextRange.BoundHeight)
TEXT_H = {
    'Data: IMU Data Integrity': 1.497, 'Data: Magnetometer Yaw Aid': 1.718, 'Data: Balancing the Table': 1.940,
    'Data: Heading Holds by Laser': 1.497, 'Data: Tilted Slews vs. Gravity': 1.275, 'DEMO ALL: The Whole Run': 0.958,
    'DEMO ALL: Requirement 1': 1.053, 'DEMO ALL: Requirement 2': 1.497, 'DEMO ALL: Requirement 3': 0.990,
    'Data: Wheel Speed Control': 1.497, 'Data: Attitude Determination at Rest': 1.275,
    'Data: Magnetometer vs. Wheel Motors': 1.718, 'Data: Magnetometer at the Presentation Spot': 1.718,
    'Data: First DEMO SPIN': 1.497, 'Data: DEMO SPIN Leveling': 1.497, 'Data: DEMO LEVEL Hold': 1.497,
    'Data: Flight Computer Failure': 1.718, 'Data: False Dead-Motor Flags': 1.718, 'Data: STEP TEST 1': 1.497,
    'Data: STEP TEST (Heading Steps)': 1.497, 'Data: FAULT TEST on the Air Bearing': 1.275,
    'Data: Wheel D Encoder Lead': 1.275, 'Data: SLEW YAW on the Air Bearing': 1.275,
}
TEXT_BOTTOM, MARGINS, SLACK = 6.95, 0.16, 0.06     # text box bottom (above the slide number); insets; spare
PLOT_X, PLOT_W, PLOT_TOP, GAP = 0.3, 9.4, 1.50, 0.02
# slide colours -> more saturated versions (same hue family)
RECOLOR = {'2E7D4F': '128A3E', 'E8F3EC': 'D2F0DC', '2F5D8A': '1559B8', 'E9EFF6': 'D6E5F8', 'B7790F': 'DB8A00',
           '9C2F25': 'C41E12', 'FBF1DE': 'FDE3B5'}


def title_of(s):
    return s.shapes.title.text_frame.text.strip() if s.shapes.title is not None else ''


def lookup(table, title):
    for k, v in table.items():
        if title.startswith(k):
            return v
    return None


def saturate(data, factor=SAT):
    im = Image.open(io.BytesIO(data)).convert('RGB')
    out = io.BytesIO()
    ImageEnhance.Color(im).enhance(factor).save(out, 'PNG')
    return out.getvalue()


def trim(data, pad=8):
    im = Image.open(io.BytesIO(data)).convert('RGB')
    box = ImageOps.invert(im.convert('L')).point(lambda v: 255 if v > 18 else 0).getbbox()
    if box:
        box = (max(box[0] - pad, 0), max(box[1] - pad, 0), min(box[2] + pad, im.width), min(box[3] + pad, im.height))
        im = im.crop(box)
    out = io.BytesIO()
    im.save(out, 'PNG')
    return out.getvalue()


def is_graphic(data):
    """Plot or diagram (mostly white background), not a photo."""
    im = Image.open(io.BytesIO(data)).convert('L')
    im.thumbnail((300, 300))
    px = list(im.get_flattened_data()) if hasattr(im, "get_flattened_data") else list(im.getdata())
    return sum(v > 235 for v in px) / len(px) > 0.35


def pictures(shapes):
    for sh in shapes:
        if sh.shape_type == 6:
            yield from pictures(sh.shapes)
        elif sh.shape_type == 13:
            yield sh


def replace_picture(slide, pic, data, box):
    """New picture with `data` in `box` (x, y, w, h inches), in the old one's place in the z-order."""
    x, y, w, h = box
    new = slide.shapes.add_picture(io.BytesIO(data), Inches(x), Inches(y), Inches(w), Inches(h))
    old = pic._element
    new.name = pic.name.replace(MARK, '') + MARK
    descr = old.nvPicPr.cNvPr.get('descr')
    if descr:
        new._element.nvPicPr.cNvPr.set('descr', descr)
    rid = old.blipFill.blip.rEmbed
    old.addprevious(new._element)
    old.getparent().remove(old)
    # An identical image is de-duplicated onto the same relationship, so drop it only if nothing uses it now
    # (python-pptx's drop_rel also drops a relationship that is still referenced once).
    if not slide._element.xpath(f'.//*[@r:embed="{rid}" or @r:link="{rid}" or @r:id="{rid}"]'):
        slide.part._rels.pop(rid)


def fit(data, slot):
    """Largest box with the image's aspect inside slot (x, y, w, h), centred."""
    x, y, w, h = slot
    iw, ih = Image.open(io.BytesIO(data)).size
    k = min(w / iw, h / ih)
    return x + (w - iw * k) / 2, y + (h - ih * k) / 2, iw * k, ih * k


prs = Presentation(SRC)
E = 914400.0
log = []

for n, s in enumerate(prs.slides, 1):
    t = title_of(s)
    # 1. layout: text block fitted to its height at the bottom, plot fills the rest
    th = lookup(TEXT_H, t)
    mcen = next((sh for sh in s.shapes if sh.has_text_frame and sh.text_frame.text.startswith('MEASURE')), None)
    pics = [p for p in pictures(s.shapes)]
    slot = None
    if th is not None and mcen is not None and len(pics) == 1:
        h = th + MARGINS + SLACK
        mcen.top, mcen.height = Inches(TEXT_BOTTOM - h), Inches(h)
        slot = (PLOT_X, PLOT_TOP, PLOT_W, TEXT_BOTTOM - h - GAP - PLOT_TOP)
    for p in pics:
        name = lookup(REDRAWN, t) if len(pics) == 1 else None
        src = lookup(SOURCE, t) if len(pics) == 1 else None
        if name:
            data = open(FIG + name, 'rb').read()
            box = fit(data, slot) if slot else fit(data, (p.left / E, p.top / E, p.width / E, p.height / E))
            replace_picture(s, p, data, box)
            log.append(f'{n}: {name} redrawn, {box[2]:.2f} x {box[3]:.2f} in')
        elif src or (p.name.find(MARK) < 0 and is_graphic(p.image.blob)):
            data = saturate(trim(open(src, 'rb').read()) if src else (trim(p.image.blob) if slot else p.image.blob))
            if slot:
                box = fit(data, slot)
            else:
                box = (p.left / E, p.top / E, p.width / E, p.height / E)
            if p._element.getparent().tag == qn('p:grpSp') and not slot:
                # picture inside a group: swap the image data in place
                part = s.part.related_part(p._element.blipFill.blip.rEmbed)
                part._blob = data
                p.name = p.name + MARK
            else:
                replace_picture(s, p, data, box)
            log.append(f'{n}: picture {p.name.replace(MARK, "")} saturated' + (f', fitted {box[2]:.2f} x {box[3]:.2f} in' if slot else ''))
        elif slot and p.name.find(MARK) >= 0:
            box = fit(p.image.blob, slot)
            p.left, p.top, p.width, p.height = (Inches(v) for v in box)

    # 2. slide colours
    for el in s._element.iter(qn('a:srgbClr')):
        v = el.get('val').upper()
        if v in RECOLOR:
            el.set('val', RECOLOR[v])

# 3. firmware -> software, everywhere text lives
WORDS = (('firmware', 'software'), ('Firmware', 'Software'), ('FIRMWARE', 'SOFTWARE'))


def fix_frame(tf, where):
    for para in tf.paragraphs:
        before = ''.join(r.text for r in para.runs)
        for r in para.runs:
            for a, b in WORDS:
                if a in r.text:
                    r.text = r.text.replace(a, b)
        after = ''.join(r.text for r in para.runs)
        if after != before:
            log.append(f'{where}: {after[:110]}')
        if 'irmware' in after.lower():
            raise SystemExit(f'"firmware" split across runs at {where}: {after}')


def walk(shapes, where):
    for sh in shapes:
        if sh.shape_type == 6:
            walk(sh.shapes, where)
        elif sh.has_text_frame:
            fix_frame(sh.text_frame, where)
        elif getattr(sh, 'has_table', False) and sh.has_table:
            for row in sh.table.rows:
                for cell in row.cells:
                    fix_frame(cell.text_frame, where + ' table')


for n, s in enumerate(prs.slides, 1):
    walk(s.shapes, f'slide {n}')
    if s.has_notes_slide:
        fix_frame(s.notes_slide.notes_text_frame, f'notes {n}')

prs.save(OUT)
print('\n'.join(log))
print('saved', OUT)
