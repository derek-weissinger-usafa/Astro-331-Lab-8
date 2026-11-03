"""Final deck rebuild (2026-11-02, Fable): projector-sized figures on the nine presented data slides and tighter
text, applied to the team's current file so their edits (e.g. the Future Work slide) survive.
    python Reports/deck_builder/rebuild_final.py <in.pptx> <out.pptx>
Figures: Reports/figures/final_2026-11-02/deck_figs.py (run it first)."""
import os, sys
from PIL import Image
from pptx import Presentation
from pptx.util import Inches, Pt

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
FIG = os.path.join(ROOT, 'Reports', 'figures', 'final_2026-11-02')
SRC, OUT = sys.argv[1], sys.argv[2]
PIC_Y, PIC_X, PIC_W, BOTTOM = 1.55, 0.3, 9.4, 6.95

prs = Presentation(SRC)


def slide_by_title(start):
    for s in prs.slides:
        if s.shapes.title is not None and s.shapes.title.text.startswith(start):
            return s
    raise KeyError(start)


def picture_shape(slide):
    return next(sh for sh in slide.shapes if sh.shape_type == 13)


def text_shape(slide, start):
    return next(sh for sh in slide.shapes if sh.has_text_frame and sh.text_frame.text.startswith(start))


def set_run_sizes(shape, size):
    for p in shape.text_frame.paragraphs:
        for r in p.runs:
            r.font.size = Pt(size)


def refig(title, fig, max_h, below_start='MEASURE', text_size=None):
    """Replace the slide's picture with fig (fitted to the slide width, at most max_h tall) and move the
    text block that starts with below_start to sit under it."""
    s = slide_by_title(title)
    old = picture_shape(s)
    old._element.getparent().remove(old._element)
    path = os.path.join(FIG, fig)
    iw, ih = Image.open(path).size
    w = PIC_W
    h = w * ih / iw
    if h > max_h:
        h = max_h
        w = h * iw / ih
    s.shapes.add_picture(path, Inches(PIC_X + (PIC_W - w) / 2), Inches(PIC_Y), Inches(w), Inches(h))
    box = text_shape(s, below_start)
    top = PIC_Y + h + 0.08
    box.top = Inches(top)
    box.height = Inches(BOTTOM - top)
    if text_size:
        set_run_sizes(box, text_size)
    return s


def mcen(slide, **parts):
    """Replace the text after a MEASURE/COMPARE/EXPLAIN/NEXT label."""
    box = text_shape(slide, 'MEASURE')
    for p in box.text_frame.paragraphs:
        lab = p.text.split(':')[0]
        if lab in parts:
            rs = p.runs
            rs[1].text = parts[lab]
            for r in rs[2:]:
                r._r.getparent().remove(r._r)


def takeaway(slide, text):
    box = text_shape(slide, 'Takeaway')
    rs = box.text_frame.paragraphs[0].runs
    rs[1].text = text
    for r in rs[2:]:
        r._r.getparent().remove(r._r)


# ---- slide 8: laser truth (bullets below, not MCEN)
refig('Hardware: Laser Heading Truth', 'd9_laser_setup.png', 3.45, below_start='Scale:')

# ---- slide 9: MAG TEST
s = refig('Data: Magnetometer Yaw Aid', 'd1_magtest.png', 3.45)
mcen(s, MEASURE=' MAG TEST on the clamped table (true heading 0): stopped, 14 wheel direction patterns, ±10° kicks, 30 min '
               'drift run; 57 min, graded on the Teensy.',
     COMPARE=' Requirement ±1°. The same data replayed with the aid off: the gyro alone drifts 5.7 °/h.',
     EXPLAIN=' Run 1 (references learned in place): 36/37, worst 1.02°. Run 2 (predicting new patterns): 10/15; the '
             'motor-magnet model moved ≈ 1° between sessions. Each new wheel pattern costs 0.3–1.2°.',
     NEXT=' Learning restored. The presentation spot bends the field up to 14° (backup), so the demo runs gyro-only.')

# ---- slide 10: balance
s = refig('Data: Balancing the Table', 'd2_balance.png', 3.45)
mcen(s, MEASURE=' Rest tilt after each trim move (DEMO LEVEL, 10-31); free-swing fits give the CG height.',
     COMPARE=' Within ±5° is comfortable for DEMO LEVEL; tilted slews need the CG within ≈ 1 mm of the pivot.',
     EXPLAIN=' 10-31: −0.9 / −0.7° at CG ≈ 13 mm. 11-01: raising the CG to ≈ 3 mm made trims land 2–4× off and the '
             'table tip twice; demo setup is stiff again (CG ≈ 10 mm), rest tilt −2.5 / +0.1°.',
     NEXT=' Tilted targets need a rigid, repeatable mass layout and CG ≲ 1 mm (future work: automatic balancer).')

# ---- slide 11: laser holds
s = refig('Data: Heading Holds by Laser', 'd3_laser_holds.png', 3.45)
mcen(s, MEASURE=' DEMO LEVEL heading holds on the floating table; laser read every 30–60 s; each run from its first reading.',
     COMPARE=' Requirement: no visible jitter or drift, |error| < 1°.',
     EXPLAIN=' Calibrated 5 min after power-on: −0.78° in 2.5 min (gyro z bias still warming). After 10+ min: a 0.25° '
             'band for 5 min with no trend; 0.15° for 3 min.',
     NEXT=' Done: power on, 10 min warm-up, CALIBRATE 180 clamped is the demo procedure.')

# ---- slide 12: heading leak
s = refig('Data: Estimator Heading Leak', 'd4_heading_leak.png', 3.55)
mcen(s, MEASURE=' Laser error after each full-turn SLEW YAW on 11-01; integrated gyro vs estimator vs laser.',
     COMPARE=' Every run passed 4/4 by the Teensy\'s own grading: the estimate said 0° every time.',
     EXPLAIN=' Errors scattered −0.1 to +2.3° and flipped sign with the turn direction; the estimator turned 360° while '
             'its gyro said 363–364°. The accelerometer update was moving heading (Code: Gravity Update Fix).',
     NEXT=' Done: heading-free accelerometer gain + yaw scale 1.0045 from a replay fit. DEMO ALL: 0.05°.')

# ---- slide 13: SD stall
s = refig('Data: First-Run Start Failures', 'd5_sd_stall.png', 3.95, text_size=13)
mcen(s, MEASURE=' Encoder counts, measured speed and duty in the first 2 s after arming (DEMO_0016, 100 Hz log).',
     COMPARE=' Expected a smooth spin-up. Instead 7 of 7 first arms after a boot flagged every wheel dead; later arms worked.',
     EXPLAIN=' The first write to a new log file searches the card for space: loop() froze ≈ 0.8 s, then the wheel loop saw '
             'one huge step.',
     NEXT=' Done: log file written and flushed before arming; a wheel-loop step > 50 ms skips the fault checks. '
          'Next: wheel loop on a timer interrupt.')

# ---- slide 14: DEMO ALL overview
s = refig('DEMO ALL: The Whole Run', 'd6_demo_all.png', 4.35, text_size=12.5)
mcen(s, MEASURE=' DEMO ALL (DEMO_0020, 11-01 23:13), first arm after a boot: heading, tilt, wheel speeds.',
     COMPARE=' Each part graded on board against its requirement; laser read at the end of each part.',
     EXPLAIN=' Hold 60 s; four slews through 360°; wheel B cut 10 s later, table held on A, C, D. No reset between parts.',
     NEXT=' Requirements 2 and 3 on the next slides; requirement 1 by laser on Heading Holds by Laser.')

# ---- slide 15: requirement 2
s = refig('DEMO ALL: Requirement 2', 'd7_req2.png', 3.5)
mcen(s, MEASURE=' Laser before and after the four slews (one full turn); worst yaw error in each 5 s target hold.',
     COMPARE=' Requirement ±1°. Before the estimator fix the same test ended +2.3° (CCW) and −1.8° (CW) off.',
     EXPLAIN=' Laser −0.05° → +0.02…−0.03°: ≈ 0.05° after 360°. Targets 0.03 / 0.02 / 0.13 / 0.05°; roll and pitch '
             'within 0.23° of rest.',
     NEXT=' Met. Replaying this run through the old estimator ends 0.46° off (1.7–2.3° on earlier runs).')

# ---- slide 16: requirement 3
s = refig('DEMO ALL: Requirement 3', 'd8_req3.png', 3.95, text_size=13)
mcen(s, MEASURE=' Attitude error and wheel speeds around the cut (motor held at zero current; controller not told).',
     COMPARE=' Pass: detected ≤ 1 s, back within ±1° ≤ 10 s, then ±1° for 30 s.',
     EXPLAIN=' Detected 0.50 s; peak 0.90° (estimate), laser peak 0.3°; 30 s hold max 0.36° yaw, 0.01° roll/pitch.',
     NEXT=' Met, on the first arm after a boot (the case that always failed before).')

prs.save(OUT)
print('saved', OUT)
