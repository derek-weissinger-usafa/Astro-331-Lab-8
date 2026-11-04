"""Code slides' speaker notes (2026-11-04): after the spoken notes, a "Source lines" section maps each line of the
slide's code panel to the file and line numbers it was taken from (PlatformIO project folder "Astro 331 Lab 8/",
commit c9b219e, the software frozen for the presentation). Re-running replaces the section.
    python Reports/deck_builder/code_notes.py <in.pptx> <out.pptx>"""
import sys
from pptx import Presentation

SRC, OUT = sys.argv[1], sys.argv[2]
HEAD = 'Source lines (Astro 331 Lab 8/src and include, commit c9b219e):'

NOTES = {
    'Code: Attitude Estimation (MEKF)': [
        'Predict (AttitudeEKF::predict, src/attitude_ekf.cpp:32-58):',
        'w = gyro - b: attitude_ekf.cpp:35.',
        'q = exp(-w dt) (x) q: attitude_ekf.cpp:39, written qmul(qconj(qFromRotVec(w * dt)), q_). '
        'qFromRotVec is the exp map: a rotation vector to a unit quaternion (include/quat_math.h).',
        'P = Phi P Phi^T + Q: Phi at attitude_ekf.cpp:42-46, Q at :48-55, P at :57.',
        'Update (AttitudeEKF::updateAccel, src/attitude_ekf.cpp:60-122):',
        'Gate on |a| - g: attitude_ekf.cpp:63-66.',
        'y = a / |a|: :69. yh = A ez: :70-71. r = y - yh: :73. H = [-[yh]x, 0]: :75-79.',
        'K = P H^T (H P H^T + R)^-1: :87-91 (R inflated as |a| leaves 1 g: :82-85).',
        'K[5][*] = 0, z bias locked: :94-95.',
        'NEW, gain taken off vertical (EkfParams::accelNoHeading): :98-104.',
        'Error state dx = K r: :107-112. Joseph covariance update: :115-116.',
        'q = exp(dx[0..2]) (x) q: :119. b += dx[3..5]: :120.',
        'Calls: ekf.predict on every gyro sample, src/main.cpp:1827; ekf.updateAccel, src/main.cpp:1836.',
        'Yaw gyro scale: src/main.cpp:1779 multiplies every gyro z sample by tune::gyroScaleZ, which starts at '
        'kGyroScaleZ = 1.0045 (include/control_params.h:146; src/tune.cpp:22).',
    ],
    'Code: Control Law & Wheel Allocation': [
        'Control law (AttitudeController::update, src/attitude_ctrl.cpp:25-50):',
        'qe = q (x) conj(q_ref): attitude_ctrl.cpp:27; short way around: :28.',
        'th = -2 vec(qe): :29.',
        'tau = -Kp th - Kd (w - w_ref) + I a_ref + w x (I w + h_wheels): :47-49. The reference rate and '
        'acceleration are rotated into the current body frame at :39-41, and the gyroscopic term is built at :44-45. '
        'The full line also has an integral term (Ki = 0 by default) and a feed-forward used only by SLEW TILT.',
        'Kp = I wn^2, Kd = 2 zeta I wn: attitude_ctrl.cpp:12-13 (rebuildGains).',
        'Allocation (wheel_geometry::allocate, include/wheel_geometry.h:27-49):',
        'M = sum(a_i a_i^T) over enabled wheels: wheel_geometry.h:28-36.',
        'y = inverse(M) tau: :37-42 (returns false if fewer than 3 wheels span xyz).',
        'u_i = -(a_i . y): :43-47.',
        'Null mode n = [+1, -1, +1, -1] (addNullSpaceTorque, wheel_geometry.h:53-58): n at :55, mode speed s at :56, '
        'u_i += J k (bias - s) n_i at :57.',
        'Calls in the 100 Hz attitude step (src/main.cpp): ctrl.update :1481; allocate :1486; the bias chosen '
        '(heading-hold planner, DEMO SPIN or FAULT TEST) :1498-1509; addNullSpaceTorque with all 4 wheels :1510.',
        'Torque to speed setpoint: wheelCmd_i += u_i / J dt at src/main.cpp:1514, kept within reach of the motor '
        'and the 80 rad/s limit at :1515-1521.',
        'Speed PI: speedLoops[i].update(...) at src/main.cpp:1620 (include/wheel_speed_ctrl.h).',
    ],
    'Code: Wheel-Failure Detection': [
        'Effort = duty - speed / 160: src/main.cpp:1563, wheels[i].axisDuty() - w / kWheelFreeSpeedRadS '
        '(160 rad/s, include/control_params.h:35).',
        'Detector (DeadMotorDetector::update, src/wheel_fault.cpp:16-48):',
        '0.1 s blocks: wheel_fault.cpp:24-26 (kDeadBlockS, control_params.h:346).',
        'Driven hard, mean |effort| >= 0.2: :28-32 (kDeadEffort :342). Not on the slide: a wheel at a standstill '
        'needs 0.4 (kDeadStillEffort :344) because static friction holds it.',
        'predicted = 800 sum(effort dt): :33 (kMotorAccelPerEffort :341).',
        'gained = (speed - speed0) sign(effort): :34.',
        'bad = gained < 0.25 |predicted|: :35-39 (kDeadResponseFrac :345); one good block resets the count.',
        'Bad for 0.2 s (2 blocks) = dead: :43-46 (kDeadDetectS :347).',
        'Out of service and coasting: src/main.cpp:1564-1566 (wheelEnabled[i] = false; wheelDead[i] = true; '
        'wheels[i].coast()).',
        'Grading (FaultTest::step, src/wheel_fault.cpp:86-128): detect, return within 1 deg, hold 30 s.',
    ],
    'Code: Magnetometer Heading Update': [
        'Heading update (AttitudeEKF::updateMagYaw, src/attitude_ekf.cpp:124-169):',
        'mI = A^T magBody: attitude_ekf.cpp:131-134.',
        'alpha = atan2(ref x mI, ref . mI): :138-139.',
        'h = A ez, H = [h^T, 0]: :141.',
        'S = h^T P h + sigma^2: :142-144.',
        'K = P H^T / S: :150-151.',
        'Rotate about vertical only, K[0..2] = (K . h) h: :152-153.',
        'K[3] = K[4] = 0: :154.',
        'Joseph covariance update: :157-163.',
        'q = exp(K[0..2] alpha) (x) q: :165. b.z += K[5] alpha: :166.',
        'Gating (MagAid::update, src/mag_aid.cpp): wheel state from low-passed speeds, stopped / spinning / '
        'gated, :136-155; return when gated or still settling, :179-187.',
        'Reference: stopped_ or spin_[pattern], mag_aid.cpp:189.',
        'Call: src/main.cpp:1804-1805 (magAid.update, then ekf.updateMagYaw).',
    ],
    'Code: DEMO ALL Sequencer': [
        'startDemo(levelOnly), part 1: src/main.cpp:577, in startFullDemo (:557); startDemo itself is at :386.',
        'Graded hold, fullDemo.holdStep: src/main.cpp:1457-1461; FullDemo::holdStep, src/full_demo.cpp:16-32.',
        'Hand-over: src/main.cpp:1476 calls beginFullSlew (:589-604): the held heading demo.startYaw() :596, '
        'demo.abort() with no disarm or brake :597, slew.start(..., YawOnly, 0, yaw0) :598.',
        'Slews complete, part 3: src/main.cpp:674 calls beginFullFault (:607-625), fault.start(w, kFaultDelayS) '
        'at :619 (wheel B, 10 s: include/control_params.h:357 and :376).',
        'Fault graded: src/main.cpp:266-267 calls finishFullDemo (:221-234), which prints "DEMO ALL COMPLETE".',
        'Log opened and flushed before arming: src/main.cpp:403 (logger.startFile("DEMO")); '
        'DataLogger::startFile, src/data_logger.cpp:40-83, header write and flush at :61-80.',
    ],
}

prs = Presentation(SRC)
done = []
for n, s in enumerate(prs.slides, 1):
    t = s.shapes.title.text_frame.text.strip() if s.shapes.title is not None else ''
    if t not in NOTES:
        continue
    tf = s.notes_slide.notes_text_frame
    spoken = tf.text.split(HEAD)[0].rstrip()
    tf.text = spoken + '\n\n' + HEAD + '\n' + '\n'.join(NOTES[t])
    done.append(n)
missing = set(NOTES) - {prs.slides[n - 1].shapes.title.text_frame.text.strip() for n in done}
if missing:
    raise SystemExit(f'slides not found: {missing}')
prs.save(OUT)
print('notes updated on slides', done, '->', OUT)
