import cv2
import numpy as np
import math
import time
import json
import serial
from dataclasses import dataclass
from openvino.runtime import Core

# ============================================================
# USER SETTINGS
# ============================================================

TEST_MODE = False
PRINT_DEBUG = False
PRINT_COMMAND = False

# --- OpenVINO model paths ---
FD_XML = r"C:\Users\kynaw\ECE496\models\face-detection-retail-0004.xml"
LM_XML = r"C:\Users\kynaw\ECE496\models\landmarks-regression-retail-0009.xml"

# --- Serial / RoArm ---
SERIAL_PORT = "COM3"
BAUDRATE = 115200

# --- Camera ---
CAM_INDEX = 1
CAM_W = 640
CAM_H = 480
CAM_FPS = 30
USE_MJPG = True
MIRROR_VIEW = True

# Try to reduce blur / lag
DISABLE_AUTOFOCUS = True
TRY_SET_FOCUS = False
MANUAL_FOCUS_VALUE = 0  

# ============================================================
# TRACKING / CAMERA MODEL
# ============================================================

DIST_TARGET_CM = 50.0
IPD_REAL_CM = 6.3
FOV_DEG = 50.0
DIST_SCALE = 0.50
FACE_CONF_THRESH = 0.35

# ============================================================
# ARM START / LIMITS
# ============================================================

X0, Y0, Z0 = 235.0, 0.0, 234.0
T_NEUTRAL = 3.14

# STRICT LIMITS: X is forward/backward (Shoulder extension).
# Kept tight (150 to 280) to prevent hyperextension and servo strain.
X_MIN, X_MAX = 150.0, 280.0
Y_MIN, Y_MAX = -250.0, 250.0
Z_MIN, Z_MAX = 80.0, 320.0

# ============================================================
# CONTROL LOOP
# ============================================================

# INCREASED RATE: 25 Hz ensures smooth, continuous trajectory 
# without overflowing the ESP32 serial buffer.
SEND_HZ = 25.0

# Tighter deadbands to allow fine adjustments without large snaps
DEADBAND_EX = 0.12
DEADBAND_EY = 0.12
DEADBAND_ED_CM = 5.0

# REDUCED SMOOTHING: Higher alpha means less delay, tracking face instantly
EMA_TARGET_CX = 0.60
EMA_TARGET_CY = 0.60
EMA_IPD = 0.50
EMA_DIST = 0.50

# Face loss behavior
FACE_HOLD_SEC = 0.50
FACE_RESET_SEC = 1.00

# ============================================================
# AXIS MAPPING / TUNING
# ============================================================
X_SIGN = 1.0
Y_SIGN = 1.0
Z_SIGN = -1.0

# ============================================================
# COMMAND IDS
# ============================================================

CMD_MOVE_INIT = 100
CMD_XYZT_DIRECT_CTRL = 1041

# ============================================================
# HELPERS
# ============================================================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def apply_deadband(v, db):
    return 0.0 if abs(v) < db else v

def ema(prev, new, alpha):
    return new if prev is None else (1.0 - alpha) * prev + alpha * new

def write_json(ser, obj):
    line = json.dumps(obj, separators=(",", ":"))
    if ser is None:
        if PRINT_COMMAND:
            print("SIM SEND:", line)
        return
    ser.write((line + "\n").encode("utf-8"))
    ser.flush()
    if PRINT_COMMAND:
        print("SEND:", line)

def send_xyzt(ser, x, y, z, t):
    write_json(ser, {
        "T": CMD_XYZT_DIRECT_CTRL,
        "x": float(round(x, 2)),
        "y": float(round(y, 2)),
        "z": float(round(z, 2)),
        "t": float(round(t, 2))
    })

def open_camera(index, width, height, fps, prefer_mjpg=True):
    backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]

    for backend in backends:
        cap = cv2.VideoCapture(index, backend)
        if not cap.isOpened():
            continue

        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if prefer_mjpg:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)

        if DISABLE_AUTOFOCUS:
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        if TRY_SET_FOCUS:
            cap.set(cv2.CAP_PROP_FOCUS, MANUAL_FOCUS_VALUE)

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        backend_id = int(cap.get(cv2.CAP_PROP_BACKEND))

        print(f"Camera opened: backend={backend_id} {actual_w}x{actual_h} @ {actual_fps:.1f} FPS")
        return cap

    return None

# ============================================================
# DATA TYPES
# ============================================================

@dataclass
class Measurement:
    face_ok: bool = False
    bbox: tuple = None
    face_score: float = 0.0

    target_cx: float = None
    target_cy: float = None

    ipd_px: float = None
    dist_cm: float = None

    ex: float = None
    ey: float = None
    ed_cm: float = None

# ============================================================
# PID
# ============================================================

class PIDAxis:
    def __init__(self, kp, ki, kd, out_min, out_max, i_min, i_max, d_alpha=0.25):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.out_min = out_min
        self.out_max = out_max
        self.i_min = i_min
        self.i_max = i_max
        self.d_alpha = d_alpha

        self.integral = 0.0
        self.prev_measurement = None
        self.d_filt = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_measurement = None
        self.d_filt = 0.0

    def update(self, error, measurement, dt):
        if dt <= 1e-5:
            return 0.0

        if self.prev_measurement is None:
            d_meas = 0.0
        else:
            d_meas = (measurement - self.prev_measurement) / dt
        self.prev_measurement = measurement

        self.d_filt = (1.0 - self.d_alpha) * self.d_filt + self.d_alpha * d_meas

        new_integral = self.integral + error * dt
        new_integral = clamp(new_integral, self.i_min, self.i_max)

        u = (
            self.kp * error +
            self.ki * new_integral -
            self.kd * self.d_filt
        )

        u_sat = clamp(u, self.out_min, self.out_max)

        if (u == u_sat) or ((u > self.out_max) and (error < 0)) or ((u < self.out_min) and (error > 0)):
            self.integral = new_integral

        return u_sat

# ============================================================
# VISION TRACKER
# ============================================================

class VisionTracker:
    def __init__(self, fd_xml, lm_xml):
        self.core = Core()

        self.fd_model = self.core.read_model(fd_xml)
        self.fd_comp = self.core.compile_model(self.fd_model, "CPU")
        self.fd_in = self.fd_comp.inputs[0].get_any_name()
        self.fd_out = self.fd_comp.outputs[0].get_any_name()
        self.fd_req = self.fd_comp.create_infer_request()

        self.lm_model = self.core.read_model(lm_xml)
        self.lm_comp = self.core.compile_model(self.lm_model, "CPU")
        self.lm_in = self.lm_comp.inputs[0].get_any_name()
        self.lm_out = self.lm_comp.outputs[0].get_any_name()
        self.lm_req = self.lm_comp.create_infer_request()

        self.cx_s = None
        self.cy_s = None
        self.ipd_s = None
        self.dist_s = None
        self.prev_bbox = None

    def reset_filters(self):
        self.cx_s = None
        self.cy_s = None
        self.ipd_s = None
        self.dist_s = None
        self.prev_bbox = None

    def _score_face(self, x0, y0, x1, y1, conf, frame_w, frame_h):
        w = max(1, x1 - x0)
        h = max(1, y1 - y0)
        area = float(w * h)

        area_norm = area / float(frame_w * frame_h)
        score = 3.0 * conf + 2.0 * area_norm

        cx = 0.5 * (x0 + x1)
        cy = 0.5 * (y0 + y1)
        dx = (cx - frame_w * 0.5) / (frame_w * 0.5)
        dy = (cy - frame_h * 0.5) / (frame_h * 0.5)
        center_penalty = math.sqrt(dx * dx + dy * dy)
        score -= 0.20 * center_penalty

        if self.prev_bbox is not None:
            px0, py0, px1, py1 = self.prev_bbox
            pcx = 0.5 * (px0 + px1)
            pcy = 0.5 * (py0 + py1)
            dist_prev = math.hypot(cx - pcx, cy - pcy)
            diag = math.hypot(frame_w, frame_h)
            score -= 0.35 * (dist_prev / diag)

        return score

    def _pick_best_face(self, detections, W, H):
        best = None
        best_score = -1e9

        for d in detections:
            conf = float(d[2])
            if conf < FACE_CONF_THRESH:
                continue

            x0 = clamp(int(d[3] * W), 0, W - 1)
            y0 = clamp(int(d[4] * H), 0, H - 1)
            x1 = clamp(int(d[5] * W), 0, W - 1)
            y1 = clamp(int(d[6] * H), 0, H - 1)

            if x1 <= x0 or y1 <= y0:
                continue

            score = self._score_face(x0, y0, x1, y1, conf, W, H)
            if score > best_score:
                best_score = score
                best = (x0, y0, x1, y1, conf, score)

        return best

    def process(self, frame, f_pixels):
        H, W = frame.shape[:2]
        meas = Measurement()

        fd_img = cv2.resize(frame, (300, 300))
        fd_blob = np.expand_dims(np.transpose(fd_img, (2, 0, 1)), 0).astype(np.float32)
        fd_out = self.fd_req.infer({self.fd_in: fd_blob})[self.fd_out][0, 0]

        best = self._pick_best_face(fd_out, W, H)
        if best is None:
            return meas

        x0, y0, x1, y1, conf, score = best

        pad = int(0.10 * max(x1 - x0, y1 - y0))
        rx0 = max(0, x0 - pad)
        ry0 = max(0, y0 - pad)
        rx1 = min(W, x1 + pad)
        ry1 = min(H, y1 + pad)

        face = frame[ry0:ry1, rx0:rx1]
        if face.size == 0:
            return meas

        meas.face_ok = True
        meas.bbox = (x0, y0, x1, y1)
        meas.face_score = score
        self.prev_bbox = (x0, y0, x1, y1)

        raw_cx = 0.5 * (x0 + x1)
        raw_cy = 0.5 * (y0 + y1)

        self.cx_s = ema(self.cx_s, raw_cx, EMA_TARGET_CX)
        self.cy_s = ema(self.cy_s, raw_cy, EMA_TARGET_CY)

        meas.target_cx = self.cx_s
        meas.target_cy = self.cy_s

        lm_img = cv2.resize(face, (48, 48))
        lm_blob = np.expand_dims(np.transpose(lm_img, (2, 0, 1)), 0).astype(np.float32)
        lm_out = self.lm_req.infer({self.lm_in: lm_blob})[self.lm_out]
        pts_norm = lm_out.reshape(-1, 2)

        fw = rx1 - rx0
        fh = ry1 - ry0

        pts = []
        for px, py in pts_norm:
            fx = int(px * fw + rx0)
            fy = int(py * fh + ry0)
            pts.append((fx, fy))

        if len(pts) >= 2:
            right_eye = pts[0]
            left_eye = pts[1]

            ipd_px_raw = float(np.linalg.norm(
                np.array(left_eye, dtype=np.float32) -
                np.array(right_eye, dtype=np.float32)
            ))

            self.ipd_s = ema(self.ipd_s, ipd_px_raw, EMA_IPD)
            meas.ipd_px = self.ipd_s

            if self.ipd_s is not None and self.ipd_s > 1.0:
                dist_raw_cm = (f_pixels * IPD_REAL_CM) / self.ipd_s
                dist_cm = DIST_SCALE * dist_raw_cm
                self.dist_s = ema(self.dist_s, dist_cm, EMA_DIST)
                meas.dist_cm = self.dist_s

        if meas.target_cx is not None and meas.target_cy is not None:
            ex = (meas.target_cx - (W / 2.0)) / (W / 2.0)
            ey = (meas.target_cy - (H / 2.0)) / (H / 2.0)

            meas.ex = apply_deadband(ex, DEADBAND_EX)
            meas.ey = apply_deadband(ey, DEADBAND_EY)

        if meas.dist_cm is not None:
            ed_cm = meas.dist_cm - DIST_TARGET_CM
            meas.ed_cm = apply_deadband(ed_cm, DEADBAND_ED_CM)

        return meas

# ============================================================
# ARM CONTROLLER
# ============================================================



class RoArmPIDController:
    def __init__(self):
        self.x_cmd = X0
        self.y_cmd = Y0
        self.z_cmd = Z0

        # Output units are mm/s
        # X relies on shoulder/distance: lower Kp than Pan/Tilt, but still much bolder than before.
        self.pid_x = PIDAxis(
            kp=12.0, ki=0.5, kd=2.0,
            out_min=-150.0, out_max=150.0,
            i_min=-30.0, i_max=30.0,
            d_alpha=0.3
        )

        # Pan & Tilt: Hugely increased Kp for fast, bold steps. 
        # Large Kd acts as a brake to prevent overshooting as it nears the target.
        self.pid_y = PIDAxis(
            kp=500.0, ki=20.0, kd=45.0,
            out_min=-400.0, out_max=400.0,
            i_min=-50.0, i_max=50.0,
            d_alpha=0.3
        )

        self.pid_z = PIDAxis(
            kp=500.0, ki=20.0, kd=45.0,
            out_min=-400.0, out_max=400.0,
            i_min=-50.0, i_max=50.0,
            d_alpha=0.3
        )

        # Let the dt and the PID velocity limits handle the step scaling.
        # Absolute hard caps per transmission frame just in case of noise spikes.
        self.max_step_x = 25.0
        self.max_step_y = 35.0
        self.max_step_z = 35.0

    def reset(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()

    def reset_pose(self):
        self.x_cmd = X0
        self.y_cmd = Y0
        self.z_cmd = Z0
        self.reset()

    def update_from_measurement(self, meas, dt):
        if not meas.face_ok:
            return self.x_cmd, self.y_cmd, self.z_cmd

        mx = 0.0 if meas.dist_cm is None else meas.dist_cm
        my = 0.0 if meas.ex is None else meas.ex
        mz = 0.0 if meas.ey is None else meas.ey

        ex_dist = 0.0 if meas.ed_cm is None else meas.ed_cm
        ey_img = 0.0 if meas.ex is None else meas.ex
        ez_img = 0.0 if meas.ey is None else meas.ey

        vx = self.pid_x.update(ex_dist, mx, dt)
        vy = self.pid_y.update(ey_img, my, dt)
        vz = self.pid_z.update(ez_img, mz, dt)

        dx = clamp(X_SIGN * vx * dt, -self.max_step_x, self.max_step_x)
        dy = clamp(Y_SIGN * vy * dt, -self.max_step_y, self.max_step_y)
        dz = clamp(Z_SIGN * vz * dt, -self.max_step_z, self.max_step_z)

        self.x_cmd = clamp(self.x_cmd + dx, X_MIN, X_MAX)
        self.y_cmd = clamp(self.y_cmd + dy, Y_MIN, Y_MAX)
        self.z_cmd = clamp(self.z_cmd + dz, Z_MIN, Z_MAX)

        return self.x_cmd, self.y_cmd, self.z_cmd

# ============================================================
# MAIN
# ============================================================

def main():
    ser = None
    if not TEST_MODE:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1, dsrdtr=None)
        try:
            ser.setRTS(False)
            ser.setDTR(False)
        except Exception:
            pass

        time.sleep(2.0)
        write_json(ser, {"T": CMD_MOVE_INIT})
        time.sleep(2.0)
        send_xyzt(ser, X0, Y0, Z0, T_NEUTRAL)
        time.sleep(0.5)
    else:
        print("TEST MODE: serial disabled")

    cap = open_camera(CAM_INDEX, CAM_W, CAM_H, CAM_FPS, prefer_mjpg=USE_MJPG)
    if cap is None or not cap.isOpened():
        print("ERROR: camera not opened.")
        if ser is not None:
            ser.close()
        return

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)

    f_pixels = (actual_w / 2.0) / math.tan(math.radians(FOV_DEG) / 2.0)

    tracker = VisionTracker(FD_XML, LM_XML)
    controller = RoArmPIDController()

    last_send = time.time()
    face_missing_since = None
    paused = False
    preview_last_t = time.time()
    preview_frames = 0
    preview_fps = 0.0

    print("\nRunning OpenVINO -> best-face -> PID RoArm tracking")
    print("Keys: ESC quit | p pause | r reset pose\n")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame read failed.")
            break

        if MIRROR_VIEW:
            frame = cv2.flip(frame, 1)

        now = time.time()

        preview_frames += 1
        if now - preview_last_t >= 1.0:
            preview_fps = preview_frames / (now - preview_last_t)
            preview_frames = 0
            preview_last_t = now

        meas = tracker.process(frame, f_pixels)

        if meas.face_ok:
            face_missing_since = None
        else:
            if face_missing_since is None:
                face_missing_since = now

            missing_dur = now - face_missing_since

            if missing_dur > FACE_HOLD_SEC:
                controller.reset()

            if missing_dur > FACE_RESET_SEC:
                tracker.reset_filters()

        send_period = 1.0 / SEND_HZ
        if now - last_send >= send_period:
            dt = now - last_send
            last_send = now

            if not paused:
                if meas.face_ok:
                    x_cmd, y_cmd, z_cmd = controller.update_from_measurement(meas, dt)
                else:
                    x_cmd, y_cmd, z_cmd = controller.x_cmd, controller.y_cmd, controller.z_cmd
            else:
                x_cmd, y_cmd, z_cmd = controller.x_cmd, controller.y_cmd, controller.z_cmd

            send_xyzt(ser, x_cmd, y_cmd, z_cmd, T_NEUTRAL)

        # Overlay
        H, W = frame.shape[:2]

        if meas.face_ok and meas.bbox is not None:
            x0, y0, x1, y1 = meas.bbox
            cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 0), 2)

        if meas.target_cx is not None and meas.target_cy is not None:
            cv2.circle(frame, (int(meas.target_cx), int(meas.target_cy)), 6, (0, 255, 255), -1)

        cv2.line(frame, (W // 2 - 15, H // 2), (W // 2 + 15, H // 2), (255, 0, 0), 2)
        cv2.line(frame, (W // 2, H // 2 - 15), (W // 2, H // 2 + 15), (255, 0, 0), 2)

        mode_text = "TEST MODE" if TEST_MODE else "LIVE ARM"
        cv2.putText(frame, mode_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 220, 255), 2)

        cv2.putText(frame, f"Pose mm: X={controller.x_cmd:.1f} Y={controller.y_cmd:.1f} Z={controller.z_cmd:.1f}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)

        cv2.putText(frame, f"Preview FPS={preview_fps:.1f}  SendHz={SEND_HZ:.1f}",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 0), 2)

        if meas.dist_cm is not None:
            cv2.putText(frame, f"Dist={meas.dist_cm:.1f} cm  Target={DIST_TARGET_CM:.1f}",
                        (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2)

        if meas.face_ok:
            cv2.putText(frame, f"FaceScore={meas.face_score:.3f}",
                        (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0), 2)

        if meas.ex is not None:
            cv2.putText(frame,
                        f"ErrX={meas.ex:.4f}  ErrY={meas.ey:.4f}  ErrD={0.0 if meas.ed_cm is None else meas.ed_cm:.2f}",
                        (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (180, 255, 180), 2)

        if paused:
            cv2.putText(frame, "PAUSED", (10, 210),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("RoArm OpenVINO PID Face Tracking", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        elif key == ord('p'):
            paused = not paused
            if paused:
                controller.reset()
        elif key == ord('r'):
            controller.reset_pose()
            tracker.reset_filters()
            send_xyzt(ser, controller.x_cmd, controller.y_cmd, controller.z_cmd, T_NEUTRAL)

    cap.release()
    cv2.destroyAllWindows()
    if ser is not None:
        ser.close()

if __name__ == "__main__":
    main()