import cv2
import numpy as np
import math
from openvino.runtime import Core

# ---- model paths ----
#Face detection CNN: extract face features, outputs mny bounding box candidates, each with the confidence score
FD_XML = r"C:\Users\16043\Desktop\ECE496\open_model_zoo\intel\face-detection-retail-0004\FP16\face-detection-retail-0004.xml"
#Head pose detection CNN: extract face features pass into yaw pitch and roll, using regression to output 3 numbers
HP_XML = r"C:\Users\16043\Desktop\ECE496\open_model_zoo\intel\head-pose-estimation-adas-0001\FP16\head-pose-estimation-adas-0001.xml"
#Face landmarks CNN: extract face features output x,y coordinates
LM_XML = r"C:\Users\16043\Desktop\ECE496\open_model_zoo\intel\landmarks-regression-retail-0009\FP16\landmarks-regression-retail-0009.xml"

core = Core()

# Face detector (expects 300x300 BGR, outputs [1,1,N,7])
fd_model = core.read_model(FD_XML)
fd_comp  = core.compile_model(fd_model, "CPU")
fd_in    = fd_comp.inputs[0].get_any_name()
fd_out   = fd_comp.outputs[0].get_any_name()

# Head-pose model (expects 60x60 BGR)
hp_model = core.read_model(HP_XML)
hp_comp  = core.compile_model(hp_model, "CPU")
hp_in    = hp_comp.inputs[0].get_any_name()

# Landmarks model (expects 48x48 BGR, outputs 5 points)
lm_model = core.read_model(LM_XML)
lm_comp  = core.compile_model(lm_model, "CPU")
lm_in    = lm_comp.inputs[0].get_any_name()
lm_out   = lm_comp.outputs[0].get_any_name()

def ema(prev, new, a=0.2):
    return new if prev is None else (1 - a) * prev + a * new

yaw_s = pitch_s = roll_s = None
dist_s = None
DEAD = 0.5  # degrees

# distance estimation constants
IPD_REAL_CM = 6.3        # average interpupillary distance
FOV_DEG = 60.0           # assumed horizontal FOV of external camera (still need to adjust)
DIST_SCALE = 43.0 / 43.0 # calibrated scale for this camera (≈ 0.70) (still need to adjust)

CAM_INDEX = 0 # external/internal camera
cap = cv2.VideoCapture(CAM_INDEX)

FD_W, FD_H = 300, 300
f_pixels = None  

while True:
    ok, frame = cap.read()
    if not ok:
        break

    frame = cv2.flip(frame, 1)

    H, W = frame.shape[:2]

    if f_pixels is None:
        f_pixels = (W / 2.0) / math.tan(math.radians(FOV_DEG) / 2.0)

    fd_img = cv2.resize(frame, (FD_W, FD_H))
    fd_blob = np.expand_dims(np.transpose(fd_img, (2, 0, 1)), 0).astype(np.float32)
    det = fd_comp.create_infer_request().infer({fd_in: fd_blob})[fd_out][0, 0]  # (N,7)

    best = None
    best_conf = 0.0
    for d in det:
        conf = float(d[2])
        if conf < 0.3:
            continue
        x0 = int(d[3] * W)
        y0 = int(d[4] * H)
        x1 = int(d[5] * W)
        y1 = int(d[6] * H)
        if conf > best_conf:
            best_conf = conf
            best = (x0, y0, x1, y1)

    if best:
        x0, y0, x1, y1 = best

        pad = int(0.1 * max(x1 - x0, y1 - y0))
        x0 = max(0, x0 - pad)
        y0 = max(0, y0 - pad)
        x1 = min(W, x1 + pad)
        y1 = min(H, y1 + pad)

        cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 0), 2)

        face = frame[y0:y1, x0:x1]
        if face.size:
            hp_img = cv2.resize(face, (60, 60))
            hp_blob = np.expand_dims(np.transpose(hp_img, (2, 0, 1)), 0).astype(np.float32)
            out_hp = hp_comp.create_infer_request().infer({hp_in: hp_blob})

            yaw   = float(out_hp["angle_y_fc"][0][0])
            pitch = float(out_hp["angle_p_fc"][0][0])
            roll  = float(out_hp["angle_r_fc"][0][0])

            yaw_s   = ema(yaw_s, yaw)
            pitch_s = ema(pitch_s, pitch)
            roll_s  = ema(roll_s, roll)

            def dz(p, c):
                return p if p is not None and abs(c - p) < DEAD else c

            yaw_d, pitch_d, roll_d = dz(yaw_s, yaw_s), dz(pitch_s, pitch_s), dz(roll_s, roll_s)

            lm_img = cv2.resize(face, (48, 48))
            lm_blob = np.expand_dims(np.transpose(lm_img, (2, 0, 1)), 0).astype(np.float32)
            out_lm = lm_comp.create_infer_request().infer({lm_in: lm_blob})[lm_out]

            lm_points = out_lm.reshape(-1, 2)  

            pts = []
            fw = (x1 - x0)
            fh = (y1 - y0)
            for (px, py) in lm_points:
                fx = int(px * fw + x0)
                fy = int(py * fh + y0)
                pts.append((fx, fy))

            right_eye = pts[0]
            left_eye  = pts[1]

            cv2.circle(frame, right_eye, 3, (255, 0, 0), -1)
            cv2.circle(frame, left_eye,  3, (255, 0, 0), -1)

            IPD_px = float(np.linalg.norm(
                np.array(left_eye, dtype=np.float32) - np.array(right_eye, dtype=np.float32)
            ))

            if IPD_px > 0 and f_pixels is not None:
                #f = focal length
                #IPD real: real world human interpupillary distance
                #IPD pixel distance (Euclidean distance) between detected eye marks
                yaw_rad = math.radians(yaw_d)  
                cos_y   = math.cos(yaw_rad)
                cos_y   = max(cos_y, 0.5)     

                Z_raw_cm = (f_pixels * IPD_REAL_CM * cos_y) / IPD_px

                #Z_raw_cm = (f_pixels * IPD_REAL_CM) / IPD_px
                Z_cam_eyes_cm = DIST_SCALE * Z_raw_cm

                dist_s = ema(dist_s, Z_cam_eyes_cm)

                cv2.putText(frame, f"Dist: {dist_s:.1f} cm",
                            (max(10, x0), max(50, y0 + 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv2.putText(frame,
                        f"Yaw:{yaw_d:.1f}  Pitch:{pitch_d:.1f}  Roll:{roll_d:.1f}",
                        (max(10, x0), max(25, y0 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("Head Pose + Landmarks + Distance (cam 1)", frame)
    if cv2.waitKey(1) == 27:  
        break

cap.release()
cv2.destroyAllWindows()
