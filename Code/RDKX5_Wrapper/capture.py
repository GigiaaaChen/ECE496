#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# capture.py - RDK X5 (CPU/OpenVINO) Head Pose + Landmarks + Distance (optimized, ASCII-safe)

import cv2
import numpy as np
import math
import time
from pathlib import Path
from openvino.runtime import Core


# -------------------------
# Paths (relative to this file)
# -------------------------
ROOT = Path(__file__).resolve().parent
MODELS = ROOT / "models"

FD_XML = str(MODELS / "face-detection-retail-0004.xml")
HP_XML = str(MODELS / "head-pose-estimation-adas-0001.xml")
LM_XML = str(MODELS / "landmarks-regression-retail-0009.xml")


# -------------------------
# Settings / knobs
# -------------------------
CAM_INDEX = 0

# Camera settings (you confirmed MJPG 1280x720 works on your setup)
CAM_W, CAM_H = 1280, 720
USE_MJPG = True
TARGET_FPS = 30  # best-effort

# Compute reduction
FD_EVERY = 5       # face detection every N frames
HP_LM_EVERY = 2    # headpose+landmarks every N frames (set 1 for smoother, 3 for faster)

CONF_TH = 0.30
FD_W, FD_H = 300, 300

# Smoothing / dead-zone
EMA_A = 0.20
DEAD_DEG = 0.50

# Distance estimation
IPD_REAL_CM = 6.3
FOV_DEG = 60.0
DIST_SCALE = 1.0  # calibrate later

# Display
SHOW_WINDOW = True   # set False if imshow does not work via VNC
DRAW_DEBUG = True


# -------------------------
# Helper functions
# -------------------------
def ema(prev, new, a=0.2):
    return new if prev is None else (1.0 - a) * prev + a * new


def deadzone(prev_out, new_out, dead=0.5):
    if prev_out is None:
        return new_out
    return prev_out if abs(new_out - prev_out) < dead else new_out


def check_model_files(xml_path: str):
    p = Path(xml_path)
    if not p.exists():
        raise FileNotFoundError(f"Missing model XML: {p}")
    bin_path = p.with_suffix(".bin")
    if not bin_path.exists():
        raise FileNotFoundError(f"Missing model BIN (must be next to XML): {bin_path}")


# -------------------------
# Validate models exist
# -------------------------
check_model_files(FD_XML)
check_model_files(HP_XML)
check_model_files(LM_XML)


# -------------------------
# OpenVINO init + compile
# -------------------------
core = Core()

fd_comp = core.compile_model(core.read_model(FD_XML), "CPU")
hp_comp = core.compile_model(core.read_model(HP_XML), "CPU")
lm_comp = core.compile_model(core.read_model(LM_XML), "CPU")

fd_in_name = fd_comp.inputs[0].get_any_name()
fd_out_name = fd_comp.outputs[0].get_any_name()

hp_in_name = hp_comp.inputs[0].get_any_name()

lm_in_name = lm_comp.inputs[0].get_any_name()
lm_out_name = lm_comp.outputs[0].get_any_name()

# Reuse infer requests (speed)
fd_req = fd_comp.create_infer_request()
hp_req = hp_comp.create_infer_request()
lm_req = lm_comp.create_infer_request()


# -------------------------
# Camera init
# -------------------------
cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    raise RuntimeError(f"Could not open camera index {CAM_INDEX}")

if USE_MJPG:
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)


# -------------------------
# State
# -------------------------
f_pixels = None

yaw_s = pitch_s = roll_s = None
yaw_out = pitch_out = roll_out = None
dist_s = None

best_bbox = None  # (x0,y0,x1,y1)
best_conf = 0.0

frame_idx = 0

# FPS meter
fps = 0.0
fps_count = 0
fps_t0 = time.time()

# Keep last eye points to draw when LM is skipped
last_left_eye = None
last_right_eye = None


# -------------------------
# Main loop
# -------------------------
try:
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Camera read failed.")
            break

        frame = cv2.flip(frame, 1)
        H, W = frame.shape[:2]

        if f_pixels is None:
            f_pixels = (W / 2.0) / math.tan(math.radians(FOV_DEG) / 2.0)

        # Face detection every FD_EVERY frames
        if best_bbox is None or (frame_idx % FD_EVERY == 0):
            fd_img = cv2.resize(frame, (FD_W, FD_H))
            fd_blob = np.expand_dims(np.transpose(fd_img, (2, 0, 1)), 0).astype(np.float32)

            out_fd = fd_req.infer({fd_in_name: fd_blob})
            det = out_fd[fd_out_name][0, 0]  # (N,7)

            best_bbox = None
            best_conf = 0.0

            for d in det:
                conf = float(d[2])
                if conf < CONF_TH:
                    continue
                if conf > best_conf:
                    x0 = int(d[3] * W)
                    y0 = int(d[4] * H)
                    x1 = int(d[5] * W)
                    y1 = int(d[6] * H)
                    best_conf = conf
                    best_bbox = (x0, y0, x1, y1)

        if best_bbox is not None:
            x0, y0, x1, y1 = best_bbox

            # Pad and clamp bbox
            pad = int(0.12 * max(x1 - x0, y1 - y0))
            x0 = max(0, x0 - pad)
            y0 = max(0, y0 - pad)
            x1 = min(W, x1 + pad)
            y1 = min(H, y1 + pad)

            if x1 > x0 and y1 > y0:
                face = frame[y0:y1, x0:x1]
                do_hplm = (frame_idx % HP_LM_EVERY == 0)

                if do_hplm and face.size:
                    # Head pose
                    hp_img = cv2.resize(face, (60, 60))
                    hp_blob = np.expand_dims(np.transpose(hp_img, (2, 0, 1)), 0).astype(np.float32)
                    out_hp = hp_req.infer({hp_in_name: hp_blob})

                    yaw = float(out_hp["angle_y_fc"][0][0])
                    pitch = float(out_hp["angle_p_fc"][0][0])
                    roll = float(out_hp["angle_r_fc"][0][0])

                    yaw_s = ema(yaw_s, yaw, EMA_A)
                    pitch_s = ema(pitch_s, pitch, EMA_A)
                    roll_s = ema(roll_s, roll, EMA_A)

                    yaw_out = deadzone(yaw_out, yaw_s, DEAD_DEG)
                    pitch_out = deadzone(pitch_out, pitch_s, DEAD_DEG)
                    roll_out = deadzone(roll_out, roll_s, DEAD_DEG)

                    # Landmarks
                    lm_img = cv2.resize(face, (48, 48))
                    lm_blob = np.expand_dims(np.transpose(lm_img, (2, 0, 1)), 0).astype(np.float32)
                    out_lm = lm_req.infer({lm_in_name: lm_blob})[lm_out_name]
                    lm_points = out_lm.reshape(-1, 2)  # normalized

                    fw = (x1 - x0)
                    fh = (y1 - y0)

                    pts = []
                    for (px, py) in lm_points:
                        fx = int(px * fw + x0)
                        fy = int(py * fh + y0)
                        pts.append((fx, fy))

                    last_right_eye = pts[0]
                    last_left_eye = pts[1]

                    # Distance from IPD (very rough)
                    ipd_px = float(np.linalg.norm(
                        np.array(last_left_eye, dtype=np.float32) -
                        np.array(last_right_eye, dtype=np.float32)
                    ))

                    if ipd_px > 1e-3:
                        yaw_used = yaw_out if yaw_out is not None else yaw_s
                        yaw_rad = math.radians(yaw_used if yaw_used is not None else 0.0)
                        cos_y = max(math.cos(yaw_rad), 0.5)

                        z_raw_cm = (f_pixels * IPD_REAL_CM * cos_y) / ipd_px
                        dist_s = ema(dist_s, DIST_SCALE * z_raw_cm, EMA_A)

                # Draw overlays
                if DRAW_DEBUG:
                    cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 0), 2)
                    cv2.putText(frame, f"FD conf: {best_conf:.2f}",
                                (x0, max(20, y0 - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    if last_right_eye is not None and last_left_eye is not None:
                        cv2.circle(frame, last_right_eye, 3, (255, 0, 0), -1)
                        cv2.circle(frame, last_left_eye, 3, (255, 0, 0), -1)

                    if yaw_out is not None:
                        cv2.putText(frame,
                                    f"Yaw:{yaw_out:.1f} Pitch:{pitch_out:.1f} Roll:{roll_out:.1f}",
                                    (10, 25),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    if dist_s is not None:
                        cv2.putText(frame, f"Dist: {dist_s:.1f} cm",
                                    (10, 55),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # FPS overlay
        fps_count += 1
        now = time.time()
        if now - fps_t0 >= 1.0:
            fps = fps_count / (now - fps_t0)
            fps_t0 = now
            fps_count = 0

        if DRAW_DEBUG:
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, H - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Show / exit
        if SHOW_WINDOW:
            cv2.imshow("RDK X5 - Pose + Landmarks + Distance (OpenVINO CPU)", frame)
            if cv2.waitKey(1) == 27:
                break
        else:
            # Headless: print occasionally
            if fps_count == 0:
                print(f"fps={fps:.1f} yaw={yaw_out} pitch={pitch_out} roll={roll_out} dist_cm={dist_s}")

        frame_idx += 1

finally:
    cap.release()
    cv2.destroyAllWindows()
