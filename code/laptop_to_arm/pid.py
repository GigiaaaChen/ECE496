#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math
import time
import json
import serial
import socket
import threading
from dataclasses import dataclass
import openvino as ov

# ============================================================
# USER SETTINGS
# ============================================================

TEST_MODE = False
PRINT_DEBUG = False
PRINT_COMMAND = False

SHOW_PREVIEW = True
SHOW_DISTANCE_TEXT = True

# --- OpenVINO model paths ---
FD_XML = r"models/face-detection-retail-0004.xml"
LM_XML = r"models/landmarks-regression-retail-0009.xml"

# Preferred devices (fallback to CPU automatically)
DEVICE_FD = "NPU"
DEVICE_LM = "NPU"

# --- Socket Server ---
SOCKET_HOST = "0.0.0.0"
SOCKET_PORT = 5000
SOCKET_BACKLOG = 5
SOCKET_ACCEPT_TIMEOUT = 1.0
SOCKET_RECV_TIMEOUT = 2.0
SOCKET_BUFFER_SIZE = 1024
TCP_IDLE_TIMEOUT_SEC = 8.0         # if no valid message for too long, close client
TCP_HEARTBEAT_REPLY = True         # reply to PING with PONG
PRINT_TCP_RAW = True               # print raw incoming TCP lines
PRINT_TCP_ACK = True               # print ACK/ERR replies sent back

# --- Serial / RoArm ---
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

# --- Camera ---
CAM_SOURCE = "/dev/v4l/by-path/platform-xhci-hcd.2.auto-usb-0:1.3:1.0-video-index0"
CAM_BACKEND = cv2.CAP_V4L2			
CAM_W = 640
CAM_H = 480
CAM_FPS = 30				
USE_MJPG = True
MIRROR_VIEW = True


# Windows testing notes:
# - If your USB camera is not the default camera, try CAM_SOURCE = 1 or 2.
# - If COM3 is wrong, change SERIAL_PORT to your actual Arduino/RoArm COM port.
# - OpenVINO defaults to CPU here for laptop stability; you can switch later if needed.

# ============================================================
# STARTUP / SAFETY / DEMO BEHAVIOR
# ============================================================

STARTUP_CAMERA_SETTLE_SEC = 1.0
STARTUP_SERIAL_SETTLE_SEC = 2.0
STARTUP_MODEL_WARMUP_RUNS = 3
STARTUP_DROP_FRAMES = 15
SEND_HOME_AFTER_INIT = True
HOME_ON_EXIT = False

# Safety / robustness
MAX_DT_SEC = 0.08                  # prevent one slow frame from causing a huge jump
CAMERA_FAIL_RETRY_SLEEP_SEC = 0.05
MAX_CAMERA_FAIL_STREAK = 20
PAUSE_HOLDS_POSITION = True
RETURN_HOME_ON_LONG_FACE_LOSS = False
RETURN_HOME_FACE_LOSS_SEC = 4.0
TRACK_ENABLE_FACE_FRAMES = 3       # need N good frames before auto motion starts
TRACK_DISABLE_FACE_LOSS_SEC = 0.60
TRACK_RESET_FILTERS_SEC = 1.20

# Reduce command spam / jitter at the arm side
SERIAL_SEND_ONLY_IF_CHANGED = True
MIN_SEND_DELTA_MM = 2.0
MIN_SEND_DELTA_RAD = 0.02

# ============================================================
# TRACKING / CAMERA MODEL / DISTANCE TUNING
# ============================================================

# Your physical target distance at the design fair
DIST_TARGET_CM = 35.0

# Keep this offset because of the camera swap / calibration mismatch.
# Effective distance used by control/display = raw_estimated_distance + DIST_ESTIMATE_OFFSET_CM
DIST_ESTIMATE_OFFSET_CM = 0.0

# Face model assumptions
IPD_REAL_CM = 6.3
FACE_CONF_THRESH = 0.35
MIN_FACE_AREA_FRAC = 0.015

# Camera model
FOV_DEG = 145.0                    # spec provided by you
DIST_SCALE = 2.1	                   # extra multiplier if you want later fine tuning

# Where you want the user's head center to appear in the frame.
# Since the camera is below the phone, you'll likely want AIM_CENTER_Y_NORM < 0.50
# so the system aims the face a bit above geometric center.
AIM_CENTER_X_NORM = 0.50
AIM_CENTER_Y_NORM = 0.40

# ============================================================
# ARM START / LIMITS & MANUAL TUNING
# ============================================================

X0, Y0, Z0 = 235.0, 0.0, 234.0
T_NEUTRAL = 3.14

X_MIN, X_MAX = 140.0, 330.0
Y_MIN, Y_MAX = -150.0, 150.0
Z_MIN, Z_MAX = 100.0, 320.0

MANUAL_STEP_MM = 10.0

# ============================================================
# CONTROL LOOP & FILTER PARAMS
# ============================================================

SEND_HZ = 20.0
DEADBAND_EX = 0.10
DEADBAND_EY = 0.10
DEADBAND_ED_CM = 2.0

EMA_TARGET_CX = 0.35
EMA_TARGET_CY = 0.35
EMA_IPD = 0.35
EMA_DIST = 0.30

X_SIGN = 1.0
Y_SIGN = 1.0
Z_SIGN = -1.0

CMD_MOVE_INIT = 100
CMD_XYZT_DIRECT_CTRL = 1041

# ============================================================
# GLOBAL SYSTEM STATE
# ============================================================

class SystemState:
    def __init__(self):
        self.mode = "AUTO"
        self.locked = False
        self.paused = False
        self.gyro_cmd = "STOP"
        self.last_cmd_time = 0.0
        self.system_ready = False
        self.status_text = "BOOTING"
        self.lock = threading.Lock()
        self.tcp_connected = False
        self.tcp_client_addr = None
        self.last_tcp_rx_time = 0.0

state = SystemState()

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

def set_status(msg):
    should_print = False
    with state.lock:
        if state.status_text != msg:
            state.status_text = msg
            should_print = True
    if should_print:
        print(f"[STATUS] {msg}")

def configure_server_socket(server):
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Low-latency small command packets
    try:
        server.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except Exception:
        pass

    server.settimeout(SOCKET_ACCEPT_TIMEOUT)


def configure_client_socket(conn):
    conn.settimeout(SOCKET_RECV_TIMEOUT)

    try:
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except Exception:
        pass

    # TCP keepalive so dead clients are detected better
    try:
        conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    except Exception:
        pass

    # Linux-specific keepalive tuning (RDK side should support these)
    try:
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 5)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 2)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
    except Exception:
        pass


def send_tcp_reply(conn, msg: str):
    if not msg.endswith("\n"):
        msg += "\n"
    conn.sendall(msg.encode("utf-8"))
    if PRINT_TCP_ACK:
        print(f"[TCP TX] {msg.strip()}")

# ============================================================
# SOCKET SERVER THREAD (ACK + ALWAYS-ALIVE + BETTER PARSING)
# ============================================================

TCP_ACCEPT_TIMEOUT_SEC = 1.0
TCP_CLIENT_TIMEOUT_SEC = 2.0
TCP_IDLE_DISCONNECT_SEC = 8.0
TCP_RECV_SIZE = 1024

VALID_COMMANDS = {
    "AUTO", "MANUAL", "LOCK", "UNLOCK", "PAUSE", "RESUME",
    "RIGHT", "LEFT", "FORWARD", "BACKWARD", "STOP", "UP", "DOWN",
    "PING", "HEARTBEAT", "KEEPALIVE"
}

def set_tcp_connection(is_connected, addr=None):
    with state.lock:
        state.tcp_connected = is_connected
        state.tcp_client_addr = addr if is_connected else None
        if is_connected:
            state.last_tcp_rx_time = time.time()

def touch_tcp_rx():
    with state.lock:
        state.last_tcp_rx_time = time.time()

def send_tcp_reply(conn, msg: str):
    try:
        if not msg.endswith("\n"):
            msg += "\n"
        conn.sendall(msg.encode("utf-8"))
        if PRINT_TCP_ACK:
            print(f"[TCP TX] {msg.strip()}")
    except Exception as e:
        print(f"[TCP TX] Failed to send reply: {e}")

def handle_socket_command(cmd: str, conn=None):
    cmd = cmd.strip().upper()
    if not cmd:
        return False

    if PRINT_TCP_RAW:
        print(f"[TCP RX CMD] {repr(cmd)}")

    touch_tcp_rx()

    if cmd in ["PING", "HEARTBEAT", "KEEPALIVE"]:
        if TCP_HEARTBEAT_REPLY and conn is not None:
            send_tcp_reply(conn, "PONG")
        return True

    handled = True

    with state.lock:
        if cmd == "AUTO":
            state.mode = "AUTO"
        elif cmd == "MANUAL":
            state.mode = "MANUAL"
        elif cmd == "LOCK":
            state.locked = True
        elif cmd == "UNLOCK":
            state.locked = False
        elif cmd == "PAUSE":
            state.paused = True
        elif cmd == "RESUME":
            state.paused = False
        elif cmd in ["RIGHT", "LEFT", "FORWARD", "BACKWARD", "STOP", "UP", "DOWN"]:
            state.gyro_cmd = cmd
            state.last_cmd_time = time.time()
        else:
            handled = False

    if conn is not None:
        if handled:
            send_tcp_reply(conn, f"ACK:{cmd}")
        else:
            send_tcp_reply(conn, f"ERR:{cmd}")

    if handled:
        print(f"[SOCKET] Handled command: {cmd}")
    else:
        print(f"[SOCKET] Unknown command: {cmd}")

    return handled

def process_recv_buffer(recv_buffer, conn):
    """
    Returns updated recv_buffer after processing as many commands as possible.
    Accepts either:
      - newline-separated commands: 'LOCK\\nUNLOCK\\n'
      - single raw packets: 'LEFT'
      - CRLF packets from some clients
    """
    # normalize CRLF -> LF
    recv_buffer = recv_buffer.replace("\r", "\n")

    # process all newline-terminated lines first
    while "\n" in recv_buffer:
        line, recv_buffer = recv_buffer.split("\n", 1)
        line = line.strip()
        if line:
            handle_socket_command(line, conn)

    # if leftover buffer is itself exactly one valid token, process it immediately
    stripped = recv_buffer.strip().upper()
    if stripped in VALID_COMMANDS:
        handle_socket_command(stripped, conn)
        recv_buffer = ""

    return recv_buffer

def socket_server_thread():
    while True:
        server = None
        try:
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)

            try:
                server.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 10)
                server.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 3)
                server.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
            except Exception:
                pass

            server.bind((SOCKET_HOST, SOCKET_PORT))
            server.listen(SOCKET_BACKLOG)
            server.settimeout(TCP_ACCEPT_TIMEOUT_SEC)

            print(f"[SOCKET] Always-alive server listening on {SOCKET_HOST}:{SOCKET_PORT}")

            while True:
                conn = None
                addr = None

                try:
                    try:
                        conn, addr = server.accept()
                    except socket.timeout:
                        continue

                    print(f"[SOCKET] Connected by {addr}")
                    conn.settimeout(TCP_CLIENT_TIMEOUT_SEC)

                    try:
                        conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                        try:
                            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 10)
                            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 3)
                            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
                        except Exception:
                            pass
                    except Exception:
                        pass

                    set_tcp_connection(True, addr)
                    recv_buffer = ""

                    # optional greeting so client knows server is ready
                    send_tcp_reply(conn, "ACK:CONNECTED")

                    while True:
                        try:
                            data = conn.recv(TCP_RECV_SIZE)

                            if not data:
                                print("[SOCKET] Client closed connection")
                                break

                            decoded = data.decode("utf-8", errors="ignore")
                            if PRINT_TCP_RAW:
                                print(f"[TCP RX RAW] {repr(decoded)}")

                            touch_tcp_rx()
                            recv_buffer += decoded
                            recv_buffer = process_recv_buffer(recv_buffer, conn)

                        except socket.timeout:
                            with state.lock:
                                idle_for = time.time() - state.last_tcp_rx_time

                            if idle_for > TCP_IDLE_DISCONNECT_SEC:
                                print(f"[SOCKET] Client idle timeout ({idle_for:.1f}s), disconnecting")
                                break

                            continue

                        except ConnectionResetError:
                            print("[SOCKET] Connection reset by peer")
                            break

                        except Exception as e:
                            print(f"[SOCKET] Client loop error: {e}")
                            break

                except Exception as e:
                    print(f"[SOCKET] Accept/client error: {e}")

                finally:
                    set_tcp_connection(False)
                    try:
                        if conn is not None:
                            conn.close()
                    except Exception:
                        pass
                    print("[SOCKET] Waiting for next client...")

        except Exception as e:
            print(f"[SOCKET] Server fatal error, recreating socket: {e}")
            time.sleep(1.0)

        finally:
            try:
                if server is not None:
                    server.close()
            except Exception:
                pass
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
    raw_dist_cm: float = None
    dist_cm: float = None
    ex: float = None
    ey: float = None
    ed_cm: float = None

# ============================================================
# PID CONTROLLER
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

        new_integral = clamp(self.integral + error * dt, self.i_min, self.i_max)
        u = self.kp * error + self.ki * new_integral - self.kd * self.d_filt
        u_sat = clamp(u, self.out_min, self.out_max)

        if (u == u_sat) or ((u > self.out_max) and (error < 0)) or ((u < self.out_min) and (error > 0)):
            self.integral = new_integral

        return u_sat

# ============================================================
# VISION TRACKER
# ============================================================

class VisionTracker:
    def __init__(self, fd_xml, lm_xml):
        self.core = ov.Core()

        available = list(self.core.available_devices)
        print(f"[OpenVINO] Available devices: {available}")

        self.device_fd = DEVICE_FD if DEVICE_FD in available else "CPU"
        self.device_lm = DEVICE_LM if DEVICE_LM in available else "CPU"

        print(f"[OpenVINO] Loading face detection on {self.device_fd}")
        self.fd_model = self.core.read_model(fd_xml)
        self.fd_comp = self.core.compile_model(self.fd_model, self.device_fd)
        self.fd_input = self.fd_comp.input(0)
        self.fd_output = self.fd_comp.output(0)
        self.fd_req = self.fd_comp.create_infer_request()

        print(f"[OpenVINO] Loading landmarks on {self.device_lm}")
        self.lm_model = self.core.read_model(lm_xml)
        self.lm_comp = self.core.compile_model(self.lm_model, self.device_lm)
        self.lm_input = self.lm_comp.input(0)
        self.lm_output = self.lm_comp.output(0)
        self.lm_req = self.lm_comp.create_infer_request()

        self.reset_filters()

    def reset_filters(self):
        self.cx_s = None
        self.cy_s = None
        self.ipd_s = None
        self.raw_dist_s = None
        self.dist_s = None
        self.prev_bbox = None

    def warmup(self):
        dummy_fd = np.zeros((1, 3, 300, 300), dtype=np.float32)
        dummy_lm = np.zeros((1, 3, 48, 48), dtype=np.float32)
        for _ in range(STARTUP_MODEL_WARMUP_RUNS):
            self.fd_req.infer({self.fd_input: dummy_fd})
            self.lm_req.infer({self.lm_input: dummy_lm})
        print("[OpenVINO] Warmup complete")

    def _score_face(self, x0, y0, x1, y1, conf, frame_w, frame_h):
        w = max(1, x1 - x0)
        h = max(1, y1 - y0)
        area_norm = (w * h) / float(frame_w * frame_h)
        score = 3.0 * conf + 2.0 * area_norm

        cx = 0.5 * (x0 + x1)
        cy = 0.5 * (y0 + y1)
        aim_x = frame_w * AIM_CENTER_X_NORM
        aim_y = frame_h * AIM_CENTER_Y_NORM
        dx = (cx - aim_x) / max(1.0, frame_w * 0.5)
        dy = (cy - aim_y) / max(1.0, frame_h * 0.5)
        score -= 0.20 * math.sqrt(dx * dx + dy * dy)

        if self.prev_bbox is not None:
            px0, py0, px1, py1 = self.prev_bbox
            prev_cx = 0.5 * (px0 + px1)
            prev_cy = 0.5 * (py0 + py1)
            dist_prev = math.hypot(cx - prev_cx, cy - prev_cy)
            score -= 0.35 * (dist_prev / math.hypot(frame_w, frame_h))

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

            area_frac = ((x1 - x0) * (y1 - y0)) / float(max(1, W * H))
            if area_frac < MIN_FACE_AREA_FRAC:
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
        fd_blob = np.transpose(fd_img, (2, 0, 1))[None, ...].astype(np.float32)

        self.fd_req.infer({self.fd_input: fd_blob})
        fd_out = self.fd_req.get_output_tensor(self.fd_output.index).data[0, 0]

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

        self.cx_s = ema(self.cx_s, 0.5 * (x0 + x1), EMA_TARGET_CX)
        self.cy_s = ema(self.cy_s, 0.5 * (y0 + y1), EMA_TARGET_CY)
        meas.target_cx = self.cx_s
        meas.target_cy = self.cy_s

        lm_img = cv2.resize(face, (48, 48))
        lm_blob = np.transpose(lm_img, (2, 0, 1))[None, ...].astype(np.float32)

        self.lm_req.infer({self.lm_input: lm_blob})
        pts_norm = self.lm_req.get_output_tensor(self.lm_output.index).data.reshape(-1, 2)

        fw = rx1 - rx0
        fh = ry1 - ry0
        pts = [(int(px * fw + rx0), int(py * fh + ry0)) for px, py in pts_norm]

        if len(pts) >= 2:
            ipd_now = math.hypot(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1])
            self.ipd_s = ema(self.ipd_s, ipd_now, EMA_IPD)
            meas.ipd_px = self.ipd_s

            if self.ipd_s > 1.0:
                raw_dist_cm = DIST_SCALE * ((f_pixels * IPD_REAL_CM) / self.ipd_s)
                self.raw_dist_s = ema(self.raw_dist_s, raw_dist_cm, EMA_DIST)
                meas.raw_dist_cm = self.raw_dist_s
                corrected_dist_cm = self.raw_dist_s + DIST_ESTIMATE_OFFSET_CM
                self.dist_s = ema(self.dist_s, corrected_dist_cm, EMA_DIST)
                meas.dist_cm = self.dist_s

        if meas.target_cx is not None and meas.target_cy is not None:
            aim_x = W * AIM_CENTER_X_NORM
            aim_y = H * AIM_CENTER_Y_NORM
            meas.ex = apply_deadband((meas.target_cx - aim_x) / (W * 0.5), DEADBAND_EX)
            meas.ey = apply_deadband((meas.target_cy - aim_y) / (H * 0.5), DEADBAND_EY)

        if meas.dist_cm is not None:
            meas.ed_cm = apply_deadband(meas.dist_cm - DIST_TARGET_CM, DEADBAND_ED_CM)

        return meas

# ============================================================
# ARM CONTROLLER
# ============================================================

class RoArmController:
    def __init__(self, ser=None):
        self.ser = ser
        self.x_cmd = X0
        self.y_cmd = Y0
        self.z_cmd = Z0
        self.last_sent = None

        # Slightly tamer gains for fair-day robustness
        self.pid_x = PIDAxis(10.0, 0.30, 2.0, -120.0, 120.0, -25.0, 25.0, 0.25)
        self.pid_y = PIDAxis(280.0, 8.0, 28.0, -220.0, 220.0, -30.0, 30.0, 0.25)
        self.pid_z = PIDAxis(280.0, 8.0, 28.0, -220.0, 220.0, -30.0, 30.0, 0.25)

        self.max_step_x = 10.0
        self.max_step_y = 12.0
        self.max_step_z = 12.0

    def attach_serial(self, ser):
        self.ser = ser

    def reset_pid(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()

    def reset_pose(self):
        self.x_cmd = X0
        self.y_cmd = Y0
        self.z_cmd = Z0
        self.reset_pid()

    def go_home(self, force_send=True):
        self.reset_pose()
        if force_send:
            self.send_current(force=True)

    def apply_manual_command(self, cmd):
        if cmd == "UP":
            self.z_cmd += MANUAL_STEP_MM
        elif cmd == "DOWN":
            self.z_cmd -= MANUAL_STEP_MM
        elif cmd == "LEFT":
            self.y_cmd += MANUAL_STEP_MM
        elif cmd == "RIGHT":
            self.y_cmd -= MANUAL_STEP_MM
        elif cmd == "FORWARD":
            self.x_cmd += MANUAL_STEP_MM
        elif cmd == "BACKWARD":
            self.x_cmd -= MANUAL_STEP_MM

        self.x_cmd = clamp(self.x_cmd, X_MIN, X_MAX)
        self.y_cmd = clamp(self.y_cmd, Y_MIN, Y_MAX)
        self.z_cmd = clamp(self.z_cmd, Z_MIN, Z_MAX)
        return self.x_cmd, self.y_cmd, self.z_cmd

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

    def send_current(self, force=False):
        payload = (round(self.x_cmd, 2), round(self.y_cmd, 2), round(self.z_cmd, 2), round(T_NEUTRAL, 2))
        should_send = True

        if SERIAL_SEND_ONLY_IF_CHANGED and self.last_sent is not None and not force:
            dx = abs(payload[0] - self.last_sent[0])
            dy = abs(payload[1] - self.last_sent[1])
            dz = abs(payload[2] - self.last_sent[2])
            dt_ang = abs(payload[3] - self.last_sent[3])
            should_send = (dx >= MIN_SEND_DELTA_MM) or (dy >= MIN_SEND_DELTA_MM) or (dz >= MIN_SEND_DELTA_MM) or (dt_ang >= MIN_SEND_DELTA_RAD)

        if should_send:
            write_json(self.ser, {
                "T": CMD_XYZT_DIRECT_CTRL,
                "x": float(payload[0]),
                "y": float(payload[1]),
                "z": float(payload[2]),
                "t": float(payload[3])
            })
            self.last_sent = payload

# ============================================================
# INITIALIZATION HELPERS
# ============================================================

def init_camera():
    set_status("OPENING CAMERA")
    cap = cv2.VideoCapture(CAM_SOURCE, CAM_BACKEND)

    if not cap.isOpened():
        print(f"[CAMERA] Failed to open {CAM_SOURCE}")
        return None

    if USE_MJPG:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    cap.set(cv2.CAP_PROP_FPS, CAM_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    time.sleep(STARTUP_CAMERA_SETTLE_SEC)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[CAMERA] Opened {actual_w}x{actual_h} @ {actual_fps:.1f} fps")

    for _ in range(STARTUP_DROP_FRAMES):
        cap.read()

    ok, frame = cap.read()
    if not ok or frame is None:
        print("[CAMERA] Failed on first frame read")
        cap.release()
        return None

    print(f"[CAMERA] First frame ok: {frame.shape}")
    return cap

def init_serial_only():
    if TEST_MODE:
        print("[SERIAL] TEST_MODE on, skipping serial")
        return None

    set_status("OPENING SERIAL")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
        ser.setRTS(False)
        ser.setDTR(False)
        time.sleep(STARTUP_SERIAL_SETTLE_SEC)
        print(f"[SERIAL] Opened {SERIAL_PORT} @ {BAUDRATE}")
        return ser
    except Exception as e:
        print(f"[SERIAL] Init failed: {e}")
        return None

def arm_safe_initialize(ser, controller):
    if ser is None:
        controller.go_home(force_send=False)
        return

    set_status("INITIALIZING ARM")
    write_json(ser, {"T": CMD_MOVE_INIT})
    time.sleep(1.0)

    if SEND_HOME_AFTER_INIT:
        controller.go_home(force_send=True)
        time.sleep(0.5)

    print("[SERIAL] Arm init complete")

# ============================================================
# MAIN LOOP
# ============================================================

def main():
    sock_thread = threading.Thread(target=socket_server_thread, daemon=True)
    sock_thread.start()

    cap = init_camera()
    if cap is None:
        set_status("CAMERA FAILED")
        return

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    # Focal pixels from HFOV. 145 deg is very wide, so keep DIST_ESTIMATE_OFFSET_CM available for tuning.
    f_pixels = (actual_w / 2.0) / math.tan(math.radians(FOV_DEG) / 2.0)

    set_status("LOADING MODELS")
    tracker = VisionTracker(FD_XML, LM_XML)
    tracker.warmup()

    ser = init_serial_only()
    controller = RoArmController(ser=ser)
    arm_safe_initialize(ser, controller)

    with state.lock:
        state.system_ready = True
    set_status("READY - HOLDING FOR FACE")

    last_send = time.time()
    face_missing_since = None
    good_face_streak = 0
    tracking_enabled = False
    long_face_loss_home_done = False
    fps_t0 = time.time()
    fps_frames = 0
    preview_fps = 0.0
    camera_fail_streak = 0

    print("\n[SYSTEM] Running. ESC quit, P pause/resume.\n")

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            camera_fail_streak += 1
            print(f"[CAMERA] Failed to read frame ({camera_fail_streak})")
            if camera_fail_streak >= MAX_CAMERA_FAIL_STREAK:
                set_status("CAMERA UNSTABLE - HOLDING POSITION")
                controller.reset_pid()
                camera_fail_streak = 0
            time.sleep(CAMERA_FAIL_RETRY_SLEEP_SEC)
            continue
        camera_fail_streak = 0

        if MIRROR_VIEW:
            frame = cv2.flip(frame, 1)

        now = time.time()

        fps_frames += 1
        if now - fps_t0 >= 1.0:
            preview_fps = fps_frames / (now - fps_t0)
            fps_frames = 0
            fps_t0 = now

        meas = tracker.process(frame, f_pixels)

        if meas.face_ok:
            good_face_streak += 1
            face_missing_since = None
            if good_face_streak >= TRACK_ENABLE_FACE_FRAMES:
                tracking_enabled = True
                long_face_loss_home_done = False
        else:
            good_face_streak = 0
            if face_missing_since is None:
                face_missing_since = now
            missing_for = now - face_missing_since
            if missing_for > TRACK_DISABLE_FACE_LOSS_SEC:
                tracking_enabled = False
                controller.reset_pid()
            if missing_for > TRACK_RESET_FILTERS_SEC:
                tracker.reset_filters()
            if RETURN_HOME_ON_LONG_FACE_LOSS and missing_for > RETURN_HOME_FACE_LOSS_SEC and not long_face_loss_home_done:
                controller.go_home(force_send=True)
                long_face_loss_home_done = True

        with state.lock:
            current_mode = state.mode
            current_locked = state.locked
            current_paused = state.paused
            current_gyro_cmd = state.gyro_cmd
            last_cmd_time = state.last_cmd_time
            system_ready = state.system_ready
            status_text = state.status_text

        if current_mode == "MANUAL" and (time.time() - last_cmd_time > 0.65):
            current_gyro_cmd = "STOP"
            with state.lock:
                state.gyro_cmd = "STOP"

        if now - last_send >= (1.0 / SEND_HZ):
            dt = clamp(now - last_send, 1e-3, MAX_DT_SEC)
            last_send = now

            if system_ready and not current_locked and not current_paused:
                if current_mode == "AUTO":
                    if tracking_enabled and meas.face_ok:
                        controller.update_from_measurement(meas, dt)
                        set_status("AUTO TRACKING")
                    else:
                        set_status("READY - HOLDING FOR FACE")
                else:
                    controller.reset_pid()
                    controller.apply_manual_command(current_gyro_cmd)
                    set_status(f"MANUAL - {current_gyro_cmd}")

                controller.send_current(force=False)
            else:
                controller.reset_pid()
                if current_locked:
                    set_status("LOCKED")
                elif current_paused:
                    set_status("PAUSED")

        H, W = frame.shape[:2]
        aim_x = int(W * AIM_CENTER_X_NORM)
        aim_y = int(H * AIM_CENTER_Y_NORM)

        if meas.face_ok and meas.bbox is not None:
            x0, y0, x1, y1 = meas.bbox
            cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 0), 2)

        if meas.target_cx is not None and meas.target_cy is not None:
            cv2.circle(frame, (int(meas.target_cx), int(meas.target_cy)), 6, (0, 255, 255), -1)

        cv2.line(frame, (aim_x - 15, aim_y), (aim_x + 15, aim_y), (255, 0, 0), 2)
        cv2.line(frame, (aim_x, aim_y - 15), (aim_x, aim_y + 15), (255, 0, 0), 2)

        ui_color = (0, 255, 0) if current_mode == "AUTO" else (0, 165, 255)
        cv2.putText(frame, f"MODE: {current_mode}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, ui_color, 2)

        if current_locked:
            cv2.putText(frame, "LOCKED", (200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        if current_paused:
            cv2.putText(frame, "PAUSED", (320, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        if current_mode == "MANUAL":
            cv2.putText(frame, f"CMD: {current_gyro_cmd}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.putText(frame, f"XYZ: {controller.x_cmd:.0f}, {controller.y_cmd:.0f}, {controller.z_cmd:.0f}",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"FPS: {preview_fps:.1f}",
                    (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(frame, f"TRACK: {'ON' if tracking_enabled else 'OFF'}", 
                    (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if tracking_enabled else (0, 0, 255), 2)
        cv2.putText(frame, f"STATUS: {status_text}",
                    (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 2)
        cv2.putText(frame, f"AIM: {AIM_CENTER_X_NORM:.2f}, {AIM_CENTER_Y_NORM:.2f}",
                    (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 2)

        if SHOW_DISTANCE_TEXT:
            if meas.dist_cm is not None:
                raw = "--" if meas.raw_dist_cm is None else f"{meas.raw_dist_cm:.1f}"
                dist_text = f"DIST: {meas.dist_cm:.1f} cm (raw {raw})"
                dist_color = (0, 255, 255)
            else:
                dist_text = "DIST: --"
                dist_color = (100, 100, 100)

            cv2.putText(frame, dist_text, (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.6, dist_color, 2)

        key = -1
        if SHOW_PREVIEW:
            cv2.imshow("RDK X5 - RoArm Controller", frame)
            key = cv2.waitKey(1) & 0xFF
        else:
            time.sleep(0.001)

        if key == 27:
            break
        elif key in [ord('p'), ord('P')]:
            with state.lock:
                state.paused = not state.paused
            controller.reset_pid()
            if PAUSE_HOLDS_POSITION:
                controller.send_current(force=True)
            set_status("PAUSED" if state.paused else "RESUMED")

    if HOME_ON_EXIT:
        controller.go_home(force_send=True)
        time.sleep(0.5)

    cap.release()
    cv2.destroyAllWindows()
    if ser is not None:
        ser.close()

if __name__ == "__main__":
    main()
