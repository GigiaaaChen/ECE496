# command.py
# Distance-only control for RoArm-M2-S (keep camera ~30 cm from eyes)

import json

# ---- Distance control targets ----
DIST_TARGET_CM = 30.0     # we want ~30 cm
DIST_TOLERANCE = 3        # avoid shaking

# ---- Elbow joint characteristics (J2) ----
ELBOW_MIN = -45.0        
ELBOW_MAX = 180.0     

# current elbow command (start neutral)
elbow_angle = 90.0        

K_DIST = -50.0            # deg per (dist_error / target_distance)

# Degree per update, rate limit so we don't jerk the arm
MAX_DELTA = 3.0           # max elbow change per update (deg)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def update_elbow(dist_cm):
    """
    Given measured distance (cm), compute desired elbow angle.
    No hardware I/O here; just math. Returns updated elbow_angle (deg).
    """
    global elbow_angle

    if dist_cm is None:
        return elbow_angle

    # distance error: if user is too far or too closefrom the target distance
    err = dist_cm - DIST_TARGET_CM

    # dead zone to avoid tiny oscillations
    if abs(err) < DIST_TOLERANCE:
        return elbow_angle

    # relative error, e.g. +0.33 if 40cm vs 30cm
    rel = err / DIST_TARGET_CM

    # ideal elbow angle before rate limiting
    ideal = 90.0 + K_DIST * rel
    ideal = clamp(ideal, ELBOW_MIN, ELBOW_MAX)

    # apply rate limit
    delta = ideal - elbow_angle
    if delta >  MAX_DELTA:
        delta =  MAX_DELTA
    if delta < -MAX_DELTA:
        delta = -MAX_DELTA

    elbow_angle = clamp(elbow_angle + delta, ELBOW_MIN, ELBOW_MAX)
    return elbow_angle


# ---- Build command for the board ----

def build_command(elbow, spd=10, acc=10):
    """
    Build a JSON command dict. For now:
      - base (b) fixed at 0
      - shoulder (s) fixed at 0
      - elbow (e) controlled
      - wrist (h) fixed at 180 (phone upright-ish)
    """
    cmd = {
        "T": 122,          # joint angle command type
        "b": 0.0,          # base joint
        "s": 0.0,          # shoulder joint
        "e": float(elbow), # elbow joint
        "h": 180.0,        # wrist joint
        "spd": int(spd),
        "acc": int(acc),
    }
    return cmd


def serialize_command(cmd):
    """Convert command dict to a JSON line the board can read."""
    return json.dumps(cmd) + "\n"


def send(cmd):
    """
    Stub for now: just print command.
    Later, replace with USB serial write.
    """
    line = serialize_command(cmd)
    print("[CMD]", line, end="")


# Small self-test if you run this file directly:
if __name__ == "__main__":
    # pretend distances
    for d in [50, 45, 40, 35, 32, 30, 28, 26]:
        e = update_elbow(d)
        c = build_command(e)
        send(c)
        print("  # dist =", d, "→ elbow =", e)
