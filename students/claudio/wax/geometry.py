"""Canvas geometry shared by setup_canvas.py and canvas_tool.py. No robot.

    frame_geometry(bl_tip, tl_tip, ...)  two taught inner corners -> frame, border, drawable corners
    rotvec_to_mat / mat_to_rotvec / tilt_deg / straight_down   rotation helpers (UR rotation vectors)

Coordinates: robot Base frame, metres. "bl"/"tl" = the frame's bottom-left / top-left inner
corner as seen standing opposite the robot (top = towards the robot).
"""

from __future__ import annotations

import math

import numpy as np


def rotvec_to_mat(r):
    r = np.asarray(r, float)
    a = np.linalg.norm(r)
    if a < 1e-12:
        return np.eye(3)
    k = r / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def mat_to_rotvec(R):
    """Robust also near 180 deg (tool pointing down is ~pi)."""
    t = np.trace(R)
    if t > -0.99:
        w = math.sqrt(max(1 + t, 0)) / 2
        q = np.array([w, (R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w),
                      (R[1, 0] - R[0, 1]) / (4 * w)])
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(max(1 + R[i, i] - R[j, j] - R[k, k], 0)) * 2
        q = np.zeros(4)
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i] = s / 4
        q[1 + j] = (R[j, i] + R[i, j]) / s
        q[1 + k] = (R[k, i] + R[i, k]) / s
    if q[0] < 0:
        q = -q
    angle = 2 * math.atan2(np.linalg.norm(q[1:]), q[0])
    n = np.linalg.norm(q[1:])
    return list(q[1:] / n * angle) if n > 1e-12 else [0.0, 0.0, 0.0]


def tilt_deg(rotvec) -> float:
    z = rotvec_to_mat(rotvec)[:, 2]
    return math.degrees(math.acos(np.clip(-z[2], -1, 1)))


def straight_down(rotvec):
    """Same heading, tool axis exactly vertical (smallest correcting rotation)."""
    R = rotvec_to_mat(rotvec)
    z, down = R[:, 2], np.array([0.0, 0.0, -1.0])
    axis = np.cross(z, down)
    s, c = np.linalg.norm(axis), float(np.dot(z, down))
    if s < 1e-9:
        return list(rotvec)
    fix = rotvec_to_mat(axis / s * math.atan2(s, c))
    return mat_to_rotvec(fix @ R)


def estimate_gap_mm(bl_tip, tl_tip, frame_mm: float) -> float:
    """Gap between tip and walls implied by the corner distance vs. the frame size (0.5 mm steps)."""
    dist = float(np.linalg.norm(np.asarray(tl_tip[:2]) - np.asarray(bl_tip[:2]))) * 1000
    return max(0.0, round((frame_mm - dist) / 2 * 2) / 2)


def frame_geometry(bl_tip, tl_tip, *, frame_mm: float, border_mm: float, gap_mm: float,
                   extra_m: float, ceiling_mm: float) -> dict:
    """From the two recorded REAL tip positions (m) to the canvas.

    The frame is a square of frame_mm (inner); its 'up' direction and depth come from the
    two corners, its width is frame_mm. gap_mm = how far the tip stayed from the walls.
    Returns metres unless the key says _mm / _deg:
      frame      4 inner frame corners (x, y): BL, TL, TR, BR
      drawable   4 corners inside the border  (x, y): BL, TL, TR, BR
      zs, zt     TCP heights: tip on the (higher) wax / ceiling   (TCP = tip + extra_m up)
      wax_z      real tip height on the wax (higher of the two corners)
      rot_deg, depth_mm, tip_dist_mm, level_mm
    """
    bl_t, tl_t = np.asarray(bl_tip, float), np.asarray(tl_tip, float)
    d = tl_t[:2] - bl_t[:2]
    tip_dist = float(np.linalg.norm(d))
    if tip_dist < 0.05:
        raise ValueError("the two corners are less than 50 mm apart — corners mixed up?")
    u = d / tip_dist                                  # frame 'up' (bottom -> top)
    right = np.array([u[1], -u[0]])                   # frame 'right'
    g, b = gap_mm / 1000, border_mm / 1000
    blc = bl_t[:2] - g * (u + right)
    tlc = tl_t[:2] + g * u - g * right
    depth = float(np.linalg.norm(tlc - blc))
    width = frame_mm / 1000
    if depth - 2 * b < 0.02 or width - 2 * b < 0.02:
        raise ValueError("frame minus border leaves no drawable area")
    frame = [blc, tlc, tlc + width * right, blc + width * right]
    bld = blc + b * u + b * right
    tld = blc + (depth - b) * u + b * right
    drawable = [bld, tld, tld + (width - 2 * b) * right, bld + (width - 2 * b) * right]
    wax_z = max(bl_t[2], tl_t[2])
    zs = wax_z + extra_m
    return {
        "frame": [tuple(map(float, p)) for p in frame],
        "drawable": [tuple(map(float, p)) for p in drawable],
        "zs": float(zs), "zt": float(zs + ceiling_mm / 1000), "wax_z": float(wax_z),
        "rot_deg": math.degrees(math.atan2(u[1], u[0])),
        "depth_mm": depth * 1000, "width_mm": frame_mm, "tip_dist_mm": tip_dist * 1000,
        "level_mm": abs(bl_t[2] - tl_t[2]) * 1000,
    }
