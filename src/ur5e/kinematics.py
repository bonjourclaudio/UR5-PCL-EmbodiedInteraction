"""UR5e forward kinematics (DH-based).

Shared by the viewer and the safety/workspace checks. Positions are in the
robot base frame, meters. fk_frames/fk_flange_position give the FLANGE frame
(tool0); fk_tcp_position(q, tcp) applies a TCP offset so the safety check
tests the same point the taught workspace is expressed in. arm_spheres(q)
gives sample points along the physical links (for the keep-out check of cell
profiles that have a solid body next to the arm, e.g. the cart). The
authoritative boundary protection is the robot's PolyScope safety planes, not this.
"""

from __future__ import annotations

import math

import numpy as np

# UR5e DH parameters (meters / radians)
DH_D = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
DH_A = [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0]
DH_ALPHA = [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0]


def _dh(theta, d, a, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,      ca,      d],
        [0.0,     0.0,     0.0,    1.0],
    ])


def fk_frames(q):
    """List of 4x4 cumulative transforms: base + each of the 6 joint frames."""
    T = np.eye(4)
    frames = [T.copy()]
    for i in range(6):
        T = T @ _dh(q[i], DH_D[i], DH_A[i], DH_ALPHA[i])
        frames.append(T.copy())
    return frames


def fk_origins(q):
    """3xN float32 array of frame origins (base + 6 joints) — for the viewer."""
    return np.array([T[:3, 3] for T in fk_frames(q)]).T.astype(np.float32)


def fk_tcp_position(q, tcp=None):
    """Position of the tool point for joint angles q (base frame, metres).
    tcp = [x, y, z, rx, ry, rz] offset relative to the flange (as set with
    set_tcp / the pendant's active TCP); None = the flange itself."""
    T = fk_frames(q)[-1]
    if tcp is not None:
        return T[:3, :3] @ np.asarray(tcp[:3], dtype=float) + T[:3, 3]
    return T[:3, 3]


def fk_flange_position(q):
    """xyz position of the flange (tool0) in the base frame."""
    return fk_frames(q)[-1][:3, 3]


# --- pose helpers (rotation vector <-> matrix; numpy only) --------------------

def rotvec_to_matrix(rv):
    """3x3 rotation matrix of a rotation vector [rx, ry, rz] (axis * angle, rad)."""
    rv = np.asarray(rv, dtype=float)
    angle = float(np.linalg.norm(rv))
    if angle < 1e-12:
        return np.eye(3)
    k = rv / angle
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def matrix_to_rotvec(R):
    """Rotation vector of a 3x3 rotation matrix (angle in 0..pi)."""
    c = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))
    angle = math.acos(c)
    if angle < 1e-9:
        return np.zeros(3)
    if math.pi - angle < 1e-6:                      # near 180 deg: axis from the diagonal
        axis = np.sqrt(np.maximum((np.diag(R) + 1.0) / 2.0, 0.0))
        i = int(np.argmax(axis))
        for j in range(3):
            if j != i and R[i, j] + R[j, i] < 0:
                axis[j] = -axis[j]
        return axis / np.linalg.norm(axis) * angle
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2.0 * math.sin(angle))
    return axis * angle


def interpolate_pose(p0, p1, t):
    """Pose a fraction t (0..1) of the way from p0 to p1 the way a linear move
    (moveL) travels: position on the straight line, orientation turning evenly
    about one axis. Poses are [x, y, z, rx, ry, rz] (m, rotation vector rad)."""
    p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    R0, R1 = rotvec_to_matrix(p0[3:6]), rotvec_to_matrix(p1[3:6])
    R = R0 @ rotvec_to_matrix(matrix_to_rotvec(R0.T @ R1) * t)
    return [*(p0[:3] + (p1[:3] - p0[:3]) * t), *matrix_to_rotvec(R)]


def flange_position_from_pose(pose, tcp=None):
    """Flange (tool0) position for a TCP pose, given the TCP offset
    [x, y, z, rx, ry, rz] relative to the flange (None = TCP is the flange)."""
    p = np.asarray(pose[:3], dtype=float)
    if tcp is None:
        return p
    R_tcp = rotvec_to_matrix(pose[3:6])
    R_flange = R_tcp @ rotvec_to_matrix(tcp[3:6]).T
    return p - R_flange @ np.asarray(tcp[:3], dtype=float)


def ik_near(pose, q_near, tcp=None, max_iter=80, tol_pos=1e-5, tol_rot=1e-5):
    """Joint angles that put the TCP at `pose`, found by iterating from q_near
    on the nominal model (damped least squares, steps limited to 0.2 rad per
    joint) — so the answer is the solution CONNECTED to q_near, the one a
    linear move arrives at when it starts from q_near and the pose is close.
    Returns a list of 6 angles, or None if it does not converge (pose out of
    reach, or at a singularity).

    Used for the arm check along linear moves: chain it from the current
    joints through closely spaced poses. Not a general IK solver, and not used
    to command the robot. (The controller's own get_inverse_kin_has_solution
    is not usable through ur_rtde 1.6.3 on PolyScope 5.9: it answers False for
    reachable poses — found 2026-11-18 in URSim 5.9.4.)"""
    q = np.array(q_near, dtype=float)
    R_tcp = rotvec_to_matrix(pose[3:6])
    if tcp is not None:
        R_t = R_tcp @ rotvec_to_matrix(tcp[3:6]).T
        p_t = np.asarray(pose[:3], dtype=float) - R_t @ np.asarray(tcp[:3], dtype=float)
    else:
        R_t, p_t = R_tcp, np.asarray(pose[:3], dtype=float)

    def err(qq):
        T = fk_frames(qq)[-1]
        return np.concatenate([p_t - T[:3, 3], matrix_to_rotvec(R_t @ T[:3, :3].T)])

    eps = 1e-6
    for _ in range(max_iter):
        e = err(q)
        if np.linalg.norm(e[:3]) < tol_pos and np.linalg.norm(e[3:]) < tol_rot:
            return [float(v) for v in q]
        J = np.zeros((6, 6))
        for j in range(6):
            dq = np.zeros(6)
            dq[j] = eps
            J[:, j] = -(err(q + dq) - e) / eps
        step = np.linalg.solve(J.T @ J + 1e-6 * np.eye(6), J.T @ e)
        biggest = float(np.max(np.abs(step)))
        if biggest > 0.2:
            step *= 0.2 / biggest
        q = q + step
    return None


# --- physical arm, as spheres ---------------------------------------------------
# The DH frames above are a skeleton; the real links sit beside it: the upper arm
# tube runs 0.138 m out along the shoulder-lift axis, the forearm 0.007 m, wrist 1
# comes back out to d4 = 0.1333 m (ur_description 4.4.0, visual_parameters.yaml —
# the same numbers the viewer's robot_model uses). Nominal geometry, no calibration.
UPPER_ARM_OFFSET = 0.138
FOREARM_OFFSET = 0.007
_SPHERE_STEP = 0.04          # spacing of sample points along a link (m)


def _along(a, b, step=_SPHERE_STEP):
    n = max(1, int(math.ceil(float(np.linalg.norm(b - a)) / step)))
    return [a + (b - a) * (i / n) for i in range(n + 1)]


def arm_spheres(q, radius=0.06):
    """Sample points on the centre lines of the moving links for joint angles q
    (base frame, metres), as a list of (link name, xyz). A link is treated as
    the set of spheres of `radius` around its points. Covered: upper arm,
    elbow, forearm, wrist 1, wrist 2, wrist 3. NOT covered: base and shoulder
    (fixed on the mount) and whatever is mounted on the flange (see
    SafetyLimits.check_arm, which adds the flange->TCP line).

    Wrist 3 ends in the flange FACE: its last sphere sits one radius behind the
    flange, so the spheres reach exactly to the flange plane, not beyond it."""
    T = fk_frames(q)
    P = [t[:3, 3] for t in T]
    side = T[1][:3, 2]                       # shoulder-lift axis = lateral direction of the arm
    out = []
    a, b = P[1] + side * UPPER_ARM_OFFSET, P[2] + side * UPPER_ARM_OFFSET
    out += [("upper arm", p) for p in _along(a, b)[1:]]            # [0] is the shoulder itself
    out += [("elbow", p) for p in _along(P[2] + side * UPPER_ARM_OFFSET, P[2] + side * FOREARM_OFFSET)]
    out += [("forearm", p) for p in _along(P[2] + side * FOREARM_OFFSET, P[3] + side * FOREARM_OFFSET)]
    out += [("wrist 1", p) for p in _along(P[3], P[4])]
    out += [("wrist 2", p) for p in _along(P[4], P[5])]
    z6 = T[6][:3, 2]
    end = P[6] - z6 * radius
    if float(np.dot(end - P[5], z6)) > 0:
        out += [("wrist 3", p) for p in _along(P[5], end)]
    else:
        out.append(("wrist 3", P[5]))
    return out
