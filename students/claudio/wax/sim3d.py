"""3D simulation of a wax drawing run — the moves draw_wax.py sends, checked and animated.

simulate(ws, steps, accel, tcp)
    Replays the step list from draw_wax.plan_steps (home, enter, every mark, leave, home).
    Every move goes through the SAME checks Robot runs before sending it, with the
    workspace profile (Robot(profile=ws)): speed limits, the target, and on a cell with a
    keep-out (cart_360) the straight-line / joint path of the arm. The first move the
    checks would refuse stops the simulation — the real run would stop there too.
    Joint angles along each straight move come from ur5e.kinematics.ik_near, chained from
    the previous pose like the controller's linear moves (nominal model, few mm off the
    real arm). Times are estimates (distance / speed + acceleration ramps).

Sim3D(port)
    viser server (the library of the compose viewer) with the framework's UR5e meshes, the
    cell keep-outs, the frame, wax, border, the tool to its real tip and the marks; the
    marks turn dark as the simulated tool reaches them. Play / pause / scrub / speed in
    its panel. Started by canvas_tool.py; open http://localhost:8766 (or the 3D tab there).

Visual only — nothing here commands the robot.
"""

from __future__ import annotations

import bisect
import math
import re
import threading
import time

import numpy as np

from ur5e.kinematics import fk_frames, ik_near, interpolate_pose, matrix_to_rotvec, rotvec_to_matrix
from ur5e.safety import SafetyViolation

KEY_STEP_M = 0.01            # keyframe spacing along straight moves (animation only)


def tcp_pose(q, tcp):
    T = fk_frames(q)[-1]
    R = T[:3, :3] @ rotvec_to_matrix(tcp[3:6])
    p = T[:3, :3] @ np.asarray(tcp[:3], float) + T[:3, 3]
    return [*map(float, p), *map(float, matrix_to_rotvec(R))]


def _mark_of(label):
    m = re.match(r"mark (\d+):", label)
    return int(m.group(1)) if m else None


def simulate(ws, steps, accel, tcp, progress=None):
    """-> dict(segments, violation, total_s, done_times). Segment: t0, dur, keys [(frac, q)], label, kind, mark."""
    limits = ws.profile(ws.target)
    q = [float(v) for v in ws.home_joints]
    pose = tcp_pose(q, tcp)
    ik = lambda p, qn: ik_near(p, qn, tcp)                            # noqa: E731
    segs, t, violation, done_times = [], 0.0, None, {}
    for k, (kind, target, speed, label) in enumerate(steps):
        try:
            if kind == "joints":
                sp, ac = speed or limits.joint_speed_max, limits.joint_accel_max
                limits.check_joint_speed(sp, ac)
                limits.check_joint_target(target, tcp=tcp)
                if limits.has_obstacles:
                    limits.check_joint_path(q, target, tcp=tcp)
                dq = float(np.max(np.abs(np.asarray(target) - np.asarray(q))))
                dur = dq / sp + (sp / ac if dq > 1e-6 else 0.0)
                keys = [(0.0, q), (1.0, [float(v) for v in target])]
                q_new = keys[-1][1]
                pose_new = tcp_pose(q_new, tcp)
            elif kind == "line":
                limits.check_tcp_speed(speed, accel)
                limits.check_pose_target(target, tcp=tcp)
                if limits.has_obstacles:
                    limits.check_linear_path(pose, target, tcp=tcp, q_start=q, ik=ik)
                dist = math.dist(pose[:3], target[:3])
                n = max(1, math.ceil(dist / KEY_STEP_M))
                keys, qq = [(0.0, q)], q
                for i in range(1, n + 1):
                    qq = ik(interpolate_pose(pose, target, i / n), qq)
                    if qq is None:
                        raise SafetyViolation(f"no joint solution {i / n:.0%} of the way (out of reach / singular)")
                    keys.append((i / n, qq))
                dur = dist / speed + (speed / accel if dist > 1e-6 else 0.0)
                q_new, pose_new = qq, [float(v) for v in target]
            else:
                dur, keys, q_new, pose_new = float(target), [(0.0, q), (1.0, q)], q, pose
        except SafetyViolation as e:
            at = target[:3] if kind == "line" else tcp_pose(target, tcp)[:3]
            violation = {"step": k + 1, "of": len(steps), "label": label, "message": str(e),
                         "at": [float(v) for v in at], "t": t}
            segs.append({"t0": t, "dur": 1.0, "keys": [(0.0, q), (1.0, q)], "label": label,
                         "kind": kind, "mark": _mark_of(label), "bad": True})
            t += 1.0
            break
        mark = _mark_of(label)
        segs.append({"t0": t, "dur": dur, "keys": keys, "label": label, "kind": kind, "mark": mark})
        t += dur
        if mark and (label.endswith("plunge") or label.endswith("engrave")):
            done_times[mark] = t
        q, pose = q_new, pose_new
        if progress and k % 25 == 0:
            progress(k / len(steps))
    if progress:
        progress(1.0)
    marks_done = [done_times[m] for m in sorted(done_times)]
    return {"segments": segs, "violation": violation, "total_s": t, "done_times": marks_done}


def q_at(segs, starts, t):
    """Joint angles at time t (keyframes interpolated linearly)."""
    i = max(0, bisect.bisect_right(starts, t) - 1)
    s = segs[i]
    f = 1.0 if s["dur"] <= 0 else min(max((t - s["t0"]) / s["dur"], 0.0), 1.0)
    keys = s["keys"]
    j = max(1, bisect.bisect_left([k[0] for k in keys], f))
    j = min(j, len(keys) - 1)
    (f0, q0), (f1, q1) = keys[j - 1], keys[j]
    a = 0.0 if f1 <= f0 else (f - f0) / (f1 - f0)
    return [x0 + (x1 - x0) * a for x0, x1 in zip(q0, q1)], s, i


def tool_mesh(length, radius=0.012, n=24, body=0.82):
    """Cylinder from the flange (z=0) to body*length, then a cone to the tip at z=length.
    numpy only (trimesh's helpers need scipy, which the image does not have)."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.stack([np.cos(a) * radius, np.sin(a) * radius], axis=1)
    zb = length * body
    v = np.concatenate([np.c_[ring, np.zeros(n)], np.c_[ring, np.full(n, zb)],
                        [[0, 0, 0], [0, 0, length]]]).astype(np.float32)
    c0, tip = 2 * n, 2 * n + 1
    f = []
    for i in range(n):
        j = (i + 1) % n
        f += [[i, j, n + j], [i, n + j, n + i],      # side
              [c0, j, i],                            # flange cap
              [n + i, n + j, tip]]                   # cone
    return v, np.array(f, np.uint32)


def load_stl(path):
    """Binary or ASCII STL -> (vertices (N,3) float, faces (M,3) int), duplicate vertices merged."""
    data = open(path, "rb").read()
    n = int.from_bytes(data[80:84], "little") if len(data) >= 84 else 0
    if len(data) == 84 + 50 * n:
        rec = np.frombuffer(data, dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]), count=n, offset=84)
        tri = rec["v"].reshape(-1, 3).astype(np.float64)
    else:
        txt = data.decode("utf-8", "replace").split()
        tri = np.array([[float(txt[i + 1]), float(txt[i + 2]), float(txt[i + 3])]
                        for i, w in enumerate(txt) if w == "vertex"], np.float64)
    if len(tri) == 0 or len(tri) % 3:
        raise ValueError(f"{path}: no triangles")
    verts, inv = np.unique(np.round(tri, 4), axis=0, return_inverse=True)
    return verts, inv.reshape(-1, 3)


def tool_from_stl(path, units="mm", rot_z_deg=0.0):
    """Tool mesh in the flange frame (tool0, metres): the lowest z of the model (the mounting
    face) goes to z = 0, the tool points along +z, optional turn about the flange axis.
    Returns (vertices, faces, tip) — tip = centre of the part furthest from the flange."""
    v, f = load_stl(path)
    v = v * {"mm": 0.001, "m": 1.0}[units]
    v[:, 2] -= v[:, 2].min()
    a = math.radians(rot_z_deg)
    R = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    v = v @ R.T
    top = v[v[:, 2] > v[:, 2].max() - 0.0005]
    tip = np.array([*top[:, :2].mean(axis=0), v[:, 2].max()])
    return v.astype(np.float32), f.astype(np.uint32), tip


# ---------------------------------------------------------------- viewer --
class Sim3D:
    SPEEDS = {"1x": 1.0, "5x": 5.0, "20x": 20.0, "100x": 100.0}

    def __init__(self, port=8766, host="0.0.0.0"):
        import viser
        from ur5e.viewer import robot_model as rm
        from ur5e.viewer.transforms import tf_to_viser
        self._rm, self._tf = rm, tf_to_viser
        self.server = viser.ViserServer(host=host, port=port)
        self.lock = threading.Lock()
        self.sim = None
        self.starts = []
        self.t = 0.0
        self.playing = False
        self._done_shown = -1
        self._ignore_slider = False
        s = self.server.scene
        s.set_up_direction("+z")
        s.add_grid("/grid", width=2.0, height=2.0, plane="xy", cell_size=0.1, section_size=1.0,
                   position=(0.0, 0.0, -0.0105))
        s.add_frame("/base", axes_length=0.12, axes_radius=0.003, origin_radius=0.006)
        self.links = {ln.name: s.add_glb(f"/robot/{ln.name}", rm.mesh_bytes(ln)) for ln in rm.LINKS}
        self.tool0 = s.add_frame("/tool0", show_axes=False)
        self.tool_nodes = []
        self.static_nodes = []
        self.marks_node = None
        self.path_node = None
        self.bad_node = None
        self._focus = None
        self._build_gui()
        self._set_q(rm.HOME_Q)

        @self.server.on_client_connect
        def _(client):
            self._aim(client)
        threading.Thread(target=self._loop, daemon=True).start()

    # -------------------------------------------------------------- gui --
    def _build_gui(self):
        g = self.server.gui
        g.configure_theme(control_layout="collapsible", control_width="medium")
        self.md = g.add_markdown("**No simulation yet.** Press *Simulate in 3D* in the canvas tool.")
        self.bar = g.add_progress_bar(0.0, visible=False)
        self.b_play = g.add_button("Play", icon="player-play")
        self.b_start = g.add_button("Back to start", icon="player-skip-back")
        self.slider = g.add_slider("time (s)", min=0.0, max=1.0, step=0.1, initial_value=0.0)
        self.dd = g.add_dropdown("speed", list(self.SPEEDS), initial_value="20x")
        self.b_bad = g.add_button("Go to the problem", color="red", visible=False)
        self.cb_path = g.add_checkbox("tool tip path", False)
        self.cb_cell = g.add_checkbox("cart / keep-outs", True)

        @self.b_play.on_click
        def _(_e):
            with self.lock:
                if self.sim and self.t >= self.sim["total_s"] - 1e-6:
                    self.t = 0.0
                self.playing = not self.playing
            self.b_play.label = "Pause" if self.playing else "Play"

        @self.b_start.on_click
        def _(_e):
            self.seek(0.0)

        @self.slider.on_update
        def _(_e):
            if not self._ignore_slider:
                self.seek(float(self.slider.value), from_slider=True)

        @self.b_bad.on_click
        def _(_e):
            if self.sim and self.sim["violation"]:
                self.seek(self.sim["violation"]["t"] + 0.5)

        @self.cb_path.on_update
        def _(_e):
            if self.path_node is not None:
                self.path_node.visible = self.cb_path.value

        @self.cb_cell.on_update
        def _(_e):
            for n in self.static_nodes:
                if getattr(n, "_is_cell", False):
                    n.visible = self.cb_cell.value

    def _aim(self, client):
        """Camera: from the front of the table (-X side, where you stand), looking at the canvas."""
        if self._focus is None:
            return
        cx, cy, cz = self._focus
        client.camera.position = (cx - 0.75, cy - 0.30, cz + 0.55)
        client.camera.look_at = (cx, cy, cz)

    def progress(self, frac, text="simulating…"):
        self.bar.visible = frac < 1.0
        self.bar.value = frac * 100
        if frac < 1.0:
            self.md.content = f"**{text}** {frac:.0%}"

    # ------------------------------------------------------------ scene --
    def set_scene(self, sc):
        """sc: keepouts [(name, box)], frame [(x,y)*4], drawable, box (x,y ranges), wax_z, wall,
        rot_deg, tip_vec (flange -> real tip per the setup, m), tcp, tool_mesh (v, f, tip) or None,
        marks [[x,y,(dark)] or [x0,y0,x1,y1]], style, home."""
        s = self.server.scene
        for n in self.static_nodes + self.tool_nodes:
            n.remove()
        self.static_nodes, self.tool_nodes = [], []
        for i, (name, b) in enumerate(sc["keepouts"]):
            dims = (b.x[1] - b.x[0], b.y[1] - b.y[0], b.z[1] - b.z[0])
            c = ((b.x[0] + b.x[1]) / 2, (b.y[0] + b.y[1]) / 2, (b.z[0] + b.z[1]) / 2)
            n = s.add_box(f"/cell/{i}", color=(150, 155, 165), dimensions=dims, position=c, opacity=0.35)
            n._is_cell = True
            self.static_nodes.append(n)
        z, wall = sc["wax_z"], sc["wall"]
        fr = np.array(sc["frame"])
        ctr = fr.mean(axis=0)
        depth = float(np.linalg.norm(fr[1] - fr[0]))
        width = float(np.linalg.norm(fr[3] - fr[0]))
        yaw = math.radians(sc["rot_deg"])
        wxyz = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
        self.static_nodes.append(s.add_box("/canvas/wax", color=(240, 222, 175), dimensions=(depth, width, 0.002),
                                           position=(ctr[0], ctr[1], z - 0.001), wxyz=wxyz))

        def ring(pts, zz):
            p = [(x, y, zz) for x, y in pts]
            return [[p[i], p[(i + 1) % len(p)]] for i in range(len(p))]
        walls = ring(sc["frame"], z) + ring(sc["frame"], z + wall) + [[(x, y, z), (x, y, z + wall)] for x, y in sc["frame"]]
        self.static_nodes.append(s.add_line_segments("/canvas/walls", np.array(walls, np.float32),
                                                     (110, 115, 125), thickness=3.0, thickness_units="screen"))
        self.static_nodes.append(s.add_line_segments("/canvas/border", np.array(ring(sc["drawable"], z + 0.0004), np.float32),
                                                     (210, 140, 40), thickness=2.0, thickness_units="screen"))
        bx, by = sc["box"]["x"], sc["box"]["y"]
        box_pts = [(bx[0], by[0]), (bx[1], by[0]), (bx[1], by[1]), (bx[0], by[1])]
        self.static_nodes.append(s.add_line_segments("/canvas/box", np.array(ring(box_pts, z + 0.0008), np.float32),
                                                     (47, 125, 91), thickness=2.0, thickness_units="screen"))
        if sc.get("home"):
            hx, hy, hz = sc["home"]
            self.static_nodes.append(s.add_icosphere("/home", radius=0.006, color=(47, 111, 179), position=(hx, hy, hz)))
            self.static_nodes.append(s.add_label("/home/label", "home", position=(0.0, 0.03, 0.01),
                                                 font_screen_scale=0.7))
        # tool: flange -> real tip along tool0 z; TCP marker where the checks look
        tip_set = np.asarray(sc["tip_vec"], float)          # tip from the setup (TCP + workaround)
        if sc.get("tool_mesh"):
            v, f, tip_mesh = sc["tool_mesh"]
            self.tool_nodes.append(s.add_mesh_simple("/tool0/tool", v, f, color=(205, 120, 60)))
            self.tool_nodes.append(s.add_icosphere("/tool0/tip_mesh", radius=0.003, color=(30, 160, 60),
                                                   position=tuple(tip_mesh)))
            self.tool_nodes.append(s.add_label("/tool0/tip_mesh/l", "STL tip", position=(0.012, 0, 0),
                                               font_screen_scale=0.7))
            if np.linalg.norm(tip_mesh - tip_set) > 0.003:
                self.tool_nodes.append(s.add_icosphere("/tool0/tip_set", radius=0.004, color=(230, 30, 30),
                                                       position=tuple(tip_set)))
                self.tool_nodes.append(s.add_label("/tool0/tip_set/l", "tip from settings (what is placed on the dots)",
                                                   position=(0.012, 0, 0), font_screen_scale=0.7))
        else:
            v, f = tool_mesh(float(tip_set[2]))
            self.tool_nodes.append(s.add_mesh_simple("/tool0/tool", v, f, color=(205, 120, 60)))
            self.tool_nodes.append(s.add_icosphere("/tool0/tip", radius=0.003, color=(30, 160, 60),
                                                   position=tuple(tip_set)))
        t = sc["tcp"]
        self.tool_nodes.append(s.add_icosphere("/tool0/tcp", radius=0.005, color=(230, 50, 50), position=tuple(t[:3])))
        self.tool_nodes.append(s.add_label("/tool0/tcp/label", "TCP", position=(0.02, 0.0, 0.0),
                                           font_screen_scale=0.7))
        self._set_marks(sc)
        self._focus = (float(ctr[0]), float(ctr[1]), float(z))
        for client in self.server.get_clients().values():
            self._aim(client)

    def _set_marks(self, sc):
        s = self.server.scene
        if self.marks_node is not None:
            self.marks_node.remove()
        z = sc["wax_z"] + 0.0006
        self.mark_style = sc["style"]
        m = sc["marks"]
        if sc["style"] == "dots":
            pts = np.array([[a[0], a[1], z] for a in m], np.float32).reshape(-1, 3)
            self.marks_node = s.add_point_cloud("/canvas/marks", pts, self._mark_colors(len(m), 0),
                                                point_size=0.0018, point_shape="circle")
        else:
            seg = np.array([[(a[0], a[1], z), (a[2], a[3], z)] for a in m], np.float32).reshape(-1, 2, 3)
            self.marks_node = s.add_line_segments("/canvas/marks", seg, self._mark_colors(len(m), 0, lines=True),
                                                  thickness=1.2, thickness_units="world")
        self.n_marks = len(m)
        self._done_shown = 0

    @staticmethod
    def _mark_colors(n, done, lines=False):
        c = np.tile(np.array([[200, 180, 140]], np.uint8), (n, 1))
        c[:done] = (40, 32, 28)
        return np.repeat(c[:, None, :], 2, axis=1) if lines else c

    # -------------------------------------------------------- simulation --
    def set_sim(self, sim, tip_vec, wax_z):
        with self.lock:
            self.sim, self.starts = sim, [sg["t0"] for sg in sim["segments"]]
            self.t, self.playing = 0.0, False
            self._tip = np.asarray(tip_vec, float)
        self.b_play.label = "Play"
        total = max(sim["total_s"], 0.1)
        self._ignore_slider = True
        self.slider.max = round(total, 1) + 0.1
        self.slider.value = 0.0
        self._ignore_slider = False
        v = sim["violation"]
        s = self.server.scene
        if self.bad_node is not None:
            self.bad_node.remove()
            self.bad_node = None
        if v:
            self.bad_node = s.add_icosphere("/problem", radius=0.012, color=(230, 30, 30), opacity=0.6,
                                            position=tuple(v["at"]))
            self.md.content = (f"**Would be REFUSED** at step {v['step']} of {v['of']}: *{v['label']}*\n\n"
                               f"{v['message']}\n\nThe run would stop there.")
        else:
            self.md.content = (f"**All {len(sim['segments'])} moves pass the checks.** "
                               f"About **{sim['total_s'] / 60:.1f} min** (estimate).")
        self.b_bad.visible = bool(v)
        # tip path from the keyframes
        pts = []
        for sg in sim["segments"]:
            for _, q in sg["keys"][1:]:
                T = fk_frames(q)[-1]
                pts.append(T[:3, 3] + T[:3, :3] @ self._tip)
        if self.path_node is not None:
            self.path_node.remove()
            self.path_node = None
        if len(pts) > 1:
            P = np.array(pts, np.float32)
            self.path_node = s.add_line_segments("/path", np.stack([P[:-1], P[1:]], axis=1), (220, 80, 70),
                                                 thickness=1.5, thickness_units="screen",
                                                 visible=self.cb_path.value)
        self.seek(0.0)

    def seek(self, t, from_slider=False):
        with self.lock:
            if not self.sim:
                return
            self.t = min(max(t, 0.0), self.sim["total_s"])
        self._render(update_slider=not from_slider)

    def _set_q(self, q):
        frames = self._rm.fk(q)
        for name, T in self._rm.link_world_poses(frames).items():
            w, p = self._tf(T)
            h = self.links[name]
            h.wxyz, h.position = w, p
        w, p = self._tf(frames["tool0"])
        self.tool0.wxyz, self.tool0.position = w, p

    def _render(self, update_slider=True):
        with self.lock:
            sim, t = self.sim, self.t
        if not sim:
            return
        q, seg, i = q_at(sim["segments"], self.starts, t)
        self._set_q(q)
        done = bisect.bisect_right(sim["done_times"], t)
        if done != self._done_shown and self.marks_node is not None:
            self.marks_node.colors = self._mark_colors(self.n_marks, done, lines=self.mark_style != "dots")
            self._done_shown = done
        if update_slider:
            self._ignore_slider = True
            self.slider.value = round(t, 1)
            self._ignore_slider = False
        if not sim["violation"] or t < sim["violation"]["t"]:
            n = len(sim["done_times"])
            self.md.content = (f"**{seg['label']}** · {done}/{n} marks · {t / 60:.1f} / "
                               f"{sim['total_s'] / 60:.1f} min" +
                               ("" if not sim["violation"] else "\n\n⚠ a later move would be refused"))
        else:
            v = sim["violation"]
            self.md.content = (f"**Would be REFUSED** at step {v['step']} of {v['of']}: *{v['label']}*\n\n"
                               f"{v['message']}\n\nThe run would stop there.")

    def _loop(self):
        last = time.monotonic()
        while True:
            time.sleep(1 / 30)
            now = time.monotonic()
            dt, last = now - last, now
            with self.lock:
                if not (self.playing and self.sim):
                    continue
                self.t += dt * self.SPEEDS.get(self.dd.value, 20.0)
                if self.t >= self.sim["total_s"]:
                    self.t, self.playing = self.sim["total_s"], False
                    self.b_play.label = "Play"
            self._render()
