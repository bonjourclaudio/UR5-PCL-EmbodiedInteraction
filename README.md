# UR5e — Embodied Interaction & Fabrication HS26

Python framework and simulator setup for the UR5e cobot course at ZHdK
Interaction Design. You program a **simulated UR5e** on your own laptop with
the exact same code that later runs on the real arm in the lab.

**→ [docs/SETUP.md](docs/SETUP.md)** — install Docker Desktop, VS Code, Git;
first start; checks. Do this before the first session.

**→ [docs/USAGE.md](docs/USAGE.md)** — the Python API, examples, sim vs. real
robot, the 3D viewer (scenes, models, toolheads, safety planes), PolyScope
programs → USB → pendant.

**→ [context/](context/README.md)** — knowledge base for your AI assistant
(ChatGPT, Claude, Copilot, Cursor, …): instructions, API facts, the framework
source as one file, the ur_rtde docs and the UR5e user manual. Give it to
your tool before you let it write robot code.

## In short

Start the simulator and the 3D viewer:

```bash
docker compose up -d ursim viewer
```

Open http://localhost:6080/vnc.html (Connect → power button → ON → START →
header icon: Remote Control) and http://localhost:7001/ (live 3D view). Then
open a Python shell inside the container and run things:

```bash
docker compose run --rm dev bash
python scripts/check_connection.py --target sim
python examples/function_templates/00_teach_workspace.py --target sim --demo
python examples/function_templates/03_move_to.py
```

The second line writes `workspace.json` — the box the examples move in
(teach it properly in four steps without `--demo`; see
`examples/function_templates/README.md`).

**Simulator = the lab robot's software version.** The simulator in
`docker-compose.yml` is pinned to the PolyScope version of the real UR5e
(`universalrobots/ursim_e-series:5.9.4` — the robot runs 5.9.1, same 5.9 line),
so programs you save in the simulator load on the real pendant. On the pendant
the version and the robot's IP are under **☰ (top right) → About**:
*URSoftware 5.9.1…* and *IP address 192.168.1.11*. The IP is static — that is
what you put into `UR_HOST` in the lab (see [docs/USAGE.md](docs/USAGE.md) §4).
In the lab your laptop needs a manual address on that network — `192.168.1.1NN`,
mask `255.255.255.0`, cable in — and `ping 192.168.1.11` must answer *before*
you start Docker (USAGE §4 has the steps).
Only the instructor changes the pinned version, after a PolyScope update.

`[screenshot: pendant ☰ → About — version and IP address]`

```python
from ur5e import Robot, Workspace

ws = Workspace.load()                          # the taught box
with Robot(target="sim", profile=ws) as robot:
    robot.move_joints(ws.home_joints)
    for cell in ws.grid(3, 3, w=0.1):          # 3 x 3 over the area, 10 % up
        robot.move_to(cell)
        if robot.read_digital_in(0):
            robot.set_digital_out(0, True)
            robot.set_digital_out(0, False)
    robot.move_joints(ws.home_joints)
```

## What's in the repo

```
docs/            SETUP.md, USAGE.md
src/ur5e/        the framework: Robot API, connection, motion checks, viewer
scripts/         check_connection.py, smoke_test_move.py, rtde_viewer.py
examples/        grid_process.py — copy it to start your own
examples/function_templates/   one script per API call + 00_teach_workspace.py (teach your work area: 8 corners + home)
examples-gripper/              the lab gripper: two-pose demo + Gripper class (experimental; mock gripper in the sim)
scenes/          viewer scenes (scene.json + models/)
toolheads/       toolhead presets for the viewer (mesh + TCP)
programs/        where the simulator saves PolyScope programs (.urp) — git-ignored
students/        your work: students/<your-name>/ — git-ignored
context/         for your AI assistant: INSTRUCTIONS.md, KNOWLEDGE.md, framework_source.md, ur_rtde docs, UR5e manual
AGENTS.md / CLAUDE.md / .github/copilot-instructions.md   picked up automatically by editor AI tools; they point into context/
docker-compose.yml   ursim (simulator) · dev (Python shell) · viewer (3D)
Start UR5e Sim.command / .bat   double-click launchers (optional)
```

Nothing is installed on your laptop besides Docker Desktop, VS Code and Git —
Python, `ur_rtde` and the simulator run in containers
(`ghcr.io/reckj/ur5e-dev`, `universalrobots/ursim_e-series`).

Updates: `git pull`, then `docker compose down && docker compose up -d ursim viewer`.
