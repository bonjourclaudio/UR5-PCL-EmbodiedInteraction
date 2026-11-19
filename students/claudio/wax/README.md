# Wax engraving

Heated tool draws an image into wax — dot-work (default, `--style dots`) or hatch lines (`--style hatch`).

| file | what |
|---|---|
| `setup_canvas.py` | guided setup with the robot: TCP check, 2 corners by freedrive, home, 4-corner check (values remembered) |
| `canvas_tool.py` + `web/` | browser tool, no robot needed: see/adjust canvas, border, corners, width, cutting level, image + dot preview, save |
| `sim3d.py` | 3D simulation (viser, like the compose viewer): the exact moves of a run, checked like `Robot` does, animated |
| `draw_wax.py` | main script: image → dots/lines → engrave (sim / real); picks up the saved setup and image settings |
| `geometry.py` | frame geometry from the two corners, rotation helpers (shared by setup + tool) |
| `imaging.py` | image loading (PNG/PGM/PPM, JPEG only if Pillow exists), dots (error diffusion) + hatch lines, SVG preview |
| `heater.py` | `HeaterGuard`: temperature thread, on/off control, latched over-temp cutoff; `FakeHotend` for the sim |
| `place_frame.py` | hover over a square to place the frame (not with the TCP-Z workaround) |
| `inbox/` | images; the tool's upload goes here |
| `out/preview.svg` | written on every run: exactly what will be engraved |
| `wax_setup.<target>.json` | saved setup: payload, TCP, corners, home, frame, image/dot settings |

## Canvas tool (browser)

On the laptop:

```bash
docker compose run --rm --no-deps -p 127.0.0.1:8765:8765 -p 127.0.0.1:8766:8766 dev python students/claudio/wax/canvas_tool.py
```

Open http://localhost:8765. The view is from above, standing opposite the robot (robot at the
top). Drag BL / TL or type coordinates, adjust the frame and image, Save. Saving writes the
workspace + setup file (previous versions kept as `.bak`). Home still comes from
`setup_canvas.py` (step 5). "✓ same as the saved workspace" tells you a Save would not change
the workspace.

**Simulate in 3D** replays the exact move list `draw_wax.py` sends (home, enter, every dot,
leave, home) and runs each move through the checks `Robot` runs with your workspace and the
lab cell (`cart_360`). The 3D tab (or http://localhost:8766) animates it: play, scrub, speed,
tool-tip path. If a move would be refused, it says which one and why — the real run would
stop there. Times are estimates; joint angles come from the nominal model (a few mm off the
real arm).

Tool model in the 3D view: add `--tool-stl students/claudio/flange.stl` to the command (STL in
mm; its lowest face = the face on the robot flange, tool along +z; `--tool-rot DEG` turns it
about the flange axis). The view marks the STL's tip and the tip from your settings; if they
are more than 3 mm apart the page warns — moves and checks always use the settings (TCP).

## Draw

Run inside `docker compose run --rm dev bash`:

```bash
python students/claudio/wax/draw_wax.py --target real --dry-run
python students/claudio/wax/draw_wax.py --target real --no-heat --hover-mm 10
python students/claudio/wax/draw_wax.py --demo-image --size-mm 80 --sim-overheat
python students/claudio/wax/draw_wax.py --help
```

Flags you type override the saved settings.
