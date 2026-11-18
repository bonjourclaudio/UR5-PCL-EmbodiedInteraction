# Wax engraving

Heated tool draws an image into wax — dot-work (default, `--style dots`) or hatch lines (`--style hatch`).

| file | what |
|---|---|
| `setup_canvas.py` | guided setup with the robot: TCP check, 2 corners by freedrive, home, 4-corner check (values remembered) |
| `canvas_tool.py` + `web/` | browser tool, no robot needed: see/adjust canvas, border, corners, image + dot preview, save |
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
docker compose run --rm --no-deps -p 127.0.0.1:8765:8765 dev python students/claudio/wax/canvas_tool.py
```

Open http://localhost:8765. The view is from above, standing opposite the robot (robot at the
top). Drag BL / TL or type coordinates, adjust the frame and image, Save. Saving writes the
workspace + setup file (previous versions kept as `.bak`). Home still comes from
`setup_canvas.py` (step 5).

## Draw

Run inside `docker compose run --rm dev bash`:

```bash
python students/claudio/wax/draw_wax.py --target real --dry-run
python students/claudio/wax/draw_wax.py --target real --no-heat --hover-mm 10
python students/claudio/wax/draw_wax.py --demo-image --size-mm 80 --sim-overheat
python students/claudio/wax/draw_wax.py --help
```

Flags you type override the saved settings.
