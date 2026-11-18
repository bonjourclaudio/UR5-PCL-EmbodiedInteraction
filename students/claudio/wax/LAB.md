# Wax canvas on the real UR5e — lab procedure

Canvas: 180 × 180 mm inner frame, aluminium walls ~15 mm above the wax,
20 mm border → 140 × 140 mm drawable area.
Only with the instructor present, speed slider down, a hand near the e-stop.

## Before: measure

- tool mass incl. mount (kg)
- tip distance from the flange face (mm), and any sideways offset
- the tip at the wax surface must be ≥ 29 mm above the table (lab cell floor),
  otherwise raise the frame on a board
- wax flat (straightedge), tip length below the heater block > wall height

## 1. Shell

```bash
docker compose run --rm --no-deps -e UR_HOST=192.168.1.11 dev bash
```

## 2. Guided setup (once per setup)

```bash
python students/claudio/wax/setup_canvas.py --target real
```

It walks you through 7 steps:

1. Payload + TCP: you type them, then set the same values on the pendant.
2. Connects read-only and checks the pendant's TCP matches.
3. **Freedrive** (pendant in **Local** mode): tip on the wax in the
   **bottom-left inner corner** of the frame (standing opposite the robot:
   nearest corner, on your left).
4. Frame size, border, wall height: computes the 140 mm area and checks it
   lies inside the lab cell.
5. **Freedrive**: home, above the canvas centre, ~50–75 mm above the wax.
6. Saves `workspace.wax-real.json` + `wax_setup.real.json`.
7. Pendant to **Remote Control**: the arm hovers over the 4 corners
   (30 mm, then 5 mm above the wax). Check the 20 mm border.

Rerun it any time; the values you typed are remembered.

## 3. First runs (pendant in Remote Control)

`draw_wax.py` reads workspace, payload and TCP from the setup file.

```bash
python students/claudio/wax/draw_wax.py --target real --size-mm 60 --dry-run
python students/claudio/wax/draw_wax.py --target real --size-mm 60 --no-heat --hover-mm 10
python students/claudio/wax/draw_wax.py --target real --size-mm 60 --no-heat
```

1. Preview: check `out/preview.svg` and the time.
2. Cold, in the air: image the right way round from the front of the table?
   (Otherwise `--rotate` / `--mirror`.)
3. Cold, touching: the tip touches the surface at every dot, also in the centre.

## 4. Hot

Only once the PT100 transmitter is on standard analog input 0 and the heater
current is checked: drop `--no-heat`, add `--sensor-temp T0 T1` for your
transmitter. Test piece first: `--size-mm 30`, tune `--dwell-min` /
`--dwell-max` / `--setpoint`.
