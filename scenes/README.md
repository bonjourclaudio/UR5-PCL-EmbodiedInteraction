# scenes/ — viewer scenes

Each sub-folder is one self-contained scene for `scripts/rtde_viewer.py`:

    scenes/<name>/scene.json      the scene (objects, toolhead + TCP, features, safety planes)
    scenes/<name>/models/*.stl    the 3D files it references (relative paths)

Zip / commit / AirDrop the whole folder. Switch scenes in the viewer page
(Scene panel). To start on a specific scene, put `VIEWER_SCENE=<name>` in a
`.env` file next to `docker-compose.yml`.

`cart_360` is the lab cell (the cart the robot stands on, with handle and mounting
plate; the pendant installation has no planes), generated from `src/ur5e/cells/cart_360.json` — do not
edit it by hand. `example_cart` is an illustration only — its placement is
assumed, not measured.
Format reference: `docs/USAGE.md` (Viewer → Scenes); full writeup in the dev
repo's `Documentation/11-viewer-scenes-and-safety-planes.md`.
Nothing in a scene limits the robot; safety planes here are drawn, not enforced.
