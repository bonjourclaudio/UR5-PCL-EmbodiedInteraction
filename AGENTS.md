# AGENTS.md

This repository is the student framework for a UR5e cobot course (ZHdK,
Embodied Interaction HS26). Students write Python in `students/<name>/`
against the `ur5e` package in `src/ur5e/`, run it in Docker against a
simulated robot, and later on the real arm.

Before doing anything else, read:

1. `context/INSTRUCTIONS.md` — the rules for assisting here (use the
   framework API only, taught workspace or absolute poses in metres/radians,
   sim first, never bypass `SafetyViolation`, no invented methods).
2. `context/KNOWLEDGE.md` — the facts: environment, full `Robot` /
   `Workspace` API, safety profiles, lab robot, typical errors.

3. `context/CART_360.md` — the lab's safety setup (`cart_360`): the arm on
   the cart may work around and below the cart top, the cart body is a
   keep-out and every move is checked along its path. Read it before writing
   anything that moves beside or below the cart top.

Then look at `examples/function_templates/` — new scripts start as a copy of
the closest template. Reference material for deeper questions:
`context/ur_rtde/` (ur_rtde docs as text) and `context/manual/` (UR5e user
manual). `src/`, `scripts/`, `examples/`, `docs/` and `docker-compose.yml`
are maintained upstream and replaced on `git pull` — do not edit them;
student work belongs in `students/<name>/`.
