This repository is a UR5e cobot student framework. Follow
`context/INSTRUCTIONS.md` and use the facts in `context/KNOWLEDGE.md`:
write Python with `from ur5e import Robot, Workspace`; positions as
fractions of the taught workspace (`ws.pose(u, v, w)`, `ws.grid`, `ws.line`)
or as absolute Base-frame poses, metres and radians only, `target="sim"` unless the student is in the lab,
never work around `SafetyViolation` (the lab cell `cart_360` treats the cart body as a keep-out and checks whole paths — see `context/CART_360.md`: go out past the cart's edge, then down, in short `move_to` legs; rehearse with `--cell cart_360`), never invent methods not listed in
`context/KNOWLEDGE.md` §4, start new scripts from
`examples/function_templates/`, keep them in `students/<name>/`, and do not
edit `src/`, `scripts/`, `examples/` or `docs/`.
