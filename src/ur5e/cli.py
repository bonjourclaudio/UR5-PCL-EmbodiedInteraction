"""Command-line plumbing shared by the examples: ``--target sim|real`` and friends.

STATUS: validated for student use (HS26) — thin argparse glue, no robot logic.

Every example script does::

    from ur5e import Robot, Workspace
    from ur5e.cli import argument_parser, robot_kwargs

    args = argument_parser(__doc__).parse_args()
    ws = Workspace.load(args.workspace)
    with Robot(**robot_kwargs(args), profile=ws) as robot:
        ...

so the same file runs in the simulator (default) and, in the lab, on the real
arm with ``--target real``.  ``--target real`` is the conscious act that enables
real-arm motion: it sets ``allow_real=True`` and declares the tool
(``--payload`` kg, ``--tcp`` x y z rx ry rz; defaults = nothing mounted, TCP at
the flange).  The host comes from ``UR_HOST`` (lab: 192.168.1.11) or ``--host``.
``--cell cart_360`` checks a sim run against the lab cell (the cart keep-out)
— what the real arm will enforce.
"""

from __future__ import annotations

import argparse
import os


class _SetCell(argparse.Action):
    """--cell NAME: select the cell profile for this run (sets UR5E_CELL)."""

    def __call__(self, parser, namespace, values, option_string=None):
        os.environ["UR5E_CELL"] = values
        setattr(namespace, self.dest, values)


def argument_parser(description: str | None = None) -> argparse.ArgumentParser:
    """ArgumentParser with the standard example options. Add your own on top."""
    ap = argparse.ArgumentParser(description=description,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", choices=("sim", "real"), default="sim",
                    help="sim (default) or real = the lab arm, supervised (implies allow_real)")
    ap.add_argument("--host", default=None,
                    help="override the robot IP/hostname (default: URSIM_HOST / UR_HOST)")
    ap.add_argument("--workspace", default=None, metavar="FILE",
                    help="workspace file (default: workspace.json or $UR5E_WORKSPACE)")
    ap.add_argument("--cell", action=_SetCell, default=None, metavar="NAME",
                    help="cell profile to check against (default: sim_open in the sim, cart_360 on the "
                         "real arm). '--cell cart_360' in the sim rehearses against the cart keep-out")
    ap.add_argument("--payload", type=float, default=0.0, metavar="KG",
                    help="real only: mass of the mounted tool in kg (default 0.0 = nothing mounted)")
    ap.add_argument("--tcp", type=float, nargs=6, default=(0, 0, 0, 0, 0, 0),
                    metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
                    help="real only: TCP offset in m / rad, as on the pendant (default: flange)")
    return ap


def robot_kwargs(args) -> dict:
    """Keyword arguments for ``Robot(...)`` from parsed arguments."""
    kw = {"target": args.target, "host": args.host}
    if args.target == "real":
        if not args.host and not os.environ.get("UR_HOST"):
            raise SystemExit("--target real needs the robot's address: start the shell with\n"
                             "  docker compose run --rm --no-deps -e UR_HOST=192.168.1.11 dev bash\n"
                             "or pass --host 192.168.1.11")
        kw.update(allow_real=True, payload_kg=args.payload, tcp=tuple(args.tcp))
    return kw
