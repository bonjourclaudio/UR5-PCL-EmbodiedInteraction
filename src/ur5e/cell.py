"""Print the cell profile — the measured lab setup the checks run against.

STATUS: HS26 student framework (thin printing helper, no robot logic).

    python -m ur5e.cell                the real-arm cell (cart_360) and the sim default
    python -m ur5e.cell cart_360       one profile by name
    python -m ur5e.cell --all          every profile

The cell is selected by ur5e.config.cell_profile_name: sim_open in the
simulator, cart_360 on the real arm; `--cell NAME` on the examples (or the
environment variable UR5E_CELL) overrides it, e.g. `--cell cart_360` to
rehearse in the simulator against the cart keep-out.
"""

from __future__ import annotations

import sys

from .config import cell_profile_name
from .safety import PROFILES, get_profile


def main(argv=None) -> None:
    args = list(sys.argv[1:] if argv is None else argv)
    if "--all" in args:
        names = sorted(PROFILES)
    elif args:
        names = args
    else:
        names = list(dict.fromkeys([cell_profile_name("real"), cell_profile_name("sim")]))
        print(f"real arm -> '{names[0]}'    simulator -> '{names[-1]}'\n")
    for name in names:
        print(get_profile(name).describe(), "\n")


if __name__ == "__main__":
    main()
