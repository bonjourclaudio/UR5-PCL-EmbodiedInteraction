"""Named connection targets.

The ONLY place the sim<->real host differs. Everything else in the framework
is host-agnostic.
"""

from __future__ import annotations

import os

# Env-overridable so the same code works from the host, from the dev container
# (URSIM_HOST=ursim via compose), and against the real arm (UR_HOST=<lan ip>).
HOSTS = {
    "sim": os.environ.get("URSIM_HOST", "127.0.0.1"),   # URSim
    "real": os.environ.get("UR_HOST", "0.0.0.0"),       # UR5e LAN IP
}


def resolve_host(target: str) -> str:
    try:
        return HOSTS[target]
    except KeyError:
        raise KeyError(f"unknown target {target!r}; known: {sorted(HOSTS)}") from None


# Hosts treated as simulation (no real-arm gate). Includes the resolved sim host.
_SIM_HOSTS = {"127.0.0.1", "localhost", "ursim"}


def is_sim(host: str) -> bool:
    return host in _SIM_HOSTS or host == HOSTS["sim"]


# --- which cell profile bounds the arm -------------------------------------------
# The cell profile is the measured physical setup (safety.py PROFILES). A taught
# Workspace is clipped to it; Robot(profile=None) uses it directly.
REAL_CELL_DEFAULT = "cart_360"       # arm on the free-standing cart (HS26, from 2026-11-18)
SIM_CELL_DEFAULT = "sim_open"

_warned_open = False


def cell_profile_name(target: str) -> str:
    """Name of the cell profile for 'sim' / 'real'.

    Default: sim -> sim_open (free exploration), real -> cart_360.
    Environment variable UR5E_CELL (or the examples' --cell flag) overrides it:
      UR5E_CELL=cart_360            use the cart cell in the SIM too — rehearse a
                                    script against the cart keep-out before the lab
      UR5E_CELL=cart_against_wall   the earlier table-only cell
      UR5E_CELL=open                real arm, INSTRUCTOR OVERRIDE: no cell — only the
                                    taught workspace box limits the arm ('real_open')
    """
    global _warned_open
    choice = os.environ.get("UR5E_CELL", "").strip()
    if choice.lower() in ("open", "none", "off"):
        if target != "real":
            return SIM_CELL_DEFAULT
        if not _warned_open:
            print("[ur5e] !! UR5E_CELL=open: the measured cell profile is NOT applied — only the taught "
                  "workspace box limits the real arm (instructor override; PolyScope planes still apply)")
            _warned_open = True
        return "real_open"
    if choice:
        return choice
    return REAL_CELL_DEFAULT if target == "real" else SIM_CELL_DEFAULT
