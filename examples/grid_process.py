"""First example process — grid traversal with sensor-gated actuation (SIM).

The skeleton students adapt for their digital-fabrication processes: traverse a
grid over the work surface; at each cell read a sensor (analog + digital) and
conditionally run the actuator tool. Built entirely on the student-facing base
functions in `ur5e.robot` — no RTDE or safety internals here.

In SIM the inputs read static 0, so every cell will 'skip'. To see the process
branch fire, set a standard digital/analog input HIGH in URSim's I/O tab, or
wire real sensors. Students replace `process_at_cell` with their actual process.

Run (dev container, URSim ON/STARTED, viewer open to watch):
    python examples/grid_process.py
    python examples/grid_process.py --target real      # lab, supervised: the grid
        # starts at the arm's CURRENT pose, so freedrive it over the table first
        # (inside the cell, see `python -m ur5e.cell`); --payload/--tcp if a tool is mounted
"""

from ur5e import Robot, grid_poses
from ur5e.cli import argument_parser, robot_kwargs

ANALOG_PIN = 0
DIGITAL_PIN = 0
ACTUATOR_PIN = 0
TRIGGER_THRESHOLD_V = 0.5   # actuate when the analog sensor exceeds this


def process_at_cell(robot: Robot) -> None:
    """The per-cell 'process'. Students replace this body (servo, dwell, etc.)."""
    robot.set_digital_out(ACTUATOR_PIN, True)
    robot.set_digital_out(ACTUATOR_PIN, False)


def main() -> None:
    args = argument_parser(__doc__).parse_args()
    with Robot(**robot_kwargs(args)) as robot:
        # Anchor the grid to the current reachable pose. Students re-anchor this
        # to the measured work surface. Small 3x3 grid, 4 cm spacing.
        origin = robot.pose()
        cells = grid_poses(origin, nx=3, ny=3, dx=0.04, dy=0.04)
        print(f"traversing {len(cells)} cells ...")

        for k, cell in enumerate(cells):
            robot.move_to(cell)
            analog = robot.read_analog_in(ANALOG_PIN)
            digital = robot.read_digital_in(DIGITAL_PIN)
            triggered = analog > TRIGGER_THRESHOLD_V or digital
            print(f"  cell {k}: analog={analog:.3f}V digital={digital} -> "
                  f"{'PROCESS' if triggered else 'skip'}")
            if triggered:
                process_at_cell(robot)

        robot.move_to(origin)  # return to start
    print("done.")


if __name__ == "__main__":
    main()
