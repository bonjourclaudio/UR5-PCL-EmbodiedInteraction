#!/bin/bash
# make_source_snapshot.sh — concatenate the framework source + templates into
# context/framework_source.md so it can be uploaded to a chat tool as one file.
# Run from anywhere; regenerate after `git pull`:
#   ./context/make_source_snapshot.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/context/framework_source.md"

FILES=(
  src/ur5e/__init__.py
  src/ur5e/robot.py
  src/ur5e/cli.py
  src/ur5e/workspace.py
  src/ur5e/safety.py
  src/ur5e/motion.py
  src/ur5e/connection.py
  src/ur5e/config.py
  src/ur5e/kinematics.py
  src/ur5e/cell.py
  src/ur5e/cells/cart_360.json
  examples/function_templates/README.md
  examples/function_templates/00_teach_workspace.py
  examples/function_templates/01_read_state.py
  examples/function_templates/02_move_joints.py
  examples/function_templates/03_move_to.py
  examples/function_templates/04_move_through.py
  examples/function_templates/05_grid_poses.py
  examples/function_templates/06_digital_io.py
  examples/function_templates/07_analog_io.py
  examples/function_templates/08_safety_rejections.py
  examples/function_templates/09_raw_rtde.py
  examples/grid_process.py
  docker-compose.yml
)

{
  echo "# ur5e framework — source snapshot"
  echo
  echo "Generated $(date -u +%Y-%m-%dT%H:%MZ) from git $(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo '?')"
  echo "by context/make_source_snapshot.sh. The files below are the student-facing"
  echo "framework (src/ur5e/, without the viewer) and the function templates, verbatim."
  echo "When this snapshot and the repo disagree, the repo is right — regenerate."
  echo
  for f in "${FILES[@]}"; do
    [ -f "$ROOT/$f" ] || { echo "warning: missing $f" >&2; continue; }
    case "$f" in
      *.py)  lang=python ;;
      *.md)  lang=markdown ;;
      *.yml) lang=yaml ;;
      *)     lang=text ;;
    esac
    echo "---"
    echo
    echo "## \`$f\`"
    echo
    echo "\`\`\`$lang"
    cat "$ROOT/$f"
    echo
    echo "\`\`\`"
    echo
  done
} > "$OUT"

echo "wrote $OUT ($(wc -c < "$OUT" | tr -d ' ') bytes)"
