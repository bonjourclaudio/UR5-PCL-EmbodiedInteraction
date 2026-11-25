#!/bin/bash
cd "$(dirname "$0")"
PY=$(command -v python3)
if [ -z "$PY" ]; then echo "Python not found. Install it from python.org and try again."; read -p "Enter to close"; exit 1; fi
echo "Creating the environment and installing the packages..."
"$PY" -m venv venv && ./venv/bin/pip install requests pyserial pillow && echo && echo "DONE. Now open 2_START.command"
read -p "Press Enter to close"
