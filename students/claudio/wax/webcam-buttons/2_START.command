#!/bin/bash
cd "$(dirname "$0")"
if [ ! -x venv/bin/python ]; then echo "First open 1_INSTALL_ONCE.command"; read -p "Enter to close"; exit 1; fi
./venv/bin/python webcams.py
read -p "Press Enter to close"
