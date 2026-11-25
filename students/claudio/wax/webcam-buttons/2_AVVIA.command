#!/bin/bash
cd "$(dirname "$0")"
if [ ! -x venv/bin/python ]; then echo "Prima apri 1_INSTALLA_UNA_VOLTA.command"; read -p "Invio per chiudere"; exit 1; fi
./venv/bin/python webcams.py
read -p "Premi Invio per chiudere"
