#!/bin/bash
cd "$(dirname "$0")"
PY=$(command -v python3)
if [ -z "$PY" ]; then echo "Python non trovato. Installalo da python.org e riprova."; read -p "Invio per chiudere"; exit 1; fi
echo "Creo l'ambiente e installo i pacchetti..."
"$PY" -m venv venv && ./venv/bin/pip install requests pyserial && echo && echo "FATTO. Ora apri 2_AVVIA.command"
read -p "Premi Invio per chiudere"
