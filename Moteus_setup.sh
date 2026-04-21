#!/bin/bash
 
VENV_DIR="./moteus_venv"
 
# Create venv if it doesn't exist
if [ ! -d "$VENV_DIR" ]; then
    read -rp "No virtual environment found. Create one? [y/N] " yn
    if [[ "$yn" =~ ^[Yy]$ ]]; then
        python3.11 -m venv "$VENV_DIR"
        echo "Virtual environment created."
    else
        echo "Exiting."
        exit 1
    fi
fi
 
# Activate and install moteus
source "$VENV_DIR/bin/activate"
pip install --upgrade pip --quiet
pip install moteus moteus-pi3hat numpy requests --quiet
echo "Dependencies installed."
 
# Run calibration as admin
echo "Starting calibration"
echo "Ensure motors are lifted"
sudo moteus_tool \
    --pi3hat-cfg "1=1;2=2;3=3;4=4" \
    -t 1,2,3,4 \
    --calibrate
 
echo "Done."