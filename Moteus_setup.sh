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
pip install moteus moteus-pi3hat numpy moteus_gui requests --quiet
echo "Dependencies installed."
 