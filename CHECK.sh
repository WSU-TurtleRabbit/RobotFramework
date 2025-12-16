#!/usr/bin/env bash

# RUN USING:    bash ./CHECK.sh

echo "--------------------------------------------------"
echo "     --- WELCOME TO ROBOT FRAMEWORK SETUP ---     "
echo "--------------------------------------------------"
echo ""

# Python Package list
PACKAGES=(
    "moteus"
    "moteus_pi3hat"
    "moteus_gui"
)

install_py_moteus() {
    # Checking for Venv
    echo "Checking for venv..."
    if check_venv; then
        echo "✓ Already inside .rframework-venv"
    else
        echo "✗ Not inside .rframework-venv"
        if [ -d ".rframework-venv" ]; then
            echo "Tempting to activating venv..."
            source .rframework-venv/bin/activate
        else
            install_venv
        fi
    fi

    if check_venv; then
        echo "Upgrading pip..."
        pip install --upgrade pip
        echo "Installing moteus packages..."
        for package in "${PACKAGES[@]}"; do
                py_package_installer "$package"
            done
    else
        echo "✗ Failed to activate .rframework-venv"
        exit 1
    fi
}

install_venv() {
    # Create the venv (Virtual Environment) with .rframework venv
    echo "Creating venv..."
    python3 -m venv .rframework-venv

    # Activiate the venv
    echo "Activating venv..."
    source .rframework-venv/bin/activate
}

check_venv() {
    if [[ -n "$VIRTUAL_ENV" && "$VIRTUAL_ENV" == *"/.rframework-venv"* ]]; then
        return 0
    else
        return 1
    fi
}

is_py_package_installed() {
    pip show "$1" >/dev/null 2>&1
}

# Install python package if not already installed
py_package_installer() {
    TO_BE_CHECKED=$1
    if is_py_package_installed "$TO_BE_CHECKED"; then
        echo "Python Package: $TO_BE_CHECKED ✓ Package installed"
    else
        echo "Python Package: $TO_BE_CHECKED ✗ Package missing"
        echo "Installing $TO_BE_CHECKED now..."
        pip install "$TO_BE_CHECKED"
        if [ $? -eq 0 ]; then
            echo "✓ $TO_BE_CHECKED installed successfully"
        else
            echo "✗ Failed to install $TO_BE_CHECKED"
            return 1
        fi
    fi
}

# Menu
PS3='What would you like to do? '
options=(
    "Run Tests"
    "Callibrate"
    "Install Python Packages"
    "Quit"
)
select opt in "${options[@]}"
do
    case $opt in
        "Run Tests")
            echo "Sorry, this feature isn't done yet..."
            ;;
        "Callibrate")
            echo "Sorry, this feature isn't done yet..."
            ;;
        "Install Python Packages")
            echo "Starting installing process..."
            install_py_moteus
            ;;
        "Quit")
            echo "Laters!"
            exit 0
            ;;
    esac
done