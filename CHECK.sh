#!/usr/bin/env bash

# RUN USING:    bash ./CHECK.sh

echo "--------------------------------------------------"
echo "     --- WELCOME TO ROBOT CHECKUP ---     "
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
    if ensure_venv; then
        echo "✓ .rframework-venv is active"
    else
        echo "✗ Failed to activate .rframework-venv"
        exit 1
    fi

    # Upgrade pip and install packages
    echo "Upgrading pip..."
    pip install --upgrade pip
    echo "Installing moteus packages..."
    for package in "${PACKAGES[@]}"; do
        py_package_installer "$package"
    done
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

activate_venv() {
    if [ -d ".rframework-venv" ]; then
        echo "Activating .rframework-venv..."
        source .rframework-venv/bin/activate
        return 0
    else
        echo "✗ .rframework-venv not found! Please create it first"
        return 1
    fi
}

ensure_venv() {
    if check_venv; then
        echo "✓ Already inside .rframework-venv"
        return 0
    else
        echo "⚠ Venv not active. Trying to activate..."
        activate_venv

        # Re-check after activation
        if ! check_venv; then
            echo "✗ Failed to activate .rframework-venv"
            return 1
        fi
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
PS3='Main Menu --> What would you like to do? '
main_options=(
    "Tests"
    "Callibration"
    "Packages"
    "Quit"
)
select opt in "${main_options[@]}"
do
    case $opt in
        "Tests")
            echo "Sorry, this feature isn't done yet..."
            ;;

        "Callibration")
            echo "Sorry, this feature isn't done yet..."
            ;;

        "Packages")
            PS3='Package Menu --> What would you like to do? '
            package_options=(
                "Check Packages"
                "Install Packages"
                "Uninstall Packages"
            )
            select p_opt in "${package_options[@]}"
            do
                case $p_opt in
                    "Check Packages" )
                        # Ensure the venv is active before checking packages
                        ensure_venv || { echo "Cannot check packages without venv"; break; }

                        for package in "${PACKAGES[@]}"; do
                            if is_py_package_installed "$package"; then
                                echo "Python Package: $package ✓ Package installed"
                            else
                                echo "Python Package: $package ✗ Package missing"
                            fi
                        done
                        ;;
                    "Install Packages" )
                        # Ensure the venv is active before installing packages
                        ensure_venv || { echo "Cannot install packages without venv"; break; }

                        echo "Starting installing process..."
                        install_py_moteus
                        ;;
                    "Uninstall Packages" )
                        echo "Uninstall feature not implemented yet"
                        ;;
                    *)
                        echo "Invalid option, try again."
                        ;;
                esac
            done
            ;;

        "Quit")
            echo "Laters!"
            exit 0
            ;;
        *)
            echo "Invalid option, try again."
            ;;
    esac
done
