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
    "pyyaml"
)

install_py_packages() {
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

# Uninstall python package if not already uninstalled
py_package_uninstaller() {
    TO_BE_CHECKED=$1
    if is_py_package_installed "$TO_BE_CHECKED"; then
        echo "Python Package: $TO_BE_CHECKED ✗ Package installed"
        echo "Uninstalling $TO_BE_CHECKED now..."
        pip uninstall -y "$TO_BE_CHECKED"
        if [ $? -eq 0 ]; then
            echo "✓ $TO_BE_CHECKED uninstalled successfully"
        else
            echo "✗ Failed to uninstall $TO_BE_CHECKED"
            return 1
        fi
    else
        echo "Python Package: $TO_BE_CHECKED ✓ Package already not installed"
    fi
}

# Parse Motor.yaml and build pi3hat-cfg string
build_pi3hat_cfg() {
    if [ ! -f "config/Motor.yaml" ]; then
        echo "✗ config/Motor.yaml not found"
        return 1
    fi

    # Extract motor map and group by bus
    # Format: bus=motor_id1,motor_id2;bus=motor_id3,motor_id4
    declare -A bus_map
    
    # Parse Motor.yaml to build bus_map
    while IFS=: read -r motor_id bus_id; do
        motor_id=$(echo "$motor_id" | xargs)  # trim whitespace
        bus_id=$(echo "$bus_id" | xargs | cut -d'#' -f1 | xargs)  # trim, remove comments, trim again
        
        # Skip empty lines and motorMap line
        if [ -z "$motor_id" ] || [ "$motor_id" = "motorMap" ]; then
            continue
        fi
        
        # Build bus_map
        if [ -z "${bus_map[$bus_id]}" ]; then
            bus_map[$bus_id]="$motor_id"
        else
            bus_map[$bus_id]="${bus_map[$bus_id]},$motor_id"
        fi
    done < <(grep -v "^#" config/Motor.yaml | grep -v "motorMap" | grep ":")
    
    # Build the pi3hat-cfg string
    local cfg_string=""
    for bus in $(printf '%s\n' "${!bus_map[@]}" | sort -n); do
        if [ -z "$cfg_string" ]; then
            cfg_string="$bus=${bus_map[$bus]}"
        else
            cfg_string="$cfg_string;$bus=${bus_map[$bus]}"
        fi
    done
    
    echo "$cfg_string"
}

# Get all motor IDs from Motor.yaml
get_motor_ids() {
    if [ ! -f "config/Motor.yaml" ]; then
        return 1
    fi
    
    grep -v "^#" config/Motor.yaml | grep ":" | awk '{print $1}' | grep -v "motorMap" | tr -d ':' | tr '\n' ',' | sed 's/,$//'
}

# Calibrate wheels using moteus_tool
calibrate_wheels() {
    echo "Building pi3hat configuration from Motor.yaml..."
    
    local all_motor_ids=$(get_motor_ids)
    if [ -z "$all_motor_ids" ]; then
        echo "✗ Failed to extract motor IDs from Motor.yaml"
        return 1
    fi
    
    echo "Available motor IDs: $all_motor_ids"
    echo ""
    read -p "Calibrate all motors or select specific? (a/s): " -n 1 -r
    echo
    
    local motor_ids=""
    if [[ $REPLY =~ ^[Aa]$ ]]; then
        motor_ids="$all_motor_ids"
        echo "✓ Calibrating all motors: $motor_ids"
    elif [[ $REPLY =~ ^[Ss]$ ]]; then
        read -p "Enter motor IDs to calibrate (comma-separated, e.g. 1,4): " motor_ids
        if [ -z "$motor_ids" ]; then
            echo "✗ No motors selected"
            return 1
        fi
        echo "✓ Calibrating motors: $motor_ids"
    else
        echo "✗ Invalid choice"
        return 1
    fi
    
    local pi3hat_cfg=$(build_pi3hat_cfg)
    if [ -z "$pi3hat_cfg" ]; then
        echo "✗ Failed to parse Motor.yaml"
        return 1
    fi
    
    echo "✓ pi3hat config: $pi3hat_cfg"
    echo ""
    echo "⚠ WARNING: Ensure all motors can spin freely before proceeding!"
    read -p "Continue with calibration? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Calibration cancelled"
        return 0
    fi
    
    echo "Starting motor calibration..."
    sudo ./.rframework-venv/bin/moteus_tool --pi3hat-cfg "$pi3hat_cfg" -t "$motor_ids" --calibrate
    
    if [ $? -eq 0 ]; then
        echo "✓ Calibration completed successfully"
    else
        echo "✗ Calibration failed"
        return 1
    fi
}

# Menu
PS3='Main Menu --> What would you like to do? '
main_options=(
    "Check Packages"
    "Install Packages"
    "Callibrate Wheels"
    "Run Tests"
    "Quit"
)
select opt in "${main_options[@]}"
do
    case $opt in
        "Check Packages")
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
            echo "Starting installing process..."
            install_py_packages
            ;;

        "Callibrate Wheels")
            ensure_venv || { echo "Cannot calibrate without venv"; break; }
            calibrate_wheels
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
