#!/usr/bin/env bash

# RUN USING:    bash ./SETUP.sh [action]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

VENV_DIR=".rframework-venv"

SYSTEM_PACKAGES=(
    "cmake"
    "libopencv-dev"
    "python3-opencv"
    "python3-venv"
    "libserial-dev"
    "libraspberrypi-dev"
    "libyaml-cpp-dev"
    "pkg-config"
    "raspberrypi-kernel-headers"
)

PYTHON_PACKAGES=(
    "moteus"
    "moteus-pi3hat"
    "moteus_gui"
    "pyyaml"
    "numpy"
    "requests"
)

print_banner() {
    echo "--------------------------------------------------"
    echo " --- WELCOME TO ROBOT FRAMEWORK SETUP & CHECKUP --- "
    echo "--------------------------------------------------"
    echo ""
}

is_package_installed() {
    local package_name=$1
    dpkg-query --show -f='${Status}' "$package_name" 2>/dev/null | grep -q "install ok installed"
}

package_installer() {
    local package_name=$1

    if is_package_installed "$package_name"; then
        echo "Dependency package: $package_name ✓ installed"
        return 0
    fi

    echo "Dependency package: $package_name ✗ missing"
    echo "Installing $package_name now..."
    if sudo apt install -y "$package_name"; then
        echo "✓ $package_name installed successfully"
    else
        echo "✗ Failed to install $package_name"
        return 1
    fi
}

package_uninstaller() {
    local package_name=$1

    if ! is_package_installed "$package_name"; then
        echo "Dependency package: $package_name ✓ already not installed"
        return 0
    fi

    echo "Dependency package: $package_name ✗ installed"
    echo "Uninstalling $package_name now..."
    if sudo apt remove -y "$package_name"; then
        echo "✓ $package_name uninstalled successfully"
    else
        echo "✗ Failed to uninstall $package_name"
        return 1
    fi
}

check_system_dependencies() {
    local status=0

    echo "Checking system dependencies..."
    for package in "${SYSTEM_PACKAGES[@]}"; do
        if is_package_installed "$package"; then
            echo "✓ $package installed"
        else
            echo "✗ $package missing"
            status=1
        fi
    done
    echo "System dependency check complete!"

    return $status
}

install_system_dependencies() {
    echo "Installing system dependencies..."
    sudo apt update || return 1

    for package in "${SYSTEM_PACKAGES[@]}"; do
        package_installer "$package" || return 1
    done

    echo "System dependency install complete!"
}

uninstall_system_dependencies() {
    echo "Uninstalling system dependencies..."
    sudo apt update || return 1

    for package in "${SYSTEM_PACKAGES[@]}"; do
        package_uninstaller "$package" || return 1
    done

    echo "System dependency uninstall complete!"
}

install_venv() {
    if [ -d "$VENV_DIR" ]; then
        echo "✓ $VENV_DIR already exists"
    else
        echo "Creating virtual environment at $VENV_DIR..."
        python3 -m venv "$VENV_DIR" || return 1
    fi

    echo "Activating $VENV_DIR..."
    # shellcheck disable=SC1091
    source "$VENV_DIR/bin/activate"
}

check_venv() {
    [ -n "$VIRTUAL_ENV" ] && [ "$(basename "$VIRTUAL_ENV")" = "$VENV_DIR" ]
}

activate_venv() {
    if [ ! -d "$VENV_DIR" ]; then
        echo "✗ $VENV_DIR not found"
        return 1
    fi

    echo "Activating $VENV_DIR..."
    # shellcheck disable=SC1091
    source "$VENV_DIR/bin/activate"
}

ensure_venv() {
    local reply

    if check_venv; then
        echo "✓ Already inside $VENV_DIR"
        return 0
    fi

    if [ ! -d "$VENV_DIR" ]; then
        read -r -p "No virtual environment found at $VENV_DIR. Create one now? [y/N] " reply
        if [[ ! $reply =~ ^[Yy]$ ]]; then
            echo "Cannot continue without a virtual environment"
            return 1
        fi

        install_venv || return 1
    else
        activate_venv || return 1
    fi

    if check_venv; then
        echo "✓ $VENV_DIR is active"
        return 0
    fi

    echo "✗ Failed to activate $VENV_DIR"
    return 1
}

is_py_package_installed() {
    pip show "$1" >/dev/null 2>&1
}

py_package_installer() {
    local package_name=$1

    if is_py_package_installed "$package_name"; then
        echo "Python package: $package_name ✓ installed"
        return 0
    fi

    echo "Python package: $package_name ✗ missing"
    echo "Installing $package_name now..."
    if pip install "$package_name"; then
        echo "✓ $package_name installed successfully"
    else
        echo "✗ Failed to install $package_name"
        return 1
    fi
}

py_package_uninstaller() {
    local package_name=$1

    if ! is_py_package_installed "$package_name"; then
        echo "Python package: $package_name ✓ already not installed"
        return 0
    fi

    echo "Python package: $package_name ✗ installed"
    echo "Uninstalling $package_name now..."
    if pip uninstall -y "$package_name"; then
        echo "✓ $package_name uninstalled successfully"
    else
        echo "✗ Failed to uninstall $package_name"
        return 1
    fi
}

check_python_packages() {
    local status=0

    if [ ! -d "$VENV_DIR" ]; then
        echo "✗ $VENV_DIR not found"
        return 1
    fi

    activate_venv || return 1

    echo "Checking Python packages..."
    for package in "${PYTHON_PACKAGES[@]}"; do
        if is_py_package_installed "$package"; then
            echo "Python package: $package ✓ installed"
        else
            echo "Python package: $package ✗ missing"
            status=1
        fi
    done

    return $status
}

install_python_packages() {
    ensure_venv || return 1

    echo "Upgrading pip..."
    pip install --upgrade pip || return 1

    echo "Installing Python packages..."
    for package in "${PYTHON_PACKAGES[@]}"; do
        py_package_installer "$package" || return 1
    done

    echo "Python package install complete!"
}

uninstall_python_packages() {
    if [ ! -d "$VENV_DIR" ]; then
        echo "✗ $VENV_DIR not found"
        return 1
    fi

    activate_venv || return 1

    echo "Uninstalling Python packages..."
    for package in "${PYTHON_PACKAGES[@]}"; do
        py_package_uninstaller "$package" || return 1
    done

    echo "Python package uninstall complete!"
}

ensure_moteus_tool() {
    local reply

    ensure_venv || return 1

    if [ -x "$VENV_DIR/bin/moteus_tool" ]; then
        return 0
    fi

    echo "✗ moteus_tool is not installed in $VENV_DIR"
    read -r -p "Install Python packages now? [y/N] " reply
    if [[ ! $reply =~ ^[Yy]$ ]]; then
        return 1
    fi

    install_python_packages || return 1
    [ -x "$VENV_DIR/bin/moteus_tool" ]
}

build_pi3hat_cfg() {
    declare -A bus_map
    local cfg_string=""
    local bus
    local motor_id
    local bus_id

    if [ ! -f "config/Motor.yaml" ]; then
        echo "✗ config/Motor.yaml not found"
        return 1
    fi

    while IFS=: read -r motor_id bus_id; do
        motor_id=$(echo "$motor_id" | xargs)
        bus_id=$(echo "$bus_id" | xargs | cut -d'#' -f1 | xargs)

        if [ -z "$motor_id" ] || [ "$motor_id" = "motorMap" ]; then
            continue
        fi

        if [ -z "${bus_map[$bus_id]}" ]; then
            bus_map[$bus_id]="$motor_id"
        else
            bus_map[$bus_id]="${bus_map[$bus_id]},$motor_id"
        fi
    done < <(grep -v '^#' config/Motor.yaml | grep ':')

    for bus in $(printf '%s\n' "${!bus_map[@]}" | sort -n); do
        if [ -z "$cfg_string" ]; then
            cfg_string="$bus=${bus_map[$bus]}"
        else
            cfg_string="$cfg_string;$bus=${bus_map[$bus]}"
        fi
    done

    echo "$cfg_string"
}

get_motor_ids() {
    if [ ! -f "config/Motor.yaml" ]; then
        echo ""
        return 1
    fi

    grep -v '^#' config/Motor.yaml | grep ':' | grep -v 'motorMap' | awk '{print $1}' | tr -d ':' | tr '\n' ',' | sed 's/,$//'
}

calibrate_wheels() {
    local all_motor_ids
    local motor_ids
    local pi3hat_cfg
    local reply

    ensure_moteus_tool || {
        echo "Cannot calibrate without moteus_tool in $VENV_DIR"
        return 1
    }

    echo "Building pi3hat configuration from config/Motor.yaml..."
    all_motor_ids=$(get_motor_ids)
    if [ -z "$all_motor_ids" ]; then
        echo "✗ Failed to extract motor IDs from config/Motor.yaml"
        return 1
    fi

    echo "Available motor IDs: $all_motor_ids"
    read -r -p "Calibrate all motors or select specific ones? (a/s): " reply

    if [[ $reply =~ ^[Aa]$ ]]; then
        motor_ids="$all_motor_ids"
        echo "✓ Calibrating all motors: $motor_ids"
    elif [[ $reply =~ ^[Ss]$ ]]; then
        read -r -p "Enter motor IDs to calibrate (comma-separated, e.g. 1,4): " motor_ids
        if [ -z "$motor_ids" ]; then
            echo "✗ No motors selected"
            return 1
        fi
        echo "✓ Calibrating motors: $motor_ids"
    else
        echo "✗ Invalid choice"
        return 1
    fi

    pi3hat_cfg=$(build_pi3hat_cfg)
    if [ -z "$pi3hat_cfg" ]; then
        echo "✗ Failed to parse config/Motor.yaml"
        return 1
    fi

    echo "✓ pi3hat config: $pi3hat_cfg"
    echo "⚠ WARNING: Ensure all motors are lifted and can spin freely before proceeding."
    read -r -p "Continue with calibration? [y/N] " reply
    if [[ ! $reply =~ ^[Yy]$ ]]; then
        echo "Calibration cancelled"
        return 0
    fi

    echo "Starting motor calibration..."
    if sudo "$SCRIPT_DIR/$VENV_DIR/bin/moteus_tool" --pi3hat-cfg "$pi3hat_cfg" -t "$motor_ids" --calibrate; then
        echo "✓ Calibration completed successfully"
    else
        echo "✗ Calibration failed"
        return 1
    fi
}

run_moteus_setup() {
    local reply

    install_python_packages || return 1

    echo "Moteus Python environment is ready in $VENV_DIR"
    read -r -p "Run wheel calibration now? [y/N] " reply
    if [[ $reply =~ ^[Yy]$ ]]; then
        calibrate_wheels
    else
        echo "Skipping calibration"
    fi
}

build_robot_framework() {
    echo "Building RobotFramework..."
    cmake -S . -B build && cmake --build build
}

run_checkup() {
    local status=0

    check_system_dependencies || status=1
    echo ""
    check_python_packages || status=1

    return $status
}

full_setup() {
    install_system_dependencies || return 1
    install_python_packages || return 1
    build_robot_framework || return 1
    echo "Setup complete!"
}

print_usage() {
    cat <<'EOF'
Usage:
  bash ./SETUP.sh                # interactive menu
  bash ./SETUP.sh checkup        # check Linux and Python dependencies
  bash ./SETUP.sh moteus-setup   # create venv, install moteus packages, optionally calibrate
  bash ./SETUP.sh full-setup     # install system deps, Python deps, and build

Actions:
  menu
  checkup
  check-system
  install-system
  uninstall-system
  check-python
  install-python
  uninstall-python
  moteus-setup
  calibrate-wheels
  build
  full-setup
  help
EOF
}

show_menu() {
    local options
    local opt

    PS3='Main Menu --> What would you like to do? '
    options=(
        "Run Checkup"
        "Check System Dependencies"
        "Install System Dependencies"
        "Uninstall System Dependencies"
        "Check Python Packages"
        "Install Python Packages"
        "Uninstall Python Packages"
        "Run Moteus Setup"
        "Calibrate Wheels"
        "Build RobotFramework"
        "Full Setup"
        "Quit"
    )

    select opt in "${options[@]}"
    do
        case $opt in
            "Run Checkup")
                run_checkup
                ;;
            "Check System Dependencies")
                check_system_dependencies
                ;;
            "Install System Dependencies")
                install_system_dependencies
                ;;
            "Uninstall System Dependencies")
                uninstall_system_dependencies
                ;;
            "Check Python Packages")
                check_python_packages
                ;;
            "Install Python Packages")
                install_python_packages
                ;;
            "Uninstall Python Packages")
                uninstall_python_packages
                ;;
            "Run Moteus Setup")
                run_moteus_setup
                ;;
            "Calibrate Wheels")
                calibrate_wheels
                ;;
            "Build RobotFramework")
                build_robot_framework
                ;;
            "Full Setup")
                full_setup
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
}

run_action() {
    case "$1" in
        menu)
            show_menu
            ;;
        check|checkup)
            run_checkup
            ;;
        check-system)
            check_system_dependencies
            ;;
        install-system)
            install_system_dependencies
            ;;
        uninstall-system)
            uninstall_system_dependencies
            ;;
        check-python)
            check_python_packages
            ;;
        install-python)
            install_python_packages
            ;;
        uninstall-python)
            uninstall_python_packages
            ;;
        moteus-setup)
            run_moteus_setup
            ;;
        calibrate-wheels)
            calibrate_wheels
            ;;
        build|build-robot-framework)
            build_robot_framework
            ;;
        full-setup)
            full_setup
            ;;
        help|-h|--help)
            print_usage
            ;;
        *)
            echo "Unknown action: $1"
            echo ""
            print_usage
            return 1
            ;;
    esac
}

main() {
    print_banner

    if [ $# -eq 0 ]; then
        show_menu
        return
    fi

    run_action "$1"
}

main "$@"