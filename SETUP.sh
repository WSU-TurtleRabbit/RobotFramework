#!/bin/bash
if [ -z "$SETUP_SELF_CLEANED" ]; then
  if grep -q $'\r' "$0" 2>/dev/null; then
    TMP_SCRIPT="$(mktemp /tmp/SETUP_clean.XXXXXX.sh)" || { echo "mktemp failed"; exit 1; }
    # Remove CR characters into tmp script
    tr -d '\r' < "$0" > "$TMP_SCRIPT" || { echo "failed to write cleaned script"; rm -f "$TMP_SCRIPT"; exit 1; }
    chmod +x "$TMP_SCRIPT" 2>/dev/null || true
    # Mark that we're the cleaned child and tell it where the tmp is so it can clean up
    export SETUP_SELF_CLEANED=1
    export SETUP_TMP_SCRIPT="$TMP_SCRIPT"
    exec bash "$TMP_SCRIPT" "$@"
    # exec replaces the shell; doesn't return
  fi
fi
# ROBOTFRAMEWORK SETUP
echo "--------------------------------------------------"
echo "     --- WELCOME TO ROBOT FRAMEWORK SETUP ---     "
echo "--------------------------------------------------"
echo ""
# Package / Dependency list
PACKAGES=(
    "cmake"
    "libopencv-dev"
    "python3-opencv"
    "libserial-dev"
    "libraspberrypi-dev"
    "pkg-config"
    "raspberrypi-kernel-headers"
)
# Check if the package is installed
is_package_installed() {
    PACKAGE_NAME=$1
    if dpkg-query --show -f='${Status}' "$PACKAGE_NAME" 2>/dev/null | grep -q "install ok installed"; then
        return 0
    else
        return 1
    fi
}
# Install package if not already installed
package_installer() {
    TO_BE_CHECKED=$1
    if is_package_installed "$TO_BE_CHECKED"; then
        echo "Dependancy Package: $TO_BE_CHECKED ✓ Dependancy installed"
    else
        echo "Dependancy Package: $TO_BE_CHECKED ✗ Dependancy missing"
        echo "Installing $TO_BE_CHECKED now..."

        sudo apt install -y "$TO_BE_CHECKED"
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
    "Check Dependencies"
    "Install Dependencies"
    "Build RobotFramework"
    "Full Setup"
    "Quit"
)
select opt in "${options[@]}"
do
    case $opt in
        "Check Dependencies")
            echo "Checking dependancies..."
            for package in "${PACKAGES[@]}"; do
                if is_package_installed "$package"; then
                    echo "✓ $package installed"
                else
                    echo "✗ $package missing"
                fi
            done
            echo "Check complete!"
            ;;

        "Install Dependencies")
            echo "Installing dependancies..."
            sudo apt update
            for package in "${PACKAGES[@]}"; do
                package_installer "$package"
            done
            echo "Install complete!"
            ;;

        "Build RobotFramework")
            echo "Building RobotFramework..."
            rm -rf build
            mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
            ;;

        "Full Setup")
            echo "Installing dependancies..."
            sudo apt update
            for package in "${PACKAGES[@]}"; do
                package_installer "$package"
            done

            echo "Building RobotFramework..."
            rm -rf build
            mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
            echo "Setup complete!"
            ;;

        "Quit")
            echo "Laters!"
            break
            ;;
    esac
done