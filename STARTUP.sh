#!/usr/bin/env bash

# Startup manager for RobotFramework.
#
# On boot the systemd service calls this script in --run-service mode,
# which connects to WiFi and launches build/RobotFramework.
#
# How to use it (run from any directory, must have sudo):
#   sudo bash ./STARTUP.sh --enable    # install service + enable autostart on boot
#   sudo bash ./STARTUP.sh --disable   # disable autostart (does not stop current run)
#        bash ./STARTUP.sh --status    # show service status + last logs
#        bash ./STARTUP.sh --now       # connect to WiFi and launch right now (no install)
#        bash ./STARTUP.sh             # show this help
#
# Configuration lives in config/Startup.yaml.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

SERVICE_NAME="robotframework"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
BINARY="build/RobotFramework"
STARTUP_CFG="config/Startup.yaml"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

die() { echo "Error: $*"; exit 1; }

require_root() {
    [ "$EUID" -eq 0 ] || die "This action requires root. Run: sudo bash ./STARTUP.sh $*"
}

# Minimal YAML field reader — handles:  key: value  and  key: "value"
read_yaml() {
    local file="$1" key="$2"
    grep -E "^\s*${key}\s*:" "$file" \
        | head -1 \
        | sed 's/[^:]*:[[:space:]]*//' \
        | tr -d '"' | tr -d "'" | xargs
}

read_config() {
    [ -f "$STARTUP_CFG" ] || die "Missing config: $STARTUP_CFG"

    WIFI_SSID="$(read_yaml "$STARTUP_CFG" ssid)"
    WIFI_PASS="$(read_yaml "$STARTUP_CFG" password)"
    WIFI_IFACE="$(read_yaml "$STARTUP_CFG" interface)"
    WIFI_POWER_SAVE="$(read_yaml "$STARTUP_CFG" power_save)"
    WIFI_DISCONNECT_BUILTIN="$(read_yaml "$STARTUP_CFG" disconnect_builtin)"
    WIFI_STATIC_IP="$(read_yaml "$STARTUP_CFG" static_ip)"
    WIFI_GATEWAY="$(read_yaml "$STARTUP_CFG" gateway)"
    ROBOT_ID="$(read_yaml "$STARTUP_CFG" id)"
    ROBOT_MODE="$(read_yaml "$STARTUP_CFG" mode)"
    STATUS_PORT="$(read_yaml "$STARTUP_CFG" port)"
    STATUS_HOST="$(read_yaml "$STARTUP_CFG" host)"
    [ -n "$STATUS_PORT" ] || STATUS_PORT="50513"
    [ -n "$STATUS_HOST" ] || STATUS_HOST="255.255.255.255"

    [ -n "$WIFI_SSID" ] || die "wifi.ssid is empty in $STARTUP_CFG"
    [ -n "$WIFI_PASS" ] || die "wifi.password is empty in $STARTUP_CFG"
    [ -n "$ROBOT_MODE" ] || ROBOT_MODE="safe"

    # Normalise mode — strip leading dash if user added one
    ROBOT_MODE="${ROBOT_MODE#-}"

    case "$ROBOT_MODE" in
        safe|capped|unsafe) ;;
        *) die "Invalid mode '$ROBOT_MODE' in $STARTUP_CFG. Use: safe | capped | unsafe" ;;
    esac
}

# ---------------------------------------------------------------------------
# Boot status
# ---------------------------------------------------------------------------

send_boot_status() {
    local host="$1" port="$2"
    local ip
    ip="$(hostname -I | awk '{print $1}')"

    echo "  Sending boot status → $host:$port ..."

    BOOT_IP="$ip" BOOT_MODE="${ROBOT_MODE:-safe}" BOOT_ID="${ROBOT_ID:-?}" \
    BOOT_HOST="$host" BOOT_PORT="$port" \
    python3 -c "
import socket, json, time, os
msg = json.dumps({
    'status':    'online',
    'id':        os.environ['BOOT_ID'],
    'ip':        os.environ['BOOT_IP'],
    'mode':      os.environ['BOOT_MODE'],
    'timestamp': int(time.time()),
})
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.sendto(msg.encode(), (os.environ['BOOT_HOST'], int(os.environ['BOOT_PORT'])))
s.close()
" 2>/dev/null \
        && echo "  ✓ Boot status sent." \
        || echo "  Warning: could not send boot status."
}

# ---------------------------------------------------------------------------
# WiFi
# ---------------------------------------------------------------------------

disconnect_builtin_nic() {
    echo "  Disconnecting built-in WiFi (wlan0)..."
    nmcli dev disconnect wlan0 &>/dev/null \
        && echo "  ✓ wlan0 disconnected." \
        || echo "  Warning: wlan0 may already be disconnected."
}

disable_power_save() {
    local iface="$1"
    echo "  Disabling power save on $iface..."
    if command -v iw &>/dev/null; then
        iw dev "$iface" set power_save off \
            && echo "  ✓ Power save off." \
            || echo "  Warning: could not disable power save on $iface."
    else
        echo "  Warning: iw not found — cannot disable power save."
    fi
}

connect_wifi() {
    local ssid="$1" pass="$2" iface="$3" power_save="$4" disconnect_builtin="$5"

    echo "Connecting to WiFi: $ssid (interface: ${iface:-auto}) ..."

    if ! command -v nmcli &>/dev/null; then
        echo "  nmcli not found — skipping WiFi step (configure manually)."
        return 0
    fi

    # Disconnect built-in NIC so it doesn't take priority over the dongle.
    if [ "${disconnect_builtin}" = "true" ]; then
        disconnect_builtin_nic
    fi

    # Disable power save before connecting so the link stays stable.
    if [ "${power_save}" = "false" ] && [ -n "$iface" ]; then
        disable_power_save "$iface"
    fi

    # Build optional interface argument array.
    local iface_args=()
    [ -n "$iface" ] && iface_args=(ifname "$iface")

    # Apply static IP settings if configured.
    local ip_args=()
    if [ -n "$WIFI_STATIC_IP" ]; then
        echo "  Applying static IP: $WIFI_STATIC_IP (gateway: ${WIFI_GATEWAY:-none}) ..."
        ip_args=(ip4 "$WIFI_STATIC_IP")
        [ -n "$WIFI_GATEWAY" ] && ip_args+=(gw4 "$WIFI_GATEWAY")
    fi

    # If a connection profile already exists for this SSID, update its IP
    # settings then bring it up. Otherwise create a new profile.
    if nmcli -t -f NAME con show | grep -qxF "$ssid"; then
        if [ -n "$WIFI_STATIC_IP" ]; then
            nmcli con modify "$ssid" \
                ipv4.method manual \
                ipv4.addresses "$WIFI_STATIC_IP" \
                ipv4.gateway "${WIFI_GATEWAY:-}" &>/dev/null
        else
            nmcli con modify "$ssid" ipv4.method auto ipv4.addresses "" ipv4.gateway "" &>/dev/null
        fi
        nmcli con up "$ssid" "${iface_args[@]}" &>/dev/null || true
    else
        nmcli dev wifi connect "$ssid" password "$pass" "${iface_args[@]}" "${ip_args[@]}" &>/dev/null \
            || { echo "  Could not connect to $ssid on ${iface:-any interface}."; return 1; }
    fi

    # Wait up to 30 s for NetworkManager to report connected.
    local i=0
    while [ $i -lt 30 ]; do
        if nmcli -t -f STATE general 2>/dev/null | grep -qx "connected"; then
            echo "  ✓ WiFi connected."
            return 0
        fi
        sleep 1
        i=$((i + 1))
    done

    echo "  ✗ WiFi connection timed out after 30 s."
    return 1
}

# ---------------------------------------------------------------------------
# Service file
# ---------------------------------------------------------------------------

write_service_file() {
    cat > "$SERVICE_FILE" <<EOF
[Unit]
Description=RobotFramework Auto-Start
# Wait until NetworkManager is up before we try to connect to WiFi ourselves.
After=NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=simple
User=root
WorkingDirectory=${SCRIPT_DIR}
ExecStart=/usr/bin/bash ${SCRIPT_DIR}/STARTUP.sh --run-service
Restart=on-failure
RestartSec=10
# Give the binary time to shut down cleanly on Ctrl-C / SIGTERM.
TimeoutStopSec=15

[Install]
WantedBy=multi-user.target
EOF
}

# ---------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------

action_enable() {
    require_root "--enable"

    [ -f "$BINARY" ] \
        || die "$BINARY not found. Build the project first: bash ./SETUP.sh build"

    echo "Installing systemd service..."
    write_service_file
    systemctl daemon-reload
    systemctl enable "$SERVICE_NAME"
    echo ""
    echo "✓ Service installed and enabled."
    echo "  The robot will auto-start on next boot."
    echo "  To start right now:  sudo systemctl start $SERVICE_NAME"
    echo "  To stop:             sudo systemctl stop  $SERVICE_NAME"
    echo "  To disable autoboot: sudo bash ./STARTUP.sh --disable"
}

action_disable() {
    require_root "--disable"

    systemctl disable "$SERVICE_NAME" 2>/dev/null || true

    if [ -f "$SERVICE_FILE" ]; then
        rm "$SERVICE_FILE"
        systemctl daemon-reload
    fi

    echo "✓ Autostart disabled. The service file has been removed."
    echo "  Any currently running instance is unaffected."
}

action_status() {
    echo "=== Service status ==="
    systemctl status "$SERVICE_NAME" --no-pager -l 2>/dev/null \
        || echo "(service not installed)"
    echo ""
    echo "=== Recent logs ==="
    journalctl -u "$SERVICE_NAME" --no-pager -n 30 2>/dev/null \
        || echo "(no logs found)"
}

action_now() {
    # Connect to WiFi and launch the binary in the current terminal session.
    read_config

    [ -f "$BINARY" ] \
        || die "$BINARY not found. Build the project first: bash ./SETUP.sh build"

    if [ "$EUID" -ne 0 ]; then
        echo "pi3hat requires root. Re-launching with sudo..."
        exec sudo -E bash "${BASH_SOURCE[0]}" --now
    fi

    connect_wifi "$WIFI_SSID" "$WIFI_PASS" "$WIFI_IFACE" "$WIFI_POWER_SAVE" "$WIFI_DISCONNECT_BUILTIN"
    send_boot_status "$STATUS_HOST" "$STATUS_PORT"

    echo "Starting RobotFramework (-${ROBOT_MODE})..."
    echo "Press Ctrl+C to stop."
    echo ""
    exec "./$BINARY" "-${ROBOT_MODE}"
}

action_run_service() {
    # Called by systemd — not meant for direct use.
    read_config
    connect_wifi "$WIFI_SSID" "$WIFI_PASS" "$WIFI_IFACE" "$WIFI_POWER_SAVE" "$WIFI_DISCONNECT_BUILTIN"
    send_boot_status "$STATUS_HOST" "$STATUS_PORT"
    echo "Starting RobotFramework (-${ROBOT_MODE})..."
    exec "${SCRIPT_DIR}/${BINARY}" "-${ROBOT_MODE}"
}

print_usage() {
    cat <<'EOF'
Usage:
  sudo bash ./STARTUP.sh --enable    # install service + enable autostart on boot
  sudo bash ./STARTUP.sh --disable   # disable autostart
       bash ./STARTUP.sh --status    # show service status and recent logs
       bash ./STARTUP.sh --now       # connect + launch right now (no install)

Configuration:  config/Startup.yaml
  Change wifi.ssid, wifi.password, wifi.interface, and robot.mode there.
EOF
}

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

case "${1:-}" in
    --enable)       action_enable ;;
    --disable)      action_disable ;;
    --status)       action_status ;;
    --now)          action_now ;;
    --run-service)  action_run_service ;;   # internal — called by systemd
    *)              print_usage ;;
esac
