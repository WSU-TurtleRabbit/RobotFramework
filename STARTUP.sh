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
#        bash ./STARTUP.sh --logs      # tail the live log file
#        bash ./STARTUP.sh --now       # connect to WiFi and launch right now (no install)
#        bash ./STARTUP.sh             # show this help
#
# Internal flags (called by systemd, not for direct use):
#   --run-service   called by ExecStart on boot — connects WiFi and launches binary
#   --restore       called by ExecStopPost on shutdown — restores the other WiFi interface
#
# Configuration lives in config/Startup.yaml.
# Logs are written to logs/service.log.
#
# System changes made at runtime and how to revert them:
#
#   WiFi interface (other than the active one) is brought down:
#     sudo ip link set wlan0 up
#     sudo nmcli dev set wlan0 autoconnect yes
#
#   Power save is disabled on the active interface:
#     sudo iw dev wlan1 set power_save on
#
#   NM connection profile is re-bound to the active interface:
#     sudo nmcli con modify "YourSSID" connection.interface-name ""
#
#   systemd service installed to /etc/systemd/system/robotframework.service:
#     sudo bash ./STARTUP.sh --disable

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

SERVICE_NAME="robotframework"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
BINARY="build/RobotFramework"
STARTUP_CFG="config/Startup.yaml"
LOG_FILE="${SCRIPT_DIR}/logs/service.log"

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"
}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

die() { log "ERROR: $*"; exit 1; }

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
    WIFI_DISCONNECT_OTHER="$(read_yaml "$STARTUP_CFG" disconnect_other)"
    WIFI_STATIC_IP="$(read_yaml "$STARTUP_CFG" static_ip)"
    WIFI_GATEWAY="$(read_yaml "$STARTUP_CFG" gateway)"
    ROBOT_ID="$(read_yaml "$STARTUP_CFG" id)"
    ROBOT_MODE="$(read_yaml "$STARTUP_CFG" mode)"
    REQUIRE_NETWORK="$(read_yaml "$STARTUP_CFG" require_network)"
    [ -n "$REQUIRE_NETWORK" ] || REQUIRE_NETWORK="true"
    ROBOT_USE_TMUX="$(read_yaml "$STARTUP_CFG" use_tmux)"
    [ -n "$ROBOT_USE_TMUX" ] || ROBOT_USE_TMUX="true"
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

    log "Config loaded — robot: ${ROBOT_ID:-?}, mode: $ROBOT_MODE, ssid: $WIFI_SSID, iface: ${WIFI_IFACE:-auto}"
}

# ---------------------------------------------------------------------------
# Boot status
# ---------------------------------------------------------------------------

send_boot_status() {
    local host="$1" port="$2"
    local ip
    ip="$(ip -4 addr show "${WIFI_IFACE:-wlan1}" 2>/dev/null | grep 'inet ' | awk '{print $2}' | cut -d/ -f1)"
    [ -n "$ip" ] || ip="$(hostname -I | awk '{print $1}')"

    log "Sending boot status → $host:$port (id=${ROBOT_ID:-?} ip=$ip mode=${ROBOT_MODE:-safe})"

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
        && log "✓ Boot status sent." \
        || log "Warning: could not send boot status."
}

# ---------------------------------------------------------------------------
# WiFi
# ---------------------------------------------------------------------------

wait_for_interface() {
    local iface="$1"
    local i=0
    log "Waiting for NetworkManager to recognise $iface..."
    while [ $i -lt 30 ]; do
        if nmcli -t -f DEVICE,STATE dev 2>/dev/null | grep -q "^${iface}:"; then
            log "✓ Interface $iface is ready."
            return 0
        fi
        sleep 1
        i=$((i + 1))
    done
    log "✗ Interface $iface did not appear in NetworkManager after 30 s."
    return 1
}

disconnect_other_nic() {
    local active="$1"
    local other

    case "$active" in
        wlan0) other="wlan1" ;;
        wlan1) other="wlan0" ;;
        *)
            log "Cannot determine other interface for $active — skipping disconnect."
            return 0
            ;;
    esac

    log "Disabling auto-connect and disconnecting $other..."
    nmcli dev set "$other" autoconnect no &>/dev/null || true
    nmcli dev disconnect "$other" &>/dev/null || true
    ip link set "$other" down &>/dev/null \
        && log "✓ $other down." \
        || log "Warning: could not bring $other down."
}

disable_power_save() {
    local iface="$1"
    log "Disabling power save on $iface..."
    if command -v iw &>/dev/null; then
        iw dev "$iface" set power_save off \
            && log "✓ Power save off." \
            || log "Warning: could not disable power save on $iface."
    else
        log "Warning: iw not found — cannot disable power save."
    fi
}

connect_wifi() {
    local ssid="$1" pass="$2" iface="$3" power_save="$4" disconnect_other="$5"

    log "Connecting to WiFi: $ssid (interface: ${iface:-auto}) ..."

    # Wait for the interface to exist before touching nmcli.
    # The Realtek dongle can take several seconds to initialise after boot.
    if [ -n "$iface" ]; then
        wait_for_interface "$iface" || return 1
    fi

    if ! command -v nmcli &>/dev/null; then
        log "nmcli not found — skipping WiFi step (configure manually)."
        return 0
    fi

    # Disconnect the other interface so it doesn't compete.
    if [ "${disconnect_other}" = "true" ] && [ -n "$iface" ]; then
        disconnect_other_nic "$iface"
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
        log "Applying static IP: $WIFI_STATIC_IP (gateway: ${WIFI_GATEWAY:-none})"
        ip_args=(ip4 "$WIFI_STATIC_IP")
        [ -n "$WIFI_GATEWAY" ] && ip_args+=(gw4 "$WIFI_GATEWAY")
    fi

    # Scan for networks before connecting so NM has fresh results.
    log "Scanning for networks on $iface..."
    nmcli dev wifi rescan ifname "$iface" &>/dev/null || true
    sleep 3

    # If a connection profile already exists for this SSID, update its IP
    # settings then bring it up. Otherwise create a new profile.
    if nmcli -t -f NAME con show | grep -qxF "$ssid"; then
        if [ -n "$WIFI_STATIC_IP" ]; then
            nmcli con modify "$ssid" \
                connection.interface-name "$iface" \
                ipv4.method manual \
                ipv4.addresses "$WIFI_STATIC_IP" \
                ipv4.gateway "${WIFI_GATEWAY:-}" &>/dev/null
        else
            nmcli con modify "$ssid" \
                connection.interface-name "$iface" \
                ipv4.method auto ipv4.addresses "" ipv4.gateway "" &>/dev/null
        fi
        nmcli con up "$ssid" "${iface_args[@]}" \
            || { log "Could not bring up $ssid."; return 1; }
    else
        nmcli dev wifi connect "$ssid" password "$pass" "${iface_args[@]}" "${ip_args[@]}" \
            || { log "Could not connect to $ssid on ${iface:-any interface}."; return 1; }
    fi

    # Wait up to 30 s for NetworkManager to report connected.
    local i=0
    while [ $i -lt 30 ]; do
        if nmcli -t -f STATE general 2>/dev/null | grep -qx "connected"; then
            log "✓ WiFi connected."
            return 0
        fi
        sleep 1
        i=$((i + 1))
    done

    log "✗ WiFi connection timed out after 30 s."
    return 1
}

# ---------------------------------------------------------------------------
# Service file
# ---------------------------------------------------------------------------

write_service_file() {
    mkdir -p "${SCRIPT_DIR}/logs"
    cat > "$SERVICE_FILE" <<EOF
[Unit]
Description=RobotFramework Auto-Start
# Wait until NetworkManager is up before we try to connect to WiFi ourselves.
After=NetworkManager.service
Wants=NetworkManager.service
# Delay startup to give the kernel time to initialise USB WiFi dongles.
ExecStartPre=/bin/sleep 5

[Service]
Type=simple
User=root
WorkingDirectory=${SCRIPT_DIR}
ExecStart=/usr/bin/bash ${SCRIPT_DIR}/STARTUP.sh --run-service
Restart=on-failure
RestartSec=10
# Give the binary time to shut down cleanly on Ctrl-C / SIGTERM.
TimeoutStopSec=15
# Re-enable the other interface when the service stops so SSH access is restored.
ExecStopPost=/usr/bin/bash ${SCRIPT_DIR}/STARTUP.sh --restore
# Log everything (startup script + RobotFramework binary) to a single file.
StandardOutput=append:${SCRIPT_DIR}/logs/service.log
StandardError=append:${SCRIPT_DIR}/logs/service.log

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

    mkdir -p "${SCRIPT_DIR}/logs"

    echo "Installing systemd service..."
    write_service_file
    systemctl daemon-reload
    systemctl enable "$SERVICE_NAME"
    echo ""
    echo "✓ Service installed and enabled."
    echo "  The robot will auto-start on next boot."
    echo "  To start right now:  sudo systemctl start $SERVICE_NAME"
    echo "  To stop:             sudo systemctl stop  $SERVICE_NAME"
    echo "  To watch the log:    bash ./STARTUP.sh --logs"
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
    echo "=== Last 50 lines of log ==="
    if [ -f "$LOG_FILE" ]; then
        tail -n 50 "$LOG_FILE"
    else
        echo "(no log file yet — has the service run?)"
    fi
}

action_logs() {
    if [ ! -f "$LOG_FILE" ]; then
        echo "No log file found at $LOG_FILE"
        echo "Has the service run yet? Try: sudo systemctl start $SERVICE_NAME"
        exit 1
    fi
    echo "Tailing $LOG_FILE  (Ctrl+C to stop)"
    echo "---"
    tail -f "$LOG_FILE"
}

action_now() {
    # Connect to WiFi and launch the binary in the current terminal session.
    read_config

    [ -f "$BINARY" ] \
        || die "$BINARY not found. Build the project first: bash ./SETUP.sh build"

    if [ "$EUID" -ne 0 ]; then
        log "pi3hat requires root. Re-launching with sudo..."
        exec sudo -E bash "${BASH_SOURCE[0]}" --now
    fi

    if ! connect_wifi "$WIFI_SSID" "$WIFI_PASS" "$WIFI_IFACE" "$WIFI_POWER_SAVE" "$WIFI_DISCONNECT_OTHER"; then
        if [ "$REQUIRE_NETWORK" = "true" ]; then
            die "Network connection failed — RobotFramework will not start (require_network: true)."
        fi
        log "Warning: network connection failed, continuing anyway (require_network: false)."
    fi
    send_boot_status "$STATUS_HOST" "$STATUS_PORT"

    log "Starting RobotFramework (-${ROBOT_MODE})..."
    cd "${SCRIPT_DIR}/build" || die "build/ directory not found"

    if [ "$ROBOT_USE_TMUX" = "true" ] && command -v tmux &>/dev/null; then
        # Kill any stale session from a previous run.
        tmux kill-session -t robotframework 2>/dev/null || true
        log "Launching in tmux session 'robotframework'."
        log "Detach with Ctrl+B then D. Reattach with: tmux attach -t robotframework"
        tmux new-session -d -s robotframework "./RobotFramework -${ROBOT_MODE}"
        tmux attach -t robotframework
    else
        [ "$ROBOT_USE_TMUX" = "true" ] && log "tmux not found — running directly."
        log "Press Ctrl+C to stop."
        exec "./RobotFramework" "-${ROBOT_MODE}"
    fi
}

action_restore() {
    # Called by ExecStopPost — brings the other interface back up after service stops.
    read_config
    local other
    case "$WIFI_IFACE" in
        wlan0) other="wlan1" ;;
        wlan1) other="wlan0" ;;
        *) return 0 ;;
    esac
    log "Restoring $other after service stop..."
    ip link set "$other" up &>/dev/null || true
    nmcli dev set "$other" autoconnect yes &>/dev/null || true
    log "✓ $other restored."
}

action_run_service() {
    # Called by systemd — not meant for direct use.
    # stdout/stderr are redirected to logs/service.log by the service file.
    log "========================================"
    log "RobotFramework service starting"
    log "========================================"
    read_config
    if ! connect_wifi "$WIFI_SSID" "$WIFI_PASS" "$WIFI_IFACE" "$WIFI_POWER_SAVE" "$WIFI_DISCONNECT_OTHER"; then
        if [ "$REQUIRE_NETWORK" = "true" ]; then
            die "Network connection failed — RobotFramework will not start (require_network: true)."
        fi
        log "Warning: network connection failed, continuing anyway (require_network: false)."
    fi
    send_boot_status "$STATUS_HOST" "$STATUS_PORT"
    log "Handing off to RobotFramework (-${ROBOT_MODE})..."
    cd "${SCRIPT_DIR}/build" || die "build/ directory not found"
    exec "./RobotFramework" "-${ROBOT_MODE}"
}

print_usage() {
    cat <<'EOF'
Usage:
  sudo bash ./STARTUP.sh --enable    # install service + enable autostart on boot
  sudo bash ./STARTUP.sh --disable   # disable autostart
       bash ./STARTUP.sh --status    # show service status and recent logs
       bash ./STARTUP.sh --logs      # tail the live log file
       bash ./STARTUP.sh --now       # connect + launch right now (no install)

Configuration:  config/Startup.yaml
Log file:       logs/service.log
EOF
}

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

case "${1:-}" in
    --enable)       action_enable ;;
    --disable)      action_disable ;;
    --status)       action_status ;;
    --logs)         action_logs ;;
    --now)          action_now ;;
    --restore)      action_restore ;;        # internal — called by ExecStopPost
    --run-service)  action_run_service ;;   # internal — called by systemd
    *)              print_usage ;;
esac
