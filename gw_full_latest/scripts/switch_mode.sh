#!/bin/bash

#-------------------------------------------------------------------------------
# switch_mode.sh - Switch LoRa Gateway between LBS and TTN/UDP modes
#-------------------------------------------------------------------------------

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Assuming the root is one level up from scripts/
GW_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

cd "$GW_ROOT" || { echo "Error: Could not enter gateway root directory $GW_ROOT"; exit 1; }

# Configuration file names
LBS_CLOUDS="clouds_lbs.json"
LBS_GW="gateway_conf_lbs.json"
TTN_CLOUDS="clouds_ttn.json"
TTN_GW="gateway_conf_ttn.json"
LORA_CLOUDS="clouds_lora.json"
LORA_GW="gateway_conf_lora.json"

ACTIVE_CLOUDS="clouds.json"
ACTIVE_GW="gateway_conf.json"

usage() {
    echo "Usage: $0 {lbs|ttn|udp|lora}"
    echo "  lbs: Switch to LoRa Basics Station mode"
    echo "  ttn/udp: Switch to Legacy TTN UDP mode"
    echo "  lora: Switch to simple LoRa mode (Local MongoDB only)"
    exit 1
}

if [ "$#" -ne 1 ]; then
    usage
fi

MODE=$1

# 1. Detect current mode by checking which backup is missing
# (Since the active one was moved out of its backup slot)
CURRENT_MODE="unknown"
if [ ! -f "$LBS_CLOUDS" ]; then
    CURRENT_MODE="lbs"
elif [ ! -f "$TTN_CLOUDS" ]; then
    CURRENT_MODE="ttn"
elif [ ! -f "$LORA_CLOUDS" ]; then
    CURRENT_MODE="lora"
fi

# 2. Handle switching
case "$MODE" in
    lbs)
        TARGET_CLOUDS="$LBS_CLOUDS"
        TARGET_GW="$LBS_GW"
        SERVICE_ACTION="start_lbs"
        ;;
    ttn|udp)
        TARGET_CLOUDS="$TTN_CLOUDS"
        TARGET_GW="$TTN_GW"
        SERVICE_ACTION="stop_lbs"
        ;;
    lora)
        TARGET_CLOUDS="$LORA_CLOUDS"
        TARGET_GW="$LORA_GW"
        SERVICE_ACTION="stop_lbs"
        ;;
    *)
        usage
        ;;
esac

echo ">>> Switching from $CURRENT_MODE to $MODE..."

if [ "$MODE" == "$CURRENT_MODE" ]; then
    echo "Error: Already in $MODE mode."
    exit 1
fi

if [ ! -f "$TARGET_CLOUDS" ] || [ ! -f "$TARGET_GW" ]; then
    echo "Error: Target configuration files ($TARGET_CLOUDS, $TARGET_GW) not found."
    exit 1
fi

# Save current active to its backup name
case "$CURRENT_MODE" in
    lbs)
        mv "$ACTIVE_CLOUDS" "$LBS_CLOUDS"
        mv "$ACTIVE_GW" "$LBS_GW"
        ;;
    ttn)
        mv "$ACTIVE_CLOUDS" "$TTN_CLOUDS"
        mv "$ACTIVE_GW" "$TTN_GW"
        ;;
    lora)
        mv "$ACTIVE_CLOUDS" "$LORA_CLOUDS"
        mv "$ACTIVE_GW" "$LORA_GW"
        ;;
    *)
        echo "Warning: Current mode unknown. Creating a timestamped backup of current config..."
        TS=$(date +%Y%m%d_%H%M%S)
        cp "$ACTIVE_CLOUDS" "${ACTIVE_CLOUDS}.bak.${TS}"
        cp "$ACTIVE_GW" "${ACTIVE_GW}.bak.${TS}"
        ;;
esac

# Activate target config
echo "Activating $MODE config..."
mv "$TARGET_CLOUDS" "$ACTIVE_CLOUDS"
mv "$TARGET_GW" "$ACTIVE_GW"

# Manage services
echo "Restarting services..."
if [ "$SERVICE_ACTION" == "start_lbs" ]; then
    sudo systemctl restart lora-gateway.service
    sudo systemctl start lora-lbs.service
else
    sudo systemctl stop lora-lbs.service
    sudo systemctl restart lora-gateway.service
fi

echo "Successfully switched to $MODE mode."
