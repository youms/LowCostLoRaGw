#!/bin/bash

################################################################################
# Automatic Gateway Configuration Cycling Script
# Matches Arduino Enhanced_Lora_DS18B20_SX12XXX_Characterization.ino
# 
# This script cycles through the same 9 parameter combinations as the Arduino:
# - MIN: SF7, BW125, CR4/5 with payload sizes 20,50,80
# - MEAN: SF9, BW125, CR4/5 with payload sizes 20,50,80  
# - MAX: SF12, BW125, CR4/5 with payload sizes 20,50,80
#
# Each configuration runs for 20 minutes to match Arduino parameter changes
################################################################################

# Configuration
FREQ="865.2"           # Frequency in MHz
CYCLE_TIME=1200        # 20 minutes in seconds (20 * 60)
LOG_DIR="./characterization_logs"
BASE_LOG_NAME="network_test"

# Create log directory if it doesn't exist
mkdir -p "$LOG_DIR"

# Function to log with timestamp
log_message() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_DIR/gateway_cycle.log"
}

# Function to kill existing gateway processes - using official cmd.sh approach
kill_gateway() {
    log_message "Stopping existing gateway processes..."
    
    # Use the same process identification as cmd.sh 
    # Find PIDs of gateway-related processes
    gateway_pids=$(ps aux | grep -e start_gw -e start_upl -e start_lpf -e lora_gateway -e post_processing -e log_gw -e util_pkt_logger -e lora_pkt_fwd | grep -v grep | awk '{print $2}')
    
    if [ -n "$gateway_pids" ]; then
        log_message "Found gateway processes: $gateway_pids"
        
        # Kill gracefully first
        echo "$gateway_pids" | xargs -r sudo kill 2>/dev/null
        sleep 3
        
        # Check if any processes are still running and force kill if necessary
        remaining_pids=$(ps aux | grep -e start_gw -e start_upl -e start_lpf -e lora_gateway -e post_processing -e log_gw -e util_pkt_logger -e lora_pkt_fwd | grep -v grep | awk '{print $2}')
        
        if [ -n "$remaining_pids" ]; then
            log_message "Force killing remaining processes: $remaining_pids"
            echo "$remaining_pids" | xargs -r sudo kill -9 2>/dev/null
        fi
        
        log_message "Gateway processes stopped."
    else
        log_message "No gateway processes found running."
    fi
}

# Function to start gateway with specific parameters
start_gateway() {
    local sf=$1
    local config_name=$2
    local log_suffix=$3
    
    log_message "Starting gateway: $config_name (SF$sf, BW125, CR5)"
    
    # Create timestamped log file for this configuration
    local timestamp=$(date '+%Y%m%d_%H%M%S')
    local log_file="$LOG_DIR/${BASE_LOG_NAME}_${log_suffix}_${timestamp}.log"
    
    # Start the gateway with specified parameters
    # Use nohup and redirect to log file, run in background
    nohup sudo ./lora_gateway --bw 125 --sf "$sf" --cr 5 --freq "$FREQ" 2>&1 | \
    python ./post_processing_gw.py 2>&1 | \
    python ./log_gw.py 2>&1 | \
    tee "$log_file" &
    
    # Store the process ID for monitoring
    GATEWAY_PID=$!
    
    log_message "Gateway started with PID $GATEWAY_PID, logging to: $log_file"
}

# Function to wait for specified duration with progress
wait_with_progress() {
    local duration=$1
    local config_name=$2
    local start_time=$(date +%s)
    local end_time=$((start_time + duration))
    
    while [ $(date +%s) -lt $end_time ]; do
        local remaining=$((end_time - $(date +%s)))
        local minutes=$((remaining / 60))
        local seconds=$((remaining % 60))
        
        printf "\r[$(date '+%H:%M:%S')] Running $config_name - Time remaining: %02d:%02d" $minutes $seconds
        sleep 10
    done
    printf "\n"
}

# Function to check if gateway is still running
check_gateway_status() {
    if ! ps -p $GATEWAY_PID > /dev/null 2>&1; then
        log_message "WARNING: Gateway process $GATEWAY_PID has stopped unexpectedly!"
        return 1
    fi
    return 0
}

# Array of configurations matching Arduino code exactly
declare -a configs=(
    "7:MIN-SF7-BW125-T20:sf7_t20"
    "7:MIN-SF7-BW125-T50:sf7_t50" 
    "7:MIN-SF7-BW125-T80:sf7_t80"
    "9:MEAN-SF9-BW125-T20:sf9_t20"
    "9:MEAN-SF9-BW125-T50:sf9_t50"
    "9:MEAN-SF9-BW125-T80:sf9_t80"
    "12:MAX-SF12-BW125-T20:sf12_t20"
    "12:MAX-SF12-BW125-T50:sf12_t50"
    "12:MAX-SF12-BW125-T80:sf12_t80"
)

# Signal handlers for graceful shutdown
cleanup() {
    log_message "Received shutdown signal, cleaning up..."
    kill_gateway
    exit 0
}

trap cleanup SIGINT SIGTERM

# Main execution
main() {
    log_message "=========================================="
    log_message "Starting Automatic Gateway Cycle Script"
    log_message "Frequency: $FREQ MHz"
    log_message "Cycle time: $CYCLE_TIME seconds (20 minutes)"
    log_message "Total configurations: ${#configs[@]}"
    log_message "Total test duration: $((CYCLE_TIME * ${#configs[@]} / 60)) minutes"
    log_message "=========================================="
    
    # Check if required files exist
    if [ ! -f "./lora_gateway" ]; then
        log_message "ERROR: lora_gateway executable not found in current directory"
        exit 1
    fi
    
    if [ ! -f "./post_processing_gw.py" ]; then
        log_message "ERROR: post_processing_gw.py not found in current directory"
        exit 1
    fi
    
    if [ ! -f "./log_gw.py" ]; then
        log_message "ERROR: log_gw.py not found in current directory"
        exit 1
    fi
    
    # Initial cleanup
    kill_gateway
    
    local cycle=1
    local config_index=0
    
    # Main loop - can run multiple cycles if needed
    while true; do
        log_message "Starting cycle $cycle..."
        
        for config in "${configs[@]}"; do
            # Parse configuration string
            IFS=':' read -r sf config_name log_suffix <<< "$config"
            
            config_index=$((config_index + 1))
            
            log_message "Configuration $config_index/${#configs[@]}: $config_name"
            
            # Kill any existing gateway processes
            kill_gateway
            
            # Wait a moment before starting new configuration
            sleep 3
            
            # Start gateway with new configuration
            start_gateway "$sf" "$config_name" "$log_suffix"
            
            # Wait for the cycle time with progress indication
            wait_with_progress $CYCLE_TIME "$config_name"
            
            # Optional: Check gateway status during run
            # check_gateway_status
        done
        
        log_message "Cycle $cycle completed. Starting next cycle..."
        cycle=$((cycle + 1))
        config_index=0
        
        # Optional: Add a break here if you only want to run one complete cycle
        # break
    done
}

# Check if script is being run as source or executed
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    # Script is being executed directly
    main "$@"
else
    # Script is being sourced
    log_message "Script loaded. Use 'main' function to start gateway cycling."
fi