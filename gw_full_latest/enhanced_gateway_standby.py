#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Enhanced Gateway Controller with Standby Mode - Python 2 Version
Advanced LoRa network characterization controller with:
- Standby-based configuration switching for reliable downlinks
- Extended configuration set with BW500-T50 variants
- Custom predefined sequence cycling
- Multi-attempt downlink strategy for reliable configuration switching
- Database markers for clean data analysis boundaries
- Efficient gateway management with automatic standby return
- Phase-based cycle management with clean monitoring windows
- Auto-detection of gateway downlink processing intervals
"""

import json
import time
import os
import sys
import subprocess
import threading
import signal
import re
from datetime import datetime, timedelta

# Extended configuration mappings - must match the node's NetworkParams.h
CONFIGURATIONS = [
    {"index": 0, "name": "MIN-SF7-BW125-T20", "sf": 7, "bw": 125, "cr": 5, "size": 20},
    {"index": 1, "name": "MIN-SF7-BW500-T20", "sf": 7, "bw": 500, "cr": 5, "size": 20},
    {"index": 2, "name": "MIN-SF7-BW125-T50", "sf": 7, "bw": 125, "cr": 5, "size": 50},
    {"index": 3, "name": "MIN-SF7-BW125-T80", "sf": 7, "bw": 125, "cr": 5, "size": 80},
    {"index": 4, "name": "MEAN-SF9-BW125-T20", "sf": 9, "bw": 125, "cr": 5, "size": 20},
    {"index": 5, "name": "MEAN-SF9-BW500-T20", "sf": 9, "bw": 500, "cr": 5, "size": 20},
    {"index": 6, "name": "MEAN-SF9-BW125-T50", "sf": 9, "bw": 125, "cr": 5, "size": 50},
    {"index": 7, "name": "MEAN-SF9-BW125-T80", "sf": 9, "bw": 125, "cr": 5, "size": 80},
    {"index": 8, "name": "MAX-SF12-BW125-T20", "sf": 12, "bw": 125, "cr": 5, "size": 20},
    {"index": 9, "name": "MAX-SF12-BW500-T20", "sf": 12, "bw": 500, "cr": 5, "size": 20},
    {"index": 10, "name": "MAX-SF12-BW125-T50", "sf": 12, "bw": 125, "cr": 5, "size": 50},
    {"index": 11, "name": "MAX-SF12-BW125-T80", "sf": 12, "bw": 125, "cr": 5, "size": 80},
    # New BW500-T50 variants
    {"index": 12, "name": "MIN-SF7-BW500-T50", "sf": 7, "bw": 500, "cr": 5, "size": 50},
    {"index": 13, "name": "MEAN-SF9-BW500-T50", "sf": 9, "bw": 500, "cr": 5, "size": 50},
    {"index": 14, "name": "MAX-SF12-BW500-T50", "sf": 12, "bw": 500, "cr": 5, "size": 50},
]

# Predefined sequence for characterization
CHARACTERIZATION_SEQUENCE = [2, 12, 6, 13, 10, 14]  # SF7-BW125-T50, SF7-BW500-T50, SF9-BW125-T50, SF9-BW500-T50, SF12-BW125-T50, SF12-BW500-T50

# Standby configuration - most reliable for downlinks
STANDBY_CONFIG_INDEX = 8  # MAX-SF12-BW125-T20

class GatewayController:
    def __init__(self, gateway_path="/home/pi/lora_gateway", cycle_minutes=5, start_sequence_index=0):
        self.gateway_path = gateway_path
        self.downlink_dir = os.path.join(gateway_path, "downlink")
        self.downlink_file = os.path.join(self.downlink_dir, "downlink-post.txt")

        # Clean up any leftover downlink files
        if os.path.exists(self.downlink_dir):
            for fname in os.listdir(self.downlink_dir):
                if fname.startswith("downlink-post"):
                    try:
                        os.remove(os.path.join(self.downlink_dir, fname))
                    except Exception as e:
                        print("Failed to delete {}: {}".format(fname, e))
        else:
            os.makedirs(self.downlink_dir)
        
        self.log_dir = os.path.join(gateway_path, "characterization_logs")
        
        # Timing configuration - preserved from original
        self.monitoring_time = cycle_minutes * 60  # USER GETS EXACTLY THIS MUCH CLEAN TIME
        self.downlink_attempts = 1  # Number of downlink attempts
        self.commands_per_attempt = 2 # 2 commands in one file
        self.downlink_multiplier = 3  # Multiplier for downlink attempts (default 1)
        self.problematic_index = -1  # Index that needs special downlink handling, -1 disables the logic
        
        # Auto-detect the gateway's downlink processing interval
        self.gateway_downlink_check = self.get_gateway_downlink_interval()
        
        # Downlink timing: gateway_check_interval + safety margin
        self.downlink_interval = self.gateway_downlink_check + 10  # 30s + 10s = 40s
        
        # Calculate buffer time using original formula
        self.node_transmission_interval = 10  # Seconds between node transmissions
        self.node_downlink_delay = 30  # Seconds node waits after receiving each downlink
        self.buffer_time = 60
        """(
            self.gateway_downlink_check +  # 30s - time to find file
            (self.commands_per_attempt * self.node_transmission_interval) +  # 2×15s - transmission time
            (self.commands_per_attempt * self.node_downlink_delay) +  # 2×30s - node processing delay
            30  # Safety margin
        )"""  # Total: 30 + 30 + 60 + 30 = 150 seconds
        
        # Calculate total cycle time (monitoring + overhead)
        self.total_cycle_time = self.monitoring_time + self.buffer_time
        self.requested_cycle_minutes = cycle_minutes
        
        # Sequence management
        self.current_sequence_index = start_sequence_index
        if not (0 <= start_sequence_index < len(CHARACTERIZATION_SEQUENCE)):
            raise ValueError("start_sequence_index must be between 0 and {} (got {})".format(len(CHARACTERIZATION_SEQUENCE)-1, start_sequence_index))
            
        # State tracking
        self.detected_node_addr = None
        self.gateway_process = None
        self.running = True
        self.initial_sync_complete = False
        self.packets_received = 0
        self.is_in_standby_mode = False
        
        # Add radio parameter tracking
        self.current_sf = None
        self.current_bw = None  
        self.current_cr = None
        
        # Setup logging
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        self.setup_logging()
        
        # Log timing configuration
        self.log_message("=== ENHANCED GATEWAY CONTROLLER WITH STANDBY MODE ===")
        self.log_message("USER REQUESTED: {} minutes of clean monitoring per configuration".format(cycle_minutes))
        self.log_message("CLEAN MONITORING TIME: {} seconds ({} minutes)".format(self.monitoring_time, self.monitoring_time / 60))
        self.log_message("CHARACTERIZATION SEQUENCE: {}".format([CONFIGURATIONS[i]['name'] for i in CHARACTERIZATION_SEQUENCE]))
        self.log_message("STANDBY CONFIG: {} (Index {})".format(CONFIGURATIONS[STANDBY_CONFIG_INDEX]['name'], STANDBY_CONFIG_INDEX))
        self.log_message("STARTING AT SEQUENCE INDEX: {} (Config: {})".format(start_sequence_index, CONFIGURATIONS[CHARACTERIZATION_SEQUENCE[start_sequence_index]]['name']))
        
        self.log_message("DOWNLINK CONFIGURATION:")
        self.log_message("BUFFER TIME CALCULATION:")
        self.log_message("  - Gateway check time: {} seconds".format(self.gateway_downlink_check))
        self.log_message("  - Command transmission: {} commands × {}s = {} seconds".format(
            self.commands_per_attempt, self.node_transmission_interval, 
            self.commands_per_attempt * self.node_transmission_interval))
        self.log_message("  - Node downlink processing: {} commands × {}s = {} seconds".format(
            self.commands_per_attempt, self.node_downlink_delay,
            self.commands_per_attempt * self.node_downlink_delay))
        self.log_message("  - Safety margin: 30 seconds")
        self.log_message("  - STANDARD BUFFER TIME: {} seconds ({:.1f} minutes)".format(
            self.buffer_time, self.buffer_time / 60.0))
        
        # Signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGHUP, self.signal_handler)

    def get_gateway_downlink_interval(self):
        """Read the downlink check interval from gateway_conf.json"""
        try:
            gateway_conf_file = os.path.join(self.gateway_path, "gateway_conf.json")
            with open(gateway_conf_file, 'r') as f:
                gateway_conf = json.load(f)
            
            # Get downlink interval from gateway_conf.json
            downlink_interval = gateway_conf["gateway_conf"].get("downlink", 30)
            print("Detected gateway downlink check interval: {} seconds".format(downlink_interval))
            return downlink_interval
            
        except Exception as e:
            print("Could not read gateway downlink interval: {} - using default 30 seconds".format(e))
            return 30  # Default fallback

    def setup_logging(self):
        """Setup logging"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = os.path.join(self.log_dir, "controller_{}.log".format(timestamp))

    def log_message(self, message, level="INFO"):
        """Log message with timestamp"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = "[{}] [{}] {}".format(timestamp, level, message)
        print(log_entry)
        
        with open(self.log_file, 'a') as f:
            f.write(log_entry + "\n")

    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.log_message("Received signal {}, shutting down...".format(signum))
        self.running = False
        # Clean downlink folder before shutdown
        self.cleanup_downlink_folder()
        self.kill_gateway_processes()
        sys.exit(0)

    def kill_gateway_processes(self):
        """Kill gateway processes using official method"""
        self.log_message("Stopping gateway processes...")
        
        cmd = "ps aux | grep -e start_gw -e lora_gateway -e post_processing -e log_gw | grep -v grep | awk '{print $2}'"
        
        try:
            result = subprocess.check_output(cmd, shell=True)
            gateway_pids = result.strip()
            
            if gateway_pids:
                self.log_message("Found gateway processes: {}".format(gateway_pids))
                subprocess.call("echo '{}' | xargs -r sudo kill".format(gateway_pids), shell=True)
                time.sleep(3)
                
                # Force kill if needed
                try:
                    result = subprocess.check_output(cmd, shell=True)
                    remaining_pids = result.strip()
                    if remaining_pids:
                        subprocess.call("echo '{}' | xargs -r sudo kill -9".format(remaining_pids), shell=True)
                except subprocess.CalledProcessError:
                    pass
                    
                self.log_message("Gateway processes stopped")
                
        except subprocess.CalledProcessError:
            self.log_message("No gateway processes found")
        except Exception as e:
            self.log_message("Error killing gateway processes: {}".format(e), "ERROR")

    def cleanup_downlink_folder(self):
        """Clean up downlink folder - same logic as original"""
        if os.path.exists(self.downlink_dir):
            for fname in os.listdir(self.downlink_dir):
                if fname.startswith("downlink"):
                    try:
                        os.remove(os.path.join(self.downlink_dir, fname))
                        self.log_message("Cleaned downlink file: {}".format(fname))
                    except Exception as e:
                        self.log_message("Failed to delete {}: {}".format(fname, e), "ERROR")
        else:
            self.log_message("Downlink directory does not exist")

    def get_buffer_time(self, config_index):
        """Calculate buffer time based on configuration index"""
        if config_index == self.problematic_index:
            # Double commands for problematic transitions
            commands = self.commands_per_attempt * self.downlink_multiplier
            buffer_time = (
                self.gateway_downlink_check +
                (commands * self.node_transmission_interval) +
                (commands * self.node_downlink_delay) +
                30
            )
            return buffer_time
        else:
            return self.buffer_time  # Standard buffer time
    
    def get_gateway_id(self):
        """Get gateway ID from gateway_conf.json (same method as post_processing_gw.py)"""
        try:
            gateway_conf_file = os.path.join(self.gateway_path, "gateway_conf.json")
            with open(gateway_conf_file, 'r') as f:
                gateway_conf = json.load(f)
            return gateway_conf["gateway_conf"]["gateway_ID"]
        except Exception as e:
            self.log_message("Error reading gateway ID: {}".format(e), "WARNING")
            return "0000000000000000"  # Fallback

    def create_database_marker(self, config_name, marker_type):
        """Create a database marker using CloudMongoDB.py subprocess"""
        try:
            from datetime import datetime
            now = datetime.now()
            
            # Get gateway ID from gateway_conf.json
            gateway_id = self.get_gateway_id()
            
            marker_data = "{}-{}".format(config_name, marker_type)
            
            self.log_message("=== DATABASE MARKER: {} ===".format(marker_data))
            
            # Format for CloudMongoDB.py
            ldata = marker_data
            pdata = "255,255,0,0,{},0,0,0".format(len(marker_data))  # Special type 255 for markers with ToA
            rdata = "125,5,12"  # Default radio data for markers  
            tdata = now.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "+01:00"  # ISO format with timezone
            gwid = gateway_id
            
            # Call CloudMongoDB.py exactly like original
            mongodb_cmd = 'python CloudMongoDB.py "{}" "{}" "{}" "{}" "{}"'.format(
                ldata, pdata, rdata, tdata, gwid
            )
            
            self.log_message("Calling: {}".format(mongodb_cmd))
            
            # Change to gateway directory and execute
            old_cwd = os.getcwd()
            os.chdir(self.gateway_path)
            
            result = subprocess.call(mongodb_cmd, shell=True)
            
            os.chdir(old_cwd)
            
            if result == 0:
                self.log_message("Database marker inserted successfully via CloudMongoDB.py")
                return True
            else:
                self.log_message("Database marker insertion failed with code: {}".format(result), "ERROR")
                return False
                
        except Exception as e:
            self.log_message("Error creating database marker: {}".format(e), "ERROR")
            return False

    def start_gateway_with_config(self, config, is_standby=False):
        """Start gateway using the complete pipeline with optimized monitoring"""
        mode_desc = "STANDBY" if is_standby else "MONITORING"
        self.log_message("Configuring gateway for {}: {}".format(mode_desc, config['name']))

        # Check if radio parameters changed
        radio_params_changed = (
            self.current_sf != config['sf'] or
            self.current_bw != config['bw'] or  
            self.current_cr != config['cr']
        )
        
        if radio_params_changed:
            self.log_message("Radio parameters changed (SF:{}->{}, BW:{}->{}, CR:{}->{}), restarting processes".format(
                self.current_sf, config['sf'], 
                self.current_bw, config['bw'],
                self.current_cr, config['cr']
            ))
            
            # CRITICAL: Clean downlink folder BEFORE killing processes
            self.cleanup_downlink_folder()
            
            # Kill existing processes and start new ones
            self.kill_gateway_processes()
            time.sleep(3)
            
            # Update current radio parameters
            self.current_sf = config['sf']
            self.current_bw = config['bw']
            self.current_cr = config['cr']
            
            # Start new gateway process with new radio parameters
            try:
                os.chdir(self.gateway_path)
                
                # Use the COMPLETE pipeline - essential for proper operation
                gateway_cmd = "sudo ./lora_gateway --bw {} --sf {} --cr {} --freq 868.0 2>&1 | python ./post_processing_gw.py 2>&1 | python ./log_gw.py 2>&1".format(
                    config['bw'], config['sf'], config['cr']
                )
                
                self.log_message("Starting complete gateway pipeline: {}".format(gateway_cmd))
                
                self.gateway_process = subprocess.Popen(
                    gateway_cmd,
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    universal_newlines=True
                )
                
                self.log_message("Complete gateway pipeline started (PID: {})".format(self.gateway_process.pid))
                
                # Start monitoring thread for the complete pipeline output
                self.monitor_thread = threading.Thread(target=self.monitor_gateway_output)
                self.monitor_thread.daemon = True
                self.monitor_thread.start()
                
                # Update standby mode state
                self.is_in_standby_mode = is_standby
                
                return True
                
            except Exception as e:
                self.log_message("Failed to start gateway: {}".format(e), "ERROR")
                return False
        
        else:
            self.log_message("EFFICIENCY: Payload size changed ({}), keeping existing processes running - saved gateway restart!".format(config['size']))
            self.log_message("Gateway continues with SF:{}, BW:{}, CR:{} - no downtime".format(
                self.current_sf, self.current_bw, self.current_cr
            ))
            # Update standby mode state even if no restart needed
            self.is_in_standby_mode = is_standby
            return True

    def switch_to_standby(self):
        """Switch gateway to standby mode for reliable downlinks"""
        standby_config = CONFIGURATIONS[STANDBY_CONFIG_INDEX]
        self.log_message("=== SWITCHING TO STANDBY MODE ===")
        self.log_message("Standby config: {}".format(standby_config['name']))
        
        if not self.start_gateway_with_config(standby_config, is_standby=True):
            self.log_message("Failed to switch to standby mode", "ERROR")
            return False
        
        # Give gateway time to stabilize in standby mode
        time.sleep(5)
        self.log_message("Gateway now in standby mode - ready for reliable downlinks")
        return True

    def switch_to_monitoring(self, target_config):
        """Switch gateway to monitoring mode for data collection"""
        self.log_message("=== SWITCHING TO MONITORING MODE ===")
        self.log_message("Target config: {}".format(target_config['name']))
        
        if not self.start_gateway_with_config(target_config, is_standby=False):
            self.log_message("Failed to switch to monitoring mode", "ERROR")
            return False
        
        # Give gateway time to stabilize in monitoring mode
        time.sleep(5)
        self.log_message("Gateway now in monitoring mode - ready for data collection")
        return True

    def monitor_gateway_output(self):
        """Monitor gateway output for packet reception"""
        if not self.gateway_process:
            return
            
        self.log_message("Monitoring gateway output for packets...")
        
        while self.running and self.gateway_process.poll() is None:
            try:
                line = self.gateway_process.stdout.readline()
                if line:
                    line = line.strip()
                    
                    # Check for packet reception or status messages
                    self.parse_gateway_output(line)
                    
            except Exception as e:
                self.log_message("Error monitoring gateway: {}".format(e), "ERROR")
                break

    def parse_gateway_output(self, line):
        """Parse complete gateway pipeline output with bulletproof packet detection"""
        
        # Look for gateway status messages
        if "Low-level gw status ON" in line or "status: gw ON" in line:
            self.log_message("Gateway status: ON - system is running")
        
        # PRIMARY DETECTION: Raw lora_gateway output
        if "--- rxlora[" in line and " dst=" in line and " src=" in line:
            self.packets_received += 1
            self.log_message("=== PACKET #{} DETECTED ===".format(self.packets_received))
            self.log_message("Raw detection: {}".format(line))
            
            # Extract source address from the raw line
            if not self.detected_node_addr:
                try:
                    src_match = re.search(r' src=(\d+)', line)
                    if src_match:
                        src_addr = int(src_match.group(1))
                        self.detected_node_addr = src_addr
                        self.log_message("Auto-detected node address: {}".format(self.detected_node_addr))
                except Exception as e:
                    self.log_message("Error parsing node address: {} - {}".format(line, e), "WARNING")
            
            # Mark initial sync as complete after first packet
            if not self.initial_sync_complete:
                self.initial_sync_complete = True
                self.log_message("=== FIRST PACKET RECEIVED - INITIAL SYNC COMPLETE! ===")

    def wait_for_initial_sync(self, timeout_minutes=10):
        """Wait for first packet in standby mode to trigger the characterization loop"""
        self.log_message("=== WAITING FOR FIRST PACKET IN STANDBY MODE ===")
        self.log_message("Starting in standby config: {}".format(CONFIGURATIONS[STANDBY_CONFIG_INDEX]['name']))
        
        # Force initial startup by resetting radio parameters
        self.current_sf = None
        self.current_bw = None
        self.current_cr = None
        
        # Start gateway in standby mode
        if not self.switch_to_standby():
            return False
        
        # Wait for first packet or timeout
        start_time = time.time()
        timeout_seconds = timeout_minutes * 60
        
        while self.running and not self.initial_sync_complete:
            elapsed = time.time() - start_time
            remaining = timeout_seconds - elapsed
            
            if remaining <= 0:
                self.log_message("Timeout waiting for first packet - check node is transmitting", "ERROR")
                return False
            
            if int(remaining) % 30 == 0:  # Log every 30 seconds
                self.log_message("Waiting for trigger packet... {:.1f} minutes remaining".format(remaining/60))
            
            time.sleep(1)
        
        if self.initial_sync_complete:
            self.log_message("=== FIRST PACKET RECEIVED - STARTING CHARACTERIZATION ===")
            self.log_message("Node address: {}".format(self.detected_node_addr or "Unknown"))
            self.log_message("Ready to start sequence with: {}".format(CONFIGURATIONS[CHARACTERIZATION_SEQUENCE[self.current_sequence_index]]['name']))
            return True
        
        return False

    def send_configuration_commands(self, config_index, config_name):
        """Send configuration commands from standby mode"""
        if not self.detected_node_addr:
            self.log_message("ERROR: No node address detected - cannot send commands", "ERROR")
            return False
        
        # Ensure we're in standby mode for reliable downlinks
        if not self.is_in_standby_mode:
            self.log_message("WARNING: Not in standby mode, switching to standby for reliable downlinks")
            if not self.switch_to_standby():
                return False
            
        command = "/@C{}#".format(config_index)
        
        self.log_message("=== STARTING DOWNLINK PHASE FROM STANDBY ===")
        self.log_message("Sending {} attempts for config: {}".format(self.downlink_attempts, config_name))
        self.log_message("Command: {} to node {}".format(command, self.detected_node_addr))
        self.log_message("Downlink mode: STANDBY ({}) for maximum reliability".format(CONFIGURATIONS[STANDBY_CONFIG_INDEX]['name']))
        
        success_count = 0
        
        for attempt in range(1, self.downlink_attempts + 1):
            self.log_message("Downlink attempt {}/{}: {}".format(attempt, self.downlink_attempts, command))
            
            try:
                # Create downlink directory if it doesn't exist
                if not os.path.exists(self.downlink_dir):
                    os.makedirs(self.downlink_dir)
                
                # Use the reliable manual method with PROPER JSON formatting
                downlink_content = '{{"status":"send_request","dst":{},"data":"{}"}}'.format(self.detected_node_addr, command)
                downlink_content_with_newline = downlink_content + '\n'
                
                # Write multiple commands
                with open(self.downlink_file, 'w') as f:
                    loops = self.commands_per_attempt * self.downlink_multiplier if config_index == self.problematic_index else self.commands_per_attempt
                
                    if config_index == self.problematic_index:
                        self.log_message("Using {} commands for problematic transition".format(loops))
                    
                    for i in range(loops):
                        f.write(downlink_content_with_newline)

                self.log_message("Created downlink file with {} commands: {}".format(loops, downlink_content))
                
                # Verify the file was written correctly
                try:
                    with open(self.downlink_file, 'r') as f:
                        written_content = f.read()
                    
                    # Count lines to verify multiple commands
                    command_count = len([line for line in written_content.strip().split('\n') if line.strip()])
                    success_count += self.commands_per_attempt
                    self.log_message("Downlink attempt {} successful - {} total commands in file".format(attempt, command_count))
                                    
                except Exception as verify_error:
                    self.log_message("Downlink attempt {} failed verification: {}".format(attempt, verify_error), "ERROR")
                    continue
                    
            except Exception as e:
                self.log_message("Error in downlink attempt {}: {}".format(attempt, e), "ERROR")
                if attempt <= self.downlink_attempts:
                    time.sleep(self.downlink_interval)
        
        # Wait for all commands to be processed
        buffer_time_to_use = self.get_buffer_time(config_index)
        self.log_message("Waiting {} seconds for downlinks to be processed...".format(buffer_time_to_use))
        time.sleep(buffer_time_to_use)
        
        self.log_message("=== DOWNLINK PHASE COMPLETE ===")
        self.log_message("Total commands written: {}".format(success_count))
        self.log_message("Successful attempts: {}/{}".format(attempt, self.downlink_attempts))
        
        return success_count > 0

    def run_cycle(self, target_config_index, is_first_cycle=False):
        """Run a cycle with unified standby-based approach"""
        target_config = CONFIGURATIONS[target_config_index]
        cycle_start = datetime.now()
        
        self.log_message("=== STARTING STANDBY-BASED CYCLE: {} ===".format(target_config['name']))
        self.log_message("CLEAN MONITORING: {} minutes (user requested)".format(self.monitoring_time / 60))
        self.log_message("BUFFER TIME: {} seconds".format(self.buffer_time))
        self.log_message("TOTAL CYCLE: {:.1f} minutes".format(self.total_cycle_time / 60.0))

        # PHASE 1: ENSURE STANDBY MODE (every cycle)
        self.log_message("=== PHASE 1: ENSURING STANDBY MODE ===")
        if not self.switch_to_standby():
            self.log_message("Failed to switch to standby mode", "ERROR")
            return False
            
        # PHASE 2: SEND DOWNLINK COMMANDS FROM STANDBY (every cycle)
        self.log_message("=== PHASE 2: SENDING DOWNLINK FROM STANDBY ===")
        self.log_message("Target config: {} (index {})".format(target_config['name'], target_config_index))
        
        if not self.send_configuration_commands(target_config_index, target_config['name']):
            self.log_message("All downlink attempts failed", "ERROR")
            # Continue anyway - node might still switch
        
        # PHASE 3: SWITCH TO MONITORING MODE
        self.log_message("=== PHASE 3: SWITCH TO MONITORING MODE ===")
        if not self.switch_to_monitoring(target_config):
            self.log_message("Failed to switch to monitoring mode", "ERROR")
            return False
        
        # CRITICAL: Clear any pending downlinks AFTER gateway is configured for monitoring
        self.cleanup_downlink_folder()
        self.log_message("Cleared downlink queue to ensure clean monitoring phase")
        
        time.sleep(5)  # Give some time for the gateway to stabilize
        
        # PHASE 4: START MARKER
        self.log_message("=== PHASE 4: START MARKER ===")
        if not self.create_database_marker(target_config['name'], "START"):
            self.log_message("Failed to create START marker", "WARNING")
        
        # PHASE 5: MONITORING EXECUTION
        self.log_message("=== PHASE 5: MONITORING EXECUTION ===")
        start_time = time.time()
        packet_count_start = self.packets_received
        
        while time.time() - start_time < self.monitoring_time and self.running:
            time.sleep(2)  # Check every 2 seconds
            
            # Log progress every minute
            elapsed = time.time() - start_time
            if int(elapsed) % 60 == 0 and elapsed > 0:
                packets_this_cycle = self.packets_received - packet_count_start
                remaining = (self.monitoring_time - elapsed) / 60.0
                self.log_message("Clean monitoring progress: {:.1f} min remaining, {} packets received".format(
                    remaining, packets_this_cycle))
        
        if not self.running:
            return False
        
        # PHASE 6: END MARKER
        self.log_message("=== PHASE 6: END MARKER ===")
        if not self.create_database_marker(target_config['name'], "END"):
            self.log_message("Failed to create END marker", "WARNING")
        
        # PHASE 7: RETURN TO STANDBY
        self.log_message("=== PHASE 7: RETURN TO STANDBY ===")
        if not self.switch_to_standby():
            self.log_message("Failed to return to standby mode", "ERROR")
            return False
        
        # Summary with clear confirmation
        total_packets_this_cycle = self.packets_received - packet_count_start
        actual_monitoring_time = time.time() - start_time
        self.log_message("=== CYCLE COMPLETED ===")
        self.log_message("Configuration: {}".format(target_config['name']))
        self.log_message("ACTUAL CLEAN MONITORING TIME: {:.2f} minutes".format(actual_monitoring_time / 60.0))
        self.log_message("REQUESTED CLEAN TIME: {} minutes".format(self.monitoring_time / 60))
        self.log_message("Packets received this cycle: {}".format(total_packets_this_cycle))
        self.log_message("Gateway returned to standby mode - ready for next cycle")
        
        return True

    def run_characterization_sequence(self, max_cycles=None):
        """Run the complete characterization sequence with standby-based cycling"""
        
        # Step 1: Wait for initial sync in standby mode
        if not self.wait_for_initial_sync():
            self.log_message("Failed to achieve initial synchronization", "ERROR")
            return False
        
        # Step 2: Run characterization cycles through predefined sequence
        self.log_message("=== STARTING STANDBY-BASED CHARACTERIZATION SEQUENCE ===")
        self.log_message("Configuration sequence order:")
        for i, config_index in enumerate(CHARACTERIZATION_SEQUENCE):
            marker = " -> START" if i == self.current_sequence_index else ""
            self.log_message("  {}: {} (index {}){}".format(i, CONFIGURATIONS[config_index]['name'], config_index, marker))
        
        self.log_message("Standby config: {} (index {})".format(CONFIGURATIONS[STANDBY_CONFIG_INDEX]['name'], STANDBY_CONFIG_INDEX))
        
        cycle_count = 0
        
        while self.running:
            # Get current target configuration from sequence
            target_config_index = CHARACTERIZATION_SEQUENCE[self.current_sequence_index]
            target_config = CONFIGURATIONS[target_config_index]
            
            self.log_message("")
            self.log_message("+" + "="*70 + "+")
            self.log_message("| CYCLE {}: {} (sequence index {}, config index {}) |".format(
                cycle_count + 1, target_config['name'], self.current_sequence_index, target_config_index).ljust(72) + "|")
            self.log_message("+" + "="*70 + "+")
            
            # Run the cycle with unified approach
            success = self.run_cycle(target_config_index)
            
            if success:
                cycle_count += 1
                self.log_message("Cycle {} completed successfully".format(cycle_count))
                
                # Advance to next configuration in sequence
                old_sequence_index = self.current_sequence_index
                self.current_sequence_index = (self.current_sequence_index + 1) % len(CHARACTERIZATION_SEQUENCE)
                next_config_index = CHARACTERIZATION_SEQUENCE[self.current_sequence_index]
                next_config = CONFIGURATIONS[next_config_index]
                
                self.log_message("SEQUENCE ADVANCED: {} -> {} (next: {})".format(
                    old_sequence_index, self.current_sequence_index, next_config['name']))
                
                if max_cycles and cycle_count >= max_cycles:
                    self.log_message("Maximum cycles ({}) reached, stopping".format(max_cycles))
                    break
                    
            else:
                self.log_message("Cycle failed, retrying in 30 seconds...", "WARNING")
                time.sleep(30)
        
        self.log_message("=== STANDBY-BASED CHARACTERIZATION SEQUENCE COMPLETED ===")
        self.log_message("Total cycles completed: {}".format(cycle_count))
        
        # Final cleanup
        self.cleanup_downlink_folder()
        
        return True


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Enhanced Gateway Controller with Standby Mode for LoRa Network Characterization')
    parser.add_argument('--gateway-path', default='/home/pi/lora_gateway',
                       help='Path to gateway directory')
    parser.add_argument('--cycle-minutes', type=int, default=5,
                       help='Minutes of CLEAN MONITORING per configuration')
    parser.add_argument('--max-cycles', type=int,
                       help='Maximum cycles to run (0 = infinite)')
    parser.add_argument('--sync-timeout', type=int, default=10,
                       help='Minutes to wait for initial packet')
    parser.add_argument('--gateway-check-interval', type=int,
                       help='Override gateway downlink check interval (default: auto-detect from gateway_conf.json)')
    parser.add_argument('--start-sequence-index', type=int, default=0,
                   help='Starting sequence index (0-5) for resuming - maps to characterization sequence')
    
    args = parser.parse_args()
    
    # Validate start-sequence-index
    if not (0 <= args.start_sequence_index < len(CHARACTERIZATION_SEQUENCE)):
        print("ERROR: start-sequence-index must be between 0 and {} (got {})".format(len(CHARACTERIZATION_SEQUENCE)-1, args.start_sequence_index))
        sys.exit(1)
    
    # Create controller with basic settings
    controller = GatewayController(
        gateway_path=args.gateway_path,
        cycle_minutes=args.cycle_minutes,
        start_sequence_index=args.start_sequence_index
    )
    
    # Apply any command-line overrides BEFORE recalculating timings
    settings_changed = False
    
    if args.gateway_check_interval:
        controller.gateway_downlink_check = args.gateway_check_interval
        settings_changed = True
        controller.log_message("Override: gateway check interval = {} seconds".format(args.gateway_check_interval))
    
    # Recalculate ALL timings if any settings were changed
    if settings_changed:
        controller.log_message("Recalculating timings due to command-line overrides...")
        
        # Recalculate downlink interval
        controller.downlink_interval = controller.gateway_downlink_check + 10
        
        # Recalculate buffer time using preserved formula
        controller.buffer_time = 60
        """(
            controller.gateway_downlink_check +
            (controller.commands_per_attempt * controller.node_transmission_interval) +
            (controller.commands_per_attempt * controller.node_downlink_delay) +
            30
        )"""
        
        # Recalculate total cycle time
        controller.total_cycle_time = controller.monitoring_time + controller.buffer_time
        
        # Log the new calculations
        controller.log_message("=== RECALCULATED TIMING CONFIGURATION ===")
        controller.log_message("Downlink attempts: {}".format(controller.downlink_attempts))
        controller.log_message("Gateway check interval: {} seconds".format(controller.gateway_downlink_check))
        controller.log_message("Commands per attempt: {}".format(controller.commands_per_attempt))
        controller.log_message("Node transmission interval: {}s".format(controller.node_transmission_interval))
        controller.log_message("Node downlink delay: {}s".format(controller.node_downlink_delay))
        controller.log_message("Buffer time: {} seconds ({:.1f} minutes)".format(
            controller.buffer_time, controller.buffer_time / 60.0))
        controller.log_message("Total cycle time: {} seconds ({:.1f} minutes)".format(
            controller.total_cycle_time, controller.total_cycle_time / 60.0))
        controller.log_message("=" * 50)
    
    # Final configuration summary
    controller.log_message("=== FINAL CONFIGURATION CONFIRMATION ===")
    controller.log_message("USER REQUESTED: {} minutes clean monitoring per config".format(args.cycle_minutes))
    controller.log_message("GUARANTEED CLEAN TIME: {} minutes".format(controller.monitoring_time / 60))
    controller.log_message("Buffer overhead: {} seconds ({:.1f} minutes)".format(
        controller.buffer_time, controller.buffer_time / 60.0))
    controller.log_message("Total time per cycle: {:.1f} minutes".format(controller.total_cycle_time / 60.0))
    controller.log_message("STANDBY MODE: {} (Index {})".format(CONFIGURATIONS[STANDBY_CONFIG_INDEX]['name'], STANDBY_CONFIG_INDEX))
    controller.log_message("CHARACTERIZATION SEQUENCE: {}".format([CONFIGURATIONS[i]['name'] for i in CHARACTERIZATION_SEQUENCE]))
    controller.log_message("STARTING SEQUENCE INDEX: {} (Config: {})".format(
        args.start_sequence_index, CONFIGURATIONS[CHARACTERIZATION_SEQUENCE[args.start_sequence_index]]['name']))
    controller.log_message("Max cycles: {}".format(args.max_cycles if args.max_cycles else "unlimited"))
    controller.log_message("=" * 50)
    
    try:
        controller.run_characterization_sequence(max_cycles=args.max_cycles)
    except KeyboardInterrupt:
        controller.log_message("Interrupted by user")
    finally:
        controller.kill_gateway_processes()

if __name__ == "__main__":
    main()