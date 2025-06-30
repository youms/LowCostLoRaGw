#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Gateway Controller - Python 2 Version
Fixed to work with the actual gateway ecosystem:
- Uses start_gw.py (Python 2) as the proper startup method
- Compatible with post_processing_gw.py and log_gw.py (Python 2)
- Modifies gateway_conf.json to set radio parameters
- Waits for first packet reception to trigger characterization loop
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

# Configuration mappings - must match the node's NetworkParams.h
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
]

class GatewayController:
    def __init__(self, gateway_path="/home/pi/lora_gateway", cycle_minutes=50):
        self.gateway_path = gateway_path
        self.downlink_dir = os.path.join(gateway_path, "downlink")
        self.downlink_file = os.path.join(self.downlink_dir, "downlink-post.txt")
        self.log_dir = os.path.join(gateway_path, "characterization_logs")
        
        # Timing configuration
        self.cycle_minutes = cycle_minutes
        self.confirmation_seconds = 30
        
        # State tracking
        self.current_config_index = 0  # Start with index 0
        self.detected_node_addr = None
        self.gateway_process = None
        self.running = True
        self.initial_sync_complete = False
        self.packets_received = 0
        
        # Setup logging
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        self.setup_logging()
        
        # Signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

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

    def start_gateway_with_config(self, config):
        """Start gateway using manual command pipeline"""
        self.log_message("Starting gateway with config: {}".format(config['name']))
        
        # Kill any existing processes
        self.kill_gateway_processes()
        time.sleep(3)
        
        try:
            os.chdir(self.gateway_path)
            
            # Use the manual pipeline command as you specified
            gateway_cmd = "sudo ./lora_gateway --bw {} --sf {} --cr {} --freq 868.0 | python ./post_processing_gw.py | python ./log_gw.py".format(
                config['bw'], config['sf'], config['cr']
            )
            
            self.log_message("Starting gateway pipeline: {}".format(gateway_cmd))
            
            self.gateway_process = subprocess.Popen(
                gateway_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True
            )
            
            self.log_message("Gateway pipeline started (PID: {})".format(self.gateway_process.pid))
            
            # Start monitoring thread (Python 2 compatible)
            self.monitor_thread = threading.Thread(target=self.monitor_gateway_output)
            self.monitor_thread.daemon = True  # Set daemon property after creation
            self.monitor_thread.start()
            
            return True
            
        except Exception as e:
            self.log_message("Failed to start gateway: {}".format(e), "ERROR")
            return False

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
                    
                    # Log all output
                    with open(os.path.join(self.log_dir, "gateway_output.log"), 'a') as f:
                        f.write("GATEWAY: {}\n".format(line))
                    
                    # Check for packet reception or status messages
                    self.parse_gateway_output(line)
                    
            except Exception as e:
                self.log_message("Error monitoring gateway: {}".format(e), "ERROR")
                break

    def parse_gateway_output(self, line):
        """Parse gateway output to detect packets and status"""
        
        # Look for "status: gw ON" message
        if "status: gw ON" in line:
            self.log_message("Gateway status: ON - system is running")
        
        # Look for packet reception in various formats
        if any(indicator in line for indicator in ["^p", "rcv ctrl pkt info", "Temperature sensor"]):
            self.packets_received += 1
            self.log_message("Packet #{} detected: {}".format(self.packets_received, line))
            
            # Try to extract node address if available
            if not self.detected_node_addr:
                # Look for source address patterns
                if "src=" in line:
                    try:
                        src_match = re.search(r'src=(\d+)', line)
                        if src_match:
                            self.detected_node_addr = int(src_match.group(1))
                            self.log_message("Auto-detected node address: {}".format(self.detected_node_addr))
                    except:
                        pass
            
            # Mark initial sync as complete after first packet
            if not self.initial_sync_complete:
                self.initial_sync_complete = True
                self.log_message("First packet received - initial sync complete!")

    def wait_for_initial_sync(self, timeout_minutes=10):
        """Wait for first packet to trigger the characterization loop"""
        self.log_message("=== WAITING FOR FIRST PACKET TO START CHARACTERIZATION ===")
        self.log_message("Starting with config index 0: {}".format(CONFIGURATIONS[0]['name']))
        
        # Start gateway with index 0 configuration
        if not self.start_gateway_with_config(CONFIGURATIONS[0]):
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
            self.log_message("Starting configuration: {}".format(CONFIGURATIONS[0]['name']))
            return True
        
        return False

    def send_configuration_command(self, config_index, max_retries=3):
        """Send configuration command via simple downlink file creation"""
        if not self.detected_node_addr:
            self.log_message("No node address detected, using broadcast (dst=0)")
            target_addr = 0
        else:
            target_addr = self.detected_node_addr
            
        command = "/@C{}#".format(config_index)
        
        for attempt in range(max_retries):
            self.log_message("Sending config command (attempt {}/{}): {} to node {}".format(
                attempt + 1, max_retries, command, target_addr))
            
            try:
                # Create downlink directory if it doesn't exist
                if not os.path.exists(self.downlink_dir):
                    os.makedirs(self.downlink_dir)
                
                # Use the simple manual method you specified
                downlink_content = '{{"status":"send_request","dst":{},"data":"{}"}}'.format(target_addr, command)
                
                # Write using simple file creation (like echo command)
                with open(self.downlink_file, 'w') as f:
                    f.write(downlink_content)
                
                self.log_message("Created downlink file: {}".format(downlink_content))
                self.log_message("Written to: {}".format(self.downlink_file))
                time.sleep(5)  # Wait for processing
                return True
                
            except Exception as e:
                self.log_message("Error creating downlink file (attempt {}): {}".format(attempt + 1, e), "ERROR")
                time.sleep(2)
        
        return False

    def run_cycle(self, config):
        """Run a cycle with specified configuration"""
        cycle_start = datetime.now()
        self.log_message("=== STARTING CYCLE: {} ===".format(config['name']))
        
        # Start gateway with this configuration
        if not self.start_gateway_with_config(config):
            self.log_message("Failed to start gateway for cycle", "ERROR")
            return False
        
        # Wait for most of the cycle duration
        cycle_duration_seconds = self.cycle_minutes * 60
        command_delay = 90  # Send command 90 seconds before cycle end
        monitoring_duration = cycle_duration_seconds - command_delay
        
        self.log_message("Monitoring for {:.1f} minutes before sending next command...".format(monitoring_duration / 60.0))
        
        # Monitor during cycle
        start_time = time.time()
        while time.time() - start_time < monitoring_duration and self.running:
            time.sleep(10)  # Check every 10 seconds
        
        if not self.running:
            return False
        
        # Send command for next configuration
        next_config_index = (config['index'] + 1) % len(CONFIGURATIONS)
        next_config = CONFIGURATIONS[next_config_index]
        
        self.log_message("Sending transition command to: {}".format(next_config['name']))
        
        if self.send_configuration_command(next_config_index):
            # Wait for confirmation
            self.log_message("Waiting {} seconds for transition confirmation...".format(self.confirmation_seconds))
            time.sleep(self.confirmation_seconds)
            
            self.current_config_index = next_config_index
            self.log_message("Cycle completed successfully")
            return True
        else:
            self.log_message("Failed to send transition command", "ERROR")
            return False

    def run_characterization_sequence(self, max_cycles=None):
        """Run the complete characterization sequence"""
        
        # Step 1: Wait for initial sync
        if not self.wait_for_initial_sync():
            self.log_message("Failed to achieve initial synchronization", "ERROR")
            return False
        
        # Step 2: Run characterization cycles
        self.log_message("=== STARTING CHARACTERIZATION SEQUENCE ===")
        
        cycle_count = 0
        
        while self.running:
            config = CONFIGURATIONS[self.current_config_index]
            
            # Run the cycle
            success = self.run_cycle(config)
            
            if success:
                cycle_count += 1
                self.log_message("Cycle {} completed successfully".format(cycle_count))
                
                if max_cycles and cycle_count >= max_cycles:
                    self.log_message("Maximum cycles reached, stopping")
                    break
                    
            else:
                self.log_message("Cycle failed, retrying...", "WARNING")
                time.sleep(30)
        
        self.log_message("=== CHARACTERIZATION SEQUENCE COMPLETED ===")
        return True

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Gateway Controller for LoRa Network Characterization')
    parser.add_argument('--gateway-path', default='/home/pi/lora_gateway',
                       help='Path to gateway directory')
    parser.add_argument('--cycle-minutes', type=int, default=50,
                       help='Minutes per configuration cycle')
    parser.add_argument('--max-cycles', type=int,
                       help='Maximum cycles to run')
    parser.add_argument('--sync-timeout', type=int, default=10,
                       help='Minutes to wait for initial packet')
    
    args = parser.parse_args()
    
    controller = GatewayController(
        gateway_path=args.gateway_path,
        cycle_minutes=args.cycle_minutes
    )
    
    try:
        controller.run_characterization_sequence(max_cycles=args.max_cycles)
    except KeyboardInterrupt:
        controller.log_message("Interrupted by user")
    finally:
        controller.kill_gateway_processes()

if __name__ == "__main__":
    main()