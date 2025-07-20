#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Node Monitoring Controller - Python 2 Version
Round-robin node monitoring controller with M1/M0 commands:
- Automated M1 (start monitoring) and M0 (stop monitoring) commands
- Gateway frequency switching between 865.2MHz (standby) and 868MHz (monitoring)
- Database markers for clean data analysis boundaries
- Phase-based node management with round-robin cycling
- Compatible with Enhanced_Lora_DS18B20_MQ9_2_Downlink_Controlled nodes
"""

import json
import time
import os
import sys
import subprocess
import threading
import signal
import argparse
import re
from datetime import datetime, timedelta

# Node mappings - configure your network nodes here
NODES = [
    {"index": 0, "address": 10, "name": "floor-0"},
    {"index": 1, "address": 20, "name": "floor-1"},
    {"index": 2, "address": 30, "name": "floor-2"},
]

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

class NodeMonitoringController:
    def __init__(self, gateway_path="/home/pi/lora_gateway", monitoring_minutes=5, 
                 monitoring_config_index=2, standby_config_index=8, start_node_index=0):
        self.gateway_path = gateway_path
        self.downlink_dir = os.path.join(gateway_path, "downlink")
        self.downlink_file = os.path.join(self.downlink_dir, "downlink-post.txt")

        # Clean up any leftover downlink files
        if os.path.exists(self.downlink_dir):
            for fname in os.listdir(self.downlink_dir):
                if fname.startswith("downlink"):
                    try:
                        os.remove(os.path.join(self.downlink_dir, fname))
                    except Exception as e:
                        print("Failed to delete {}: {}".format(fname, e))
        else:
            os.makedirs(self.downlink_dir)
        
        self.log_dir = os.path.join(gateway_path, "node_monitoring_logs")
        
        # Node configuration
        self.monitoring_minutes = monitoring_minutes
        self.monitoring_time = monitoring_minutes * 60  # Convert to seconds
        self.monitoring_config_index = monitoring_config_index
        self.standby_config_index = standby_config_index
        self.current_node_index = start_node_index
        
        # Validate start_node_index
        if not (0 <= start_node_index < len(NODES)):
            raise ValueError("start_node_index must be between 0 and {} (got {})".format(len(NODES)-1, start_node_index))
        
        # Validate configuration indexes
        if not (0 <= monitoring_config_index < len(CONFIGURATIONS)):
            raise ValueError("monitoring_config_index must be between 0 and {} (got {})".format(len(CONFIGURATIONS)-1, monitoring_config_index))
        if not (0 <= standby_config_index < len(CONFIGURATIONS)):
            raise ValueError("standby_config_index must be between 0 and {} (got {})".format(len(CONFIGURATIONS)-1, standby_config_index))
        
        # Gateway timing configuration
        self.gateway_downlink_check = self.get_gateway_downlink_interval()
        self.downlink_interval = self.gateway_downlink_check + 10
        self.commands_per_attempt = 2  # Write 2 commands per downlink attempt
        self.downlink_attempts = 1
        
        # Calculate buffer time for command processing
        self.node_transmission_interval = 10  # Seconds between node transmissions  
        self.node_downlink_delay = 30  # Seconds node waits after receiving each downlink
        self.buffer_time = 60 
        """ (
            self.gateway_downlink_check +
            (self.commands_per_attempt * self.node_transmission_interval) +
            (self.commands_per_attempt * self.node_downlink_delay) +
            30  # Safety margin
        ) """
        
        # State tracking
        self.gateway_process = None
        self.monitor_thread = None
        self.running = True
        self.detected_nodes = set()
        self.packets_received = 0
        
        # Gateway frequency constants (matching the Arduino code)
        self.standby_frequency = 865.2  # MHz - for standby mode SF12
        self.monitoring_frequency = 868.0  # MHz - for monitoring mode SF7
        
        # Setup logging
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        self.setup_logging()
        
        # Log configuration
        self.log_message("=== NODE MONITORING CONTROLLER INITIALIZED ===")
        self.log_message("Monitoring time per node: {} minutes".format(monitoring_minutes))
        self.log_message("Total nodes: {}".format(len(NODES)))
        self.log_message("Starting with node: {} ({})".format(start_node_index, NODES[start_node_index]['name']))
        self.log_message("Monitoring config: {} ({})".format(monitoring_config_index, CONFIGURATIONS[monitoring_config_index]['name']))
        self.log_message("Standby config: {} ({})".format(standby_config_index, CONFIGURATIONS[standby_config_index]['name']))
        self.log_message("Buffer time: {} seconds ({:.1f} minutes)".format(
            self.buffer_time, self.buffer_time / 60.0))
        self.log_message("=" * 50)
        
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
        """Clean up downlink folder"""
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

    def create_database_marker(self, marker_type, node_name=None, from_node=None, to_node=None):
        """Create a database marker using CloudMongoDB.py subprocess"""
        try:
            from datetime import datetime
            now = datetime.now()
            
            # Get gateway ID
            gateway_id = self.get_gateway_id()
            
            # Format marker data based on type
            if marker_type == "PHASE_CHANGE" and from_node and to_node:
                marker_data = "PHASE_CHANGE-{}-{}".format(from_node, to_node)
            elif node_name:
                marker_data = "{}-{}".format(node_name, marker_type)
            else:
                marker_data = marker_type
            
            self.log_message("=== DATABASE MARKER: {} ===".format(marker_data))
            
            # Format for CloudMongoDB.py (same as enhanced_gateway_controller.py)
            ldata = marker_data
            pdata = "255,255,0,0,{},0,0,0".format(len(marker_data))  # Special type 255 for markers with ToA
            rdata = "125,5,12"  # Default radio data for markers
            tdata = now.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "+01:00"  # ISO format
            gwid = gateway_id
            
            # Call CloudMongoDB.py
            mongodb_cmd = 'python CloudMongoDB.py "{}" "{}" "{}" "{}" "{}"'.format(
                ldata, pdata, rdata, tdata, gwid
            )
            
            self.log_message("Calling: {}".format(mongodb_cmd))
            
            # Execute in gateway directory
            old_cwd = os.getcwd()
            os.chdir(self.gateway_path)
            
            result = subprocess.call(mongodb_cmd, shell=True)
            
            os.chdir(old_cwd)
            
            if result == 0:
                self.log_message("Database marker inserted successfully")
                return True
            else:
                self.log_message("Database marker insertion failed with code: {}".format(result), "ERROR")
                return False
                
        except Exception as e:
            self.log_message("Error creating database marker: {}".format(e), "ERROR")
            return False

    def start_gateway_with_config(self, frequency, sf, bw=125, cr=5):
        """Start gateway using the complete pipeline"""
        self.log_message("Starting gateway: {:.1f}MHz SF{} BW{} CR{}".format(frequency, sf, bw, cr))
        
        # CRITICAL: Clean downlink folder BEFORE killing processes
        self.cleanup_downlink_folder()
        
        # Kill existing processes and start new ones
        self.kill_gateway_processes()
        time.sleep(3)
        
        # Start new gateway process with radio parameters
        try:
            old_cwd = os.getcwd()
            os.chdir(self.gateway_path)
            
            # Use the COMPLETE pipeline - this is essential for proper operation
            gateway_cmd = "sudo ./lora_gateway --bw {} --sf {} --cr {} --freq {:.1f} 2>&1 | python ./post_processing_gw.py 2>&1 | python ./log_gw.py 2>&1".format(
                bw, sf, cr, frequency
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
            
            os.chdir(old_cwd)
            return True
            
        except Exception as e:
            self.log_message("Failed to start gateway: {}".format(e), "ERROR")
            os.chdir(old_cwd)
            return False

    def monitor_gateway_output(self):
        """Monitor gateway output for packet detection and logging"""
        try:
            for line in iter(self.gateway_process.stdout.readline, ''):
                if not self.running:
                    break
                
                line = line.strip()
                
                # Log important messages (simplified version)
                if "rxlora" in line:
                    self.packets_received += 1
                    self.log_message("Packet received: {}".format(line))
                elif "ERROR" in line:
                    self.log_message("Gateway error: {}".format(line), "ERROR")
                
        except Exception as e:
            self.log_message("Error monitoring gateway output: {}".format(e), "ERROR")

    def send_downlink_command(self, node_address, command):
        """Send a downlink command to a specific node"""
        self.log_message("Sending command '{}' to node {}".format(command, node_address))
        
        try:
            # Create downlink directory if needed
            if not os.path.exists(self.downlink_dir):
                os.makedirs(self.downlink_dir)
            
            # Format downlink command
            downlink_content = '{{"status":"send_request","dst":{},"data":"{}"}}'.format(node_address, command)
            downlink_content_with_newline = downlink_content + '\n'
            
            # Write multiple commands for reliability (as in enhanced_gateway_controller.py)
            with open(self.downlink_file, 'a') as f:
                for i in range(self.commands_per_attempt):
                    f.write(downlink_content_with_newline)
                    f.flush()
            
            self.log_message("Downlink command written: {} copies".format(self.commands_per_attempt))
            
            # Wait for command processing
            self.log_message("Waiting {} seconds for command processing...".format(self.buffer_time))
            time.sleep(self.buffer_time)
            
            return True
            
        except Exception as e:
            self.log_message("Error sending downlink command: {}".format(e), "ERROR")
            return False

    def monitor_single_node(self, node, is_first_node=False):
        """Monitor a single node through complete cycle"""
        node_addr = node['address']
        node_name = node['name']
        
        self.log_message("=== STARTING MONITORING CYCLE: {} (Address {}) ===".format(node_name, node_addr))
        
        # PHASE 1: Send M1 command (gateway STAYS in standby mode for first node)
        self.log_message("=== PHASE 1: SEND M1 COMMAND ===")
        
        if is_first_node:
            # For first node, gateway is already in standby mode from startup
            self.log_message("First node - gateway already in standby mode (865.2MHz SF12)")
        else:
            # For subsequent nodes, ensure gateway is in standby mode
            if not self.start_gateway_with_config(self.standby_frequency, 12, 125, 5):
                self.log_message("Failed to start standby gateway", "ERROR")
                return False
        
        if not self.send_downlink_command(node_addr, "/@M1#"):
            self.log_message("Failed to send M1 command", "ERROR")
            return False
        
        self.log_message("M1 command sent - node should switch to monitoring mode")
        
        # PHASE 2: Switch gateway to monitoring mode (868MHz SF7)
        self.log_message("=== PHASE 2: SWITCH TO MONITORING MODE ===")
        if not self.start_gateway_with_config(self.monitoring_frequency, 7, 125, 5):
            self.log_message("Failed to start monitoring gateway", "ERROR")
            return False
        
        # Clear any pending downlinks after gateway restart
        self.cleanup_downlink_folder()
        self.log_message("Cleared downlink queue for clean monitoring")
        
        # PHASE 3: Insert START marker
        self.log_message("=== PHASE 3: START MARKER ===")
        if not self.create_database_marker("MONITORING_START", node_name):
            self.log_message("Failed to create START marker", "WARNING")
        
        # PHASE 4: Monitor for specified duration
        self.log_message("=== PHASE 4: MONITORING EXECUTION ===")
        self.log_message("Monitoring {} for {} minutes...".format(node_name, self.monitoring_minutes))
        
        start_time = time.time()
        packet_count_start = self.packets_received
        
        while time.time() - start_time < self.monitoring_time and self.running:
            time.sleep(2)  # Check every 2 seconds
            
            # Log progress every minute
            elapsed = time.time() - start_time
            if int(elapsed) % 60 == 0 and elapsed > 0:
                packets_this_cycle = self.packets_received - packet_count_start
                remaining = (self.monitoring_time - elapsed) / 60.0
                self.log_message("Monitoring progress: {:.1f} min remaining, {} packets received".format(
                    remaining, packets_this_cycle))
        
        if not self.running:
            return False
        
        # PHASE 5: Insert END marker
        self.log_message("=== PHASE 5: END MARKER ===")
        if not self.create_database_marker("MONITORING_END", node_name):
            self.log_message("Failed to create END marker", "WARNING")
        
        # PHASE 6: Send M0 command (node enters sleep) - STAY in monitoring mode
        self.log_message("=== PHASE 6: SEND M0 COMMAND ===")
        self.log_message("Sending M0 while staying in monitoring mode temporarily...")
        
        if not self.send_downlink_command(node_addr, "/@M0#"):
            self.log_message("Failed to send M0 command", "ERROR")
            return False
        
        self.log_message("M0 command sent - node entering POST_MONITOR_SLEEP")
        
        # PHASE 7: Return to standby mode (ready for next node or standby collection)
        self.log_message("=== PHASE 7: RETURN TO STANDBY MODE ===")
        if not self.start_gateway_with_config(self.standby_frequency, 12, 125, 5):
            self.log_message("Failed to start standby gateway", "ERROR")
            return False
        
        self.log_message("Gateway back to standby mode - ready to receive remaining active nodes")
        
        # Summary
        total_packets_this_cycle = self.packets_received - packet_count_start
        actual_monitoring_time = time.time() - start_time
        self.log_message("=== CYCLE COMPLETED ===")
        self.log_message("Node: {} (Address {})".format(node_name, node_addr))
        self.log_message("Actual monitoring time: {:.2f} minutes".format(actual_monitoring_time / 60.0))
        self.log_message("Requested time: {} minutes".format(self.monitoring_minutes))
        self.log_message("Packets received: {}".format(total_packets_this_cycle))
        self.log_message("Network cleanup: {} now sleeping, remaining nodes still active".format(node_name))
        
        return True

    def run_monitoring_sequence(self, max_cycles=None):
        """Run the complete node monitoring sequence"""
        self.log_message("=== STARTING NODE MONITORING SEQUENCE ===")
        self.log_message("Node monitoring order:")
        for i, node in enumerate(NODES):
            marker = " -> START" if i == self.current_node_index else ""
            self.log_message("  {}: {} (Address {}){}".format(i, node['name'], node['address'], marker))
        
        if max_cycles:
            self.log_message("Maximum cycles: {}".format(max_cycles))
        else:
            self.log_message("Running indefinitely (Ctrl+C to stop)")
        
        # STARTUP PHASE: Initialize in standby mode to receive from all nodes
        self.log_message("=== STARTUP PHASE: STANDBY MODE ===")
        self.log_message("Starting gateway in standby mode (865.2MHz SF12) to receive from all nodes...")
        if not self.start_gateway_with_config(self.standby_frequency, 12, 125, 5):
            self.log_message("Failed to start initial standby gateway", "ERROR")
            return False
        
        self.log_message("Gateway ready - all nodes should be transmitting on 865.2MHz SF12")
        self.log_message("Collecting packets from all nodes before starting round-robin monitoring...")
        
        # Optional: Brief initial collection period to confirm all nodes are active
        initial_wait = 30  # seconds
        self.log_message("Initial collection period: {} seconds".format(initial_wait))
        start_time = time.time()
        initial_packet_count = self.packets_received
        
        while time.time() - start_time < initial_wait and self.running:
            time.sleep(2)
            elapsed = time.time() - start_time
            if int(elapsed) % 10 == 0 and elapsed > 0:
                packets_collected = self.packets_received - initial_packet_count
                remaining = initial_wait - elapsed
                self.log_message("Standby collection: {} packets received, {:.0f}s remaining".format(
                    packets_collected, remaining))
        
        packets_collected = self.packets_received - initial_packet_count
        self.log_message("Initial collection complete: {} packets from all nodes".format(packets_collected))
        
        cycle_count = 0
        
        while self.running:
            if max_cycles and cycle_count >= max_cycles:
                self.log_message("Reached maximum cycles ({}), stopping".format(max_cycles))
                break
            
            cycle_count += 1
            self.log_message("=== STARTING CYCLE {} ===".format(cycle_count))
            
            # Monitor each node in sequence
            for i in range(len(NODES)):
                if not self.running:
                    break
                
                current_node = NODES[self.current_node_index]
                
                # Insert phase change marker (except for first node of first cycle)
                if not (cycle_count == 1 and i == 0):
                    prev_node_index = (self.current_node_index - 1) % len(NODES)
                    prev_node = NODES[prev_node_index]
                    if not self.create_database_marker("PHASE_CHANGE", None, 
                                                     prev_node['name'], current_node['name']):
                        self.log_message("Failed to create PHASE_CHANGE marker", "WARNING")
                
                # Monitor current node
                is_first_node = (cycle_count == 1 and i == 0)
                if not self.monitor_single_node(current_node, is_first_node):
                    self.log_message("Failed to monitor node {}, continuing...".format(current_node['name']), "ERROR")
                
                # Move to next node
                self.current_node_index = (self.current_node_index + 1) % len(NODES)
            
            self.log_message("=== CYCLE {} COMPLETED ===".format(cycle_count))
        
        self.log_message("=== NODE MONITORING SEQUENCE COMPLETED ===")
        self.log_message("Total cycles completed: {}".format(cycle_count))
        
        # Final cleanup
        self.cleanup_downlink_folder()
        
        return True

def main():
    parser = argparse.ArgumentParser(description='Node Monitoring Controller')
    parser.add_argument('--gateway-path', default='/home/pi/lora_gateway',
                       help='Path to gateway directory')
    parser.add_argument('--monitoring-minutes', type=int, default=5,
                       help='Minutes of monitoring per node')
    parser.add_argument('--monitoring-config-index', type=int, default=2,
                       help='Node monitoring configuration index')
    parser.add_argument('--standby-config-index', type=int, default=8,
                       help='Node standby configuration index')  
    parser.add_argument('--max-cycles', type=int,
                       help='Maximum cycles to run (0 = infinite)')
    parser.add_argument('--start-node-index', type=int, default=0,
                       help='Starting node index (0-N) for resuming')
    
    args = parser.parse_args()
    
    # Create controller
    controller = NodeMonitoringController(
        gateway_path=args.gateway_path,
        monitoring_minutes=args.monitoring_minutes,
        monitoring_config_index=args.monitoring_config_index,
        standby_config_index=args.standby_config_index,
        start_node_index=args.start_node_index
    )
    
    # Final configuration summary
    controller.log_message("=== FINAL CONFIGURATION ===")
    controller.log_message("Monitoring time per node: {} minutes".format(args.monitoring_minutes))
    controller.log_message("Total nodes: {}".format(len(NODES)))
    controller.log_message("Monitoring config: {} ({})".format(args.monitoring_config_index, CONFIGURATIONS[args.monitoring_config_index]['name']))
    controller.log_message("Standby config: {} ({})".format(args.standby_config_index, CONFIGURATIONS[args.standby_config_index]['name']))
    controller.log_message("Max cycles: {}".format(args.max_cycles if args.max_cycles else "unlimited"))
    controller.log_message("Starting node: {} ({})".format(
        args.start_node_index, NODES[args.start_node_index]['name']))
    controller.log_message("=" * 50)
    
    try:
        controller.run_monitoring_sequence(max_cycles=args.max_cycles)
    except KeyboardInterrupt:
        controller.log_message("Interrupted by user")
    finally:
        controller.kill_gateway_processes()

if __name__ == "__main__":
    main()