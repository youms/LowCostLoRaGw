####################################################
# Configuration for CloudLBS.py and lbs_daemon.py
# LoRa Basics Station uplink forwarding to TTN V3
####################################################

# TTN LNS WebSocket endpoint.
# Change the region prefix if your gateway is registered outside EU:
#   eu1  -> Europe
#   nam1 -> North America
#   au1  -> Australia
lorawan_server = "eu1.cloud.thethings.network"
lorawan_port   = 8887

# API key from the TTN console:
#   Console -> Gateways -> <your gateway> -> API Keys
#   -> Add API Key -> grant "Link as Gateway to a Gateway Server for traffic exchange..."
# The key looks like: NNSXS.XXXXXXXXXXXXXXXXXX.YYYYYYYYYY...
api_key = "NNSXS.4FV6CIBUYLEWUN567DEPCGSBBWWLBXOGVY73KVQ.NFSF56AKP37VQG55EHF72WVHCSVIO6QNDTUHPCDQYPRMDBWZ7RQA"

# Queue file: CloudLBS.py appends one updf JSON line per received packet.
# lbs_daemon.py reads and clears this file to forward packets to TTN.
# Must be in a directory writable by the user running the gateway.
queue_file = "/home/pi/lora_gateway/lbs_queue.txt"

# Downlink file: lbs_daemon.py writes TTN dnmsg JSON here for inspection.
# This is intentionally separate from downlink/downlink.txt (the Semtech txpk
# file the lora_gateway binary reads). Writing raw LBS dnmsg to that file
# would crash lora_gateway because the key set differs from txpk format.
downlink_file = "/home/pi/lora_gateway/lbs_downlink.txt"

# Source filter: leave empty [] to forward packets from all LoRaWAN devices.
# To restrict, list DevAddr values in hex: ["0x26011721", "0x26011722"]
source_list = []
