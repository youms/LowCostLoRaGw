#!/bin/bash
# MongoDB Lock File Fix Script
# Fixes the "old lock file" error preventing MongoDB from starting

echo "MongoDB Lock File Fix Script"
echo "============================="

# Stop MongoDB service
echo "1. Stopping MongoDB service..."
sudo systemctl stop mongodb

# Remove the stale lock file
echo "2. Removing stale lock file..."
sudo rm -f /var/lib/mongodb/mongod.lock

# Check and fix permissions
echo "3. Fixing permissions..."
sudo chown -R mongodb:mongodb /var/lib/mongodb
sudo chmod 755 /var/lib/mongodb

# Start MongoDB service
echo "4. Starting MongoDB service..."
sudo systemctl start mongodb

# Wait a moment for service to start
sleep 3

# Check status
echo "5. Checking MongoDB status..."
sudo systemctl status mongodb --no-pager

# Test connection
echo "6. Testing MongoDB connection..."
if mongo --eval "db.runCommand({ping: 1})" > /dev/null 2>&1; then
    echo "✓ MongoDB is running and accessible"
    echo "✓ Gateway can now connect to MongoDB"
else
    echo "✗ MongoDB connection test failed"
    echo "Checking logs..."
    sudo tail -5 /var/log/mongodb/mongod.log
fi

echo ""
echo "If MongoDB is now running, restart your LoRa gateway:"
echo "sudo ./lora_gateway --bw 125 --cr 5 --sf 12 | python post_processing_gw.py | python log_gw.py"
