#!/usr/bin/env python
#-------------------------------------------------------------------------------
# Copyright 2016 Congduc Pham, University of Pau, France.
# 
# Congduc.Pham@univ-pau.fr
#
# This file is part of the low-cost LoRa gateway developped at University of Pau
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with the program.  If not, see <http://www.gnu.org/licenses/>.
#-------------------------------------------------------------------------------

# Script to export all data from MongoDB ReceivedData collection to CSV
# Following the existing code patterns from MongoDB.py and clear_database.py

import pymongo
from pymongo import MongoClient
import datetime
import csv
import sys
import os

def export_to_csv(output_path="."):
	"""
	Export all documents from ReceivedData collection to CSV
	Following existing MongoDB.py patterns for connection and error handling
	"""
	try:
		# client MongoDB (following existing pattern)
		client = MongoClient()

		# open database messages (following existing pattern)
		db = client.messages
		
		# check if collection exists and isn't empty
		if db.ReceivedData.count() > 0:
			print("MongoDB: exporting data from ReceivedData collection...")
			
			# Generate datetime string: YYYYMMDD_HHMMSS format (matching PHP script)
			now = datetime.datetime.now()
			datetime_str = now.strftime('%Y%m%d_%H%M%S')
			
			# Create filename with datetime: received_data_YYYYMMDD_HHMMSS.csv
			filename = "received_data_" + datetime_str + ".csv"
			filepath = os.path.join(output_path, filename)
			
			print("MongoDB: creating CSV file: " + filename)
			
			# Get all documents sorted by time (matching PHP script query)
			cursor = db.ReceivedData.find().sort("time", 1)
			
			# Open CSV file for writing
			with open(filepath, 'w') as csvfile:
				# CSV header (matching PHP script columns)
				csvfile.write("type;gateway_eui;node_eui;seq;snr;rssi;len;toa;cr;datarate;time;data\n")
				
				# Export each document
				for doc in cursor:
					# Handle potential missing fields with defaults
					type_val = str(doc.get('type', ''))
					gateway_eui = str(doc.get('gateway_eui', ''))
					node_eui = str(doc.get('node_eui', ''))
					seq = str(doc.get('seq', ''))
					snr = str(doc.get('snr', ''))
					rssi = str(doc.get('rssi', ''))
					len_val = str(doc.get('len', ''))
					toa = str(doc.get('toa', ''))
					cr = str(doc.get('cr', ''))
					datarate = str(doc.get('datarate', ''))
					
					# Format time (matching PHP script format: Y-m-d H:i:s)
					if 'time' in doc and doc['time']:
						time_str = doc['time'].strftime('%Y-%m-%d %H:%M:%S')
					else:
						time_str = ''
					
					data = str(doc.get('data', ''))
					
					# Write CSV row (matching PHP script format with semicolon separator)
					row = ";".join([
						type_val,
						gateway_eui,
						node_eui,
						seq,
						snr,
						rssi,
						len_val,
						toa,
						cr,
						datarate,
						time_str,
						data
					])
					csvfile.write(row + "\n")
			
			print("MongoDB: export completed successfully")
			print("MongoDB: file saved as: " + filepath)
			
		else:
			print("MongoDB: ReceivedData collection is empty - no data to export")
			
	except Exception as e:
		print("MongoDB: error while exporting to CSV: " + str(e))

def main():
	"""
	Main function to execute the CSV export operation
	Usage: python export_csv.py [output_directory]
	"""
	# Default output path is current directory
	output_path = "."
	
	# Check if output directory is provided as argument
	if len(sys.argv) > 1:
		output_path = sys.argv[1]
		
		# Check if directory exists
		if not os.path.exists(output_path):
			print("Error: Directory '" + output_path + "' does not exist")
			sys.exit(1)
		
		if not os.path.isdir(output_path):
			print("Error: '" + output_path + "' is not a directory")
			sys.exit(1)
	
	print("MongoDB: starting CSV export operation...")
	print("MongoDB: output directory: " + os.path.abspath(output_path))
	
	export_to_csv(output_path)
	
	print("MongoDB: export operation completed")

if __name__ == "__main__":
	main()
