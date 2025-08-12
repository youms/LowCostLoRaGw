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

# Script to import CSV data into MongoDB ReceivedData collection
# Following the existing code patterns from MongoDB.py and clear_database.py

import pymongo
from pymongo import MongoClient
import datetime
import csv
import sys
import os
import json

def parse_datetime(datetime_str):
	"""
	Parse datetime string in format: YYYY-MM-DD HH:MM:SS
	Returns datetime object or None if parsing fails
	"""
	try:
		return datetime.datetime.strptime(datetime_str, '%Y-%m-%d %H:%M:%S')
	except (ValueError, TypeError):
		return None

def parse_numeric_field(value_str):
	"""
	Parse numeric field, return int if possible, otherwise return original string
	"""
	try:
		# Try to convert to int first
		if '.' not in value_str:
			return int(value_str)
		else:
			return float(value_str)
	except (ValueError, TypeError):
		return value_str

def import_from_csv(csv_file_path, clear_existing=False):
	"""
	Import data from CSV file into ReceivedData collection
	Following existing MongoDB.py patterns for connection and error handling
	"""
	try:
		# client MongoDB (following existing pattern)
		client = MongoClient()

		# open database messages (following existing pattern)
		db = client.messages
		
		# Check if CSV file exists
		if not os.path.exists(csv_file_path):
			print("MongoDB: error - CSV file not found: " + csv_file_path)
			return False
		
		print("MongoDB: importing data from CSV file: " + csv_file_path)
		
		# Clear existing data if requested
		if clear_existing:
			if db.ReceivedData.count() > 0:
				print("MongoDB: clearing existing data from ReceivedData collection...")
				db.ReceivedData.remove({})
				print("MongoDB: existing data cleared")
		
		# Expected CSV columns (matching export format)
		expected_columns = [
			'type', 'gateway_eui', 'node_eui', 'seq', 'snr', 'rssi', 
			'len', 'toa', 'cr', 'datarate', 'time', 'data'
		]
		
		imported_count = 0
		skipped_count = 0
		
		# Open and read CSV file
		with open(csv_file_path, 'r') as csvfile:
			# Read first line to check header
			first_line = csvfile.readline().strip()
			
			# Check if header matches expected format
			header_columns = first_line.split(';')
			
			if header_columns != expected_columns:
				print("MongoDB: warning - CSV header doesn't match expected format")
				print("MongoDB: expected: " + ';'.join(expected_columns))
				print("MongoDB: found:    " + ';'.join(header_columns))
				
				# Ask user if they want to continue
				response = raw_input("Continue with import? (y/n): ")
				if response.lower() != 'y':
					print("MongoDB: import cancelled by user")
					return False
			
			# Process each data row
			line_number = 1  # Start from 1 since we already read the header
			
			for line in csvfile:
				line_number += 1
				line = line.strip()
				
				# Skip empty lines
				if not line:
					continue
				
				# Split line by semicolon
				fields = line.split(';')
				
				# Check if we have the right number of fields
				if len(fields) != len(expected_columns):
					print("MongoDB: warning - line " + str(line_number) + " has " + str(len(fields)) + " fields, expected " + str(len(expected_columns)))
					skipped_count += 1
					continue
				
				try:
					# Parse datetime
					time_obj = parse_datetime(fields[10])  # time field
					if time_obj is None:
						print("MongoDB: warning - line " + str(line_number) + " has invalid time format: " + fields[10])
						skipped_count += 1
						continue
					
					# Create document (matching CloudMongoDB.py structure)
					doc = {
						"type": parse_numeric_field(fields[0]),
						"gateway_eui": fields[1],
						"node_eui": parse_numeric_field(fields[2]),
						"seq": parse_numeric_field(fields[3]),
						"snr": parse_numeric_field(fields[4]),
						"rssi": parse_numeric_field(fields[5]),
						"len": parse_numeric_field(fields[6]),
						"toa": parse_numeric_field(fields[7]),
						"cr": parse_numeric_field(fields[8]),
						"datarate": fields[9],
						"time": time_obj,
						"data": fields[11]
					}
					
					# Insert document into collection
					db.ReceivedData.insert_one(doc)
					imported_count += 1
					
					# Progress indicator every 100 records
					if imported_count % 100 == 0:
						print("MongoDB: imported " + str(imported_count) + " records...")
				
				except Exception as e:
					print("MongoDB: error processing line " + str(line_number) + ": " + str(e))
					skipped_count += 1
					continue
		
		print("MongoDB: import completed")
		print("MongoDB: " + str(imported_count) + " records imported successfully")
		if skipped_count > 0:
			print("MongoDB: " + str(skipped_count) + " records skipped due to errors")
		
		return True
		
	except Exception as e:
		print("MongoDB: error during CSV import: " + str(e))
		return False

def main():
	"""
	Main function to execute the CSV import operation
	Usage: python import_csv.py <csv_file> [--clear]
	"""
	if len(sys.argv) < 2:
		print("Usage: python import_csv.py <csv_file> [--clear]")
		print("")
		print("Arguments:")
		print("  csv_file    Path to the CSV file to import")
		print("  --clear     Optional: Clear existing data before import")
		print("")
		print("Examples:")
		print("  python import_csv.py received_data_20250810_143022.csv")
		print("  python import_csv.py /path/to/data.csv --clear")
		sys.exit(1)
	
	csv_file_path = sys.argv[1]
	clear_existing = False
	
	# Check for --clear flag
	if len(sys.argv) > 2 and sys.argv[2] == "--clear":
		clear_existing = True
	
	print("MongoDB: starting CSV import operation...")
	print("MongoDB: CSV file: " + csv_file_path)
	
	if clear_existing:
		print("MongoDB: existing data will be cleared before import")
		response = raw_input("Are you sure you want to clear existing data? (y/n): ")
		if response.lower() != 'y':
			print("MongoDB: import cancelled by user")
			sys.exit(1)
	
	success = import_from_csv(csv_file_path, clear_existing)
	
	if success:
		print("MongoDB: import operation completed successfully")
	else:
		print("MongoDB: import operation failed")
		sys.exit(1)

if __name__ == "__main__":
	main()
