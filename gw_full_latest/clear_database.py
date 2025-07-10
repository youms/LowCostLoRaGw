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

# Script to clear all data from MongoDB ReceivedData collection
# Following the existing code patterns from MongoDB.py

import pymongo
from pymongo import MongoClient

def clear_all_data():
	"""
	Clear all documents from ReceivedData collection
	Following existing MongoDB.py patterns for connection and error handling
	"""
	try:
		# client MongoDB (following existing pattern)
		client = MongoClient()

		# open database messages (following existing pattern)
		db = client.messages
		
		# check if collection exists and isn't empty
		if db.ReceivedData.count() > 0:
			print("MongoDB: clearing all data from ReceivedData collection...")
			
			# remove all documents from ReceivedData collection
			# equivalent to: db.ReceivedData.remove({})
			result = db.ReceivedData.remove({})
			
			print("MongoDB: all documents deleted")
			print("MongoDB: collection cleared successfully")
		else:
			print("MongoDB: ReceivedData collection is already empty")
			
	except Exception as e:
		print("MongoDB: error while clearing database: " + str(e))

def main():
	"""
	Main function to execute the database clearing operation
	"""
	print("MongoDB: starting database clear operation...")
	clear_all_data()
	print("MongoDB: clear operation completed")

if __name__ == "__main__":
	main()

