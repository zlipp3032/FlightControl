from dronekit import connect, VehicleMode
import sys
import time
import socket
import os
import Queue
import multiprocessing	
import controlSimple, receive, log
import vehicleState
import defaultConfig

import numpy as np
import argparse
from datetime import datetime


#get enviromental variables
AdHocIP = '192.168.2.254'
peerReadPort = 5001
bufferlength = 1000
myAddr = (AdHocIP, peerReadPort)
logPath = '/Users/zlipp3032/Documents/MastersThesisUAS/FlightCode/No_Vehicle_v1/Data/'
broadcastIP= '192.168.2.255'
transmitAddress = (broadcastIP,peerReadPort)
ThisVehicle = 1
vehicle = 1

defaultParams = defaultConfig.getParams()


#create message queues
loggingQueue = multiprocessing.Queue()
receiveQueue = multiprocessing.Queue()

startTime=datetime.now()

# Connect to the Pixhawk 2.0 Cube
#         - It should be noted that the connection string will need to be
#           modified pending the connect type
#             + Connect string used for wireless connection to SOLO
#                 'udpin:0.0.0.0:14550'
#             + Connect string used for serial2 connection to SOLO via UP-Board
#                 'dev/ttyAMA0' (Note the baudrate needs to be adjusted,
#                 baud = 57600 for this connectino string)
# Setup option parsing to get connection string
#parser = argparse.ArgumentParser(description='Commands vehicle using simple.')
#parser.add_argument('--connect',
#                      help="Vehicle connect target string. If not specified, SITL automatic")
#args = parser.parse_args()
#connection_string = args.connect
#sitl = None
# Start SITL if no connection string specifiied
#if not connection_string:
#    import dronekit_sitl
#    sitl = dronekit_sitl.start_default()
#    connection_string = sitl.connection_string()
# Connect to UDP endpoint (and wait for defualt attributes to accumulate)
#print '\nConnecting to vehicle on: %s' % connection_string
#vehicle = connect(connection_string, wait_ready = True, baud=115200, rate=10) #230400 or 115000 or 250000)

receiveThread = receive.Receiver(receiveQueue,AdHocIP,peerReadPort,bufferlength,defaultParams.config['ignoreSelfPackets'])

fileSuffix =  '_v' + str(int(ThisVehicle))
logThread = log.Logger(loggingQueue,logPath,defaultParams.expectedMAVs,startTime,fileSuffix)

controlThread = controlSimple.Controller(loggingQueue,receiveQueue,defaultParams,startTime,vehicle)


threads = []
threads.append(controlThread)
threads.append(receiveThread)
threads.append(logThread)

receiveThread.start()
print 'Started Receive'

logThread.start()
print 'Started Logging'

controlThread.start()
print 'Started Control'


def hasLiveThreads(threads):
	return True in [t.is_alive() for t  in threads]


while hasLiveThreads(threads):
	try:
		[t.join(1) for t in threads
		if t is not None and t.is_alive()]
		
	except KeyboardInterrupt:
		print "killing threads"
		for t in threads:
			t.stop()
	
print "exiting Main"
