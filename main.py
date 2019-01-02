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

defaultParams = defaultConfig.getParams()


#create message queues
loggingQueue = multiprocessing.Queue()
receiveQueue = multiprocessing.Queue()

startTime=datetime.now()

receiveThread = receive.Receiver(receiveQueue,AdHocIP,peerReadPort,bufferlength,defaultParams.config['ignoreSelfPackets'])

fileSuffix =  '_v' + str(int(ThisVehicle))
logThread = log.Logger(loggingQueue,logPath,defaultParams.expectedMAVs,startTime,fileSuffix)

controlThread = controlSimple.Controller(loggingQueue,receiveQueue,defaultParams,startTime)


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
