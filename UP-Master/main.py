from dronekit import connect, VehicleMode
import sys
import time
import argparse
import socket
import os
import Queue
import threading
import receiveClass_v5,loggingClass, controlSimple_v2
from defaultParams import *
#import receive, control, log
from rigidBodyState import *
import numpy as np
from datetime import datetime

#Define the Agent on the network
localIP = '192.168.2.1'
Port = 5001
myAddr = (localIP,Port)
bufferLength = 1000
expectedMAVs = 2
logPath = '/home/zsl/Desktop/multiFlightProgram/Data'
#broadcastIP = '192.168.0.255' #Will be necessary when we introduce multiple agents
#transmitAddr = (broadcastIP,Port)
d = 500

startTime = datetime.now()

#Create Message Queues
receiveQueue = Queue.Queue()
logQueue = Queue.Queue()

# Connect to the Pixhawk 2.0 Cube
#         - It should be noted that the connection string will need to be
#           modified pending the connect type
#             + Connect string used for wireless connection to SOLO
#                 'udpin:0.0.0.0:14550'
#             + Connect string used for serial2 connection to SOLO via UP-Board
#                 'dev/ttyAMA0' (Note the baudrate needs to be adjusted,
#                 baud = 57600 for this connectino string)
# Setup option parsing to get connection string
parser = argparse.ArgumentParser(description='Commands vehicle using simple.')
parser.add_argument('--connect',
                      help="Vehicle connect target string. If not specified, SITL automatic")
args = parser.parse_args()
connection_string = args.connect
sitl = None
# Start SITL if no connection string specifiied
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()
# Connect to UDP endpoint (and wait for defualt attributes to accumulate)
print '\nConnecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready = True, baud=115200, rate=10) #230400 or 115000 or 250000)

receiveThread = receiveClass_v5.Receiver(receiveQueue,localIP,Port,bufferLength,d)
logThread = loggingClass.Logging(logQueue,logPath,expectedMAVs,startTime)
controlThread = controlSimple_v2.Control(logQueue,receiveQueue,startTime,localIP,defaultParams,vehicle)

threads = []
threads.append(receiveThread)
threads.append(logThread)
threads.append(controlThread)

receiveThread.start()
logThread.start()
controlThread.start()

def hasLiveThreads(threads):
    return True in [t.isAlive() for t in threads]

while hasLiveThreads(threads):
    try:
        [t.join(1) for t in threads
        if t is not None and t.isAlive()]

    except KeyboardInterrupt:
        print "Killing Threads"
        for t in threads:
            t.stop()

print "Exiting mainProgram"

