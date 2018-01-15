# Main flight program - send commands to Pixhawk via connect method
# Zachary Lippay

from dronekit import connect, VehicleMode, Vehicle
import sys
import time
import argparse
import socket
import os
import Queue
import threading
import receiveClass_v5,loggingClass, controlSimple_v3
from defaultParams import *
#import receive, control, log
from rigidBodyState import *
import numpy as np
from datetime import datetime

#Define the Agent on the network
localIP = '192.168.0.6'
Port = 5001
myAddr = (localIP,Port)
bufferLength = 1000
expectedMAVs = 1
logPath = 'C:\Users\Zack\Desktop\MastersThesisUAS\SOLOCode\ComTests\ControlCom\junk'
#broadcastIP = '192.168.0.255' #Will be necessary when we introduce multiple agents
#transmitAddr = (broadcastIP,Port)
d = 500

startTime = datetime.now()

#Create Message Queues
receiveQueue = Queue.Queue()
logQueue = Queue.Queue()

receiveThread = receiveClass_v5.Receiver(receiveQueue,localIP,Port,bufferLength,d)
logThread = loggingClass.Logging(logQueue,logPath,expectedMAVs,startTime)
controlThread = controlSimple_v3.Control(logQueue,receiveQueue,startTime,localIP,defaultParams)

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
