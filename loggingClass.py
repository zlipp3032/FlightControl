import socket
import threading
from rigidBodyState import *
import collections
import logging
import os
import time
import mutil
import mutilNeighSet
import Queue
from datetime import datetime


class Logging(threading.Thread):
    def __init__(self,logQueue,logPath,n,startTime):
        threading.Thread.__init__(self)
        self.logQueue=logQueue
        self.stoprequest = threading.Event()
        self.startTime = startTime
        self.file = open(os.path.join(logPath, self.startTime.strftime('%Y_%m_%d__%H_%M_%S_log.csv')), 'w')
        headerString=''
        headerString+='Time, RelTime,'
        self.expectedMAVs = n
        for i in range(1,n+1):
            headerString+=(mutil.vsToCSVHeaders())
            if(i!=n):
                headerString+=','
        headerString+='\n'

    def stop(self):
        self.stoprequest.set()
        print "Stop flag set - Log"

    def run(self):
        while( not self.stoprequest.is_set()):
            while( not self.logQueue.empty()):
#                if(self.logQueue.qsize()>5):
#                    print "Log Queue Size: " + str(self.logQueue.qsize())
                try:
                    msg = self.logQueue.get(True, 0.5)
                    self.logMessage(msg)
#                    print msg.content
                    self.logQueue.task_done()
                except Queue.Empty:
                    thread.sleep(0.001)
                    break # no more messages
        self.file.flush()
        os.fsync(self.file.fileno())
        self.file.close()
        print "Log Stopped"

    def logMessage(self, msg):
        outString = ''
        RigidBodies = msg.content['RigidBodies']
        thisBodyState = msg.content['thisBodyState']
        outString+=str(datetime.now()) + ','
        outString+=str((datetime.now() - thisBodyState.startTime).total_seconds()) + ',' #relative time
	outString+=mutil.vsToCSV(thisBodyState)
		#        for i in range(1,thisBodyState.parameters.expectedMAVs+1):
        #print RigidBodies
	#print thisBodyState
	for i in range(1,thisBodyState.parameters.expectedMAVs+1): #may need to add 1 to the for loop input argument when dealing with more agents
            try:
                if(True):
                    outString+=mutilNeighSet.vsToCSV(RigidBodies[i])
		    #print 'thisGuy'
                else:
                    outString+=mutil.vsToCSV(thisBodyState)
		    #print 'No This Guy'
            except KeyError:
                print "Attempted to log nonexistent vehicle: " + str(i)
                outString += str(i) + ','
            if(i!=thisBodyState.parameters.expectedMAVs):
                outString+=','
	#print outString
        self.file.write(outString)
        self.file.write('\n')

