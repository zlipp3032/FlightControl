# Send received data to Logging Queue and compute the control inputs using a PD controller for following a leader trajectory.
# Zachary Lippay
import time
import logging
from rigidBodyState import *
import os
import Queue
import threading
import math as m
import numpy as np
from datetime import datetime, timedelta

logging.basicConfig(level=logging.WARNING)

class Control(threading.Thread):
    def __init__(self,logQueue,receiveQueue,startTime,localIP,defaultParams):
        threading.Thread.__init__(self)
        self.isRunning=True
        self.logQueue = logQueue
        self.receiveQueue = receiveQueue
        self.RigidBodies = {}
        self.rigidBodyState = RigidBodyState()
        self.rigidBodyState.startTime = datetime.now()
        self.counter = 0
        self.stoprequest = threading.Event()
        self.startTime = startTime
        self.rigidBodyState.ID = 1#BE SURE TO UPDATE THIS WHEN IT COMES TIME FOR MULTIAGENT TESTING!!!!!
        self.rigidBodyState.parameters = defaultParams
        self.lastGPSContact = -1

    def stop(self):
        self.stoprequest.set()
        print "Stop Flag Set - Control"

    def run(self):
        while(not self.stoprequest.is_set()):
            while(not self.receiveQueue.empty()):
                try:
                    msg = self.receiveQueue.get(False)
#                    print msg.content
                    self.updateGlobalStatewithData(msg)
#                    print self.rigidBodyState
                    self.getRigidBodyState(msg)
                    self.receiveQueue.task_done()
                except Queue.Empty:
                    break #no more messages
#            self.getRigidBodyState()
            # Implement a failsafe to ensure the computer is recceving data from the computer before it computes the control
            if(not self.rigidBodyState.isGPS):
                self.checkGPS()
            if(self.rigidBodyState.isGPS and True):
                if(not self.checkAbort()):
                    self.computeControl()
            
            self.RigidBodies[self.rigidBodyState.ID] = self.rigidBodyState
#            print self.rigidBodyState.leader
#            self.computeControl()
#            self.pushStatetoTxQueue()
            self.pushStatetoLoggingQueue()
            time.sleep(self.rigidBodyState.parameters.Ts)
        self.stop()
        print "Control Stopped"

    def updateGlobalStatewithData(self,msg):
        self.decodeMessage(msg)

    def decodeMessage(self,msg):
        if(msg.content.ID>0):
            ID = int(msg.content.ID)# + self.rigidBodyState.ID
            self.RigidBodies[ID] = msg.content
 #           self.rigidBodyState.position = msg.content.position
            self.rigidBodyState.timeout.peerLastRx[ID] = datetime.now()
        else:
            ID = int(msg.content.ID) + self.rigidBodyState.ID
            self.RigidBodies[ID] = msg.content
            self.rigidBodyState.timeout.peerLastRx[int(msg.content.ID)] = datetime.now()

        
            
    def getRigidBodyState(self,msg):
        self.rigidBodyState.timeout.peerLastRx[msg.content.ID] = datetime.now() #This might cause issues the way this is currently set up...need to fix...
#        self.rigidBodyState.position = self.RigidBodies[ID]
        if(msg.content.ID>0):
            self.rigidBodyState.position = msg.content.position
            self.rigidBodyState.velocity = msg.content.velocity
        else:
            self.rigidBodyState.leader = msg.content.leader
#        print self.rigidBodyState.position
        self.rigidBodyState.time = datetime.now()
        self.counter+=1
       # self.rigidBodyState.velocity = BackEuler()

    def checkAbort(self):
        if(self.checkTimeouts()):
            self.rigidBodyState.abortReason = "Timeout"
            self.rigdBodyState.RCLatch = True
            self.rigidBodyState.isGPS = False
            self.releaseControl()
#            self.landRigidBodySOLO()
            return True
        #print "Flight Mode: " + str(self.vehicle.mode)
#        if(not (self.vehicle.mode == ControlMode)):
#            self.rigidBodyState.RCLatch = True
#            self.rigidBodyState.isGPS = False
#            self.releaseControl()
#            self.landRigidBodySOLO()
#            return True

    def checkGPS(self):
        #Check Timeouts
        if(self.checkTimeouts()):
            print "No GPS - Timeout"
            self.rigidBodyState.RCLatch = True
            return False
        #Check configuration
        if(not self.rigidBodyState.parameters.isComplete):
           self.rigidBodyState.RCLatch = True
           return False
#        if(not (self.vehicle.mode == ControlMode)):
#           print "Wont't Engage - control Mode"
#           print "Current Mode: %s" % self.vehicle.mode
#           self.rigidBodyState.RCLatach = True
#           return False
        self.rigidBodyState.RCLatch = True # Set the Latch
        self.rigidBodyState.isGPS = True # GPS is being received
        return True

    # Check for a GPS timeout - If no GPS, control should not be engaged
    def checkTimeouts(self):
        didTimeout = False
        if(datetime.now() - timedelta(seconds = self.lastGPSContact) < datetime.now() - timedelta(seconds = self.rigidBodyState.parameters.GPSTimeout)):
            print "GPS Timeout"
            self.rigidBodyState.timeout.GPSTimeoutTime = time.time()
            didTimeout = True
        return didTimeout
        
  #  def pushStatetoTxQueue(self):
  #      msg = Message()
  #      msg.type = "UAV"
  #      msg.sendTime = datetime.now()
  #      msg.content = self.rigidBodyState
#        self.transmitQueue.put(msg)
  #      return msg

    def pushStatetoLoggingQueue(self):
        msg = Message()
        msg.type = "UAV_LOG"
        msg.sendTime = time.time()
        msg.content = {}
        msg.content['thisBodyState'] = self.rigidBodyState
        msg.content['RigidBodies'] = self.RigidBodies
#        print msg.content
        self.logQueue.put(msg)

    def computeControl(self):
        self.rigidBodyState.control.ux = -self.rigidBodyState.parameters.kp*(self.rigidBodyState.position.x - self.rigidBodyState.leader.qgx) - self.rigidBodyState.parameters.kd*(self.rigidBodyState.velocity.vx - self.rigidBodyState.leader.pgx)
        self.rigidBodyState.control.uy = -self.rigidBodyState.parameters.kp*(self.rigidBodyState.position.y - self.rigidBodyState.leader.qgy) - self.rigidBodyState.parameters.kd*(self.rigidBodyState.velocity.vy - self.rigidBodyState.leader.pgy)
        self.rigidBodyState.control.uz = -self.rigidBodyState.parameters.kp*(self.rigidBodyState.position.z - self.rigidBodyState.leader.qgz) - self.rigidBodyState.parameters.kd*(self.rigidBodyState.velocity.vz - self.rigidBodyState.leader.pgz)
#        print self.rigidBodyState.control
        #Implement channel override algorithm here
#        self.scaleControl()

#    def scaleControl(self):

#        self.sendCommand()

#    def sendCommand(self)
    
