# This is the controller class that will be used to compute the control values, override the channels and send data to the logging queue.
# Zachary Lippay
#
from dronekit import connect, VehicleMode, Vehicle
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
    def __init__(self,logQueue,receiveQueue,startTime,localIP,defaultParams,vehicle):
        threading.Thread.__init__(self)
        self.isRunning=True
        self.logQueue = logQueue
        self.receiveQueue = receiveQueue
        self.RigidBodies = {}
        self.rigidBodyState = RigidBodyState()
        self.rigidBodyState.startTime = datetime.now()
        self.counter = 0
        self.stoprequest = threading.Event()
        self.vehicle = vehicle
        self.startTime = startTime
        self.rigidBodyState.ID = 1#BE SURE TO UPDATE THIS WHEN IT COMES TIME FOR MULTIAGENT TESTING!!!!!
        self.rigidBodyState.parameters = defaultParams
        self.rigidBodyState.lastGPSContact = -1
        self.rigidBodyState.parameters.isTakeOff = False
        self.TargetAltitude = self.rigidBodyState.parameters.TargetAltitude
        
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
            self.RigidBodies[self.rigidBodyState.ID] = self.rigidBodyState
#            print self.rigidBodyState.leader
            # Implement a failsafe to ensure the computer is recceving data from the computer before it computes the control
            if(not self.rigidBodyState.isGPS):
                self.checkGPS()
            if(self.rigidBodyState.isGPS and True):
                if(not self.checkAbort()):
                    if(self.rigidBodyState.parameters.isTakeOff):
                        self.computeControl()
                    else:
                        self.sendTakeOff(self.TargetAltitude)
#            self.pushStatetoTxQueue()
            self.pushStatetoLoggingQueue()
            time.sleep(self.rigidBodyState.parameters.Ts)
        self.stop()
        self.releaseControl()
        self.vehicle.close()
        print "Control Stopped"


        
    def updateGlobalStatewithData(self,msg):
        if (msg.type == 'UAV'):
            self.decodeMessage(msg)
        else:
            self.decodeMessage(msg)


        
    def decodeMessage(self,msg):
        if(msg.content.ID>0):
            ID = int(msg.content.ID)# + self.rigidBodyState.ID
            self.RigidBodies[ID] = msg.content.position
            self.RigidBodies[ID] = msg.content.velocity
            self.rigidBodyState.timeout.peerLastRx[ID] = datetime.now()
        else:
            ID = int(msg.content.ID) + self.rigidBodyState.ID
            self.RigidBodies[ID] = msg.content.leader
            self.rigidBodyState.timeout.peerLastRx[int(msg.content.ID)] = datetime.now()

        
            
    def getRigidBodyState(self,msg):
        self.rigidBodyState.timeout.peerLastRx[msg.content.ID] = datetime.now() #This might cause issues the way this is currently set up...need to fix...
#        self.rigidBodyState.position = self.RigidBodies[ID]
        if(msg.content.ID>0):
            self.rigidBodyState.position = msg.content.position
            self.rigidBodyState.velocity = msg.content.velocity
            self.rigidBodyState.attitude = msg.content.attitude
        else:
            self.rigidBodyState.leader = msg.content.leader
#        print self.rigidBodyState.position
        self.rigidBodyState.time = datetime.now()
        self.counter+=1
        #self.rigidBodyState.channels = dict(zip(self.vehicle.channels.keys(),elf.vehicle.channels.values())) #Necessary to be able to erialize it
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
        if(datetime.now() - timedelta(seconds = self.rigidBodyState.lastGPSContact) < datetime.now() - timedelta(seconds = self.rigidBodyState.parameters.GPSTimeout)):
            print "GPS Timeout"
            self.rigidBodyState.timeout.GPSTimeoutTime = time.time()
            didTimeout = True
        return didTimeout


    
  #  def pushStatetoTxQueue(self):
  #      msg = Message()
  #      msg.type = 'UAV'
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


        
    def sendTakeOff(self,TargetAltitude):
        print 'Basic pre-arm checks'
        self.vehicle.mode = VehicleMode("ALT_HOLD")
        while not self.vehicle.is_armable:
            print 'Waiting for vehicle to initialize...'
            time.sleep(1)
        print 'Arming Motors'
        self.vehicle.armed = True
        time.sleep(5) #Wait Five Seconds before taking off
#        self.vehicle.simple_takeoff(TargtAltitude)
        #Will need to use channel overrides and likely PD controller to take off vertically without GPS
        if(self.rigidodyState.position.z>=TargetAltitude*0.95):
             self.rigidBodyState.parameters.isTakeOff = True
             time.sleep(1)
        else:
           self.rigidBodyState.control.ux = self.rigidBodyState.parameters.kp*(self.rigidBodyState.position.x - self.rigidBodyState.initPos.x) + self.rigidBodyState.parameters.kd*(self.rigidBodyState.velocity.vx - 0)
           self.rigidBodyState.control.uy = self.rigidBodyState.parameters.kp*(self.rigidBodyState.position.y - self.rigidBodyState.initPos.y) + self.rigidBodyState.parameters.kd*(self.rigidBodyState.velocity.vy - 0)
           self.rigidBodyState.control.uz = self.rigidBodyState.parameters.kp*(self.rigidBodyState.position.z - TargetAltitude) + self.rigidBodyState.parameters.kd*(self.rigidBodyState.velocity.vz - 0)
           self.rotateToBodyFrame()
           self.scaleAndSendControl()
        

        
    def computeControl(self):
        self.rigidBodyState.control.ux = self.rigidBodyState.parameters.kp*(self.rigidBodyState.position.x - self.rigidBodyState.leader.qgx) + self.rigidBodyState.parameters.kd*(self.rigidBodyState.velocity.vx - self.rigidBodyState.leader.pgx)
        self.rigidBodyState.control.uy = self.rigidBodyState.parameters.kp*(self.rigidBodyState.position.y - self.rigidBodyState.leader.qgy) + self.rigidBodyState.parameters.kd*(self.rigidBodyState.velocity.vy - self.rigidBodyState.leader.pgy)
        self.rigidBodyState.control.uz = self.rigidBodyState.parameters.kp*(self.rigidBodyState.position.z - self.rigidBodyState.leader.qgz) + self.rigidBodyState.parameters.kd*(self.rigidBodyState.velocity.vz - self.rigidBodyState.leader.pgz)
#        print self.rigidBodyState.control
        #Implement channel override algorithm here
        self.rotateToBodyFrame()
        self.scaleAndSendControl()



    def rotateToBodyFrame(self):
        print "hello"
        
    def scaleAndSendControl(self):
        #Scale the compute control values to match the format used in vehicle.channel.overrides{}
        #Will need to rotate the control from the inertial frame to the body frame
        ROLL = 1
        PITCH = 2
        THROTTLE = 3 
        YAW = 1020
        # Saturate to keep commands in range of input values
        self.rigidBodyState.command.Roll = self.saturate(ROLL,1000,2000)
        self.rigidBodyState.command.Pitch = self.saturate(PITCH,1000,2000)
        self.rigidBodyState.command.Throttle = self.saturate(THROTTLE,1000,2000)
        self.rigidBodyState.command.Yaw = self.saturate(YAW,1000,2000)
        self.vehicle.channels.overrides = {'1': self.rigidBodyState.command.Roll,'2': self.rigidBodyState.command.Yaw,'3': self.rigidBodyState.command.Throttle,'4': self.rigidBodyState.command.Yaw}



        
    def saturate(self, value, minimum, maximum):
        out = max(value,minimum)
        out = min(out,maximum)
        return out


    
    #This function is used to clear commands being sent to the pixhawk.
    def releaseControl(self):
        self.vehicle.channels.overrides = {}
