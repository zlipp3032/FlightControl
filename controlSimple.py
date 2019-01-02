#from dronekit import connect,VehicleMode, Vehicle
import time
import logging
from vehicleState import *
import os
import Queue
import threading
import recordtype
import jsonpickle
import math as m
from datetime import datetime, timedelta
import numpy as np
import copy
from curtsies import Input
import defaultConfig

#controlMode = VehicleMode("Stabalize")

class Controller(threading.Thread):

    def __init__(self,loggingQueue,receiveQueue,defaultParams,startTime):
        threading.Thread.__init__(self)
        self.loggingQueue = loggingQueue
        self.receiveQueue = receiveQueue
        self.stateVehicles = {}
        #self.vehicleState.parameters = defaultParams
        self.stoprequest = threading.Event()
        self.startTime = startTime
        self.vehicleState = FullVehicleState()
        self.vehicleState.ID = 1
        self.vehicleState.startTime = datetime.now()
        self.vehicleState.parameters = defaultParams
        #self.config = self.vehicleState.parameters.config
        #self.gains = self.vehicleState.parameters.gains
        self.lastGCSContact = -1
        #self.vehicle = vehicle


    def stop(self):
        self.stoprequest.set()
        print "Stop flag set - Control"
    def run(self):
        while(not self.stoprequest.is_set()):
            loopStartTime = datetime.now()
            while(not self.stoprequest.is_set()):
                try:
                    msg = self.receiveQueue.get(False)
                    self.parseMessage(msg)
                    #print(msg)
                except Queue.Empty:
                    break
            if(not self.vehicleState.parameters.config['initPos']):
                self.vehicleState.parameters.config['initPos'] = self.setInitialPos()
                print('Getting Home Location')
            # This is where the switcher sequence will go
            if(self.vehicleState.parameters.config['isGPS'] and True):
                if(not self.checkAbort()):
                    self.switchFlightSequence()
            # Log the data and control the rate of the loop
            self.pushStateToLoggingQueue()
            time.sleep(self.vehicleState.parameters.Ts)
        self.stop()
        print "Control Stopped"


    def switchFlightSequence(self):
        arg = self.vehicleState.flightSeq
        flightSequence = {0: self.idleFunction, 1: self.takeoff, 2: self.hover, 3: self.flocking, 4: self.landing}
        Keyboard_Command_Handler = flightSequence.get(arg, lambda: 'Invalid Command')
        Keyboard_Command_Handler()

    def idleFunction(self):
        print("Waiting for Command")

    def hover(self):
        if(not self.checkAbort()):
            self.computeControl()
            print('Hovering')

    def flocking(self):
        if(not self.checkAbort()):
            self.computeControl()
            print('Flocking')

    def landing(self):
        if(not self.checkAbort()):
            desDest = self.vehicleState.position['z'] - self.vehicleState.initPos['z']
            self.vehicleState.leader['qgz'] = self.vehicleState.initPos['z']
            self.computeLandingVelocity(desDest)

    def takeoff(self):
        if(not self.vehicleState.parameters.config['isTakeoff']):
            if(not self.vehicleState.position['z'] <= -self.vehicleState.parameters.config['targetAltitude']*0.95):
                if(not self.checkAbort()):
                    desDest = self.vehicleState.position['z'] - self.vehicleState.leader['qgz']
                    self.computeTakeoffVelocity(desDest)
            else:
                print("Reached target altitude")
                self.vehicleState.parameters.config['isTakeoff'] = True
                self.vehicleState.parameters.config['isHovering'] = True
                self.getLeaderData()
        else:
            if(self.vehicleState.parameters.config['isHovering']):
                if(not self.checkAbort()):
                    self.hover()
            else:
                if(not self.checkAbort()):
                    self.landing()
    

    def computeTakeoffVelocity(self,desDest):
        if(abs(desDest) >= self.vehicleState.parameters.config['stoppingDistance']):
            self.vehicleState.leader['pgz'] = (self.vehicleState.parameters.config['desiredSpeed']*desDest)/abs(desDest)
            if (not self.checkAbort()):
                self.computeControl()
                print("Taking off")
        else:
            self.vehicleState.leader['pgz'] = (self.vehicleState.parameters.config['desiredSpeed']*desDest)/self.vehicleState.parameters.config['stoppingDistance']
            if(not self.checkAbort()):
                self.computeControl()
                print("Approaching target altitude")

    def computeLandingVelocity(self,desDest):
        if(abs(desDest) >= self.vehicleState.parameters.config['stoppingDistance']):
            self.vehicleState.leader['pgz'] = (self.vehicleState.parameters.config['desiredSpeed']*desDest)/abs(desDest)
            if(not self.checkAbort()):
                self.computeControl()
                print("Landing")
        elif(self.vehicleState.position['z'] >= (self.vehicleState.initPos['z'] - 0.05)):
            #self.vehicle.channels.overrides = {'3': 1000}
            #self.vehicle.armed = False
            self.vehicleState.parameters.config['isTakeoff'] = False
            print("Vehicle landed")
        else:
            self.vehicleState.leader['pgz'] = (self.vehicleState.parameters.config['desiredSpeed']*desDest)/self.vehicleState.parameters.config['stoppingDistance']
            if(not self.checkAbort()):
                self.computeControl()
                print("Approaching landing")
                    
    def parseMessage(self,msg):
        ID = msg.content['ID']
        self.stateVehicles[ID] = copy.deepcopy(BasicVehicleState())
        self.stateVehicles[ID].ID = ID
        self.stateVehicles[ID].counter += 1
        self.stateVehicles[ID].position = msg.content['position']
        self.stateVehicles[ID].velocity = msg.content['velocity']
        self.stateVehicles[ID].timestamp = msg.sendTime
        if (ID ==  self.vehicleState.ID):
            self.updateGlobalStateWithData(ID,msg)
        #print(self.stateVehicles)
        
    def updateGlobalStateWithData(self,ID,msg):
        self.vehicleState.position = self.stateVehicles[ID].position
        self.vehicleState.velocity = self.stateVehicles[ID].velocity
        self.vehicleState.attitude = msg.content['attitude']
        self.vehicleState.timestamp = self.stateVehicles[ID].timestamp
        self.vehicleState.flightSeq = msg.content['flightSeq']
        # Update the leader states
        self.vehicleState.leader['qgx'] = msg.content['leader']['qgx']
        self.vehicleState.leader['qgy'] = msg.content['leader']['qgy']
        self.vehicleState.leader['qgz'] = msg.content['leader']['qgz']
        

    def pushStateToLoggingQueue(self):
        msg = Message()
        msg.type = "UAV_LOG"
        msg.sendTime = time.time()
        msg.content = {}
        msg.content['thisState'] = copy.deepcopy(self.vehicleState)
        msg.content['stateVehicles'] = copy.deepcopy(self.stateVehicles)
        self.loggingQueue.put(msg)

    def setInitialPos(self):
        initPosSet = False
        #print(self.vehicleState.position['z'])
        if(self.vehicleState.position['z']): #and self.vehicleState.position['z']<-0.115):
            self.vehicleState.initPos = self.vehicleState.position
            initPosSet = True
        return initPosSet

    def checkAbort(self):
        #if(self.checkTimeouts()):
        #    self.rigidBodyState.abortReason = "Timeout"
        #    self.rigdBodyState.RCLatch = True
        #    self.rigidBodyState.isGPS = False
        #    self.releaseControl()
        #    return True
        #! Check the proper flight mode
        #if(not self.vehicle.mode == 'STABILIZE'):
        #    self.releaseControl()
        #    return True
        return False

    def computeControl(self):
        qi = np.array([[self.vehicleState.position['x']],[self.vehicleState.position['y']],[self.vehicleState.position['z']]])
        pi = np.array([[self.vehicleState.velocity['vx']],[self.vehicleState.velocity['vy']],[self.vehicleState.velocity['vz']]])
        qg =  np.array([[self.vehicleState.leader['qgx']],[self.vehicleState.leader['qgy']],[self.vehicleState.leader['qgz']]])
        qg_prev = np.array([[self.vehicleState.leader['qgx_prev']],[self.vehicleState.leader['qgy_prev']],[self.vehicleState.leader['qgz_prev']]])
        pg_prev = np.array([[self.vehicleState.leader['pgx_prev']],[self.vehicleState.leader['pgy_prev']],[self.vehicleState.leader['pgz_prev']]])
        #print(qg_prev)
        if(self.vehicleState.leader['qgx_prev']):
            pg = ((1 - self.vehicleState.parameters.gains['leadVelGain'])/self.vehicleState.parameters.Ts)*(qg - qg_prev) + self.vehicleState.parameters.gains['leadVelGain']*pg_prev
            self.vehicleState.leader['pgx'] = pg[0,0]
            self.vehicleState.leader['pgy'] = pg[1,0]
            self.vehicleState.leader['pgz'] = pg[2,0]
            #print(pg)
        #except TypeError:
        #    print('No leader')
        self.vehicleState.leader['qgx_prev'] = self.vehicleState.leader['qgx']
        self.vehicleState.leader['qgy_prev'] = self.vehicleState.leader['qgy']
        self.vehicleState.leader['qgz_prev'] = self.vehicleState.leader['qgz']
        #print('hello')

    def antiWindup(self,value,lowLimit,highLimit,accumulator,toAdd):
        if(value>=highLimit): #Saturation and anti-windup
            if(toAdd < 0):
                accumulator = accumulator + toAdd
        elif(value<=lowLimit):
            if(toAdd > 0):
                accumulator = accumulator + toAdd		
        else:
            accumulator = accumulator + toAdd
        return accumulator

    def vectorNorm(self,vec):
        eta = np.sqrt(m.pow(vec[0,0],2) + m.pow(vec[1,0],2) + m.pow(vec[2,0],2))
        return eta

    def saturate(self, value, minimum, maximum):
        out = max(value,minimum)
        out = min(out,maximum)
        return out

    def releaseControl(self):
        self.vehicle.channels.overrides = {}
        print self.vehicle.channels.overrides
        print "Channels Cleared"

    def diffFunction(self,pos1,pos2):
        delta = pos2 - pos1
        return delta
