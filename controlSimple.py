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
            try:
                self.updateGlobalStateWithData()
            except KeyError:
                print('No Messages')
            self.pushStateToLoggingQueue()
            time.sleep(self.vehicleState.parameters.Ts)
        self.stop()
        print "Control Stopped"


    def parseMessage(self,msg):
        ID = msg.content['ID']
        self.stateVehicles[ID] = copy.deepcopy(BasicVehicleState())
        self.stateVehicles[ID].ID = ID
        self.stateVehicles[ID].position = msg.content['position']
        self.stateVehicles[ID].velocity = msg.content['velocity']
        self.stateVehicles[ID].timestamp = msg.sendTime
        #print(self.stateVehicles)
        
    def updateGlobalStateWithData(self):
        #print(msg.content)
        ID = self.vehicleState.ID
        self.vehicleState.position = self.stateVehicles[ID].position
        self.vehicleState.velocity = self.stateVehicles[ID].velocity
        self.vehicleState.timestamp = self.stateVehicles[ID].timestamp
        #print(self.vehicleState.position)

    def pushStateToLoggingQueue(self):
        msg = Message()
        msg.type = "UAV_LOG"
        msg.sendTime = time.time()
        msg.content = {}
        msg.content['thisState'] = copy.deepcopy(self.vehicleState)
        msg.content['stateVehicles'] = copy.deepcopy(self.stateVehicles)
        self.loggingQueue.put(msg)
        
        
