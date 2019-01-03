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

#controlMode = VehicleMode("STABILIZE")

class Controller(threading.Thread):

    def __init__(self,loggingQueue,receiveQueue,defaultParams,startTime,vehicle):
        threading.Thread.__init__(self)
        self.loggingQueue = loggingQueue
        self.receiveQueue = receiveQueue
        self.stateVehicles = {}
        #self.vehicleState.parameters = defaultParams
        self.stoprequest = threading.Event()
        self.startTime = startTime
        self.vehicleState = FullVehicleState()
        self.vehicleState.ID = 1 # !!!!!!!!!!IMPORTANT NOTE: Be sure to update this value for each vehicle!!!!!!!!
        self.vehicleState.startTime = datetime.now()
        self.vehicleState.parameters = defaultParams
        self.vehicle = vehicle
        self.lastGCSContact = -1
        self.prepTakeoff()

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
            # Need to put a GPS conditional here to ensure we have position data
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
        self.vehicleState.leader['qgx'] = self.vehicleState.hover['x']
        self.vehicleState.leader['qgy'] = self.vehicleState.hover['y']
        self.vehicleState.leader['qgz'] = self.vehicleState.hover['z']
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
                self.getHoverData()
        else:
            if(self.vehicleState.parameters.config['isHovering']):
                if(not self.checkAbort()):
                    self.hover()
            else:
                if(not self.checkAbort()):
                    self.landing()


    def prepTakeoff(self):
        #self.vehicle.mode = VehicleMode('STABILIZE')
        print("Arming motors")
        #self.vehicle.channels.overrides = {'3': 1000}
        time.sleep(2)
        #self.vehicle.armed = True

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

    def computeLeaderVelocity(self,qg):
        qg_prev = np.array([[self.vehicleState.leader['qgx_prev']],[self.vehicleState.leader['qgy_prev']],[self.vehicleState.leader['qgz_prev']]])
        pg_prev = np.array([[self.vehicleState.leader['pgx_prev']],[self.vehicleState.leader['pgy_prev']],[self.vehicleState.leader['pgz_prev']]])
        if(not self.vehicleState.parameters.config['hoverVel']):
            pg = ((1 - self.vehicleState.parameters.gains['leadVelGain'])/self.vehicleState.parameters.Ts)*(qg - qg_prev) + self.vehicleState.parameters.gains['leadVelGain']*pg_prev
            print('goodbye')
        else:
            pg =  np.array([[self.vehicleState.leader['pgx']],[self.vehicleState.leader['pgy']],[self.vehicleState.leader['pgz']]])
        self.vehicleState.leader['qgx_prev'] = self.vehicleState.leader['qgx']
        self.vehicleState.leader['qgy_prev'] = self.vehicleState.leader['qgy']
        self.vehicleState.leader['qgz_prev'] = self.vehicleState.leader['qgz']
        self.vehicleState.leader['pgx_prev'] = self.vehicleState.leader['pgx']
        self.vehicleState.leader['pgy_prev'] = self.vehicleState.leader['pgy']
        self.vehicleState.leader['pgz_prev'] = self.vehicleState.leader['pgz']
        self.vehicleState.leader['pgx'] = copy.copy(pg[0,0])
        self.vehicleState.leader['pgy'] = copy.copy(pg[1,0])
        self.vehicleState.leader['pgz'] = copy.copy(pg[2,0])
        return pg
             
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
        self.vehicleState.attitude['yaw'] = msg.content['attitude']['yaw']
        #self.vehicleState.attitude['roll'] = self.vehicle.attitude.roll
        #self.vehicleState.attitude['pitch'] = self.vehicle.attitude.pitch
        #self.vehicleState.channels = self.vehicle.channels
        #self.vehicleSate.droneState['batt_volt'] = self.vehicle.battery.voltage
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


    def getHoverData(self):
        self.vehicleState.parameters.config['hoverVel'] = True
        self.vehicleState.hover = self.vehicleState.position
        self.vehicleState.leader['pgx'] = 0
        self.vehicleState.leader['pgy'] = 0
        self.vehicleState.leader['pgz'] = 0

    def computeControl(self):
        gains = self.vehicleState.parameters.gains
        Ts = self.vehicleState.parameters.Ts
        qi = np.array([[self.vehicleState.position['x']],[self.vehicleState.position['y']],[self.vehicleState.position['z']]])
        pi = np.array([[self.vehicleState.velocity['vx']],[self.vehicleState.velocity['vy']],[self.vehicleState.velocity['vz']]])
        qg =  np.array([[self.vehicleState.leader['qgx']],[self.vehicleState.leader['qgy']],[self.vehicleState.leader['qgz']]])
        pg = self.computeLeaderVelocity(qg)
        intPrep = np.dot(self.vehicleState.parameters.Ts,(qg - qi))
        kp = np.array([[gains['kpx'], 0, 0], [0, gains['kpy'], 0], [ 0, 0, gains['kpz']]])
        kd = np.array([[gains['kdx'], 0, 0], [0, gains['kdy'], 0], [ 0, 0, gains['kdz']]])
        ki = np.array([[gains['kix'], 0, 0], [0, gains['kiy'], 0], [ 0, 0, gains['kiz']]])
        # Integrate the position error
        accPosPrev = np.array([[self.vehicleState.accumulator['intXPosError']],[self.vehicleState.accumulator['intYPosError']],[self.vehicleState.accumulator['intZPosError']]])
        accPosError = self.antiWindupVec(qg,np.array([[-10],[-10],[-10]]),np.array([[-10],[-10],[-10]]),accPosPrev,intPrep)
        # Compute the control (note the negative feedback gives us the following deltas)
        dq = qg - qi
        dp = pg - pi
        uk = np.dot(kp,dq) + np.dot(kd,dp) + np.dot(ki,accPosError)
        # Estimate the desired velocity
        pkp = np.array([[self.vehicleState.controlState['vx_des']],[self.vehicleState.controlState['vy_des']],[self.vehicleState.controlState['vz_des']]])
        ukp = np.array([[self.vehicleState.controlState['ux_des']],[self.vehicleState.controlState['uy_des']],[self.vehicleState.controlState['uz_des']]])
        temp1 = (1 - Ts*gains['velGain'])
        temp2 = Ts*temp1
        pk = np.dot(temp1,pkp) + np.dot(temp2,ukp)
        self.updateControlState(pi,pk,uk,accPosError,Ts)
        #print(accPosError)
        

    def updateControlState(self,pi,pk,uk,accPosError,Ts):
        config = self.vehicleState.parameters.config
        gains = self.vehicleState.parameters.gains
        self.vehicleState.accumulator['intXPosError'] = accPosError[0,0]
        self.vehicleState.accumulator['intYPosError'] = accPosError[1,0]
        self.vehicleState.accumulator['intZPosError'] = accPosError[2,0]
        self.vehicleState.controlState['vx_des'] = pk[0,0]
        self.vehicleState.controlState['vy_des'] = pk[1,0]
        self.vehicleState.controlState['vz_des'] = pk[2,0]
        self.vehicleState.controlState['ux_des'] = uk[0,0]
        self.vehicleState.controlState['uy_des'] = uk[1,0]
        self.vehicleState.controlState['uz_des'] = uk[2,0]
        # Update the integrator for velocity term
        accVelError = (self.vehicleState.velocity['vz'] - self.vehicleState.controlState['vz_des'])*Ts
        self.vehicleState.accumulator['intZVelError'] = self.antiWindup(pk[2,0],-10,10,self.vehicleState.accumulator['intZVelError'],accVelError)
        # Compute the attitude and thrust commands
        self.vehicleState.controlState['thrust'] = config['quadMass']*(config['grav'] - uk[2,0] + gains['kw_vel']*(pi[2,0] - pk[2,0]) + gains['intGain']*self.vehicleState.accumulator['intZVelError'])/(np.cos(self.vehicleState.attitude['roll'])*np.cos(self.vehicleState.attitude['pitch']))
        self.vehicleState.controlState['pitch'] = np.arctan(((uk[0,0] - gains['ku_vel']*(pi[0,0] - pk[0,0]))*np.cos(self.vehicleState.attitude['yaw']) + (uk[1,0] - gains['kv_vel']*(pi[1,0] - pk[1,0]))*np.sin(self.vehicleState.attitude['yaw']))/(-config['grav'] + uk[2,0] - gains['kw_vel']*(pi[2,0] - pk[2,0]) - gains['intGain']*self.vehicleState.accumulator['intZVelError']))
        self.vehicleState.controlState['roll'] = np.arctan(((uk[0,0] - gains['ku_vel']*(pi[0,0] - pk[0,0]))*np.cos(self.vehicleState.controlState['pitch'])*np.sin(self.vehicleState.attitude['yaw']) - (uk[1,0] - gains['kv_vel']*(pi[1,0] - pk[1,0]))*np.cos(self.vehicleState.controlState['pitch'])*np.cos(self.vehicleState.attitude['yaw']))/(-config['grav'] + uk[2,0] - gains['kw_vel']*(pi[2,0] - pk[2,0]) - gains['intGain']*self.vehicleState.accumulator['intZVelError']))
        self.scaleAndSendControl()

    def scaleAndSendControl(self):
        config = self.vehicleState.parameters.config
        y = self.vehicleState.controlState['thrust']
        #y = self.vehicle.battery.voltage
        x = 1
        ROLL = 1500 + (500/config['rollLimit'])*self.vehicleState.controlState['roll']
        PITCH = 1500 + (500/config['pitchLimit'])*self.vehicleState.controlState['pitch']
        THROTTLE = 1000*(1 + (y - 4.869)/(0.2067*(x*x - 23.71)))
        # Saturate the commands to keep in range of the spec'd input values
        self.vehicleState.controlState['roll_PWM'] = self.saturate(ROLL,1000,2000)
        self.vehicleState.controlState['pitch_PWM'] = self.saturate(PITCH,1000,2000)
        self.vehicleState.controlState['throttle_PWM'] = self.saturate(THROTTLE,1000,2000)
        #self.vehicle.channels.overrides = {'1': self.vehicleState.controlState['roll_PWM'], '2': self.vehicleState.controlState['pitch_PWM'], '3':self.vehicleState.controlState['throttle_PWM']}
        
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

    def antiWindupVec(self,value,lowLimit,highLimit,accumulator,toAdd):
        for i in range(0,3):
            if(value[i,0]>=highLimit[i,0]): #Saturation and anti-windup
                if(toAdd[i,0] < 0):
                    accumulator[i,0] = accumulator[i,0] + toAdd[i,0]
            elif(value[i,0]<=lowLimit[i,0]):
                if(toAdd[i,0] > 0):
                    accumulator[i,0] = accumulator[i,0] + toAdd[i,0]		
            else:
                accumulator[i,0] = accumulator[i,0] + toAdd[i,0]
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
