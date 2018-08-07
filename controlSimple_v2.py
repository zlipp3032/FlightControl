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
from pymavlink import mavutil

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
        self.startTime = startTime
        self.rigidBodyState.ID = 1 # BE SURE TO UPDATE THIS WHEN IT COMES TIME FOR MULTIAGENT TESTING!!!!!
        self.rigidBodyState.parameters = defaultParams
        self.vehicle = vehicle
        self.lastGPSContact = -1
        self.prepTakeoff()
        self.scoobyDoo = MessageState()

    def stop(self):
        self.stoprequest.set()
        print "Stop Flag Set - Control"

    def run(self):
        while(not self.stoprequest.is_set()):
        	while(not self.receiveQueue.empty()):
                	try:
                            msg = self.receiveQueue.get(False)
                            self.decodeData(msg)
                            self.receiveQueue.task_done()
                	except Queue.Empty:
                    		break #no more messages
            	# Implement a failsafe to ensure the UP is recceving data from the computer before it computes the control
            	if(not self.rigidBodyState.isGPS):
                	self.checkGPS()
            	if(not self.rigidBodyState.parameters.InitPos):
                	self.rigidBodyState.parameters.InitPos = self.setInitialPos()
                	self.getLeaderData()
			self.setStaticFlockingLeader()
                	self.rigidBodyState.leader.qgz = -self.rigidBodyState.parameters.targetAltitude
                	print 'Setting Initial Position'
            	if(self.rigidBodyState.isGPS and True):
                	self.switch_Flight_Sequence()
            	#self.pushStatetoTxQueue()
            	self.pushStatetoLoggingQueue()
            	self.counter = self.counter + 1
            	time.sleep(self.rigidBodyState.parameters.Ts)
        self.releaseControl()
        self.stop()
        print "Control Stopped"


    def switch_Flight_Sequence(self):
        arg = self.rigidBodyState.flightSeq
        flightSequence = {0: self.idleFunction, 1: self.takeoff, 2: self.hover, 3: self.flocking, 4: self.landing}#, 5: self.prepTakeoff}
        Keyboard_Command_Handler = flightSequence.get(arg, lambda: 'Invalid Command')
        Keyboard_Command_Handler()

    def idleFunction(self):
        print "Waiting for Command"
        
    def landing(self):
        if( not self.checkAbort()):
            desDest = self.rigidBodyState.position.z - self.rigidBodyState.initPos.zo
            self.rigidBodyState.leader.qgz = self.rigidBodyState.initPos.zo
            self.computeLandingVelocity(desDest)          

    def flocking(self):
        print "flocking"
        if(not self.checkAbort()):
	    #self.hover()
            self.computeFlockingControl()

            
    def takeoff(self):
        if(not self.rigidBodyState.parameters.isTakeoff):
            if(not self.rigidBodyState.position.z <= -self.rigidBodyState.parameters.targetAltitude*0.95):
                if(not self.checkAbort()):
                    desDest =  self.rigidBodyState.position.z - self.rigidBodyState.leader.qgz # Fix this to match targetAltitude!!!
                    self.computeTakeoffVelocity(desDest)
            else:
                print "Reached Target Altitude"
                self.rigidBodyState.parameters.isTakeoff = True
                self.rigidBodyState.parameters.isHovering = True
                self.getLeaderData()
		self.setStaticFlockingLeader()
        else:
            if(self.rigidBodyState.parameters.isHovering):
                if(not self.checkAbort()):
                    self.hover()
            else:
                if(not self.checkAbort()):
                    self.landing()

    def hover(self):
        if(not self.checkAbort()):
            self.computePDControl()
            print "Hovering"
            
    def prepTakeoff(self):
        self.vehicle.mode = VehicleMode('STABILIZE')
        #print 'Basic Prearm Checks'
        print 'Arming Motors'
        #self.vehicle.channels.overrides = {'3':1000}
        time.sleep(2)
        #self.vehicle.armed = True

    def computeTakeoffVelocity(self,desDest):
        if(abs(desDest) >= self.rigidBodyState.parameters.stoppingDistance):
            self.rigidBodyState.leader.pgz = (self.rigidBodyState.parameters.desiredSpeed*desDest)/abs(desDest)
            if( not self.checkAbort()):
                self.computePDControl()
                print 'Taking Off'
        else:
            self.rigidBodyState.leader.pgz = (self.rigidBodyState.parameters.desiredSpeed*desDest)/self.rigidBodyState.parameters.stoppingDistance
            if(not self.checkAbort()):
                self.computePDControl()
                print 'Approaching Target Altitude'

    def computeLandingVelocity(self,desDest):
        if(abs(desDest) >= self.rigidBodyState.parameters.stoppingDistance):
            self.rigidBodyState.leader.pgz = (self.rigidBodyState.parameters.desiredSpeed*desDest)/abs(desDest)
            #print self.rigidBodyState.leader
            if(not self.checkAbort()):
                self.computePDControl()
                print 'Landing'
        elif(self.rigidBodyState.position.z >= (self.rigidBodyState.initPos.zo-0.05)):
            #self.vehicle.channels.overrides = {'3':1000}
            self.vehicle.armed = False
            self.rigidBodyState.parameters.isTakeoff = False
	    print "Vehicle Landed"
        else:
            self.rigidBodyState.leader.pgz = (self.rigidBodyState.parameters.desiredSpeed*desDest)/self.rigidBodyState.parameters.stoppingDistance
            if(not self.checkAbort()):
                self.computePDControl()
                print 'Approaching Landing'


#    def updateGlobalStatewithData(self):
#        #if(msg.content.ID>0):
#        ID = int(self.scoobyDoo.ID)# + self.rigidBodyState.ID
#       	if(ID==self.rigidBodyState.ID):
#            self.RigidBodies[ID] = self.rigidBodyState
#        else:
#            self.RigidBodies[ID] = self.scoobyDoo
 #           self.rigidBodyState.position = msg.content.position
 #           print self.RigidBodies
#        self.rigidBodyState.timeout.peerLastRx[ID] = datetime.now()


    def decodeData(self,msg):
        ID = int(msg.content[1])
        if(ID == self.rigidBodyState.ID):
        	self.rigidBodyState.position.x = float(msg.content[2])
        	self.rigidBodyState.position.y = float(msg.content[3])
        	self.rigidBodyState.position.z = float(msg.content[4])
        	self.rigidBodyState.velocity.vx = float(msg.content[5])
        	self.rigidBodyState.velocity.vy = float(msg.content[6])
        	self.rigidBodyState.velocity.vz = float(msg.content[7])
        	#self.rigidBodyState.attitude.roll = float(msg.content[8])
        	#self.rigidBodyState.attitude.pitch = float(msg.content[9])
        	self.rigidBodyState.attitude.yaw = float(msg.content[10])
            	self.rigidBodyState.flightSeq = int(msg.content[11])
            	self.rigidBodyState.attitude.roll = self.vehicle.attitude.roll
            	self.rigidBodyState.attitude.pitch = self.vehicle.attitude.pitch
            	self.rigidBodyState.channels = self.vehicle.channels
            	self.RigidBodies[int(ID)] = self.rigidBodyState
        else:
            	self.scoobyDoo.ID = ID
            	self.scoobyDoo.position.x = float(msg.content[2])
           	self.scoobyDoo.position.y = float(msg.content[3])
            	self.scoobyDoo.position.z = float(msg.content[4])
            	self.scoobyDoo.velocity.vx = float(msg.content[5])
            	self.scoobyDoo.velocity.vy = float(msg.content[6])
            	self.scoobyDoo.velocity.vz = float(msg.content[7])
            	self.RigidBodies[ID] = self.scoobyDoo


#    def decodeMessage(self,msg):
#        if(msg.content.ID>0):
#            ID = int(msg.content.ID)# + self.rigidBodyState.ID
#            self.RigidBodies[ID] = msg.content
 #           self.rigidBodyState.position = msg.content.position
	    #print self.RigidBodies
#            self.rigidBodyState.timeout.peerLastRx[ID] = datetime.now()
#        else:
#	    print "This should not be happening"
#            ID = int(msg.content.ID) + self.rigidBodyState.ID
#            self.RigidBodies[ID] = msg.content
#            self.rigidBodyState.timeout.peerLastRx[int(msg.content.ID)] = datetime.now()

    def getLeaderData(self):
        self.rigidBodyState.leader.qgx = self.rigidBodyState.position.x
        self.rigidBodyState.leader.qgy = self.rigidBodyState.position.y
        self.rigidBodyState.leader.qgz = self.rigidBodyState.position.z
        self.rigidBodyState.leader.pgx = 0
        self.rigidBodyState.leader.pgy = 0
        self.rigidBodyState.leader.pgz = 0

    def setStaticFlockingLeader(self):
	print 'Set the Flocking Leader at average of the two agent Position!'
	self.rigidBodyState.leader.flock.qgx = (self.rigidBodyState.initPos.xo + self.scoobyDoo.position.x)/2
	self.rigidBodyState.leader.flock.qgy = (self.rigidBodyState.initPos.yo + self.scoobyDoo.position.y)/2
	self.rigidBodyState.leader.flock.qgz = (self.rigidBodyState.initPos.zo + self.scoobyDoo.position.z)/2
        self.rigidBodyState.leader.flock.pgx = 0
        self.rigidBodyState.leader.flock.pgy = 0
        self.rigidBodyState.leader.flock.pgz = 0



    def setInitialPos(self):
        initPosSet = False
        if(self.rigidBodyState.position.z<-0.115):
            self.rigidBodyState.initPos.xo = self.rigidBodyState.position.x        
            self.rigidBodyState.initPos.yo = self.rigidBodyState.position.y
            self.rigidBodyState.initPos.zo = self.rigidBodyState.position.z
            initPosSet = True
        print initPosSet
        return initPosSet

    
    def checkAbort(self):
        if(self.checkTimeouts()):
            self.rigidBodyState.abortReason = "Timeout"
            self.rigdBodyState.RCLatch = True
            self.rigidBodyState.isGPS = False
            self.releaseControl()
            return True
        #! Check the proper flight mode
        if(not self.vehicle.mode == 'STABILIZE'):
            self.releaseControl()
            return True
        return False

    def checkGPS(self):
        #! Check Timeouts
        if(self.checkTimeouts()):
        	print "No GPS - Timeout"
        	self.rigidBodyState.RCLatch = True
        	return False
        #! Check configuration
        if(not self.rigidBodyState.parameters.isComplete):
        	self.rigidBodyState.RCLatch = True
        	return False
        self.rigidBodyState.RCLatch = True # Set the Latch
        self.rigidBodyState.isGPS = True # GPS is being received
        return True

    #! Check for a GPS timeout - If no GPS, control should not be engaged
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

    def computePDControl(self):
        #! Compute the difference vectors
        dq = np.matrix([[self.diffFunction(self.rigidBodyState.position.x,self.rigidBodyState.leader.qgx)],[self.diffFunction(self.rigidBodyState.position.y,self.rigidBodyState.leader.qgy)],[self.diffFunction(self.rigidBodyState.position.z,self.rigidBodyState.leader.qgz)]])
        dp = np.matrix([[self.diffFunction(self.rigidBodyState.velocity.vx,self.rigidBodyState.leader.pgx)],[self.diffFunction(self.rigidBodyState.velocity.vy,self.rigidBodyState.leader.pgy)],[self.diffFunction(self.rigidBodyState.velocity.vz,self.rigidBodyState.leader.pgz)]])
        #! Compute Desired Accelerations
        kp = np.matrix([[self.rigidBodyState.parameters.kpx, 0, 0], [0, self.rigidBodyState.parameters.kpy, 0], [0, 0, self.rigidBodyState.parameters.kpz]])
        kd = np.matrix([[self.rigidBodyState.parameters.kdx, 0, 0], [0, self.rigidBodyState.parameters.kdy, 0], [0, 0, self.rigidBodyState.parameters.kdz]])
        uk = kp*dq + kd*dp
        #! Estimate the velocity command using a low pass filter
        velGain = 0.1
        pkp = np.matrix([[self.rigidBodyState.previousState.velPrev_x], [self.rigidBodyState.previousState.velPrev_y], [self.rigidBodyState.previousState.velPrev_z]])
        ukp = np.matrix([[self.rigidBodyState.previousState.accPrev_x],[self.rigidBodyState.previousState.accPrev_y],[self.rigidBodyState.previousState.accPrev_z]])
        pk = (1-self.rigidBodyState.parameters.Ts*velGain)*pkp + self.rigidBodyState.parameters.Ts*(1-(velGain*self.rigidBodyState.parameters.Ts)/2)*ukp
        self.updateControlState(uk,pk)

    def computeFlockingControl(self):
        #! Set Position Vectors and Estimate Next State
        qi = np.matrix([[self.rigidBodyState.position.x],[self.rigidBodyState.position.y],[self.rigidBodyState.position.z]])
        pi = np.matrix([[self.rigidBodyState.velocity.vx],[self.rigidBodyState.velocity.vy],[self.rigidBodyState.velocity.vz]])
        qihat = qi + self.rigidBodyState.parameters.Ts*pi
        qj = np.matrix([[self.scoobyDoo.position.x],[self.scoobyDoo.position.y],[self.scoobyDoo.position.z]])
        pj = np.matrix([[self.scoobyDoo.velocity.vx],[self.scoobyDoo.velocity.vy],[self.scoobyDoo.velocity.vz]])
        qjhat = qj + self.rigidBodyState.parameters.Ts*pj
        #! Difference Functions
        dq = qj - qi
        dp = pj - pi
        dqhat = qjhat - qihat        
        #! Attraction / Repulsion
        Phi = self.rigidBodyState.parameters.alpha2/(self.rigidBodyState.parameters.alpha1 + 1) - self.rigidBodyState.parameters.alpha2/(self.rigidBodyState.parameters.alpha1 + (m.pow(self.vectorNorm(dqhat),2)/m.pow(self.rigidBodyState.parameters.desDist,2)))
        AR = Phi*dq
        #! Velocity Consensus
        VC = self.rigidBodyState.parameters.beta*dp
        #! Guidance Term
        kGamma1 = np.matrix([[self.rigidBodyState.parameters.gamma1, 0, 0], [0, self.rigidBodyState.parameters.gamma1, 0], [0, 0, self.rigidBodyState.parameters.gamma3]])
        kGamma2 = np.matrix([[self.rigidBodyState.parameters.gamma2, 0, 0], [0, self.rigidBodyState.parameters.gamma2, 0], [0, 0, self.rigidBodyState.parameters.gamma4]])
        qg = np.matrix([[self.rigidBodyState.leader.flock.qgx],[self.rigidBodyState.leader.flock.qgy],[self.rigidBodyState.leader.flock.qgz]])
        pg = np.matrix([[self.rigidBodyState.leader.flock.pgx],[self.rigidBodyState.leader.flock.pgy],[self.rigidBodyState.leader.flock.pgz]])
        GT = kGamma1*(qg-qi) + kGamma2*(pg-pi)
        #! Flock Correction to Guidance Term
        FC = (1/self.rigidBodyState.parameters.expectedMAVs)*(kGamma1*dq + kGamma2*dp)
        #! Compute Desired Accelerations
        uk = AR + VC + GT - FC
        #! Estimate the velocity command using a low pass filter
        velGain = 0.1
        pkp = np.matrix([[self.rigidBodyState.previousState.velPrev_x], [self.rigidBodyState.previousState.velPrev_y], [self.rigidBodyState.previousState.velPrev_z]])
        ukp = np.matrix([[self.rigidBodyState.previousState.accPrev_x],[self.rigidBodyState.previousState.accPrev_y],[self.rigidBodyState.previousState.accPrev_z]])
        pk = (1-self.rigidBodyState.parameters.Ts*velGain)*pkp + self.rigidBodyState.parameters.Ts*(1-(velGain*self.rigidBodyState.parameters.Ts)/2)*ukp
        self.updateControlState(uk,pk)

        
    def updateControlState(self,uk,pk):
        #! Estimate the velocity command using a low pass filter
        #velGain = 0.1
        #pkp = np.matrix([[self.rigidBodyState.previousState.velPrev_x], [self.rigidBodyState.previousState.velPrev_y], [self.rigidBodyState.previousState.velPrev_z]])
        #ukp = np.matrix([[self.rigidBodyState.previousState.accPrev_x],[self.rigidBodyState.previousState.accPrev_y],[self.rigidBodyState.previousState.accPrev_z]])
        #pk = (1-self.rigidBodyState.parameters.Ts*velGain)*pkp + self.rigidBodyState.parameters.Ts*(1-(velGain*self.rigidBodyState.parameters.Ts)/2)*ukp
        #self.updateControlState(uk,pk)
        #! Update the command state
        self.rigidBodyState.command.ux = uk[0,0]
        self.rigidBodyState.command.uy = uk[1,0]
        self.rigidBodyState.command.uz = uk[2,0]
        self.rigidBodyState.command.vel_est_x = pk[0,0]
        self.rigidBodyState.command.vel_est_y = pk[1,0]
        self.rigidBodyState.command.vel_est_z = pk[2,0]       
        #! Update the previous State 
        self.rigidBodyState.previousState.velPrev_x = pk[0,0]
        self.rigidBodyState.previousState.velPrev_y = pk[1,0]
        self.rigidBodyState.previousState.velPrev_z = pk[2,0]
        self.rigidBodyState.previousState.accPrev_x = uk[0,0] 
        self.rigidBodyState.previousState.accPrev_y = uk[1,0]
        self.rigidBodyState.previousState.accPrev_z = uk[2,0]
        #! Integrate the velocity error
        accVelError = self.rigidBodyState.velocity.vz - self.rigidBodyState.command.vel_est_z 
        self.rigidBodyState.command.accVelZError = self.antiWindup(self.rigidBodyState.command.vel_est_z,-10,10,accVelError,self.rigidBodyState.velocity.vz)
        self.computeAttitudeThrustCommands()

    def computeAttitudeThrustCommands(self):
        self.rigidBodyState.test.throttle = self.rigidBodyState.parameters.quadMass*(self.rigidBodyState.parameters.gravity - self.rigidBodyState.command.uz + self.rigidBodyState.parameters.kw_vel*(self.rigidBodyState.velocity.vz - self.rigidBodyState.command.vel_est_z) + self.rigidBodyState.parameters.intGain*self.rigidBodyState.command.accVelZError)/(np.cos(self.rigidBodyState.attitude.roll)*np.cos(self.rigidBodyState.attitude.pitch))
        self.rigidBodyState.test.pitch = np.arctan(((self.rigidBodyState.command.ux-self.rigidBodyState.parameters.ku_vel*(self.rigidBodyState.velocity.vx - self.rigidBodyState.command.vel_est_x))*np.cos(self.rigidBodyState.attitude.yaw) + (self.rigidBodyState.command.uy-self.rigidBodyState.parameters.kv_vel*(self.rigidBodyState.velocity.vy - self.rigidBodyState.command.vel_est_y))*np.sin(self.rigidBodyState.attitude.yaw))/(-self.rigidBodyState.parameters.gravity + self.rigidBodyState.command.uz - self.rigidBodyState.parameters.kw_vel*(self.rigidBodyState.velocity.vz - self.rigidBodyState.command.vel_est_z) - self.rigidBodyState.parameters.intGain*self.rigidBodyState.command.accVelZError))
        self.rigidBodyState.test.roll = np.arctan(((self.rigidBodyState.command.ux-self.rigidBodyState.parameters.ku_vel*(self.rigidBodyState.velocity.vx - self.rigidBodyState.command.vel_est_x))*np.cos(self.rigidBodyState.test.pitch)*np.sin(self.rigidBodyState.attitude.yaw) - (self.rigidBodyState.command.uy-self.rigidBodyState.parameters.kv_vel*(self.rigidBodyState.velocity.vy - self.rigidBodyState.command.vel_est_y))*np.cos(self.rigidBodyState.test.pitch)*np.cos(self.rigidBodyState.attitude.yaw))/(-self.rigidBodyState.parameters.gravity + self.rigidBodyState.command.uz - self.rigidBodyState.parameters.kw_vel*(self.rigidBodyState.velocity.vz - self.rigidBodyState.command.vel_est_z) - self.rigidBodyState.parameters.intGain*self.rigidBodyState.command.accVelZError))
        self.rigidBodyState.test.yaw = self.rigidBodyState.attitude.yaw
        self.scaleAndSendControl()


    def scaleAndSendControl(self):
        #! Scale the compute control values to match the format used in vehicle.channel.overrides{}
        ROLL =  1500 + (500/self.rigidBodyState.parameters.rollLimit)*self.rigidBodyState.test.roll
        PITCH = 1500 + (500/self.rigidBodyState.parameters.pitchLimit)*self.rigidBodyState.test.pitch
        THROTTLE = 1000 + 32.254*self.rigidBodyState.test.throttle - 0.257*self.rigidBodyState.test.throttle*self.rigidBodyState.test.throttle#972 + 48.484*self.rigidBodyState.test.throttle + 1.3241*self.rigidBodyState.test.throttle*self.rigidBodyState.test.throttle#
        YAW = self.rigidBodyState.attitude.yaw
        #! Saturate to keep commands in range of input values
        self.rigidBodyState.command.Roll = self.saturate(ROLL,1000,2000)
        self.rigidBodyState.command.Pitch = self.saturate(PITCH,1000,2000)
        self.rigidBodyState.command.Throttle = self.saturate(THROTTLE,1000,2000)
        self.rigidBodyState.command.Yaw = self.saturate(YAW,1000,2000)
        #self.vehicle.channels.overrides = {'1': self.rigidBodyState.command.Roll,'2': self.rigidBodyState.command.Pitch,'3': self.rigidBodyState.command.Throttle}
        print self.counter

    def antiWindup(self,value,lowLimit,highLimit,accumulator,toAdd):
        if(value>highLimit): #Saturation and anti-windup
            if(toAdd > 0):
                accumulator = accumulator + toAdd
        if(value<lowLimit):
            if(toAdd < 0):
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
