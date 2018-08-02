import socket
from rigidBodyState import *
import collections
import Queue
import threading
from datetime import datetime
import math as m
import json

class Receiver(threading.Thread):
    def __init__(self,receiveQueue,localIP,Port,bufferLength,d):
        threading.Thread.__init__(self)
        self.localIP = localIP
        self.IP = '' #Symbolic name meaning local host
        self.Port = Port
        self.UDPTIMEOUT = 10 #value in seconds
        self.bufferLength = bufferLength
        self.localAddr = (self.IP,self.Port)
        self.reso = 39.68 # Using the mean value of the resolution should yield better error when following a command
        self.resoAngle = 1.43 
	self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST, 1)
        self.socket.settimeout(self.UDPTIMEOUT)
        self.socket.bind(self.localAddr)
        self.receiveQueue=receiveQueue
        self.stoprequest = threading.Event()
        self.rigidBodyState = RigidBodyState()

    def stop(self):
        self.stoprequest.set()
        print "Stop flag set - Receive"

    def run(self):
        while( not self.stoprequest.is_set()):
            try:
                self.receiveMessage()
            except Queue.Empty:
                break
        print "Receive Stopped"
        
    def receiveMessage(self):
        try:
            udpData = self.socket.recvfrom(self.bufferLength)
            msg = Message()
#            msg.content = MessageState()#RigidBodyState()
            msg.sendTime = datetime.now() #This is acutally the time we received the message from the computer
            PIG = udpData[0].split(',')
	    msg.content = PIG
#            print PIG 
#!            self.decodeData(PIG,msg)
#            print msg.content.position
#	    msg.content.ID = ord(PIG[0])-48 #This value will likely be zero to inidicate it is from the computer.
#	    print msg.content.ID
#	    print msg.content.position
#            msg.content.position = self.rigidBodyState.position
#            msg.content.velocity = self.rigidBodyState.velocity
#            msg.content.leader = self.rigidBodyState.leader
#            if(not msg.content.ID == 0):
#                msg.content.position = self.rigidBodyState.position
#                msg.content.velocity = self.rigidBodyState.velocity
#                msg.content.attitude = self.rigidBodyState.attitude
#            else:
#                msg.content.leader = self.rigidBodyState.leader
           #msg.content = self.rigidBodyState
	   # print 'Receive Data'
           # print "Leader: %s" % msg.content.leader
            self.receiveQueue.put(msg)#,True,0.01)
	    #self.receiveQueue.put(PIG)
#	    print msg.content
#	    print self.receiveQueue.qsize()
#            print "Yes"
            pass
        except socket.error, e:
            if not e.args[0] == 'timed out':
                raise e
            else: print "timeout"

    def decodeData(self,PIG,msg):
        reso = (self.reso/2)/1000
	resoAng = (self.resoAngle/2)*(m.pi/180)
#        print ord(PIG[0])
        #if( not ord(PIG[0])==48):
#            print "Rigid Body"
	msg.content.ID = int(PIG[1])
	msg.content.position.x = float(PIG[2])
	msg.content.position.y = float(PIG[3])
        msg.content.position.z = float(PIG[4])
	msg.content.velocity.vx = float(PIG[5])
	msg.content.velocity.vy = float(PIG[6])
	msg.content.velocity.vz = float(PIG[7])
	msg.content.attitude.roll = float(PIG[8])
	msg.content.attitude.pitch = float(PIG[9])
	msg.content.attitude.yaw = float(PIG[10])
	#! Decode the x position
        #if( not ord(PIG[2])==127):
        #	if( ord(PIG[1])==43):
        #        	msg.content.position.x = ord(PIG[2])*reso
        #        else:
        #            	msg.content.position.x = -ord(PIG[2])*reso
        #else:
       # 	if( ord(PIG[1])==43):
       #             	msg.content.position.x = 44*reso
       #         else:
       #             	msg.content.position.x = -44*reso
       # #! Decode the y position
       # if( not ord(PIG[4])==127):
       # 	if( ord(PIG[3])==43):
       #         	msg.content.position.y = ord(PIG[4])*reso
       # 	else:
       #             	msg.content.position.y = -ord(PIG[4])*reso
       # else:
       #         if( ord(PIG[3])==43):
       # 	        msg.content.position.y = 44*reso
       #         else:
       #         	msg.content.position.y = -44*reso
       # #! Decode the z position
       # if( not ord(PIG[6])==127):
       #         if( ord(PIG[5])==43):
       # 	        msg.content.position.z = ord(PIG[6])*reso
       #         else:
       #         	msg.content.position.z = -ord(PIG[6])*reso
       # else: 
       #         if( ord(PIG[5])==43):
       # 	        msg.content.position.z = 44*reso
       #         else:
       #         	msg.content.position.z = -44*reso
       # #! Decode the x velocity
       # if( not ord(PIG[8])==127):
       #         if( ord(PIG[7])==43):
       # 	        msg.content.velocity.vx = ord(PIG[8])*reso
       #         else:
       #         	msg.content.velocity.vx = -ord(PIG[8])*reso
       # else: 
       #         if( ord(PIG[7])==43):
       #         	msg.content.velocity.vx = 44*reso
       #         else:
       #         	msg.content.velocity.vx = -44*reso
       # #! Decode the y velocity
       # if( not ord(PIG[10])==127):
       #         if( ord(PIG[9])==43):
       # 	        msg.content.velocity.vy = ord(PIG[10])*reso
       #         else:
       #             	msg.content.velocity.vy = -ord(PIG[10])*reso
       # else:
       #         if( ord(PIG[9])==43):
       #  	        msg.content.velocity.vy = 44*reso
       #         else:
       #         	msg.content.velocity.vy = -44*reso
       # #! Decode the z velocity
       # if( not ord(PIG[12])==127):
       #         if( ord(PIG[11])==43):
       # 	        msg.content.velocity.vz = ord(PIG[12])*reso
       #         else:
       #         	msg.content.velocity.vz = -ord(PIG[12])*reso
       # else:
       #         if( ord(PIG[11])==43):
       # 	        msg.content.velocity.vz = 44*reso
       #         else:
       #         	msg.content.velocity.vz = -44*reso
       # #! Decode the ROLL
       # if( not ord(PIG[14])==127):
       #         if( ord(PIG[13])==43):
       # 	        msg.content.attitude.roll = ord(PIG[14])*resoAng
       #         else:
       #         	msg.content.attitude.roll = -ord(PIG[14])*resoAng
       # else:
       #         if( ord(PIG[13])==43):
       # 	        msg.content.attitude.roll = 44*resoAng
       #         else:
       #         	msg.content.attitude.roll = -44*resoAng
       # #! Decode the PITCH
       # if( not ord(PIG[16])==127):
       #         if( ord(PIG[15])==43):
       # 	        msg.content.attitude.pitch = ord(PIG[16])*resoAng
       #         else:
       #         	msg.content.attitude.pitch = -ord(PIG[16])*resoAng
       # else:
       #         if( ord(PIG[15])==43):
       # 	        msg.content.attitude.pitch = 44*resoAng
       #         else:
       #         	msg.content.attitude.pitch = -44*resoAng
       # #! Decode the YAW
       # if( not ord(PIG[18])==127):
       #         if( ord(PIG[17])==43):
       # 	        msg.content.attitude.yaw = ord(PIG[18])*resoAng
       #         else:
       #         	msg.content.attitude.yaw = -ord(PIG[18])*resoAng
       # else:
       #         if( ord(PIG[17])==43):
       #             	msg.content.attitude.yaw = 44*resoAng
       #         else:
       #             	msg.content.attitude.yaw = -44*resoAng
        ###########################################################################else:
#            print "Leader"
            # Decode the leader x position
        #    if( not ord(PIG[2])==127):
        #        if( ord(PIG[1])==43):
        #            self.rigidBodyState.leader.qgx = ord(PIG[2])*reso
        #        else:
        #            self.rigidBodyState.leader.qgx = -ord(PIG[2])*reso
        #    else: #ord(PIG[2]) = 127
        #        if( ord(PIG[1])==43):
        #            self.rigidBodyState.leader.qgx = 44*reso
        #        else:
        #            self.rigidBodyState.leader.qgx = -44*reso
            # Decode the leader y position
        #    if( not ord(PIG[4])==127):
        #        if( ord(PIG[3])==43):
        #            self.rigidBodyState.leader.qgy = ord(PIG[4])*reso
        #        else:
        #            self.rigidBodyState.leader.qgy = -ord(PIG[4])*reso
        #    else: #ord(PIG[4]) = 127
        #        if( ord(PIG[3])==43):
        #            self.rigidBodyState.leader.qgy = 44*reso
        #        else:
        #            self.rigidBodyState.leader.qgy = -44*reso
        #    # Decode the z position
        #    if( not ord(PIG[6])==127):
        #        if( ord(PIG[5])==43):
        #            self.rigidBodyState.leader.qgz = ord(PIG[6])*reso
        #        else:
        #            self.rigidBodyState.leader.qgz = -ord(PIG[6])*reso
        #    else: #ord(PIG[6]) = 127
        #        if( ord(PIG[5])==43):
        #            self.rigidBodyState.leader.qgz = 44*reso
        #        else:
        #            self.rigidBodyState.leader.qgz = -44*reso
        #    #Decode the x velocity
        #    if( not ord(PIG[8])==127):
        #        if( ord(PIG[7])==43):
        #            self.rigidBodyState.leader.pgx = ord(PIG[8])*reso
        #        else:
        #            self.rigidBodyState.leader.pgx = -ord(PIG[8])*reso
        #    else: #ord(PIG[2]) = 127
        #        if( ord(PIG[7])==43):
        #            self.rigidBodyState.leader.pgx = 44*reso
        #        else:
        #            self.rigidBodyState.leader.pgx = -44*reso
        #    # Decode the y velocity
        #    if( not ord(PIG[10])==127):
        #        if( ord(PIG[9])==43):
        #            self.rigidBodyState.leader.pgy = ord(PIG[10])*reso
        #        else:
        #            self.rigidBodyState.leader.pgy = -ord(PIG[10])*reso
        #   else: #ord(PIG[4]) = 127
         #       if( ord(PIG[9])==43):
        #            self.rigidBodyState.leader.pgy = 44*reso
        #        else:
        #            self.rigidBodyState.leader.pgy = -44*reso
        #    # Decode the z velocity
        #    if( not ord(PIG[12])==127):
        #        if( ord(PIG[11])==43):
        #            self.rigidBodyState.leader.pgz = ord(PIG[12])*reso
        #        else:
        #            self.rigidBodyState.leader.pgz = -ord(PIG[12])*reso
        #    else: #ord(PIG[6]) = 127
        #        if( ord(PIG[11])==43):
        #            self.rigidBodyState.leader.pgz = 44*reso
        #        else:
        #            self.rigidBodyState.leader.pgz = -44*reso
