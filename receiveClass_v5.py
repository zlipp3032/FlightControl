import socket
from rigidBodyState import *
import collections
import Queue
import threading
from datetime import datetime
import math as m
import json

class Receiver(threading.Thread):
    def __init__(self,receiveQueue,localIP,Port,bufferLength):
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
            self.receiveQueue.put(msg)
            pass
        except socket.error, e:
            if not e.args[0] == 'timed out':
                raise e
            else: print "timeout"

    def decodeData(self,PIG,msg):
        reso = (self.reso/2)/1000
        resoAng = (self.resoAngle/2)*(m.pi/180)
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
