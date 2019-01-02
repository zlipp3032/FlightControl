import collections
from vehicleState import *
import socket
import Queue
import logging
import multiprocessing
import jsonpickle
import signal
import zlib
from datetime import datetime
import copy

class Receiver(multiprocessing.Process):
    def __init__(self, receiveQueue, AdHocIP, port, bufferlength, ignoreSelfPackets):
        multiprocessing.Process.__init__(self)
        self.s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.AdHocIP = AdHocIP
        self.port = port
        self.IP = ""
        self.s.bind((self.IP,self.port))
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.receiveQueue = receiveQueue
        self.stoprequest = multiprocessing.Event()
        self.s.settimeout(1)
        self.ignoreSelfPackets = ignoreSelfPackets
    def stop(self):
        self.stoprequest.set()
        print "Stop flag set - Receive"
    def run(self):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        while( not self.stoprequest.is_set()):
            try:
                self.receiveMessage()
            except Queue.Empty:
                break
        print "Receive Stopped"

    def receiveMessage(self):
        try:
            mp = self.s.recvfrom(4096)
            if(self.ignoreSelfPackets and mp[1] == (self.AdHocIP,self.port)):
                pass
            else:
                msg = Message()
                try:
                    raw_state  = jsonpickle.decode(mp[0]) #mp[0].split(',')
                    #print(raw_state['ID'])
                    msg.content = raw_state
                    #msg.content['timestamp'] = datetime.now()
                    msg.sendTime = datetime.now()
                    self.receiveQueue.put(msg)
                    pass
                except ValueError:
					print "received invalid packet"
        except socket.error, e:
            if not e.args[0] == 'timed out':
                raise e
            else:
                print "timeout"
