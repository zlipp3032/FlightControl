# This script allocates space for the variables (e.g. defines the variables) used in the flight program
# Zachary Lippay

from recordtype import recordtype

Timeout = recordtype('Timeout',['localTimeoutTime','GCSLastRx',('peerLastRx',{})],default = None)
Command = recordtype('Command',['Roll','Pitch','Throttle','Yaw'], default = None)

Position = recordtype('Position',[('x',0),('y',0),('z',0)], default=None)
Velocity = recordtype('Velocity',[('vx',0),('vy',0),('vz',0)], default = None)
ComputeControl = recordtype('ComputerControl',[('ux',0),('uy'),('uz',0)], default = None)
Leader = recordtype('Leader',[('qgx',0),('qgy',0),('qgz',0),('pgx',0),('pgy',0),('pgz',0)], default = None)
Attitude = recordtype('Attitude',[('roll',0),('pitch',0),('yaw',0)], default = None)
InitialPosition = recordtype('InitialPosition',[('xo',0),('yo',0),('zo',0),('vxo',0),('vyo',0),('vzo',0)], default = None)
Parameter = recordtype('Parameter',['Ts','peerTimeout','GPSTimeout','expectedMAVs','kp','kd','isComplete','isTakeOff','TargetAltitude'], default = None)

RigidBodyState = recordtype('RigidBodyState', [('startTime', None),'ID','time',('attitude',Attitude()),('RCLatch',False),('isGPS',False),'lastGPSContact',('position',Position()),('velocity',Velocity()),('initPos',InitialPosition()),('command',Command()),('parameters',Parameter()),('timeout',Timeout()),('control',ComputeControl()),('leader',Leader())], default = None)

Message = recordtype('Message', 'type,sendTime,content', default = None)
