print('Vehicle State')
from recordtype import recordtype
import numpy as np
from collections import OrderedDict

Timeout = recordtype('Timeout',['localTimeoutTime','GCSLastRx',('peerLastRx',{})],default = None) 
Command = recordtype('Command',['Roll','Pitch','Throttle','Yaw',
                                'ux','uy','uz','vel_est_x','vel_est_y','vel_est_z',
                                    ('accVelZError',0),('AR',0),('VC',0),('GT',0),
                                    ('FC',0),('accPosXError',0),('accPosYError',0),
                                    ('accPosZError',0)],default = None)

PreviousState = recordtype('PreviousState',[('velPrev_x',0),
                                            ('velPrev_y',0),('velPrev_z',0),('accPrev_x',0),
                                                ('accPrev_y',0),('accPrev_z',0)], default = None)

Position = recordtype('Position',['x','y','z'], default=None)
Velocity = recordtype('Velocity',[('vx',0),('vy',0),('vz',0)], default = None)


#ComputeControl = recordtype('ComputerControl',[('ux',0),('uy'),('uz',0)], default = None)

Flocking = recordtype('Flocking',['qgx','qgy','qgz',('pgx',0),('pgy',0),('pgz',0)], default = None)
Leader = recordtype('Leader',['qgx','qgy','qgz','pgx','pgy','pgz',('flocking',Flocking())], default = None)

Attitude = recordtype('Attitude',[('roll',0),('pitch',0),('yaw',0)], default = None)
AttThrust = recordtype('AttThrust',['roll','pitch','throttle','yaw'], default = None)

InitialPosition = recordtype('InitialPosition',['xo','yo','zo'], default = None)

#Parameter = recordtype('Parameter',['Ts','peerTimeout','GPSTimeout',
#                                    'expectedMAVs','isComplete','TargetAltitude','kpx','kdx',
#                                        'kpy','kdy','kpz','kdz','targetAltitude','quadMass',
#                                        'gravity','ku_vel','kv_vel','kw_vel','rollLimit','pitchLimit',
#                                        'throttleLimit','stoppingDistance','desiredSpeed','isTakeoff',
#                                        'intGain','InitPos','isLanding','isHovering','isFlocking','alpha1',
#                                        'alpha2','beta','gamma1','gamma2','gamma3','gamma4','desiredDistance',
#                                        'kix','kiy','kiz'], default = None)

Parameter = recordtype('Parameter',['Ts','receivedTime','expectedMAVs','isComplete','GPSTimeout','gains','config','txStateType'], default = None)

RigidBodyState = recordtype('RigidBodyState', [('startTime', None),'ID',
                                                   'batt','time','channels',('test',AttThrust()),
                                                   ('attitude',Attitude()),('RCLatch',False),('isGPS',False),
                                                   'lastGPSContact',('position',Position()),('velocity',Velocity()),
                                                   ('initPos',InitialPosition()),('command',Command()),
                                                   ('parameters',Parameter()),('timeout',Timeout()),
                                                   ('leader',Leader()),('previousState',PreviousState()),
                                                   'flightSeq'], default = None)

MessageState = recordtype('MessageState',['ID',('position',Position()),('velocity',Velocity()),
                                          ('attitude',Attitude())],default = None)

Message = recordtype('Message', 'type,sendTime,content', default = None)

class BasicVehicleState(object):
    def __init__(self,other=None):
        self.ID = None
        self.timestamp = None
        self.position = {'x': None, 'y': None, 'z': None}
        self.velocity =  {'vx': None, 'vy': None, 'vz': None}
        self.isPropagated = False
        self.counter = 0
        self.isFlocking = False
        #if other is not None:
        #    for k in self.__dict__.keys():
        #        self.__dict__[k] = other.__dict__[k]

    def getCSVLists(self):
        headers = []
        values = []

        headers.append('ID')
        values.append(self.ID)
        headers.append('Counter')
        values.append(self.counter)
        headers.append('Timestamp')
        values.append(self.timestamp)

        headers += ['xPos','yPos','zPos']
        values += [self.position['x'], self.position['y'], self.position['z']]

        headers += ['xVel', 'yVel', 'zVel']
        values += [self.velocity['vx'], self.velocity['vy'], self.velocity['vz']]

        out = OrderedDict(zip(headers,values))
        return out

        #return

class FullVehicleState(BasicVehicleState):
    def __init__(self):
        super(FullVehicleState, self).__init__()
        self.time = 0.00
        
    def getCSVLists(self):
        base = super(FullVehicleState,self).getCSVLists()
        headers = base.keys()
        values = base.values()

        out = OrderedDict(zip(headers,values))
        return out
        
    


def recordTypeToLists(rt,prefix =''):
	headers = []
	values = []
	d = rt._asdict()
	for k in d.keys():
		item = d[k]
		if isinstance(item,np.matrix):
			(h,v)=vecToCSV(item,k)
			headers+=h
			values+=v
		elif isinstance(item,rtTypes):	#If this is a valid recordtype (hack)
			(h,v) = recordTypeToLists(item,prefix + k+'_')
			headers+=h
			values += v
		elif isinstance(item,dict):
			for k2 in item.keys():
				if isinstance(item[k2],np.matrix):
					(h,v) = vecToCSV(item[k2],str(k)+str(k2))
					headers+=h
					values+=v
				else:
					headers.append(prefix + k2)
					values.append(item[k2])
		else:
			headers.append(prefix+k)
			values.append(item)

	return (headers,values)

def vecToCSV(mat,prefix):
	outKey = []
	outValue = []		
	for j in range(0,len(mat)):
		outKey.append(prefix+'_'+str(j))
		outValue.append(mat[j,0])
	return (outKey,outValue)
