from recordtype import recordtype

Timeout = recordtype('Timeout',['localTimeoutTime','GCSLastRx',('peerLastRx',{})],default = None)
Command = recordtype('Command',['Roll','Pitch','Throttle','Yaw','ux','uy','uz','vel_est_x','vel_est_y','vel_est_z',('accVelZError',0)], default = None)

PreviousState = recordtype('PreviousState',[('velPrev_x',0),('velPrev_y',0),('velPrev_z',0),('accPrev_x',0),('accPrev_y',0),('accPrev_z',0)], default = None)

Position = recordtype('Position',['x','y','z'], default=None)
Velocity = recordtype('Velocity',[('vx',0),('vy',0),('vz',0)], default = None)


#ComputeControl = recordtype('ComputerControl',[('ux',0),('uy'),('uz',0)], default = None)

Leader = recordtype('Leader',['qgx','qgy','qgz','pgx','pgy','pgz'], default = None)

Attitude = recordtype('Attitude',[('roll',0),('pitch',0),('yaw',0)], default = None)
AttThrust = recordtype('AttThrust',['roll','pitch','throttle','yaw'], default = None)

InitialPosition = recordtype('InitialPosition',['xo','yo','zo'], default = None)

Parameter = recordtype('Parameter',['Ts','peerTimeout','GPSTimeout','expectedMAVs','isComplete','TargetAltitude','kpx','kdx','kpy','kdy','kpz','kdz','targetAltitude','quadMass','gravity','ku_vel','kv_vel','kw_vel','rollLimit','pitchLimit','throttleLimit','stoppingDistance','desiredSpeed','isTakeoff','intGain','InitPos','isLanding'], default = None)

RigidBodyState = recordtype('RigidBodyState', [('startTime', None),'ID','time','channels',('test',AttThrust()),('attitude',Attitude()),('RCLatch',False),('isGPS',False),'lastGPSContact',('position',Position()),('velocity',Velocity()),('initPos',InitialPosition()),('command',Command()),('parameters',Parameter()),('timeout',Timeout()),('leader',Leader()),('previousState',PreviousState())], default = None)

Message = recordtype('Message', 'type,sendTime,content', default = None)

