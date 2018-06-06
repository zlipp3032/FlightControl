from rigidBodyState_log import Parameter
import time
import numpy as np
import math as m

defaultParams = Parameter()
defaultParams.Ts = 0.05#20Hz
defaultParams.expectedMAVs = 1
defaultParams.kpx = 0.9
defaultParams.kdx = 2.0
defaultParams.kpy = 0.9
defaultParams.kdy = 2.0
defaultParams.kpz = 0.4#0.09
defaultParams.kdz = 1.4#0.20
defaultParams.quadMass = 1.5#0.605# Value determined experimetnally from scale - Actual SOLO mass is 1.5kg # (units in kg)
defaultParams.gravity = 9.81 #units in  m/s/s
# Velocity Controller Gains
defaultParams.ku_vel = 3.5
defaultParams.kv_vel = 3.5
defaultParams.kw_vel = 3.5
defaultParams.rollLimit = 0.7845 # (0.7845 rad  = 45 deg) the upper limit for this drone
defaultParams.pitchLimit = 0.7845 # (0.5236 rad = 30 deg) the upper limit for this drone (1.5708 rad = 90 deg = m.pi/2 rad)
defaultParams.throttleLimit = defaultParams.quadMass*defaultParams.gravity - 0 #This is the hover value with a bias adjustment pending experiments
defaultParams.stoppingDistance =  0.25*1.5#defaultParams.targetAltitude*(1-0.75) # 0.75 says the stopping distance occurs when the drone is 75% of the target altitude 
defaultParams.desiredSpeed = -0.02 # Units in m/s
defaultParams.isTakeoff = False
defaultParams.targetAltitude = 1.5 #  unit is in meters
defaultParams.intGain = 0

