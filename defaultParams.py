from rigidBodyState import Parameter
import time
import numpy as np
import math as m


defaultParams = Parameter()
defaultParams.GPSTimeout = 1.0
#defaultParams.TargetAltitude = 100 #Units in centimeters --> Might not be the best choice, consider changing
defaultParams.isComplete = True #Ensure the Default parameters were loaded correctly into the program
defaultParams.Ts = 0.05#20Hz
defaultParams.expectedMAVs = 2 # For indoor experiments, include this agent's "MAV" ---> For outdoor, do not include this agent's "MAV"
#! PD Controller Gains
defaultParams.kpx = 0.6#0.6
defaultParams.kdx = 1.3#1.3
defaultParams.kpy = 0.6#0.6
defaultParams.kdy = 1.3#1.3
defaultParams.kpz = 0.4#0.09
defaultParams.kdz = 1.4#0.20
#! Physical Properties
defaultParams.quadMass = 1.2# 1.7 for heavy UP 1.2 for light UP # Value determined experimetnally from scale - Actual SOLO mass is 1.5kg # (units in kg)
defaultParams.gravity = 9.81 #units in  m/s/s
#! Velocity Controller Gains
defaultParams.ku_vel = 3.5
defaultParams.kv_vel = 3.5
defaultParams.kw_vel = 1.5
#! Scaling Parameters
defaultParams.rollLimit = 0.7845 # (0.7845 rad  = 45 deg) the upper limit for this drone
defaultParams.pitchLimit = 0.7845 # (0.5236 rad = 30 deg) the upper limit for this drone (1.5708 rad = 90 deg = m.pi/2 rad)
defaultParams.throttleLimit = defaultParams.quadMass*defaultParams.gravity - 0 #This is the hover value with a bias adjustment pending experiments
#! Velocity speed control parameters
defaultParams.stoppingDistance =  0.2#0.8*0.75#defaultParams.targetAltitude*(1-0.75) # 0.75 says the stopping distance occurs when the drone is 75% of the target altitude 
defaultParams.desiredSpeed = -0.2 # Units in m/s
#defaultParams.isTakeoff = False
defaultParams.targetAltitude = 0.75 #  unit is in meters
defaultParams.intGain = 0.5
#! Algorithmic parameters
defaultParams.isTakeoff = False
defaultParams.InitPos = False
defaultParams.isLanding = False
defaultParams.isHovering = False
defaultParams.isFlocking = False
#! Flocking Controller Gains
defaultParams.alpha1 = 0.001
defaultParams.alpha2 = 0.1
defaultParams.beta = 0.3
defaultParams.gamma1 = 0.08#0.1#0.08#0.02
defaultParams.gamma2 = 0.60#0.66#0.6#0.22
defaultParams.gamma3 = 0.11#0.16#0.14#0.04
defaultParams.gamma4 = 0.70#0.76#0.7#0.38
defaultParams.desiredDistance = 1.8#2#1.6 # (unit in meters) Desired distance for agents in flock
