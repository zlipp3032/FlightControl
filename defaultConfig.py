from vehicleState import Parameter
import time
import numpy as np
import math as m
#from dronekit import VehicleMode

def getParams():
    defaultParams = Parameter()
    defaultParams.receivedTime = time.time()
    defaultParams.Ts = 0.05
    defaultParams.expectedMAVs = 2
    defaultParams.isComplete = True
    defaultParams.GPSTimeout = 1.0
    defaultParams.txStateType = 'basic'
    #defaultParams.desiredPosition = np.array([[-1],[-0.5],[0]])
    defaultParams.gains = {'intGain': 0.5, 'leadVelGain': 0.0,
                           'kpx': 0.3, 'kdx': 1.2, 'kix': 0,
                           'kpy': 0.3, 'kdy': 1.2, 'kiy': 0,
                           'kpz': 0.3, 'kdz': 1.3, 'kiz': 0,
                           'ku_vel': 3.5, 'kv_vel': 3.5, 'kw_vel': 1.5, # Middle-Loop velocity controller gains
                           'alpha1': 0.001, 'alpha2': 0.09, # << Flocking gains
                           'beta': 0.25, # Flocking gains
                           'gamma1': 0.3, 'gamma2': 1.2, 'gamma3': 0.3, 'gamma4': 1.3, # Flocking gains
                           'd': 1.7 # Flocking gains>>
                           }
    defaultParams.config = {'rollLimit': 0.7845, # Used in the PWM scaling function
                            'pitchLimit': 0.7845, # Used in the PWM scaling function
                            'stoppingDistance': 0.2, # Used to switch velocity computation in potential function
                            'desiredSpeed': -0.2, # Desired speed in velocity potential function
                            'targetAltitude': 0.75, # Target altitude on takeoff
                            'quadMass': 1.67,
                            'grav': 9.81,
                            'isGPS': True,
                            'isTakeoff': False,
                            'initPos': False,
                            'isLanding': False,
                            'isHovering': False,
                            'isFlocking': False,
                            'ignoreSelfPackets': False
                            }
        
    
                           
    
    return defaultParams
