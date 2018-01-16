# Default parameters used in the entire flight program
# Zachary Lippay
from rigidBodyState import Parameter
import time
import numpy as np

defaultParams = Parameter()
defaultParams.Ts = 0.05#20Hz
defaultParams.expectedMAVs = 1
defaultParams.kp = 5 # Proportional gain for PD controller to follow leader
defaultParams.kd = 0.9 # Derivative gain for PD controller to follow leader
defaultParams.GPSTimeout = 1.0
defaultParams.TargetAltitude = 100 #Units in centimeters --> Might not be the best choice, consider changing
defaultParams.isComplete = True #Ensure the Default parameters were loaded correctly into the program
