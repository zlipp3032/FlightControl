from rigidBodyState import Parameter
import time
import numpy as np

defaultParams = Parameter()
defaultParams.Ts = 0.05#20Hz
defaultParams.expectedMAVs = 1
defaultParams.kp = 5 # Proportional gain for PD controller to follow leader
defaultParams.kd = 0.9 # Derivative gain for PD controller to follow leader
