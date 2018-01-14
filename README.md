# FlightControl
Communication and control of copter using dronekit-api on wireles network

Please note that controlFunction_v2.m is the main MATLAB transmit function. This requires the intstrument control toolbox because it utilizes the udp() function.

mainFlightProgram_v5.py is just the "receive data" communication side of things and does not connect to dronekit (e.g. there is no command sent to the copter) --> This program should work as long as the proper network parameters are used and the correct version numbers of each file are used. 

mainFlightProgram_v6.py will implement a control algorithm that uses vehicle.channel.overrides{} to send {ROLL,PITCH,THROTTLE,YAW} commands to the PX4-cube flight controller using the --connect method. This program is not yet operational and is still in its early stages.
