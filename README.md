# FlightControl
Communication and control of copter using dronekit-api in GPS denied environment. 

The overall purpose of this project is to implement a digital control algorithm on a micro-computer that will send commands to a quadcopter in a GPS denied environment. This micro-computer will be onboard the quadcopter and will recceive spatial data, velocities, and angles (e.g. roll, pitch, yaw) from an external computer. The external computer is connected to an OptiTrak vision sensing setup using the Motive GUI. Matlab will be used to initiate the experiments on the external computer. This code will use data pulled from Motive, process the data, encode the data using an ASCII format, and send the data to the Network. The micro-computer will be using Python programming and will receive the data, decode the data, compute the control parameters, override the channels to approriate commands.

Control method used is a PD controller on a leader trajectory.

# Main Functions
controlFunction_v2.m is the main MATLAB transmit function. This requires the intstrument control toolbox because it utilizes the udp() function.

mainFlightProgram_v5.py is just the "receive data" communication side of things and does not connect to dronekit (e.g. there is no command sent to the copter) --> This program should work as long as the proper network parameters are used and the correct version numbers of each file are used. 

mainFlightProgram_v6.py will implement a control algorithm that uses vehicle.channel.overrides{} to send {ROLL,PITCH,THROTTLE,YAW} commands to the PX4-cube flight controller using the --connect method --> This program is not yet operational and is still in its early stages.
