# FlightControl
Communication and control of copter using dronekit-api in GPS denied environment. 

The overall purpose of this project is to implement a digital control algorithm on a micro-computer that will send commands to a quadcopter in a GPS denied environment. This micro-computer will be onboard the quadcopter and will recceive spatial data, velocities, and angles (e.g. roll, pitch, yaw) from an external computer. The external computer is connected to an OptiTrak vision sensing setup using the Motive GUI. Matlab will be used to initiate the experiments on the external computer. This code will use data pulled from Motive, process the data, encode the data using an ASCII format, and send the data to the Network. The micro-computer will be using Python programming and will receive the data, decode the data, compute the control parameters, override the channels to approriate commands.

Control method used is a PID controller on a virtual leader for the single agent commands.

# Main Functions
main.py implements a control algorithm that uses vehicle.channel.overrides{} to send {ROLL,PITCH,THROTTLE,YAW} commands to the PX4-cube flight controller using the --connect method --> This program runs but has not been tested for proper copter control.

The takeoff, hover, and landing sequences have been tested and verified. The flocking sequence is currently being tested.
