function [xx,yy,zz] = sendState(q,p,theta,id,lead_data,flight_seq)

% PLEASE READ
% Please note this function works when a udp bordacast has been established.
% This can be done use the following lines of code in your main script.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Establish the broadcast signal to the network and define the port --> open port 5001
%global u
%u = udp('192.168.0.255',5001);
%fopen(u);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global u

% Create the necessary structs
position.x = round(q(1),5);
position.y = round(q(2),5);
position.z = round(q(3),5);
velocity.vx = round(p(1),5);
velocity.vy = round(p(2),5);
velocity.vz = round(p(3),5);
attitude.roll = round(theta(1),6);
attitude.pitch = round(theta(2),6);
attitude.yaw = round(theta(3),6);
leader.qgx = 1.5;
leader.qgy = lead_data;
leader.qgz = -0.5;

% Update the temporary state
scooby.timestamp = 0;
scooby.ID = id;
scooby.position = position;
scooby.velocity = velocity;
scooby.leader = leader;
scooby.attitude = attitude;
scooby.flightSeq = flight_seq;


msg = jsonencode(scooby)



% Send the data to the network
try
   fwrite(u,msg)
catch e
   fclose(u);
   delete(u);
   print('error')
   clear u
end
   
% Message = 'Message Sent';
clear msg
end