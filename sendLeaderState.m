function [xx,yy,zz] = sendLeaderState(qg,pg)

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
xx = ['0','0','0','0'];
yy = ['0','0','0','0'];
zz = ['0','0','0','0'];

% Write the current state to the data string 
msg = '0,+,0,+,0,+,0,+,0,+,0,+,0'; % This is simulating that we have the information for one 
%                                       agent in the format 
%                                       'rigidBody.ID, +/- , x , +/- , y , +/- , z , +/- , vx , +/- , vy , +/- , vz '
%                                       rigidBody.ID = 0 --> Leader Dynamics

% Update X value to msg
if qg(1)>0
    msg(3) = '+';
elseif qg(1)<0
    msg(3) = '-';
end
xx(1,1) = msg(3); %For debugging purposes
msg(5) = int8(abs(qg(1)));
if abs(qg(1)) >= 127
    msg(5) = int8(126);
elseif msg(5) == ','
    msg(5) = int8(127);
end 
xx(1,2) = msg(5); % For debugging purposes

%Update Y value to msg
if qg(2)>0
    msg(7) = '+';
elseif qg(2)<0
    msg(7) = '-';
end
yy(1,1) = msg(7); % For debugging purposes
msg(9) = int8(abs(qg(2)));
if abs(qg(2)) >= 127
    msg(9) = int8(126);
elseif msg(9) == ','
    msg(9) = int8(127);
end 
yy(1,2) = msg(9); %For debugging purposes

%Update Z value to msg
if qg(3)>0
    msg(11) = '+';
elseif qg(3)<0
    msg(11) = '-';
end
zz(1,1) = msg(11); % For debugging purposes
msg(13) = int8(abs(qg(3)));
if abs(qg(3)) >= 127
    msg(13) = int8(126);
elseif msg(13) == ','
    msg(13) = int8(127);
end
zz(1,2) = msg(13); %For debugging purposes

%Update VX value to msg
if pg(1)>0
    msg(15) = '+';
elseif pg(1)<0
    msg(15) = '-';
end
xx(1,3) = msg(15); %For debugging purposes
msg(17) = int8(abs(pg(1)));
if abs(pg(1)) >= 127
    msg(17) = int8(126);
elseif msg(17) == ','
    msg(17) = int8(127);
end 
xx(1,4) = msg(17); % For debugging purposes

%Update VY value to msg
if pg(2)>0
    msg(19) = '+';
elseif pg(2)<0
    msg(19) = '-';
end
yy(1,3) = msg(19); % For debugging purposes
msg(21) = int8(abs(pg(2)));
if abs(pg(2)) >= 127
    msg(21) = int8(126);
elseif msg(21) == ','
    msg(21) = int8(127);
end 
yy(1,4) = msg(21); %For debugging purposes

%Update VZ value to msg
if pg(3)>0
    msg(23) = '+';
elseif pg(3)<0
    msg(23) = '-';
end
zz(1,3) = msg(23); % For debugging purposes
msg(25) = int8(abs(pg(3)));
if abs(qg(3)) >= 127
    msg(25) = int8(126);
elseif msg(25) == ','
    msg(25) = int8(127);
end
zz(1,4) = msg(25); %For debugging purposes



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

