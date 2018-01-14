function [xx,yy,zz] = sendState_v7(q,p)

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
msg = '1,+,0,+,0,+,0,+,0,+,0,+,0'; % This is simulating that we have the information for one 
%                                       agent in the format 
%                                       'rigidBody.ID, +/- , x , +/- , y , +/- , z , +/- , vx , +/- , vy , +/- , vz '
    
% Update X value to msg
if q(1)>0
    msg(3) = '+';
elseif q(1)<0
    msg(3) = '-';
end
xx(1,1) = msg(3); %For debugging purposes
msg(5) = int8(abs(q(1)));
if abs(q(1)) >= 127
    msg(5) = int8(126);
elseif msg(5) == ','
    msg(5) = int8(127);
end 
xx(1,2) = msg(5); % For debugging purposes

%Update Y value to msg
if q(2)>0
    msg(7) = '+';
elseif q(2)<0
    msg(7) = '-';
end
yy(1,1) = msg(7); % For debugging purposes
msg(9) = int8(abs(q(2)));
if abs(q(2)) >= 127
    msg(9) = int8(126);
elseif msg(9) == ','
    msg(9) = int8(127);
end 
yy(1,2) = msg(9); %For debugging purposes

%Update Z value to msg
if q(3)>0
    msg(11) = '+';
elseif q(3)<0
    msg(11) = '-';
end
zz(1,1) = msg(11); % For debugging purposes
msg(13) = int8(abs(q(3)));
if abs(q(3)) >= 127
    msg(13) = int8(126);
elseif msg(13) == ','
    msg(13) = int8(127);
end
zz(1,2) = msg(13); %For debugging purposes

%Update VX value to msg
if p(1)>0
    msg(15) = '+';
elseif p(1)<0
    msg(15) = '-';
end
xx(1,3) = msg(15); %For debugging purposes
msg(17) = int8(abs(p(1)));
if abs(p(1)) >= 127
    msg(17) = int8(126);
elseif msg(17) == ','
    msg(17) = int8(127);
end 
xx(1,4) = msg(17); % For debugging purposes

%Update VY value to msg
if p(2)>0
    msg(19) = '+';
elseif p(2)<0
    msg(19) = '-';
end
yy(1,3) = msg(19); % For debugging purposes
msg(21) = int8(abs(p(2)));
if abs(p(2)) >= 127
    msg(21) = int8(126);
elseif msg(21) == ','
    msg(21) = int8(127);
end 
yy(1,4) = msg(21); %For debugging purposes

%Update VZ value to msg
if p(3)>0
    msg(23) = '+';
elseif p(3)<0
    msg(23) = '-';
end
zz(1,3) = msg(23); % For debugging purposes
msg(25) = int8(abs(p(3)));
if abs(q(3)) >= 127
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

