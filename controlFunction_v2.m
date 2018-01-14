% Example for PD controller on Leaader position and velocity

clear
clc
close all

global STOPTIME Ts kp kd

STOPTIME = 250;
Ts = 0.1;
TIME = [0:Ts:STOPTIME];

% Establish the broadcast signal to the network and define the port --> open port 5001
global u
u = udp('192.168.0.255',5001);
fopen(u);

% Allocate memory to known matrices
q = zeros(length(TIME),3);
p = zeros(length(TIME),3);
encq = zeros(length(TIME),3);
encp = zeros(length(TIME),3);
encqg = zeros(length(TIME),3);
encpg = zeros(length(TIME),3);
stringer = zeros(length(TIME),4,3);
stringerLeader = zeros(length(TIME),4,3);
scoobiedoo = ones(length(TIME)); % This value is used in the computation of th z position

% Set the initial conditions for the rigid body
qo = [20 24 0]; %Arranged in x,y,z fashion
po = [0 0 0]; %Arranged in vx,vy,vz fashion --> Assumed vehicle starts from rest
qi = qo;
pi = po;

% Leader Stuff
leaderamp = [100 -150 250];
velamp = 5;
qgo = [0 0 0];
pgo = [0 0 0];

% %Leader Trajectory --> Hover
% qgdesired = [leaderamp(1).*ones(length(TIME),1) leaderamp(2).*ones(length(TIME),1) leaderamp(3).*ones(length(TIME),1)];
% pgdesired = zeros(length(TIME),3);
% Lead Trajectory -- > Circle
qgdesired = [leaderamp(1).*sin(TIME)' leaderamp(2).*cos(TIME)' zeros(length(TIME),1)]; %[leaderamp(1).*ones(length(TIME),1) leaderamp(2).*ones(length(TIME),1) leaderamp(3).*ones(length(TIME),1)];
pgdesired = velamp.*[cos(TIME)' -sin(TIME)' ((max(TIME).*ones(length(TIME),1))-TIME')./max(TIME)]; %zeros(length(TIME),3);
% Get the leader dynamics
[qg,pg] = leadDynamics_HOVER(qgo,pgo,qgdesired,pgdesired);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% This section is used for processing %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Data path --> Data received from python script
% data = 'C:\Users\Zack\Desktop\MastersThesisUAS\SOLOCode\ComTests\ControlCom\junk\2018_01_13__15_53_15_log.csv';
% [xData,yData,zData,vxData,vyData,vzData,relTime,scaleTime] = dataProcessor_v2(data);
% TIMESCALER = scaleTime/max(TIME);
% tscale = TIME.*TIMESCALER;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% This section is used for processing %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The velocity filter gain used in the Backgwards Euler computation of the
%       velocity
velFilterGain = 0.1;

% This is the upper bound on the spatial  dimensions used in simulation
%       (e.g. let qmax:=[xmax,ymax,zmax], then d:=max(qmax)=max(max(xmax),max(ymax),max(zmax))). 
d = 500; %(cm) Longest distance used for determining resolution of movement

%Define the parameters used in the control
kp = 5; % Proportional Gain for PD controller
kd = 0.9; % Derivative Gain for PD controller

pause(3) % Wait three seconds for dramatic effect

for i = 1:length(TIME)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % tscale needs to be uncommented for processing purposes %
%    tscale(i) = tscale(i) + min(relTime);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ucont(i,:) = controlPDonLeader(qi,pi,qg(i,:),pg(i,:));
    q(i,:) = qi+Ts.*pi+(1/2)*Ts^2.*ucont(i,:);
    p(i,:) = pi+Ts.*ucont(i,:);
    
    % Call the functions
    [encqg(i,:),encpg(i,:)] = encodeData_v3(qg(i,:),pg(i,:),d);
    [encq(i,:),encp(i,:)] = encodeData_v3(q(i,:),p(i,:),d);
    [stringerLeader(i,:,1),stringerLeader(i,:,2),stringerLeader(i,:,3)] = sendLeaderState(encqg(i,:),encpg(i,:));     
    [stringer(i,:,1),stringer(i,:,2),stringer(i,:,3)] = sendState_v7(encq(i,:),encp(i,:)); 
%     [decq(i,:),decp(i,:)] = decodeData_v3(stringer(i,:,1),stringer(i,:,2),stringer(i,:,3),d);
    pause(0.02) %50Hz
    
    clear qi pi
    qi = q(i,:);
    pi = p(i,:);
    
end

fclose(u)
% figure(1)
% subplot(3,1,1)
% plot(TIME,q(:,1),'b --',TIME,qg(:,1),'g -',TIME,decq(:,1),'r :')
% % hold('on')
% % plot(relTime,xData,'- k')
% % hold('off')
% legend('Rigid Body Position','Leader Position','Decode Position')
% title('X Data')
% grid('on')
% subplot(3,1,2)
% plot(TIME,p(:,1),'b --',TIME,pg(:,1),'g -',TIME,decp(:,1),'r :')
% % hold('on')
% % plot(relTime,vxData,'- k')
% % hold('off')
% legend('Rigid Body Velocity','Leader Velocity','Decode Velocity')
% grid('on')
% subplot(3,1,3)
% plot(TIME,ucont(:,1),'b -')%,TIME,pg(:,3),'g -',TIME,decp(:,3),'r :')
% % hold('on')
% % plot(relTime,vxData,'- k')
% % hold('off')
% legend('Control')%,'Leader Velocity','Decode Velocity')
% grid('on')
% 
% 
% figure(2)
% subplot(3,1,1)
% plot(TIME,q(:,2),'b --',TIME,qg(:,2),'g -',TIME,decq(:,2),'r :')
% % hold('on')
% % plot(relTime,xData,'- k')
% % hold('off')
% legend('Rigid Body Position','Leader Position','Decode Position')
% title('Y Data')
% grid('on')
% subplot(3,1,2)
% plot(TIME,p(:,2),'b --',TIME,pg(:,2),'g -',TIME,decp(:,2),'r :')
% % hold('on')
% % plot(relTime,vxData,'- k')
% % hold('off')
% legend('Rigid Body Velocity','Leader Velocity','Decode Velocity')
% grid('on')
% subplot(3,1,3)
% plot(TIME,ucont(:,2),'b -')%,TIME,pg(:,3),'g -',TIME,decp(:,3),'r :')
% % hold('on')
% % plot(relTime,vxData,'- k')
% % hold('off')
% legend('Control')%,'Leader Velocity','Decode Velocity')
% grid('on')
% 
% 
% 
% figure(3)
% subplot(3,1,1)
% plot(TIME,q(:,3),'b --',TIME,qg(:,3),'g -',TIME,decq(:,3),'r :')
% % hold('on')
% % plot(relTime,xData,'- k')
% % hold('off')
% legend('Rigid Body Position','Leader Position','Decode Position')
% title('Z Data')
% grid('on')
% subplot(3,1,2)
% plot(TIME,p(:,3),'b --',TIME,pg(:,3),'g -',TIME,decp(:,3),'r :')
% % hold('on')
% % plot(relTime,vxData,'- k')
% % hold('off')
% legend('Rigid Body Velocity','Leader Velocity','Decode Velocity')
% grid('on')
% subplot(3,1,3)
% plot(TIME,ucont(:,3),'b -')%,TIME,pg(:,3),'g -',TIME,decp(:,3),'r :')
% % hold('on')
% % plot(relTime,vxData,'- k')
% % hold('off')
% legend('Control')%(,'Leader Velocity','Decode Velocity')
% grid('on')
