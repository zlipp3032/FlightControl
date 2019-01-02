% Example for PD controller on Leaader position and velocity

clear
clc
close all

%global STOPTIME Ts kp kd

STOPTIME = 250;
Ts = 0.05;
time = [0:Ts:STOPTIME];

% Establish the broadcast signal to the network and define the port --> open port 5001
global u
u = udp('192.168.0.255',5001);
fopen(u);

q1 = zeros(length(time),3);
p1 = zeros(length(time),3);
theta = zeros(length(time),3);
q2 = zeros(length(time),3);
p2 = zeros(length(time),3);
flight_seq = 4;

for i = 1:length(time)
    
    q1(i,:) = [sin((2*pi/Ts)*0.001*time(i)) -cos((2*pi/Ts)*0.001*time(i)) -time(i)/100];
    p1(i,:) = [(2*pi/Ts)*0.001*cos((2*pi/Ts)*0.001*time(i)) -(2*pi/Ts)*0.001*sin((2*pi/Ts)*0.001*time(i)) -1/100];
    q2(i,:) = [cos((2*pi/Ts)*0.001*time(i)) -sin((2*pi/Ts)*0.001*time(i)) time(i)/100];
    p2(i,:) = [-(2*pi/Ts)*0.001*sin((2*pi/Ts)*0.001*time(i)) -(2*pi/Ts)*0.001*cos((2*pi/Ts)*0.001*time(i)) 1/100];
    lead_data(i) = -time(i)/100;
    
    sendState(q1(i,:),p1(i,:),theta(i,:),1,lead_data(i),flight_seq); 
    sendState(q2(i,:),p2(i,:),theta(i,:),2,lead_data(i),flight_seq);
%     disp(['Data Sent: ' + time(i)])
    pause(0.02) %50Hz

    
end

fclose(u)
