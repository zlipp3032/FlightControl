function [Data,relTime,NeighborSet] = dataProcessor_log(data)
% data = '/Users/zlipp3032/Documents/MastersThesisUAS/Code/ComTests/Control/junk/2018_01_12__23_10_12_log.csv';

% Import data from specified path
A = importdata(data);

% Define the points
relTime = A.data(:,1); %(s) Time stamp since the code was initialized
Data = zeros(length(relTime),4,10);
NeighborSet = zeros(length(relTime),6,2);

% Import the Data
% ID = A.data(:,2);
Data(:,1,1) = A.data(:,3); % Recv Roll Command
Data(:,2,1) = A.data(:,4); % Recv Pitch Command
Data(:,3,1) = A.data(:,5); % Recv Throttle Command
Data(:,4,1) = A.data(:,6); % Recv Yaw Command
Data(:,1,2) = A.data(:,7); % Recv Velocity Vx
Data(:,2,2) = A.data(:,8); % Recv Velocity Vy
Data(:,3,2) = A.data(:,9); % Recv Velocity Vz
Data(:,1,3) = A.data(:,10); % Recv Roll
Data(:,2,3) = A.data(:,11); % Recv Pitch
Data(:,3,3) = A.data(:,12); % Recv Yaw
Data(:,1,4) = A.data(:,13); % Recv X Pos
Data(:,2,4) = A.data(:,14); % Recv Y Pos
Data(:,3,4) = A.data(:,15); % Recv Z Pos
Data(:,1,5) = A.data(:,16); % Leader X Pos
Data(:,2,5) = A.data(:,17); % Leader Y Pos
Data(:,3,5) = A.data(:,18); % Leader Z Pos
Data(:,1,6) = A.data(:,19); % Actual Roll PWM
Data(:,2,6) = A.data(:,20); % Actual Pitch PWM
Data(:,3,6) = A.data(:,21); % Actual Throttle PWM
Data(:,4,6) = A.data(:,22); % Actual Yaw PWM
Data(:,1,7) = A.data(:,23); % PD X Acceleration
Data(:,2,7) = A.data(:,24); % PD Y Acceleration
Data(:,3,7) = A.data(:,25); % PD Z Acceleration
Data(:,1,8) = A.data(:,26); % X Velocity Estimate
Data(:,2,8) = A.data(:,27); % Y Velocity Estimate
Data(:,3,8) = A.data(:,28); % Z Velocity Estimate
Data(:,1,9) = A.data(:,29); % ACommanded Roll PWM
Data(:,2,9) = A.data(:,30); % Commanded Pitch PWM
Data(:,3,9) = A.data(:,31); % Commanded Throttle PWM
Data(:,4,9) = A.data(:,32); % Commanded Yaw PWM
Data(:,1,10) = A.data(:,33); % z integrator error in velocity

NeighborSet(:,1,1) = A.data(:,35); % Agent 1 X Velocity
NeighborSet(:,2,1) = A.data(:,36); % Agent 1 Y Velocity
NeighborSet(:,3,1) = A.data(:,37); % Agent 1 Z Velocity
NeighborSet(:,4,1) = A.data(:,38); % Agent 1 X Position
NeighborSet(:,5,1) = A.data(:,39); % Agent 1 Y Position
NeighborSet(:,6,1) = A.data(:,40); % Agent 1 Z Position

NeighborSet(:,1,2) = A.data(:,43); % Agent 2 X Velocity
NeighborSet(:,2,2) = A.data(:,44); % Agent 2 Y Velocity
NeighborSet(:,3,2) = A.data(:,45); % Agent 2 Z Velocity
NeighborSet(:,4,2) = A.data(:,46); % Agent 2 X Position
NeighborSet(:,5,2) = A.data(:,47); % Agent 2 Y Position
NeighborSet(:,6,2) = A.data(:,48); % Agent 2 Z Position

% scaleTime = max(relTime) - min(relTime);


end