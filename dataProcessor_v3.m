function [Data,relTime,scaleTime,uData,posData,lData] = dataProcessor_v3(data,qo,po)
% data = '/Users/zlipp3032/Documents/MastersThesisUAS/Code/ComTests/Control/junk/2018_01_12__23_10_12_log.csv';

% Import data from specified path
A = importdata(data);

% Define the points
relTime = A.data(:,1); %(s) Time stamp since the code was initialized
Data = zeros(length(relTime),3,2);
posData = zeros(length(relTime),3,2);

% Import the Data
% ID = A.data(:,2);
Data(:,1,1) = A.data(:,3); %RecvX
Data(:,2,1) = A.data(:,4); %RecvY
Data(:,3,1) = A.data(:,5); %RecvZ
Data(:,1,2) = A.data(:,6); %RecvVx
Data(:,2,2) = A.data(:,7); %RecvVy
Data(:,3,2) = A.data(:,8); %%RecvVz
lData(:,1) = A.data(:,9);
lData(:,2) = A.data(:,10);
lData(:,3) = A.data(:,11);
uData(:,1) = -A.data(:,12);
uData(:,2) = -A.data(:,13);
uData(:,3) = -A.data(:,14);
scaleTime = max(relTime) - min(relTime);

% Compute the theoretical next position based on the control input
qi = qo;
pi = po;
to = min(relTime);
for i = 1:length(relTime)
    DT = relTime(i,1) - to;
    posData(i,:,1) = qi+DT.*pi+(1/2)*DT^2.*uData(i,:);
    posData(i,:,2) = pi+DT.*uData(i,:);
    clear qi pi to
    qi = posData(i,:,1);
    pi = posData(i,:,2);
    to = relTime(i,1);
end

end