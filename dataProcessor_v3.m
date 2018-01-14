function [xData,yData,zData,vxData,vyData,vzData,relTime,scaleTime,uData] = dataProcessor_v3(data)
% data = '/Users/zlipp3032/Documents/MastersThesisUAS/Code/ComTests/Control/junk/2018_01_12__23_10_12_log.csv';

% Import data from specified path
A = importdata(data);

% Define the points
relTime = A.data(:,1); %(s) Time stamp since the code was initialized
% ID = A.data(:,1);
xData = A.data(:,3);
yData = A.data(:,4);
zData = A.data(:,5);
vxData = A.data(:,6);
vyData = A.data(:,7);
vzData = A.data(:,8);
uData(:,1) = -A.data(:,12);
uData(:,2) = -A.data(:,13);
uData(:,3) = -A.data(:,14);
scaleTime = max(relTime) - min(relTime);
% TIMESCALER = scaleTime/max(t);
% tscaler = t*TIMESCALER;
% tscale = zeros(length(t));
% tscale(1) = min(relTime);
% for i = 1:length(t)
%     tscale(i) = min(relTime)+t(i);
% end

end