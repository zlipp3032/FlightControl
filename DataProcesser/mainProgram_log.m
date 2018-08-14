%% Process data logged from python log script

clear
clc
close all

%! Choose which experiment you would like to analyze
n = 3;

%% Experiment ---> Experiment_11_10
% Agent 1 - 192.168.2.2
dataPathAg1 = '/Users/zlipp3032/Documents/MastersThesisUAS/Test Results/Experiment_11_10/Agent1/';
exp11 = '2018_08_14__16_07_12_log';
exp12 = '2018_08_14__16_11_11_log';
exp13 = '2018_08_14__16_16_24_log';

% Agent 2 - 192.168.2.1 
dataPathAg2 = '/Users/zlipp3032/Documents/MastersThesisUAS/Test Results/Experiment_11_10/Agent2/';
exp21 = '2018_08_14__16_08_07_log';
exp22 = '2018_08_14__16_12_21_log';
exp23 = '2018_08_14__16_17_39_log';

switch n
    case 1
        exp1 = exp11;
        exp2 = exp21;
        startFlocking = 5.94;
        endFlocking = 21.15;
        d = 1.7;
    case 2
        exp1 = exp12;
        exp2 = exp22;
        startFlocking = 6.5;
        endFlocking = 30.28;
        d = 1.7;
    case 3
        exp1 = exp13;
        exp2 = exp23;
        startFlocking = 6.44;
        endFlocking = 42.8;
        d = 1.7;      
    otherwise
        disp('Incorrect Experiment')
end
%% Receive and Decode the Data from path ----> 'C:\Users\local_admin\Documents\MATLAB\zsl\Experiment_11_2\Experiment_xx_yy\AgentN\'
% Data path --> Data received from python script
dataPath1 = [dataPathAg1 exp1 '.csv'];
[rcvData1,relTime1,NeighSet1] = dataProcessor_log_flock(dataPath1);
% [rcvDataProcess] = processData(rcvData,relTime);

deltaDist1 = NeighSet1(:,:,1) - NeighSet1(:,:,2);
for h = 1:length(relTime1)
%     processedData(h,:) = scaleAmp .* (rcvData(h,:,3) + shiftData);
    time1(h,:) = relTime1(h) - relTime1(1);
    distAgent1(h) = sqrt(deltaDist1(h,4)^2 + deltaDist1(h,5)^2 + deltaDist1(h,6)^2);
end



% Data path --> Data received from python script
dataPath2 = [dataPathAg2 exp2 '.csv'];
[rcvData2,relTime2,NeighSet2] = dataProcessor_log_flock(dataPath2);
% [rcvDataProcess] = processData(rcvData,relTime);

deltaDist2 = NeighSet2(:,:,2) - NeighSet2(:,:,1);
for h = 1:length(relTime2)
%     processedData(h,:) = scaleAmp .* (rcvData(h,:,3) + shiftData);
    time2(h,:) = relTime2(h) - relTime2(1);
    distAgent2(h) = sqrt(deltaDist2(h,4)^2 + deltaDist2(h,5)^2 + deltaDist2(h,6)^2);
end

%% Post Processers
% [leaderPosition1,leaderPosition2] = postProcessing(NeighSet1,NeighSet2,time1,time2);
% [leaderPosition1,leaderPosition2] = postProcessing(NeighSet1,rcvData1,time1,time1);

% uk1 = flockingProcessor1_log(rcvData1,NeighSet1,time1,leaderPosition1);

% uk2 = flockingProcessor2_log(rcvData2,NeighSet2,time2,leaderPosition2);

% figure()
% subplot(3,1,1)
% plot(time1,uk1(1,:),'b',time2,uk2(1,:),'r')
% title('Flocking Desired Accelerations')
% xlabel('Time (s)')
% ylabel('X Accel (m/s/s)')
% grid('on')
% subplot(3,1,2)
% plot(time1,uk1(2,:),'b',time2,uk2(2,:),'r')
% xlabel('Time (s)')
% ylabel('Y Accel (m/s/s)')
% grid('on')
% subplot(3,1,3)
% plot(time1,uk1(3,:),'b',time2,uk2(2,:),'r')
% xlabel('Time (s)')
% ylabel('Z Accel (m/s/s)')
% grid('on')
% legend('Agent 1','Agent 2')


%# Average Distance from the leader

Leader_Average_Power = 0;
Agent_Average_Power = 0;
shaggy = find(time1 > startFlocking);
scooby = find(time1 < endFlocking);
good_time = scooby(end) - shaggy(1);

avgDistLeader = ((rcvData1(:,1:3,11) - NeighSet1(:,4:6,1)) + (rcvData1(:,1:3,11) - NeighSet1(:,4:6,2)))/2;
for i = 1:length(time1)
    leadNorm(i) = sqrt(avgDistLeader(i,1)^2 + avgDistLeader(i,2)^2 + avgDistLeader(i,3)^2) ;
end

for i = shaggy(1):scooby(end)
    Leader_Average_Power = Leader_Average_Power + leadNorm(i);
    Agent_Average_Power = Agent_Average_Power + distAgent1(i);
end

Leader_Average_Power = Leader_Average_Power/good_time
Agent_Average_Power = d - Agent_Average_Power/good_time


%% Plot the Single Agent Results
% plotStuff(rcvData1,time1,endFlocking)
% plotStuff(rcvData2,time2,endFlocking)




%% Plots the two Agent Results

int = 'interpreter';
la = 'latex';
fsize = 12;
scale = 100;
rick = time1(1);
carl = endFlocking;time1(end);


% posError1 = rcvData1(:,:,4) - rcvData1(:,:,5);
% posError2 = rcvData2(:,:,4) - rcvData2(:,:,5);
% 
% figure()
% subplot(3,1,1)
% plot(time1,posError1(:,1,1),'b -',time2,posError2(:,1,1),'r -','linewidth',1.2)
% title('Position Error',int,la,'FontSize',fsize)
% xlabel('Time (s)',int,la,'FontSize',fsize)
% ylabel('X Pos Error',int,la,'FontSize',fsize)
% xlim([0 carl])
% grid('on')
% subplot(3,1,2)
% plot(time1,posError1(:,2,1),'b -',time2,posError2(:,2,1),'r -','linewidth',1.2)
% xlim([0 carl])
% xlabel('Time (s)',int,la,'FontSize',fsize)
% ylabel('Y Pos Error',int,la,'FontSize',fsize)
% grid('on')
% subplot(3,1,3)
% plot(time1,posError1(:,3,1),'b -',time2,posError2(:,3,1),'r -','linewidth',1.2)
% xlabel('Time (s)',int,la,'FontSize',fsize)
% ylabel('Z Pos Error',int,la,'FontSize',fsize)
% xlim([0 carl])
% legend('Agent 1','Agent 2')
% grid('on')

figure()
subplot(3,1,1)
plot(time1,rcvData1(:,1,4),'b -',time2,rcvData2(:,1,4),'r -',time1,rcvData1(:,1,11),'g --',time2,rcvData2(:,1,11),'k --','linewidth',1.2)
title('Position',int,la,'FontSize',fsize)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('X Pos (m)',int,la,'FontSize',fsize)
xlim([rick carl])
grid('on')
subplot(3,1,2)
plot(time1,rcvData1(:,2,4),'b -',time2,rcvData2(:,2,4),'r -',time1,rcvData1(:,2,11),'k --',time2,rcvData2(:,2,11),'k --','linewidth',1.2)
xlim([rick carl])
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Y Pos (m)',int,la,'FontSize',fsize)
grid('on')
subplot(3,1,3)
plot(time1,rcvData1(:,3,4),'b -',time2,rcvData2(:,3,4),'r -',time1,rcvData1(:,3,11),'k --',time2,rcvData2(:,3,11),'k --','linewidth',1.2)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Z Pos (m)',int,la,'FontSize',fsize)
xlim([rick carl])
legend('Agent 1','Agent 2','Flock Leader')
grid('on')

dVector = d.*ones(2,1);
t = [time1(1),time1(end)];

figure()
plot(time1,distAgent1,'b -',t,dVector,'k --','linewidth',1.2)
title('Inter-Agent Distances',int,la,'FontSize',fsize)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Inter-Agent Distance (m)',int,la,'FontSize',fsize)
xlim([rick carl])
ylim([0 3])
legend('Distance','Desired Distance')
grid('on')

figure()
plot(time1,leadNorm,'b -','linewidth',1.2)
title('Avg Flock Distance to Leader',int,la,'FontSize',fsize)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Distance (m)',int,la,'FontSize',fsize)
xlim([rick carl])
ylim([0 1])
grid('on')






