% Process data logged from python log script --> Be sure to 

clear
clc
close all


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% This section is used for processing %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Agent 2 - 192.168.2.1
% Good Data
% 2018_07_31__15_34_02_log_1
% Bad Data
% 2018_07_31__15_25_12_log_1

% Agent 1 - 192.168.2.2 
% Good Data
% 2018_07_31__15_32_56_log_2
% Bad Data
% 2018_08_01__15_43_53_log_2

% Data path --> Data received from python script
dataPath1 = 'C:\Users\local_admin\Documents\MATLAB\zsl\UP_Data\2018_08_03__17_01_47_log.csv';
[rcvData1,relTime1,NeighSet1] = dataProcessor_log(dataPath1);


% [rcvDataProcess] = processData(rcvData,relTime);

% % Data path --> Data received from python script
% dataPath2 = 'C:\Users\local_admin\Documents\MATLAB\zsl\Data\2018_07_31__15_25_12_log_1.csv';
% [rcvData2,relTime2,scaleTime2] = dataProcessor_log(dataPath2);
% % [rcvDataProcess] = processData(rcvData,relTime);

for h = 1:length(relTime1)
%     processedData(h,:) = scaleAmp .* (rcvData(h,:,3) + shiftData);
    time1(h,:) = relTime1(h) - relTime1(1);
end
% flockingProcessor_log(rcvData1,NeighSet1,time1)


% 
% for h = 1:length(relTime2)
% %     processedData(h,:) = scaleAmp .* (rcvData(h,:,3) + shiftData);
%     time2(h,:) = relTime2(h) - relTime2(1);
% end


int = 'interpreter';
la = 'latex';
fsize = 12;
scale = 100;
carl = time1(end);

%% Plots the Position Errors
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




plotStuff(rcvData1,time1)
% plotStuff(rcvData2,time2)


