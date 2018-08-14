function [leaderPosition1,leaderPosition2] = postProcessing(NeighSet1,Data,time1,time2)

%# For agent 1
for i = 1:length(time1)
%    leaderPosition1(i,:) = (NeighSet1(1,:,1) + NeighSet1(1,:,2))/2;
   deltaDist1(i,:) = NeighSet1(i,:,1) - NeighSet1(i,:,2);
   d(i) = sqrt(deltaDist1(i,4)^2 + deltaDist1(i,5)^2 + deltaDist1(i,6)^2);
end

%# For agent 2
for i = 1:length(time2)
%    leaderPosition2(i,:) = (NeighSet2(i,:,1) + NeighSet2(i,:,2))/2;
   leaderPosition2(i,:) = (Data(i,:,1) + Data(i,:,2))/2;
   leaderPosition1(i,:) = Data(i,:,11);
end

% figure()
% subplot(3,1,1)
% plot(time1,leaderPosition1(:,1),'b',time2,leaderPosition2(:,1),'r')
% grid('on')
% subplot(3,1,2)
% plot(time1,leaderPosition1(:,2),'b',time2,leaderPosition2(:,2),'r')
% grid('on')
% subplot(3,1,3)
% plot(time1,leaderPosition1(:,3),'b',time2,leaderPosition2(:,3),'r')
% grid('on')

figure()
plot(time1,d)
grid on

end
