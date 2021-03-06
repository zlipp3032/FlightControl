function plotStuff(rcvData,time)

int = 'interpreter';
la = 'latex';
fsize = 12;
scale = 100;
rick = time(1);
carl = time(end);

figure()
subplot(3,1,1)
plot(time,rcvData(:,1,1),'b -',time,rcvData(:,1,3),'r --','linewidth',1.2)
title('Attitude',int,la,'FontSize',fsize)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Roll (rad)',int,la,'FontSize',fsize)
xlim([rick carl])
grid('on')
subplot(3,1,2)
plot(time,rcvData(:,2,1),'b -',time,rcvData(:,2,3),'r --','linewidth',1.2)
xlim([rick carl])
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Pitch (rad)',int,la,'FontSize',fsize)
grid('on')
subplot(3,1,3)
plot(time,rcvData(:,3,1),'b -',time,rcvData(:,3,3),'r --','linewidth',1.2)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Thrust (N)',int,la,'FontSize',fsize)
xlim([rick carl])
legend('Command','Actual')
grid('on')

figure()
subplot(3,1,1)
plot(time,rcvData(:,1,9),'b .',time,rcvData(:,1,6),'r .','linewidth',1.2)
title('Channel PWM Commands',int,la,'FontSize',fsize)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Roll Override',int,la,'FontSize',fsize)
xlim([rick carl])
grid('on')
subplot(3,1,2)
plot(time,rcvData(:,2,9),'b .',time,rcvData(:,2,6),'r .','linewidth',1.2)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Pitch Override',int,la,'FontSize',fsize)
xlim([rick carl])
grid('on')
subplot(3,1,3)
plot(time,rcvData(:,3,9),'b .',time,rcvData(:,3,6),'r .','linewidth',1.2)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Throttle Override',int,la,'FontSize',fsize)
xlim([rick carl])
legend('Command','Actual')
grid('on')


figure()
subplot(3,1,1)
plot(time,rcvData(:,1,2),'r -',time,rcvData(:,1,8),'b -','Linewidth',1.2)
title('Velocity',int,la,'FontSize',fsize)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Vx (m/s)',int,la,'FontSize',fsize)
xlim([rick carl])
grid on
subplot(3,1,2)
plot(time,rcvData(:,2,2),'r -',time,rcvData(:,2,8),'b -','Linewidth',1.2)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Vy (m/s)',int,la,'FontSize',fsize)
xlim([rick carl])
grid('on')
subplot(3,1,3)
plot(time,rcvData(:,3,2),'r -',time,rcvData(:,3,8),'b -','Linewidth',1.2)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Vz (m/s)',int,la,'FontSize',fsize)
xlim([rick carl])
grid('on')
legend('Actual','Command')


figure()
subplot(3,1,1)
plot(time,rcvData(:,1,4),'b -',time,rcvData(:,1,5),'k --',time,rcvData(:,1,11),'g --','linewidth',1.2)
title('Local Position (NED)',int,la,'FontSize',fsize)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('X Position (m)',int,la,'FontSize',fsize)
xlim([rick carl])
grid('on')
subplot(3,1,2)
plot(time,rcvData(:,2,4),'b -',time,rcvData(:,2,5),'k --',time,rcvData(:,2,11),'g --','linewidth',1.2)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Y Position (m)',int,la,'FontSize',fsize)
xlim([rick carl])
grid('on')
subplot(3,1,3)
plot(time,rcvData(:,3,4),'b -',time,rcvData(:,3,5),'k --',time,rcvData(:,3,11),'g --','linewidth',1.2)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Z Position (m)',int,la,'FontSize',fsize)
xlim([rick carl])
grid('on')
legend('Actual','Hover Desired','Multi Desired')

figure()
subplot(3,1,1)
plot(time,rcvData(:,1,7),'linewidth',1.2)
title('Desired Acceleration',int,la,'FontSize',fsize)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Ux (m/s/s)',int,la,'FontSize',fsize)
xlim([rick carl])
grid('on')
subplot(3,1,2)
plot(time,rcvData(:,2,7),'linewidth',1.2)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Uy (m/s/s)',int,la,'FontSize',fsize)
xlim([rick carl])
grid('on')
subplot(3,1,3)
plot(time,rcvData(:,3,7),'linewidth',1.2)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Uz (m/s/s)',int,la,'FontSize',fsize)
xlim([rick carl])
grid('on')

figure()
subplot(3,1,1)
plot(time,rcvData(:,1,10),'linewidth',1.2)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Vel Int Term (m/s)',int,la,'FontSize',fsize)
xlim([rick carl])
grid('on')
subplot(3,1,2)
plot(time,rcvData(:,2,10),'linewidth',1.2)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Pos Int Term (m)',int,la,'FontSize',fsize)
xlim([rick carl])
grid('on')
subplot(3,1,3)
plot(time,rcvData(:,3,10),'linewidth',1.2)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Batt Percent',int,la,'FontSize',fsize)
xlim([rick carl])
grid('on')


end