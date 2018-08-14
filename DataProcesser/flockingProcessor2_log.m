function u_flock = flockingProcessor2_log(Data,NeighSet,time,leaderPosition)

%# I am not harrassing Jess!
Ts = 0.05; % Sample Rate of the code
qhati = zeros(3,length(time));
qhatj = zeros(3,length(time));
dqhat = zeros(3,length(time));
qNorm = zeros(length(time),1);
Phi = zeros(length(time),1);
AR = zeros(3,length(time));
VC = zeros(3,length(time));
GT = zeros(3,length(time));
FC = zeros(3,length(time));
u_flock = zeros(3,length(time));

%# Flocking Gains need to match those in the Python script!!!
alpha1 = 0.001;
alpha2 = 0.3;
beta = 0.4;
gamma1 = 0.4;
gamma2 = 0.7;
gamma3 = 0.44;
gamma4 = 0.9;
d = 3.0;
Ei = 2;

kGamma1 = [gamma1 0 0; 0 gamma1 0; 0 0 gamma3];
kGamma2 = [gamma2 0 0; 0 gamma2 0; 0 0 gamma4];
%# Note that the order of subtraction needs to match with the proper
%# agent!!!
deltaData = NeighSet(:,:,1) - NeighSet(:,:,2); 

for i = 1:length(time)
    qhati(:,i) = Data(i,1:3,4)' + Ts.*Data(i,1:3,2)';%NeighSet(i,4:6,1) + Ts.*NeighSet(i,1:3,1);
    qhatj(:,i) = NeighSet(i,4:6,1) + Ts.*NeighSet(i,1:3,1);
    dqhat(:,i) = qhatj(:,i) - qhati(:,i);
    qNorm(i) = sqrt((dqhat(1,i))^2 + (dqhat(2,i))^2 + (dqhat(3,i))^2);   
    %# Attraction / Repulsion
    Phi(i) = alpha2/(alpha1 + 1) - alpha2/(alpha1+((qNorm(i))^2/d^2));
    AR(:,i) = Phi(i).*deltaData(i,4:6);
    %# Velocity Consensus
    VC(:,i) = beta.*deltaData(i,1:3);
    %# Guidance Term
    GT(:,i) = kGamma1*(leaderPosition(i,4:6)' - Data(i,1:3,4)') + kGamma2*([0;0;0] - Data(i,1:3,2)');%kGamma1*(Data(i,1:3,5)' - Data(i,1:3,4)') + kGamma2*([0;0;0] - Data(i,1:3,2)');
    %#  Flock Correction to Guidance
    FC(:,i) = (1/Ei).*kGamma1*deltaData(i,4:6)' + (1/Ei).*kGamma2*deltaData(i,1:3)';
    %# Compute flocking Control
    u_flock(:,i) = AR(:,i) + VC(:,i) + GT(:,i) - FC(:,i);   
end

% figure()
% subplot(3,1,1)
% plot(time,u_flock(1,:),'r --',time,Data(:,1,7),'b -','linewidth',1.2)
% title('Flocking Controller Validation')
% xlabel('Time (s)')
% ylabel('X Accel (m/s/s)')
% grid on
% subplot(3,1,2)
% plot(time,u_flock(2,:),'r --',time,Data(:,2,7),'b -','linewidth',1.2)
% xlabel('Time (s)')
% ylabel('Y Accel (m/s/s)')
% grid on
% subplot(3,1,3)
% plot(time,u_flock(3,:),'r --',time,Data(:,3,7),'b -','linewidth',1.2)
% xlabel('Time (s)')
% ylabel('Z Accel (m/s/s)')
% grid on
% legend('Flocking','Actual')

% Data(:,1:3,5) is the Leader Data


end