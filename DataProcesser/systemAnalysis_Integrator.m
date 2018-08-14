%% Design the PI gains for the velocity controller

% % % % % % 
% % % % % % clear
% % % % % % clc
% % % % % % 
% % % % % % % Choose Gains
% % % % % % 
% % % % % % kiw = 0.5;
% % % % % % kw = 1.5;
% % % % % % 
% % % % % % % Find 2nd order system parameters
% % % % % % wn = sqrt(kiw);
% % % % % % zeta = kw/(2*wn);
% % % % % % 
% % % % % % % CT 
% % % % % % xCT = roots([1 kw kiw]);
% % % % % % 
% % % % % % % DT
% % % % % % % Ts = 1/10;
% % % % % % % xDT = roots([1, -(2+kw*Ts)/(1+kw*Ts+(kiw*Ts)^2), 1/(1+kw*Ts+(kiw*Ts)^2)])
% % % % % % % 
% % % % % % % z(1) = (xCT(1)*Ts+2)/(2-xCT(1)*Ts)
% % % % % % % z(2) = (xCT(2)*Ts+2)/(2-xCT(2)*Ts)
% % % % % % 
% % % % % % break

%% Closed-Loop PD Design with Linear velocity control

clear
clc
close all

% this analysis does not include the integrator
k = 3.5;
kp = 0.04;%0.4;%0.9;%0.06;%0.03;%
kd = 0.32;%1.4;%2;%0.5;%%0.35;%

% Just considering the PD controller on the double-integrator, we can
% assume the model takes the form of a standard second order damped
% oscillator. Thus, we can design our PD gains based on this type of
% analysis.
% PD_poles = roots([1 kd kp])
zeta_PD = kd/(2*sqrt(kp))

% Poles with velocity controller included
roots([1, k+kd, k*kd+kp, k*kp])

Ts = 0.1;
s = tf('s');
G = (s+k)/(s*(s+k));
Gc = (kd*s+kp)/s;
Gzoh = 1/((Ts/2)*s+1);

% figure()
% rlocus(Gzoh*G*Gc)

Tcl = minreal((Gzoh*G*Gc)/(1+Gzoh*G*Gc))*((s+k)/(s+k));
% Scl = 1/(1+Gzoh*G*Gc)
% Tcl = ((s+k)*(kd*s+kp))/(s^3+(k+kd)*s^2+(k*kd+kp)*s+k*kp) % CT model with
                                                            % no 'zoh'
stepinfo(Tcl)

STOPTIME = 100;
time = [0:0.01:STOPTIME];
r = 2+0.*time; %sin(15.*time);
% d = 5 + sin(3.*time); %0.*time;
% w = [r;d]';
% CLTF = [T S*G];
x = lsim(Tcl,r,time);
e = r'-x;


int = 'interpreter';
la = 'latex';
lw = 1.5;
fsize = 12;
scale = 100;
endtime = 110;%time(end);

figure()
subplot(2,1,1)
plot(time,x,'b -','linewidth',lw)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Position (m)',int,la,'FontSize',fsize)
grid on
subplot(2,1,2)
plot(time,e,'b -','linewidth',lw)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Error (m)',int,la,'FontSize',fsize)
grid on



%% Closed-Loop PD Design with Linear velocity control

clear
clc
close all

% this analysis DOES include the integrator!!!!
kw = 1.5;
kiw = 0.5;
kp = 0.09;%0.4;%0.03;
kd = 0.2;%1.4;%0.35;

Ts = 0.1;
s = tf('s');
G = (s+kw+kiw/s)/(s*(s+kw+kiw/s));
Gc = (kd*s+kp)/s
% For DT, uncomment the below TF
Gzoh = 1;%1/((Ts/2)*s+1)

roots([1 (kiw+kd) (kiw+kd*kw+kp) (kd*kiw+kp*kw) kp*kiw])


% figure()
% rlocus(Gzoh*G*Gc)

Tcl = minreal((Gzoh*G*Gc)/(1+Gzoh*G*Gc))%minreal((Gzoh*G*Gc)/(1+Gzoh*G*Gc))
Scl = minreal(1/(1+Gzoh*G*Gc))*(((s+kw+kiw/s))/(s*(s+kw+kiw/s)))


STOPTIME = 20;
time = [0:0.001:STOPTIME];
r = 2+0.*time; %sin(15.*time);
% d = 5 + sin(3.*time); %0.*time;
% w = [r;d]';
% CLTF = [T S*G];
x = lsim(Tcl,r,time);
e = r'-x;

int = 'interpreter';
la = 'latex';
lw = 1.5;
fsize = 12;
scale = 100;
endtime = 110;%time(end);

figure()
subplot(2,1,1)
plot(time,x,'b -','linewidth',lw)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Position (m)',int,la,'FontSize',fsize)
grid on
subplot(2,1,2)
plot(time,e,'b -','linewidth',lw)
xlabel('Time (s)',int,la,'FontSize',fsize)
ylabel('Error (m)',int,la,'FontSize',fsize)
grid on







