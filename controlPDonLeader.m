function u = controlPDonLeader(q,p,qg,pg)

% kp = 1; % Proportional Gain
% kd = 0.1;% Derivative Gain

global kp kd
u = kp*(qg-q)+kd*(pg-p);

end