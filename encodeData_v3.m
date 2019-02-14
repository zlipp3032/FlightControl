function [encq,encp,enctheta] = encodeData_v3(q,p,theta,d);

% d = 500; %(cm) Longest distance used for determining resolution of movement
reso = d/126; %(cm) Resolution of the data sent
% encx = x/reso;
% ency = y/reso;
% encz = z/reso;
encq = q./reso;
% encvx = vx/reso;
% encvy = vy/reso;
% encvz = vz/reso;
encp = p./reso;
enctheta = theta./reso;
end
