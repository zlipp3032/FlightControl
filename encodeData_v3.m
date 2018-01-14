% This function is used to encode the spatial data obtained with a resolution obtained from the longest distance in the arena boundary and the number of integers in the ASCII index (I use 126 here because of a decoding issue in my Python script).
% Zachary Lippay

function [encq,encp] = encodeData_v3(q,p,d);

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
end

