function [delq,delp] = decodeData_v3(x,y,z,d)

reso = d/126; %(cm) Resolution of the data sent
delq = zeros(1,3);
delp = zeros(1,3);

% Decode the x-data
if x(2) ~= 127
    if x(1) == 43
        delq(1) = x(2)*reso;
    else %y1 == 45
        delq(1) = -x(2)*reso;
    end
else %z1 = 127
    if x(1) == 43
        delq(1) = 44*reso;
    else % z1 == 45
        delq(1) = -44*reso;
    end
end     
    
% Decode the y-data    
if y(2) ~= 127
    if y(1) == 43
        delq(2) = y(2)*reso;
    else %y1 == 45
        delq(2) = -y(2)*reso;
    end
else %z1 = 127
    if y(1) == 43
        delq(2) = 44*reso;
    else % z1 == 45
        delq(2) = -44*reso;
    end
end    

% Decode the z-data
if z(2) ~= 127
    if z(1) == 43
        delq(3) = z(2)*reso;
    else % z1 == 45
        delq(3) = -z(2)*reso;
    end
else %z1 = 127
    if z(1) == 43
        delq(3) = 44*reso;
    else % z1 == 45
        delq(3) = -44*reso;
    end
end
    
% Decode the vx-data
if x(4) ~= 127
    if x(3) == 43
        delp(1) = x(4)*reso;
    else %y1 == 45
        delp(1) = -x(4)*reso;
    end
else %z1 = 127
    if x(3) == 43
        delp(1) = 44*reso;
    else % z1 == 45
        delp(1) = -44*reso;
    end
end     
    
% Decode the vy-data    
if y(4) ~= 127
    if y(3) == 43
        delp(2) = y(4)*reso;
    else %y1 == 45
        delp(2) = -y(4)*reso;
    end
else %z1 = 127
    if y(3) == 43
        delp(2) = 44*reso;
    else % z1 == 45
        delp(2) = -44*reso;
    end
end    

% Decode the vz-data
if z(4) ~= 127
    if z(3) == 43
        delp(3) = z(4)*reso;
    else % z1 == 45
        delp(3) = -z(4)*reso;
    end
else %z1 = 127
    if z(3) == 43
        delp(3) = 44*reso;
    else % z1 == 45
        delp(3) = -44*reso;
    end
end

end