% function [qg,pg] = leadDynamics_HOVER(t)
% 
% qg = zeros(length(t),3); % Allocate space to leader position vector --> (x,y,z)
% pg = zeros(length(t),3); % Allocate space to leader veloicity vector --> (vx,vy,vz)
% 
% % All units are in cm.
% qg(:,1) = 300.*ones(length(t),1);
% qg(:,2) = -250.*ones(length(t),1);
% qg(:,3) = 350.*ones(length(t),1);
% 
% end

function [qg,pg] = leadDynamics_HOVER(qgo,pgo,qdesired,pdesired)

    global STOPTIME Ts

    qg(1,:) = qgo;
    pg(1,:) = pgo;
    
    for j = 1:STOPTIME/Ts
        
%         e(j,:) = qdesired(j,:) - qg(j,:);
        %This is set up so the conroller is computed based on the error in
        %the leader's position verse the leader's desired position
        ug(j,:) = -(qg(j,:)-qdesired(j,:))-2*(pg(j,:)-pdesired(j,:));
        qg(j+1,:) = qg(j,:) + Ts.*pg(j,:) + (1/2)*Ts.*ug(j,:);
        pg(j+1,:) = pg(j,:) + Ts.*ug(j,:);
    end
    
    
end