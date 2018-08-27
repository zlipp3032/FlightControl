function [rcvData,time,distAgent] = getData(dataPath)

    %# Get data from the flight logs
    [rcvData,relTime,NeighSet] = dataProcessor_log_flock(dataPath);
    
    %# Get flight time and inter-agent distances
    time = zeros(length(relTime),1);
    distAgent = zeros(length(relTime),1);
    deltaDist = NeighSet(:,:,1) - NeighSet(:,:,2);
    
    for h = 1:length(relTime)
        time(h,:) = relTime(h) - relTime(1);
        distAgent(h) = sqrt(deltaDist(h,4)^2 + deltaDist(h,5)^2 + deltaDist(h,6)^2);
    end

end