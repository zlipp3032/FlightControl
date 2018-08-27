function thrustCurve(data,time,m)
    
    ka = 0.8; % accel filter gain
    Ts = 0.05; % sample time of system
    batt_volt = data(:,3,10);
    PWM_nrml = zeros(length(time),1);
    z_accel = zeros(length(time),1);
    thrust_actual = zeros(length(time),1);
    thrust_est = zeros(length(time),1);
    PWM_est = zeros(length(time),1);
    a_prev = 0;
    v_prev = 0;
    
    kthrust = 1.0; % Computed using a least square approach in excel with 20 data points
    
    for i = 1:length(time)
        PWM_nrml(i) = (data(i,3,6) - 1000)/1000;
        z_accel(i) = (1-ka)*(data(i,3,2) - v_prev)/Ts + ka*a_prev;
        thrust_actual(i) = (-z_accel(i) + 9.81)*m/(cos(data(i,1,3))*cos(data(i,2,3)));
        thrust_est(i) = kthrust*PWM_nrml(i)^2*batt_volt(i)^2;
        PWM_est(i) = 1000*sqrt(data(i,3,1)/(kthrust*batt_volt(i)^2)) + 1000;
        clear a_prev v_prev
        a_prev = z_accel(i);
        v_prev = data(i,3,2);
    end

%     thrust_error = 
    
    figure()
    plot(time,data(:,3,1),time,thrust_actual,time,thrust_est)
    figure()
    plot(time,data(:,3,6),time,PWM_est)
    

end