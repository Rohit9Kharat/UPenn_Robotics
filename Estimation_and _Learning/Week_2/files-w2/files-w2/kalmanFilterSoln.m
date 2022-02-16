function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    
    dt = t - previous_t; %small infinitesimal time element
    
    A_state_mat = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];  % state-estimation matrix
         
    C_mix_mat = [1 0 0 0; 0 1 0 0]; % mixing matrix
         
    z_t = [x,y].';
    
    ang_vel_m = [dt*dt/4 0 dt/2 0; 0 dt*dt/4 0 dt/2; dt/2 0 1 0; 0 dt/2 0 1];
               
    ang_vel_z = [0.01 0; 0 0.01]; 
                          
    P = A_state_mat * param.P * transpose(A_state_mat) +  ang_vel_m;          
    
    R = ang_vel_z;
    
    Kal_mat = P * transpose(C_mix_mat) * inv(R + C_mix_mat * P * transpose(C_mix_mat));
    
    predict = A_state_mat * (state.') + Kal_mat*(z_t - C_mix_mat * A_state_mat * (state.'));
    
    param.P = P - Kal_mat * C_mix_mat * P;
    
    state = predict.';
    
    predictx = predict(1) + predict(3)*(0.330);
    predicty = predict(2) + predict(4)*(0.330);
    
    
    
    
    
    
end
