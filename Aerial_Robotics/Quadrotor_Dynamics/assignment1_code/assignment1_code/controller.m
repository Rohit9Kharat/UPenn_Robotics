function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters




% FILL IN YOUR CODE HERE
% 1st assignment
%u = params.mass*params.gravity;


kv = 20;
kp = 220;
doub_der_Z = 0;

e = s_des - s;
%der_e = s_des.vel(2) - s.vel(2);
% 2nd assignment
u = params.mass*(doub_der_Z + kp*e(1) + kv*e(2) + params.gravity);

end

