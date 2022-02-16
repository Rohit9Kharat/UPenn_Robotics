function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
  %state.pos = [y ; z]; state.vel = [y_dot; z_dot]; state.rot = (phi);
  %state.omega = (phi_dot);
%
%   des_state: The desired states are:
   %des_state.pos = [y ; z]; des_state.vel = [y_dot; z_dot]; 
   %des_state.acc =[y_ddot; z_ddot];
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

%u1 = 0;
%u2 = 0;

% FILL IN YOUR CODE HERE
Kv_z = 20;
Kp_z = 200;
Kv_phi = 50;
Kp_phi = 1000;
Kv_y = 12;
Kp_y = 12;

y = state.pos(1);
z = state.pos(2);
y_vel = state.vel(1);
z_vel = state.vel(2);

y_c = des_state.pos(1);
z_c = des_state.pos(2);
y_vel_c = des_state.vel(1);
z_vel_c = des_state.vel(2);

y_acn_c = des_state.acc(1);
z_acn_c = des_state.acc(2);

phi = state.rot(1);
ang_vel = state.omega(1);

%e = des_s - s;
e_z = z_c - z;
e_z_vel = z_vel_c - z_vel;

e_y = y_c - y;
e_y_vel = y_vel_c - y_vel;


%u1 = params.mass(params.gravity - kvz*z_dot + kpz*(z0 - z));
%phic = -(1/params.gravity)*(kvy*(-y_dot) + kpy*(y0 - y));
%u2 = params.Ixx(phi_ddot + kvphi*(phic_dot - phi_dot) + kpphi*e(phic - phi));

phi_c = -(1/params.gravity)*(y_acn_c + Kp_y*e_y + Kv_y*e_y_vel);

%  u1
u1 = params.mass*(params.gravity + z_acn_c + Kp_z*e_z + Kv_z*e_z_vel);
% u2
phidot_c = 0;
u2 = params.Ixx*(Kp_phi*(phi_c - phi) + Kv_phi*(phidot_c - ang_vel));

end

