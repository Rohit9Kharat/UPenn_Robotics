function qdd = eom(params, th, phi, dth, dphi, u)
  % This is the starter file for the week5 assignment

  % Provided params are
  % params.g: gravitational constant
  % params.mr: mass of the "rod"
  % params.ir: rotational inertia of the rod
  % params.d: distance of rod CoM from the wheel axis
  % params.r: wheel radius

  % Provided states are:
  % th: wheel angle (relative to body)
  % phi: body pitch
  % dth, dphi: time-derivatives of above
  % u: torque applied at the wheel

  X = (params.mr*params.r*params.d*cos(phi));
  Y = (params.ir+params.mr*(params.d)^2);
  Z = (params.mr*params.g*params.d);
  P = (params.mr*(params.r)^2);
  Q = (params.r*params.mr*params.d*cos(phi));
  R = (params.mr*params.r*params.d);
  
  tau_twerk = u;
  
  phi_bdy_pitch = ((X*R*sin(phi)*dphi^2-Z*P*sin(phi)+X*tau_twerk+P*tau_twerk)/(X*Q-Y*P));
  
  th_wheel_ang = (-(X*phi_bdy_pitch+Y*phi_bdy_pitch-Z*sin(phi)+tau_twerk)/X);
  
  qdd = [th_wheel_ang;phi_bdy_pitch];
  % THE STUDENT WILL FILL THIS OUT
end