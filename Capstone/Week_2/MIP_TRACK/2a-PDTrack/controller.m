
function u = controller(params, t, x, xd)
  % x = current position
  % xd = current velocity

  % Use params.traj(t) to get the reference trajectory
  % e.g. (x - params.traj(t)) represents the instaneous trajectory error

  % params can be initialized in the initParams function, which is called before the simulation starts
  
  % SOLUTION GOES HERE -------------
  k_p = -2500;
  k_v = -110;
  
  %lumda = -0.1;
  err = x - params.traj(t);
  
  x0 = params.traj(0);
  % vt = x0*exp(lumda*t);
  u = k_v*(xd)^3 + k_p * err;
end