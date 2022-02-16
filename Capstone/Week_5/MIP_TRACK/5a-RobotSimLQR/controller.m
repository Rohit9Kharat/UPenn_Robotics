
function u = controller(params, t, X)
  % You have full state feedback available
  k = [-0.0088,   -1.0552,   -0.0112,   -0.1248];
  u = -k*X;
  % After doing the steps in simLinearization, you should be able to substitute the linear controller u = -K*x
  
end

