
function x_hat_vector = EKFstudent(t, z)
  % In this exercise, you will batch-process this data: you are provided a vector of timestamps (of length T), and a 3xT matrix of observations, z.
  x_hat_vector = zeros(2,length(t));

  % Student completes this
  M = [0.1 0;0 0.1];
  
  R = [0.009 0 0;0 0.009 0;0 0 0.0076];
  
  u = 1;
  
  x_hat_vector(:,1) = [2,0];
  
  Q(:,:,1) = [5 5;5 5];
  
  h_vector = [u*sind(x_hat_vector(1,1));u*cosd(x_hat_vector(1,1));x_hat_vector(2,1)];
  
  xkp_o = x_hat_vector(:,1);
  
  
  for i=2:length(t)
      dt = t(i) - t(i-1);
      A = [1 dt;0 1];
      xkp = A*x_hat_vector(:,i-1);
      Pkp = A*Q(:,:,i-1)*A' + M;
      H = [u*cosd(x_hat_vector(1,i))*(pi/180) 0; -u*sind(x_hat_vector(1,i))*(pi/180) 0;0 1];
      h_vector = [u*sind(xkp(1)); u*cosd(xkp(1)); xkp(2)];
      K = Pkp*H'/(H*Pkp*H' + R);
      x_hat_vector(:,i) = xkp + K*(z(:,i) - h_vector);
      Q(:,:,i) = (eye(2) - K*H)*Pkp;
      
end
