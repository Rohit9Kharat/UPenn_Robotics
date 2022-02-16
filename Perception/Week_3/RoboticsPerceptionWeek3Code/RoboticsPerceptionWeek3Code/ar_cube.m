function [proj_points, t, R] = ar_cube(H,render_points,K)
%% ar_cube
% Estimate your position and orientation with respect to a set of 4 points on the ground
% Inputs:
%    H - the computed homography from the corners in the image
%    render_points - size (N x 3) matrix of world points to project
%    K - size (3 x 3) calibration matrix for the camera
% Outputs: 
%    proj_points - size (N x 2) matrix of the projected points in pixel
%      coordinates
%    t - size (3 x 1) vector of the translation of the transformation
%    R - size (3 x 3) matrix of the rotation of the transformation
% Written by Stephen Phillips for the Coursera Robotics:Perception course

% YOUR CODE HERE: Extract the pose from the homography
% if H(3,3)<0
%     H = -H;
% end

h1_col = H(:,1);
h2_col = H(:,2);
h3_col = H(:,3);

t = h3_col/norm(h1_col);

[U, S, V] = svd([h1_col, h2_col, cross(h1_col, h2_col)]); 

R = U*[1 0 0; 0 1 0; 0 0 det(U*V')]*V';

P = render_points;
n = size(P, 1);

%proj_points = (K*(R*X' + t));
%proj_points = (K*(R*render_points(n,:)' + t));
proj_points = (K*(R*P' + repmat(t, 1, n)))';

% proj_points(:,1) = proj_points(:,1)/ proj_points(:,3);
% proj_points(:,2) = proj_points(:,2)/ proj_points(:,3);
% proj_points(:,3) = proj_points(:,3)/ proj_points(:,3);

proj_points = proj_points./repmat(proj_points(:,3), 1, 3);
proj_points = proj_points(:,1:2);
end