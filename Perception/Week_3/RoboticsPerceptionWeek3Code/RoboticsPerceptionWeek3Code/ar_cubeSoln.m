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

%conditional implemenation on homography matrix
if H(3, 3) < 0
    H = -H;
end

%declaration of homography and translation coordinates
h1 = H(:,1);
h2 = H(:,2);
t = H(:,3);

%norm function for homography coordinates
h1_pos_norm = norm(h1);  
h2_pos_norm = norm(h2);

%back-substitution of values after implementation of norm function
h1 = h1/h1_pos_norm;
h2 = h2/h2_pos_norm;
t = t/h1_pos_norm;

%Singular Value Decomposition function
[U, ~, V] = svd([h1, h2, cross(h1, h2)]);  

%computation for rotation matrix
R = U*[1 0 0; 0 1 0; 0 0 det(U*V')]*V';

% YOUR CODE HERE: Project the points using the pose

%storing size for (N*M) matrix of world points to project
num_points = size(render_points,1);

%computation for projected points on object for 3D rendering
proj_points = (K*(R*render_points' + repmat(t, 1, num_points)))';

proj_points(:,1) = proj_points(:,1) ./ proj_points(:,3);
proj_points(:,2) = proj_points(:,2) ./ proj_points(:,3);
proj_points(:,3) = proj_points(:,3) ./ proj_points(:,3);

proj_points = proj_points(:,1:2);
end
