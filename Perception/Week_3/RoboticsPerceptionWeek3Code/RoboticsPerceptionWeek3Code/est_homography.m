function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
H = [];
A = [];
for i = 1:length(video_pts)
    x_v = video_pts(i,1);
    y_v = video_pts(i,2);
    
    x_l_point = logo_pts(i,1);
    y_l_point = logo_pts(i,2);
    
    a_x = [ -x_v, -y_v, -1, 0, 0, 0, x_v*x_l_point, y_v*x_l_point, x_l_point];
    a_y = [ 0, 0, 0, -x_v, -y_v, -1, x_v*y_l_point, y_v*y_l_point, y_l_point];
    A = cat(1,A,a_x,a_y);
end
[~,~,H] = svd(A);  % A in the shape of 9x1

H = reshape(H(:,end),[3,3])';
% [U,S,V] = svd(A);
% h = V(:,9);
% h = reshape(h,3,3);
% H = h';0
end

