% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% % the number of grids for 1 meter.
 myResolution = param.resol;
% % the origin of the map in pixels
 myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;


% You should put the given initial pose into pose for j=1, ignoring the j=1 ranges.
% The pose(:,j) should be the pose when ranges(:,j) were measured.

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 200;                           % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles

P = [ repmat(myPose(:,1), [1, M]); zeros(1, M) ];
size(P)

std_x_pos =  0.016364 * 2;
std_y_pos =  0.017823 * 2;
std_theta_ang =  0.0090428 * 2;

score = 1200;

[ cell_max_y, cell_max_x] = size(map);

for j = 1:N

    

    P(1,:) = P(1,:) + randn(1, M) * std_x_pos;
    P(2,:) = P(2,:) + randn(1, M) * std_y_pos;
    P(3,:) = P(3,:) + randn(1, M) * std_theta_ang;
    P(4,:) = zeros(1, M);

    robo_x_pos = P(1, :) * myResolution + myOrigin(1);
    robo_y_pos = P(2, :) * myResolution + myOrigin(2);
    robo_orient = P(3, :);
    
    occ_cell_x = zeros(M, 1081);
    occ_cell_y = zeros(M, 1081);

    for range = 1:1081 % for each time,
        
        tetha_ang = robo_orient + scanAngles(range);
        
        top_rotn = [ cos(tetha_ang); sin(tetha_ang) ];
        
        bottom_rotn = [ -sin(tetha_ang); cos(tetha_ang) ];
        
        dis_loc = ranges(range, j) *  myResolution;
        
        occ_cell_loc = [ dis_loc; 0 ];
        
        occ_x = top_rotn' * occ_cell_loc + robo_x_pos';
        
        occ_y = bottom_rotn' * occ_cell_loc + robo_y_pos';
        
        occ_mat = [ occ_x' ; occ_y' ];
        
        occ_mat = floor(occ_mat);
        

        occ_cell_x(:,range) = occ_mat(1,:);
        occ_cell_y(:,range) = occ_mat(2,:);
     end

     P_col = size(P,2);
     
     for i = 1:P_col
         
         occ_cell = [ occ_cell_x(i,:); occ_cell_y(i,:) ];
         
         i_x_robot_pos = ( occ_cell(1,:) > 0);
         occ_cell = occ_cell(:,i_x_robot_pos);
         i_x_robot_pos = ( occ_cell(1,:) < cell_max_x);
         occ_cell = occ_cell(:,i_x_robot_pos);

         i_x_robot_pos = ( occ_cell(2,:) > 0);
         occ_cell = occ_cell(:,i_x_robot_pos);
         i_x_robot_pos = ( occ_cell(2,:) < cell_max_y);
         occ_cell = occ_cell(:,i_x_robot_pos);
         
         Sc = [cell_max_y 1];
         i_x_robot_pos = Sc * (occ_cell - [0; 1]);
         P(4, i) = sum(map(i_x_robot_pos));

     end

     [m, index] = max(P(4,:));
     
     myPose(1, j) = P(1, index);
     myPose(2, j) = P(2, index);
     myPose(3, j) = P(3, index);

     sc_fil_point = min(m, score);
     i_x_robot_pos = ( P(4,:) >= sc_fil_point);
     
     P = P(:,i_x_robot_pos);
     P_col = size(P,2);
     
     mis = M - P_col;
     i_x_robot_pos = randi(P_col, mis, 1);
     
     P = [ P P(:,i_x_robot_pos) ];
  end


myPose = myPose;
end