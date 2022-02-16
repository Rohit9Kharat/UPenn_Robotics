% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.

myResol = param.resol;
% % the initial map size in pixels
myMap = zeros(param.size);
% % the origin of the map in pixels
myorigin = param.origin; 
% 
% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);
num = size(scanAngles,1);

for j = 1:N % for each time,
       

    x_pos = pose(1, j);
    y_pos = pose(2, j);
    theta_ang = pose(3, j);

    i_x_robot_pos = ceil(x_pos * myResol) + myorigin(1);
    i_y_robot_pos = ceil(y_pos * myResol) + myorigin(2);
    

    % Find grids hit by the rays (in the gird myMap coordinate)
    
    range_col = ranges(:, j);
    
    x_occ_cell = range_col .* cos(scanAngles + theta_ang) + x_pos;
    y_occ_cell = -range_col .* sin(scanAngles + theta_ang) + y_pos;
    
    occ_cell_i_x = ceil(x_occ_cell * myResol) + myorigin(1);
    occ_cell_i_y = ceil(y_occ_cell * myResol) + myorigin(2);

    % Find occupied-measurement cells and free-measurement cells
    
    occ = sub2ind(size(myMap), occ_cell_i_y, occ_cell_i_x); 
    free = [];
    
    for k = 1:num
        [free_ix, free_iy] = bresenham(i_x_robot_pos, i_y_robot_pos, occ_cell_i_x(k), occ_cell_i_y(k));  
        free = [free; free_iy, free_ix];
    end
    free = sub2ind(size(myMap), free(:, 1), free(:, 2));

    % Update the log-odds
    
    myMap(occ) = myMap(occ) + lo_occ;
    myMap(free) = myMap(free) - lo_free;

    % Saturate the log-odd values
    
    myMap(myMap > lo_max) = lo_max;
    myMap(myMap < lo_min) = lo_min;

    %% Visualize the myMap as needed
%     imagesc(myMap); hold on;
%     plot(i_x_robot_pos, i_y_robot_pos, 'rx', 'LineWidth', 3); % indicate robot location
%     pause(0.001);
end

end