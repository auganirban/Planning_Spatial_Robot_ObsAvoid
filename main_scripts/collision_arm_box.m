clc;
clear;
close all;

addpath("supporting_kinlib"); 
addpath("supporting_colli_funcs"); 
addpath("supporting_colli_models");

global J_st;

%////////////////////// Inputs/////////////////////////
theta = [0.307, -1.130, -0.503, 2.458, 0.8130, -1.216, -0.122];
window_position = [0.65; 0.3; 0.3]; % Joint positions of the left arm of Baxter robot
% theta = zeros(1, 7);

% ///////// Obtain sets to check collision between them //////////
[g_current, ~, ~, ~, g_intermediate] = FK_URDF(theta);
J_st = JS_URDF(theta);
[link_cylinder_array] = arm_cylinder_model(g_intermediate);
[~, box_vertices_array, box_normals_array] = get_obstacle_wall(window_position);

% //////////////// Works related to collision check ////////////////
dist_array = 1000*ones(1, 7); contact_nrml_array = zeros(6, 7); contact_pts_array = zeros(3, 2, 7);
jcon_array = zeros(6, 7, 7);
for box_num = 1:4  % loop through the number of obstacles
    box_bvec = cmp_bvec(box_normals_array(:, :, box_num), box_vertices_array(:, :, box_num));

    % Initial guess to solve nonlinear optimization problem
    % to find the closest distance between the box and the cylinder
    for num_link = 1:7
        current_cylinder = link_cylinder_array{1, num_link};
        x0 = [box_vertices_array(:, 1, box_num); current_cylinder.init_pt];

        % Call collision check method
        [x1, x2, min_d] = check_colli(box_normals_array(:, :, box_num), box_bvec, x0, current_cylinder.Trns, current_cylinder.rc, current_cylinder.hc);
        
        if dist_array(num_link) > min_d
            dist_array(num_link) = min_d;
            contact_nrml_array(1:3, num_link) = (x2 - x1)/norm(x2 - x1);
            contact_pts_array(:, :, num_link) = [x1, x2];
        end
    end
end

% Comment this part if you do not need to visualize the closest points.
for i = 1:7
    cnt_pts = contact_pts_array(:, :, i);
    
    % Plot the closest point pair
    plot3([cnt_pts(1, 1), cnt_pts(1, 2)], [cnt_pts(2, 1), cnt_pts(2, 2)], [cnt_pts(3, 1), cnt_pts(3, 2)], 'ko', [cnt_pts(1, 1), cnt_pts(1, 2)], [cnt_pts(2, 1), cnt_pts(2, 2)], [cnt_pts(3, 1), cnt_pts(3, 2)], 'g');
end

% Compute the contact Jacobians
for i = 1:7
    jcon_array(:, :, i) = contact_jacobian2(i, contact_pts_array(:, 2, i));
end
