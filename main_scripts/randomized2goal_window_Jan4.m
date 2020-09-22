clc;
clear;
close all;
clearvars -global;

% Define global variables
global h;                  global Tau;         global beta;                global dof;
global count;              global tree;        global grasped_box;         global not_reached_flag;
global num_of_contacts;    global task_dimen;  global closest_sofar_index; global safe_dist;
global box_vertices_array; global box_normals_array;

seed_val = 99;
rng(seed_val);
hfig  = figure(1); hold on; view(164, 47);
xlabel("x [m]"); ylabel("y [m]"); zlabel("z [m]");

%////////////////////////////////////////////////////////////////////////
%///////////////////////// Input set ////////////////////////////////////
%////////////////////////////////////////////////////////////////////////
% Holding object
grasped_box = load("box_grasped3_exp.mat");

% Initialize video (uncomment if you need to record the video)
video_filename = strcat("../files_traj/paper_plan_video_seed", num2str(seed_val));
myVideo = VideoWriter(video_filename); % open video file
myVideo.FrameRate = 30;               % can adjust this, 5 - 10 works well for me
open(myVideo);
% close(myVideo);

%/////////////////// Inputs for window and wall /////////////////////////
window_position = [0.902; 0.013; 0.298];
window_w = 0.265; window_h = 0.215; window_t = 0.020;
[~, box_vertices_array, box_normals_array] = get_obstacle_wall(window_position);

% /////////////// Inputs for generating random samples //////////////////
% Parameters to control the bounding box of sample generation
% box_h = 0.200; box_w = 0.800; box_d = 0.050; wall_thickness = 0.020;
box_h = 0.400; box_w = 0.800; box_d = 0.150; wall_thickness = 0.020;
% Get the bound on sample generation
[v1_min, v1_max, v2_min, v2_max] = bound_position_sample(window_position, box_h, box_w, box_d, wall_thickness);

% Planning algorithm parameters
beta = 0.02; Tau = 0.5; h = 0.01; reach_dist = 0.010;
dof = 7; safe_dist = 0.010; num_of_contacts = 7; count = 1; task_dimen = 6;

% Define initial joint configuration
theta_initial = [0.0947, -1.3848, -0.3447, 2.2595, 0.1863, -0.8544, -0.0747]';
[g_initial, ~] = FK_RealBaxter(theta_initial);
A = Mat2DQ(g_initial);

% Define goal pose of end-effector
% g_final = [g_initial(1:3, 1:3), [window_position(1)+0.050; window_position(2); window_position(3)]; zeros(1, 3), 1];
theta_final = [-1.3061, -0.5065, 0.4091, 0.9871, -0.4022, -0.4525, 0.0855]';
[g_final, ~] = FK_RealBaxter(theta_final);
R_g_final = g_final(1:3, 1:3);
B = Mat2DQ(g_final);

scatter3(g_initial(1, 4), g_initial(2, 4), g_initial(3, 4), 'r', 'filled');
hold on;
scatter3(g_final(1, 4), g_final(2, 4), g_final(3, 4), 'g', 'filled');
xlim([0, 1.2]); ylim([-0.8, 0.9]); zlim([-0.1, 0.9]); view(258, 19);
%////////////////////// End of input set ////////////////////////////////

%%%%%%%%%%%%%%%%%%%% Initialize %%%%%%%%%%%%%%%%%%%
count = 1; theta_SCLERP(:, count) = theta_initial;

%%%%%%%%%%%%%%%%%%%% Planning loop %%%%%%%%%%%%%%%%%%%
% Initiate the tree
initial_dist = norm(g_final(1:3, 4) - g_initial(1:3, 4));
% the last element of this array is the parent node index
tree.nodes = [g_initial(1:3, 4)', rotm2quat(g_initial(1:3, 1:3)), 0];
tree.nodesDQ = A;
tree.nodesAng = theta_initial';
tree.node_count = 1;
tree.edges = [];              % To do: find the edges

% Main loop of iterative motion plan using PATH solver
old_small = 100;
while tree.node_count < 1000
    
    %%%%%%%%%% Generate a temporary random goal pose %%%%%%%%%%%%%%
    if old_small > 0.020
        choose_side = randi([1, 3]);
    else
        choose_side = randi([2, 3]);
    end
    
    if choose_side == 1
        rand_pose1 = generate_rand_se3(v1_min, v1_max);
    elseif choose_side == 2
        rand_pose1 = generate_rand_se3([window_position(1)-0.010, window_position(2)-0.100, window_position(2)-0.100], [window_position(1)+0.010, window_position(2)+0.100, window_position(2)+0.100]);
        % rand_pose1 = generate_rand_se3([window_position(1)-0.5*window_t, window_position(2)-0.5*window_w, window_position(2)-0.5*window_h], [window_position(1)+0.5*window_t, window_position(2)+0.5*window_w, window_position(2)+0.5*window_h]);
    else
        rand_pose1 = generate_rand_se3(v2_min, v2_max);
    end
    
    position_rand = rand_pose1(1:3);
    g_rand = [quat2rotm(rand_pose1(4:end)'), position_rand; 0, 0, 0, 1];
    DQ_rand = Mat2DQ(g_rand);
    
    %%%%% Find the closest node of g_rand from the tree %%%%%%%%%
    [closest_node_indx, closest_dist] = find_closest_node(g_rand);
    position_closest = tree.nodes(closest_node_indx, 1:3);
    DQ_closest = tree.nodesDQ(closest_node_indx, :);
    theta0 = tree.nodesAng(closest_node_indx, :)';
    
    %%%%%%% Move DQ_closest to DQ_rand %%%%%%%%%%%%%
    [achieved_g, achieved_DQ, achieved_jointvec] = get2goal(theta0, DQ_closest, DQ_rand);
    [not_reached_flag, old_small] = addNodes2TreeStep(achieved_g, closest_node_indx, achieved_jointvec, old_small, B);
    if ~not_reached_flag
        break;
    end
    
    %%%%%%% Move DQ_closest to DQ_final %%%%%%%%%%%%%
    [achieved_g2, achieved_DQ2, achieved_jointvec2] = get2goal(theta0, DQ_closest, B);
    [not_reached_flag, old_small] = addNodes2TreeStep(achieved_g2, closest_node_indx, achieved_jointvec2, old_small, B);
    if ~not_reached_flag
        break;
    end    
end

% Extract the path
path_node_ind = extract_rrt_path2(closest_sofar_index);

% Visualize
motion_angle_vec = [];
for nd_idx = 2:length(path_node_ind)-1
    th_init = tree.nodesAng(path_node_ind(nd_idx), :)';
    DQ_A = tree.nodesDQ(path_node_ind(nd_idx), :);
    DQ_B = tree.nodesDQ(path_node_ind(nd_idx+1), :);
    [~, ~, ~, theta_vec] = get2goal_video(th_init, DQ_A, DQ_B, myVideo);
    motion_angle_vec = [motion_angle_vec, theta_vec];
end
close(myVideo);

% Save tree and path
name_of_datafile = strcat("../files_traj/data_seed", num2str(seed_val), ".mat");
save(name_of_datafile, "tree", "path_node_ind", "motion_angle_vec");
fprintf(strcat("Data saved in file named: ", name_of_datafile, "\n"));

% Write data to a file
writeTraj2file(name_of_datafile, seed_val);
