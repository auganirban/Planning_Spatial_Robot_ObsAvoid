clc;
clear;
close all;
clearvars -global;

% Define global variables
global box_vertices_array; global box_normals_array; 
global g_intermediate;     global grasped_box;

hfig  = figure(1); hold on; view(164, 47);
xlabel("x [m]"); ylabel("y [m]"); zlabel("z [m]");

%////////////////////////////////////////////////////////////////////////
%///////////////////////// Input set ////////////////////////////////////
%////////////////////////////////////////////////////////////////////////
% Holding object
grasped_box = load("box_grasped3_exp.mat");

%/////////////////// Inputs for window and wall /////////////////////////
window_position = [0.902; 0.013; 0.298];
window_w = 0.265; window_h = 0.215; window_t = 0.020;
[~, box_vertices_array, box_normals_array] = get_obstacle_wall(window_position);

% /////////////////////////////////////////////////////////////////////
%/////////////////////// Load data file ///////////////////////////////
%//////////////////////////////////////////////////////////////////////
data_loader = load("data_seed99.mat");

% /////////////////////////////////////////////////////////////////////
%////////////////////// Plot the robot with grasped object ////////////
%//////////////////////////////////////////////////////////////////////
q_o_array = data_loader.motion_angle_vec;
plot_ind = [1, 150, 339, 480, 612, 780, 909];
plot_ind = [339];
for i = 1:length(plot_ind)
    [g, ~, ~, ~, g_intermediate] = FK_RealBaxter(q_o_array(:, plot_ind(i)));
    [link_cylinder_array] = arm_cylinder_model(g_intermediate);
    plot_motion_paper(hfig);
end

% Plot all the joint angles
figure(2); hold on;
for i = 1:size(q_o_array, 1)
    plot(q_o_array(i, :));
end
xlabel("iterations");
ylabel("$$\Theta$$ rad", "interpreter", "latex");
hlegend = legend({"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"}, "orientation", "vertical");
hlegend.NumColumns=4;

%%%%%%%%%%%% paper purpose %%%%%%%%%%%%%%%%%%%%%%
data_loader = load("data_seed99.mat");
tree = data_loader.tree;
path_node_ind = data_loader.path_node_ind;
