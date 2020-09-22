clc;
clear;
close all;

% Load the data from mat file
left_joint_angles = load("/home/simbot/Desktop/Complementarity_MotionPlan/main_scripts/data_seed5000000.mat");

% Write data to a file
fileID = fopen("/home/simbot/Desktop/Complementarity_MotionPlan/traj_files/joint_vec_left.txt",'w');
fprintf(fileID,"%2.4f, %2.4f, %2.4f, %2.4f, %2.4f, %2.4f, %2.4f\n", left_joint_angles.motion_angle_vec);
fclose(fileID);

