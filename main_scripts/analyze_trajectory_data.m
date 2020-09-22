clc;
clear;
close all;

% Load the file with joint trajectory data
js_traj_data = load("../files_traj/joint_vec_left.txt");
tvec = 1:1:size(js_traj_data, 1);

jt1_data = js_traj_data(:, 1);
nn = size(jt1_data, 1);

% for i = 1:7
%     max(js_traj_data(:, i) - [js_traj_data(1,i); js_traj_data(1:(nn-1),i)])
% end

% plot the data for each joints
for i = 1:size(js_traj_data, 2)
    figure(i)
    plot(js_traj_data(:, i));
    hold on;
    smoothened = spline(tvec, js_traj_data(:, i));
    plot(ppval(smoothened, tvec));
end
