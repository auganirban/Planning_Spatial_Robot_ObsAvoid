clc;

% Run this file to add paths of the supporting function files and box models 
% Add path to PATH solver
addpath("supporting_quatlib");     addpath("supporting_colli_funcs"); addpath("supporting_random"); 
addpath("supporting_kinlib");      addpath("pathmexa64");             addpath("supporting_colli_models");
addpath("supporting_planninglib"); addpath("supporting_visualize");   addpath("supporting_other");
addpath("supporting_load_boxmodels");

fprintf("All the relevant directories are added to the matlab's search path. \n");