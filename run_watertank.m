% run_watertank.m
clear; clc;

% Instantiate the object from your blueprint
init_watertank;

% Define the options for THIS specific training run1
% 2 var to the power of 2
init_sampling.num_corners = 4;       
init_sampling.num_quasi_random = 10;
init_sampling.seed = 100;
 % 1 to generate data or 0 to use prev traces
init_sampling.recompute = 1;        

falsif_sampling.num_corners = 4; 
falsif_sampling.num_quasi_random = 10;
falsif_sampling.seed = 200;
falsif_sampling.recompute = 0;
%  set to 0 for random global search 1
falsif_sampling.local_max_obj_eval = 0;
% set to default when not defined
% falsif_sampling.scaling = 0.8;

% Step 3: Start the Imitation Learning Loop!
disp('Starting Water Tank Imitation Learning...');
res = pb_watertank.algo2(...
    'init_sampling', init_sampling, ...
    'falsif_sampling', falsif_sampling, ...
    'num_training', 5, ...             
    'max_num_cex_traces', 20, ...
    'max_num_cex_samples', 20);