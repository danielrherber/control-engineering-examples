close all; clear; clc

% add all folders to your path
file_path = which(mfilename('fullpath'));
folder_name = fullfile(fileparts(file_path));
addpath(genpath(folder_name))

% initialize list of examples to run
ex = {};

% examples in 1-introduction
ex{end+1} = @ex_co_ob;
ex{end+1} = @ex_expm;
ex{end+1} = @ex_linearization;
ex{end+1} = @ex_modal;
ex{end+1} = @ex_modred;
ex{end+1} = @ex_simulation;
ex{end+1} = @ex_sysid;
ex{end+1} = @ex_transferfunction;
ex{end+1} = @ex_zoh;

% examples in 2-linear-control-design
ex{end+1} = @ex_fullfeedback;
ex{end+1} = @ex_integral_control;
ex{end+1} = @ex_mimo_place;
ex{end+1} = @ex_observer;
ex{end+1} = @ex_pi;
ex{end+1} = @ex_simulink;
% ex{end+1} = @; % ADD ex_simulink_model or ex_simulink_model2
ex{end+1} = @ex_specs;
ex{end+1} = @ex_water;
% ex{end+1} = @; % ADD ex_water_tank_control

% examples in 3-nonlinear-control-design
ex{end+1} = @ex_feedback_linearization;
ex{end+1} = @ex_gain_scheduling;
ex{end+1} = @ex_sliding_mode;

% examples in 4-optimal-control
ex{end+1} = @ex_fmincon;
ex{end+1} = @ex_lqr;
ex{end+1} = @ex_single_shooting;
ex{end+1} = @ex_single_step_dt;

% examples in 5-robust-stochastic-control
ex{end+1} = @ex_Hinfinty;
ex{end+1} = @ex_lqg;
% ex{end+1} = @; % ADD scalar_kalman_ex_r2019b

% number of examples
n = length(ex);

% initialize progress bar
f = waitbar(0, 'Starting');

% run through each example
for idx = 1:n

    % run the example
    local_function(ex{idx})

    % increment progress bar
    waitbar(idx/n, f, sprintf('Progress: %d %%', floor(idx/n*100)));

end

% delete the progress bar
delete(f)

% display inuse licenses
license('inuse')

% local function (to control the script workspace)
function local_function(fun)

fun()

end