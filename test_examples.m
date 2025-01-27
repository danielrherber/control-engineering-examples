close all; clear; clc

% add all folders to your path
file_path = which(mfilename('fullpath'));
folder_name = fullfile(fileparts(file_path));
addpath(genpath(folder_name))

% initialize list of examples to run
ex = {};

% examples in 1-introduction
ex{end+1} = @ex_controllability_observability;
ex{end+1} = @ex_expm;
ex{end+1} = @ex_linearization;
ex{end+1} = @ex_matlab_basics;
ex{end+1} = @ex_modal;
ex{end+1} = @ex_modal_interactive;
ex{end+1} = @ex_modred;
ex{end+1} = @ex_simulation;
ex{end+1} = @ex_sysid;
ex{end+1} = @ex_tf_bode;
ex{end+1} = @ex_zoh;

% examples in 2-linear-control-design
ex{end+1} = @ex_full_state_feedback;
ex{end+1} = @ex_integral_control;
ex{end+1} = @ex_mi_place;
ex{end+1} = @ex_observer;
ex{end+1} = @ex_pi;
ex{end+1} = @ex_pidtuner;
ex{end+1} = @ex_simulink_bd;
ex{end+1} = @() sim('ex_simulink_model');
ex{end+1} = @ex_specifications;
ex{end+1} = @ex_water_tank;
% ex{end+1} = @() sim('ex_water_tank_control'); % FIX: causes error

% examples in 3-nonlinear-control-design
ex{end+1} = @ex_feedback_linearization;
ex{end+1} = @ex_gain_scheduling;
ex{end+1} = @ex_sliding_mode;

% examples in 4-optimal-control
ex{end+1} = @ex_direct_transcription;
ex{end+1} = @ex_fmincon;
ex{end+1} = @ex_lqr_robot_eqs;
ex{end+1} = @ex_lqr_robot_linear;
% ex{end+1} = @() sim('ex_lqr_robot_simulink'); % FIX: causes error
ex{end+1} = @ex_simulation_optimal_control;

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