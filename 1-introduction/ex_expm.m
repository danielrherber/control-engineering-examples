% ex_expm.m
% simulation of an LTI system from Example 3.3 Transfer Function of a
% Discrete Time System in LSC using various methods, including the matrix
% exponential
close all; clear; clc

% state-space matrices
A = [0 1; -20 -12];
B = [0;1];
C = [1 1];
D = 0;

% simulation conditions
t0 = 0;
tfinal = 50;
X0 = [1;-1];

%--------------------------------------------------------------------------
% simulation-based approach
%--------------------------------------------------------------------------
% input function
u = @(t) sin(t);

% derivative function (linear system)
f = @(t,x) A*x + B*u(t);

% simulation options
OPTIONS = odeset('RelTol',1e-4,'AbsTol',1e-4);

% simulation
[T_sim,X_sim] = ode45(@(t,x) f(t,x),[t0 tfinal],X0,OPTIONS);

%--------------------------------------------------------------------------
% symbolic state equation and matrix exponential approach
% x = Part1*x0 + Part2 (time-domain LTV state equation solution)
%--------------------------------------------------------------------------
% define the symbolic variables
syms t t0 tau s

% symbolic input function
U = sin(tau); % <- note tau for the integral

% zero input solution
Part1 = expm(A*t);

% alternative method based on the inverse Laplace transform
Part1_alt = ilaplace(inv(s*eye(2)-A));
isequal(Part1,Part1_alt)

% zero state solution
Part2 = int(expm(A*(t-tau))*B*U,tau,t0,t);

% convert to Matlab functions
Part1_fun = matlabFunction(Part1);
Part2_fun = matlabFunction(Part2);

% assign initial state condition to first column
X_sym = zeros(2,length(T_sim));
X_sym(:,1) = X0;

% determine the state at the time points recorded in the simulation
for k = 1:length(T_sim)-1
    tk = T_sim(k);
    tk1 = T_sim(k+1);
    X_sym(:,k+1) = Part1_fun(tk1-tk)*X_sym(:,k) + Part2_fun(tk1,tk);
end

% plot the simulation results (see function below)
plot_example(T_sim,X_sim,X_sym)

%--------------------------------------------------------------------------
% plotting code (not the main content)
%--------------------------------------------------------------------------
function plot_example(T_sim,X_sim,X_sym)

% colors
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
% nicegreen = [109, 195, 80]/255;
% nicegray = [242, 242, 242]/255;
% xmediumgray = [170, 170, 170]/255;

% initialize figure
hf = figure; hf.Color = 'w'; hold on
LineWidth = 1;

% plot simulation results
plot(T_sim,X_sim(:,1),'Color',nicered,'linewidth',LineWidth,"DisplayName","x_1 ode45")
plot(T_sim,X_sim(:,2),'Color',niceblue,'linewidth',LineWidth,"DisplayName","x_2 ode45")

% plot symbolic method results
plot(T_sim,X_sym(1,:),'.','Color',nicered,'linewidth',LineWidth,"DisplayName","x_1 symbolic")
plot(T_sim,X_sym(2,:),'.','Color',niceblue,'linewidth',LineWidth,"DisplayName","x_2 symbolic")

xlabel('Time [sec]')
ylabel('States')

legend();

% initialize figure
hf = figure; hf.Color = 'w'; hold on
LineWidth = 1;

% plot simulation results
plot(T_sim,abs(X_sim(:,1)-X_sym(1,:)'),'Color',nicered,'linewidth',LineWidth,"DisplayName","x_1")
plot(T_sim,abs(X_sim(:,2)-X_sym(2,:)'),'Color',niceblue,'linewidth',LineWidth,"DisplayName","x_2")

xlabel('Time [sec]')
ylabel('State Error')

legend();

ha = gca;
ha.YScale = 'log';

ylim([1e-19 1e0])

end