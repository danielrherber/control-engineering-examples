% ex_expm.m
% simulation of an LTI system using various methods, including the matrix
% exponential
% [reference] Example 3.3 Transfer Function of a Discrete Time System
% [course] Session 2 - Analysis of State-Space Models (1)
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
% plotting code
% (not the main content)
function plot_example(T_sim,X_sim,X_sym)

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
LineWidth = 1;
MarkerSize = 12;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth,'MarkerSize',MarkerSize};

% initialize figure
hf = figure; hf.Color = 'w'; hold on
t = tiledlayout('flow','GridSize',[2 1],'TileSpacing','tight');

%--- states
nexttile(t,[1 1]); hold on

% plot simulation results
plot(T_sim,X_sim(:,1),plotOpts{:},'Color',nicered,"DisplayName","$x_1$ simulation")
plot(T_sim,X_sim(:,2),plotOpts{:},'Color',niceblue,"DisplayName","$x_2$ simulation")

% plot symbolic method results
plot(T_sim,X_sym(1,:),'.',plotOpts{:},'Color',nicered,"DisplayName","$x_1$ exact")
plot(T_sim,X_sym(2,:),'.',plotOpts{:},'Color',niceblue,"DisplayName","$x_2$ exact")

xlabel('Time [sec]')
ylabel('States')

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

hl = legend(); hl.Interpreter = 'latex';

%--- errors
nexttile(t,[1 1]); hold on

% plot simulation results
plot(T_sim,abs(X_sim(:,1)-X_sym(1,:)'),plotOpts{:},'Color',nicered,"DisplayName","$x_1$")
plot(T_sim,abs(X_sim(:,2)-X_sym(2,:)'),plotOpts{:},'Color',niceblue,"DisplayName","$x_2$")

xlabel('Time [sec]')
ylabel('State Error')

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;
ha.YScale = 'log';

hl = legend(); hl.Interpreter = 'latex';

ylim([1e-19 1e0])

end