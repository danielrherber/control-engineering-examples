% ex_linearization.m
% simulation of nonlinear and linearized model in Example 2.7. Rocket
% with Air Resistance in LSC
close all; clear; clc

% assumed parameters
g = 1; b = 1; m = 1;

% nominal thrust
T0 = 3;

% velocity for stationary condition
V0 = sqrt((T0-m*g)/b);

% user-defined thrust input
% T = @(t) T0 + (t>=30)*2*T0 - (t>=60)*2.25*T0;
T = @(t) T0 + sin(t); % <- also try this

% linear matrices
A = -2*b/m*V0; B = 1/m;

% nonlinear state derivative function
fnonlinear = @(t,v) -b/m*sign(v)*v^2 + T(t)/m - g;

% linear state derivative function
flinear = @(t,vdelta) A*vdelta + B*(T(t)-T0);

% time horizon bounds
tspan = [0 100];

% initial state
X0 = 0.5;

% ode options (more accurate simulation)
OPTIONS = odeset('Reltol',1e-12);

% nonlinear simulation
[T_nonlin,X_nonlin] = ode45(@(t,v) fnonlinear(t,v),tspan,X0,OPTIONS);

% shift linear simulation initial value
X0linear = X0 - V0;

% linear simulation
[T_lin,X_lin] = ode45(@(t,v) flinear(t,v),tspan,X0linear,OPTIONS);

% plot the simulation results (see function below)
% note the X_lin offset
plot_example(tspan,V0,T_nonlin,X_nonlin,T_lin,X_lin+V0)

%--------------------------------------------------------------------------
% plotting code (not the main content)
%--------------------------------------------------------------------------
function plot_example(TSPAN,V0,T_nonlin,X_nonlin,T_lin,X_lin)

% colors
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
% nicegreen = [109, 195, 80]/255;
% nicegray = [242, 242, 242]/255;
xmediumgray = [170, 170, 170]/255;

% initialize figure
hf = figure; hf.Color = 'w'; hold on
LineWidth = 1;

plot(TSPAN,[V0 V0],'--','Color',xmediumgray,'LineWidth',LineWidth,'DisplayName','V0');
plot(T_nonlin,X_nonlin,'Color',niceblue,'LineWidth',LineWidth,'DisplayName','Nonlinear Model')
plot(T_lin,X_lin,'Color',nicered,'LineWidth',LineWidth,'DisplayName','Linearized Model')

xlabel('Time [sec]')
ylabel('State v')

legend();

end