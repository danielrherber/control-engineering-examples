% ex_linearization.m
% simulation of a nonlinear and linearized model of a rocket with air
% resistance
% [reference] Example 2.7 Rocket with Air Resistance in LSC
% [course] Session 1 - Modeling for Control Design
close all; clear; clc

% assumed parameters
g = 1; b = 1; m = 1;

% nominal thrust
T0 = 3;

% velocity for stationary condition
V0 = sqrt((T0-m*g)/b);

% user-defined thrust input
T = @(t) T0 + (t>=30)*2*T0 - (t>=60)*2.25*T0;
% T = @(t) T0 + sin(t); % <- also try this

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
% plotting code
% (not the main content)
function plot_example(TSPAN,V0,T_nonlin,X_nonlin,T_lin,X_lin)

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
nicegray = [170, 170, 170]/255;
LineWidth = 1;
MarkerSize = 12;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth,'MarkerSize',MarkerSize};

% initialize figure
hf = figure; hf.Color = 'w'; hold on

plot(TSPAN,[V0 V0],'--',plotOpts{:},'Color',nicegray,'DisplayName','V0');
plot(T_nonlin,X_nonlin,plotOpts{:},'Color',niceblue,'DisplayName','Nonlinear Model')
plot(T_lin,X_lin,plotOpts{:},'Color',nicered,'DisplayName','Linearized Model')

xlabel('Time [sec]')
ylabel('State v')

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

legend();

end