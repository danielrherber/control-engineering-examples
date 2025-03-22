% ex_sliding_mode.m
% simulation of a sliding mode controller on a 2d nonlinear affine control
% system
% [reference] Section 9.15.1 Fundamentals of Sliding Control (pp. 695–700)
% in Mechanical Engineer's Handbook
% [course] Session 8 - Nonlinear Control (2)
close all; clear; clc

% select the controller to be used
% controller_option = 0; % no control
% controller_option = 1; % discontinuous controller (will take some time to simulate)
controller_option = 2; % continuous controller
% controller_option = 3; % simple gain controller

% nonlinear function
a = 1;
b = 2;
f = @(t,x1,x2) a*cos(x1) + b*cos(x2 + 1) + exp(-t);

% bound on nonlinear f
k = a + b + 1;
% k = 1; % <- try this incorrect bound

% nonlinear state derivative function
F = @(t,x) [x(2); f(t,x(1),x(2)) + u(t,x(1),x(2),k,controller_option)];

% simulation options
TSPAN = [0 20];
X0 = [0.5 0.5];
OPTIONS = odeset('RelTol',1e-6);

% simulate
[T,X] = ode45(@(t,x) F(t,x),TSPAN,X0,OPTIONS);

% plot states
plot_example(T,X,"States")

% plot control
plot_example(T,u(T,X(:,1),X(:,2),k,controller_option),"Control")

% plot state-space and switching surface
plot_example_switching(X)

%--------------------------------------------------------------------------
% control law
function U = u(t,x1,x2,k,controller_option)

% distance from the switching surface
sigma = x1 + x2;

% sign of the distance from the switching surface
sign_sigma = sign(sigma);

% continuous approximation for the sign function
approx_sign_sigma = 2./(1 + exp(-400*sigma)) - 1;

% control
switch controller_option
    case 0 % no control
        U = zeros(size(t));
    case 1 % discontinuous controller (will take some time to simulate)
        U = -( abs(x2) + k + 1).*sign_sigma;
    case 2 % continuous controller
        U = -(abs(x2) + k + 1).*approx_sign_sigma;
    case 3 % simple gain controller
        U = -k*approx_sign_sigma;
end

end

%--------------------------------------------------------------------------
% approximate sign function plot used above
% figure; hold on
% Y = linspace(-1,1,1000);
% plot(Y,2./(1 + exp(-400*Y)) - 1)
% return

%--------------------------------------------------------------------------
% plotting code for states and control
% (not the main content)
function plot_example(T,X,str)

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
LineWidth = 1;
MarkerSize = 12;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth,'MarkerSize',MarkerSize};

% initialize figure
hf = figure; hf.Color = 'w'; hold on

switch str
    case "States"
        plot(T,X(:,1),plotOpts{:},'Color',niceblue,'DisplayName','$x_1$');
        plot(T,X(:,2),plotOpts{:},'Color',nicered,'DisplayName','$x_2$');

    case "Control"
        plot(T,X(:,1),plotOpts{:},'Color',niceblue,'DisplayName','$u$');
end

xlabel('Time [s]')
ylabel(str)

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

hl = legend(); hl.Location = "best"; hl.Interpreter = "latex"; hl.FontSize = FontSize;

end

%--------------------------------------------------------------------------
% plotting code for state-space and switching surface
% (not the main content)
function plot_example_switching(X)

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
LineWidth = 1;
MarkerSize = 12;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth,'MarkerSize',MarkerSize};

% initialize figure
hf = figure; hf.Color = 'w'; hold on

plot(X(:,1),X(:,2),plotOpts{:},'Color',niceblue,'DisplayName','Simulation trajectory');
plot(X(:,1),-X(:,1),plotOpts{:},'Color',nicered,'DisplayName','Switching surface');

xlabel('$x_1$','Interpreter',"latex")
ylabel('$x_2$','Interpreter',"latex")

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

hl = legend(); hl.Location = "best"; hl.FontSize = FontSize;

end