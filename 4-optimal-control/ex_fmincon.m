close all; clear; clc

% solve the following finite-dimensional optimization problem:
% min J = x1^2 + a1*x2^2;
% subject to: g = a2/x1 - x2 <= 0

% for visualizing some things (see below)
iteration_plot_setup

% problem parameters
a1 = 2; a2 = 1.5;

% fmincon options (to display iteration progress and other plots)
options = optimoptions('fmincon','Display','iter',...
    'PlotFcn',{'optimplotfvalconstr','optimplotfirstorderopt'},...
    'OutputFcn',@outfun);

% solve the optimization problem using fmincon
% type help fmincon to see all the inputs
[x,fval] = fmincon(@(x) objective(x,a1),[1;1],[],[],[],[],[],[],...
    @(x) constraints(x,a2),options);

% display final (optimal) value of x and the objective function with this x
disp(x)
disp(fval)

% objective function
function J = objective(x,a1)

% objective function value
J = x(1)^2 + a1*x(2)^2;

end

% constraint function
function [g,h] = constraints(x,a2)

% inequality constraints
g = a2/x(1) - x(2);

% inequality constraints
h = [];

end









% NOTE: this stuff below is just to visualize the optimization algorithm
% progress. It is not needed in the class examples.

% optional customized output function to displaying optimization algorithm
% progress
function stop = outfun(x,optimValues,state)

% don't stop the optimization routing here
stop = false;

% call up figure 2
figure(2); hold on

% get previous optimization variable value
global x_old
if isempty(x_old)
    x_old = x;
end

% plot current optimization variables (2D problem)
plot([x_old(1) x(1)],[x_old(2) x(2)],'.-','markersize',16)

% update x_old
x_old = x;

% display the current iteration number
text(x(1)+0.01,x(2)-0.01,string(optimValues.iteration))

end

function iteration_plot_setup

% initialize figure 2
hf = figure(2); hf.Color = 'w'; clf; hold on
xlabel('x_1')
ylabel('x_2')

% known solution to this problem
plot(1.456475488389924,1.029883936327159,'.k','markersize',16)

global x_old
x_old = [];

end