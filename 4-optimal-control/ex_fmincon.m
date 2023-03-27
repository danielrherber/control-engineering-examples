% ex_fmincon.m
% example using fmincon to solve a 2d finite-dimensional optimization
close all; clear; clc

% solve the following finite-dimensional optimization problem:
% min J = x1*x2
% subject to: h1 = 2*x1 + 2*x2 = perimeter
%             g1 = x1 >= 0
%             g2 = x2 >= 0

% for visualizing some things (see below)
iteration_plot_setup

% problem parameters
perimeter = 20;

% fmincon options (to display iteration progress and other plots)
options = optimoptions('fmincon','Display','iter',...
    'PlotFcn',{'optimplotfvalconstr','optimplotfirstorderopt'},...
    'OutputFcn',@outfun);

% solve the optimization problem using fmincon
% type help fmincon to see all the inputs
[x,fval] = fmincon(@(x) objective(x),[1;1],[],[],[],[],[],[],...
    @(x) constraints(x,perimeter),options);

% display final (optimal) value of x and the objective function with this x
disp(x)
disp(fval)

% customize the Optimization Plot Function figure
hf = figure(1); hf.Color = 'w';
ha = gca; ha.YScale = 'log';

%--------------------------------------------------------------------------
% objective function
function J = objective(x)

% objective function value
J = x(1)*x(2);
% J = x(1)^4 + x(2)^2 + x(1)*x(2); % another objective function

end

%--------------------------------------------------------------------------
% constraint function
function [g,h] = constraints(x,perimeter)

% inequality constraints, g(x) <= 0
g1 = -x(1);
g2 = -x(2);
g = [g1;g2];

% equality constraints, h(x) = 0
h = 2*x(1) + 2*x(2) - perimeter;

end

%--------------------------------------------------------------------------
% NOTE: this stuff below is just to visualize the optimization algorithm
% progress. It is not needed in the class examples.
%--------------------------------------------------------------------------
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