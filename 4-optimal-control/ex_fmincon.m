% ex_fmincon.m
% example using fmincon to solve a finite-dimensional optimization problem
% with the initial problem of finding the maximum area rectangle with a
% prescribed perimeter value
% [course] Session 09 - Optimal Control (1)
close all; clear; clc; clear global p_old

% solve the following finite-dimensional optimization problem:
% min J = -p1*p2
% subject to: h1 : 2*p1 + 2*p2 = perimeter
%             g1 : p1 >= 0
%             g2 : p2 >= 0

% for visualizing some things (see below)
iteration_plot_setup

% problem parameters
perimeter = 20;

% fmincon options (to display iteration progress and other plots)
options = optimoptions('fmincon','Display','iter',...
    'PlotFcn',{'optimplotfvalconstr','optimplotfirstorderopt'},...
    'OutputFcn',@outfun);

% initial guess
p_guess = [1;1];

% solve the optimization problem using fmincon
% type help fmincon to see all the inputs
[p_opt,J_opt] = fmincon(@(p) objective(p),p_guess,[],[],[],[],[],[],...
    @(p) constraints(p,perimeter),options);

% display final (optimal) p value and the optimal objective function value
disp(p_opt)
disp(J_opt)

% customize the Optimization Plot Function figure
FontSize = 12;
hf = figure(1); hf.Color = 'w';
ha = hf.Children(7); ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;
ha = hf.Children(5); ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;
ha.YScale = 'log';

%--------------------------------------------------------------------------
% objective function
function J = objective(p)

% objective function value
J = -p(1)*p(2); % maximize area (as a minimization problem)
% J = p(1)^4 + p(2)^2 + p(1)*p(2); % another objective function

end

%--------------------------------------------------------------------------
% constraint function
function [g,h] = constraints(p,perimeter)

% inequality constraints, g(x) <= 0
g1 = -p(1);
g2 = -p(2);
g = [g1;g2];

% equality constraints, h(x) = 0
h = 2*p(1) + 2*p(2) - perimeter;

end

%--------------------------------------------------------------------------
% plotting code to visualize the optimization algorithm
% (not the main content)
%--------------------------------------------------------------------------
% optional customized output function to displaying optimization algorithm
% progress
function stop = outfun(p,optimValues,state)

% colors and other parameters
LineWidth = 1;
MarkerSize = 12;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth,'MarkerSize',MarkerSize};

% don't stop the optimization routing here
stop = false;

% call up figure 2
figure(2); hold on

% get previous optimization variable value
global p_old
if isempty(p_old)
    p_old = p;
end

% plot current optimization variables (2D problem)
plot([p_old(1) p(1)],[p_old(2) p(2)],'.-',plotOpts{:})

% update x_old
p_old = p;

% display the current iteration number
text(p(1)+0.01,p(2)-0.01,string(optimValues.iteration),'FontSize',FontSize)

end

function iteration_plot_setup

% colors and other parameters
FontSize = 12;

% initialize figure 2
hf = figure(2); hf.Color = 'w'; clf; hold on

xlabel('$p_1$','Interpreter','latex','FontSize',FontSize)
ylabel('$p_2$','Interpreter','latex','FontSize',FontSize)

global x_old
x_old = [];

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

end