% ex_feedback_linearization.m
% simulation of a nonlinear control law based on feedback linearization for
% a nonlinear mass-damper-spring system
% [reference] Chapter 9, Section 14 in Mechanical Engineer's Handbook
% [course] Session 8 - Nonlinear Control (2)
close all; clear; clc

% controller gains
k1c = 2;
k2c = 2;

% reference - one before (r0) and one after (r1) time t1
mycase = 2; % select a case
syms t
switch mycase
    case 1
        t1 = 8;
        r0 = 1;
        r1 = -1;
    case 2
        t1 = 6;
        r0 = 1;
        r1 = sin(2*t);
end

% create the needed reference functions (see function below)
[r,rd,rdd] = generate_references(t1,r0,r1);

% model parameters
b = 10;
k1 = 10;
k2 = 100;
M = 200;

% nonlinear functions for controllability form
f = @(x) (-b*x(2).*abs(x(2)) - k1*x(1) - k2*x(1).^3)/M;
g = @(x) 1/M;

% state derivative function with feedback linearization
deriv = @(x,w) [x(2); f(x) + g(x)*(w - f(x))/g(x)];

% simulation options
TSPAN = [0 20];
X0 = [0.5, 0.5];
OPTIONS = odeset('RelTol',1e-10);

% simulate
[T,X] = ode45(@(t,x) deriv(x,control(x,k1c,k2c,r,rd,rdd,t)),TSPAN,X0,OPTIONS);

% compute control (w and u/F)
for k = 1:size(X,1)
    Xk  = X(k,:);
    W(k) = control(Xk,k1c,k2c,r,rd,rdd,T(k));
    U(k) = (W(k) - f(Xk))/g(Xk);
end

% plot states (see function below)
plot_example_states(T,X,r(T),rd(T))

% plot states (see function below)
plot_example_control(T,W,U)

%--------------------------------------------------------------------------
% controller
function w = control(x,k1,k2,r,rd,rdd,t)

% errors
e1 = r(t) - x(1);
e2 = rd(t) - x(2);

% control law
w = rdd(t) + k1*e1 + k2*e2;

end

%--------------------------------------------------------------------------
% create reference functions, including time derivatives
function [r,rd,rdd] = generate_references(t1,ra,rb)

% initialize symbolic variable
syms t

% reference
raF = matlabFunction(ra,'Vars',t);
rbF = matlabFunction(rb,'Vars',t);

% first time derivative of r
raD = matlabFunction(diff(ra,t),'Vars',t);
rbD = matlabFunction(diff(rb,t),'Vars',t);

% second time derivative of r
raDD = matlabFunction(diff(ra,t,2),'Vars',t);
rbDD = matlabFunction(diff(rb,t,2),'Vars',t);

% create piecewise functions
r = @(t) (t<t1).*raF(t) + (t>=t1).*rbF(t);
rd = @(t) (t<t1).*raD(t) + (t>=t1).*rbD(t);
rdd = @(t) (t<t1).*raDD(t) + (t>=t1).*rbDD(t);

end

%--------------------------------------------------------------------------
% plotting code for states
% (not the main content)
function plot_example_states(T,X,R1,R2)

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
nicegreen = [109, 195, 80]/255;
nicegray = [170, 170, 170]/255;
LineWidth = 1;
MarkerSize = 12;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth,'MarkerSize',MarkerSize};

% initialize figure
hf = figure; hf.Color = 'w'; hold on

plot(T,R1,'--',plotOpts{:},'Color',nicegray,'DisplayName','$r$');
plot(T,R2,'--',plotOpts{:},'Color',nicegreen,'DisplayName','$\dot{r}$');
plot(T,X(:,1),plotOpts{:},'Color',nicered,'DisplayName','$x_1$');
plot(T,X(:,2),plotOpts{:},'Color',niceblue,'DisplayName','$x_2$');

xlabel('Time [sec]')
ylabel('Signals')

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

hl = legend(); hl.Location = "best"; hl.Interpreter = "latex"; hl.FontSize = FontSize;

end

%--------------------------------------------------------------------------
% plotting code for control
% (not the main content)
function plot_example_control(T,W,U)

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
LineWidth = 1;
MarkerSize = 12;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth,'MarkerSize',MarkerSize};

% initialize figure
hf = figure; hf.Color = 'w'; hold on

plot(T,W,plotOpts{:},'Color',nicered,'DisplayName','$w$');
plot(T,U,plotOpts{:},'Color',niceblue,'DisplayName','$u$');

xlabel('Time [sec]')
ylabel('Signals')

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

hl = legend(); hl.Location = "best"; hl.Interpreter = "latex"; hl.FontSize = FontSize;

end