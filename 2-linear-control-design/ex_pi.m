% ex_pi.m
% simulation of simple P and PI controllers on a third-order LTI system
% [reference] Section 8.1 The PID Regulator Family and Design Methods (pp.
% 278–290) in CE
% [course] Session 6 - Linear Control (3)
close all; clear; clc

% augmented system matrices
A = [-0.14 0.33 -0.33;
    0.1 -0.28 0;
    0 1.7 -0.77];
B = [0; 0; -0.025];
Bv = [1; 0; 0];
C = [2 0 0];

% simulation options
TSPAN = [0 100];
X0 = [0 0 0];
OPTIONS = odeset('RelTol',1e-13,'AbsTol',1e-10);

%--------------------------------------------------------------------------
% parameters (try changing these)
%--------------------------------------------------------------------------
r = 10; % reference (constant)
v = 0; % disturbance (constant)

Kp = 50; % proportional term gain
Ti = 3; % integral time constant
% Kp = 4.74; % <- try this too
% Ti = 2.04; % <- try this too

%--------------------------------------------------------------------------
% P controller
%--------------------------------------------------------------------------
% simulate (see state derivative function below)
[Tp,Xp] = ode45(@(t,x) derivP(t,x,r,Kp,A,B,Bv,C,v),TSPAN,X0,OPTIONS);

% calculate output and control
Yp = C*Xp';
Up = Kp*(r-Yp);

%--------------------------------------------------------------------------
% PI controller
%--------------------------------------------------------------------------
% initial integrator state
X0(end+1) = 0;

% simulate (see state derivative function below)
[TI,XI] = ode45(@(t,x) derivPI(t,x,r,Kp,A,B,Bv,C,v,Ti),TSPAN,X0,OPTIONS);

% calculate output and control
YI = C*XI(:,1:end-1)';
UI = Kp*(r-YI') + Kp/Ti*XI(:,end);

% plot results (see function below)
plot_example(Tp,Yp,Up,TI,YI,UI,r)

%--------------------------------------------------------------------------
% P controller derivative function
%--------------------------------------------------------------------------
function Dx = derivP(t,x,r,Kp,A,B,Bv,C,v)

% output
y = C*x;

% error
e = r-y;

% control
u = Kp*e;

% state derivative function
Dx = A*x + B*u + Bv*v;

end

%--------------------------------------------------------------------------
% PI controller derivative function
%--------------------------------------------------------------------------
function Dx = derivPI(t,x,r,Kp,A,B,Bv,C,v,Ti)

% output
y = C*x(1:end-1);

% error
e = r-y;

% extract integral state
integral_error = x(end);

% control
u = Kp*e + Kp/Ti*integral_error;

% state derivative function
Dx = A*x(1:end-1) + B*u + Bv*v;
Dx(end+1) = r-y;

end

%--------------------------------------------------------------------------
% plotting code
% (not the main content)
function plot_example(Tp,Yp,Up,Ti,Yi,Ui,r)

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
nicegray = [170, 170, 170]/255;
LineWidth = 1;
MarkerSize = 12;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth,'MarkerSize',MarkerSize};

% initialize figure (control)
hf = figure(1); hf.Color = 'w'; hold on

plot(Tp,Up,plotOpts{:},'Color',niceblue,'DisplayName',strcat("u for P controller"))
plot(Ti,Ui,plotOpts{:},'Color',nicered,'DisplayName',strcat("u for PI controller"))

xlabel('Time [s]')
ylabel('Control')

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

hl = legend(); hl.Location = "best";

% initialize figure (output)
hf = figure(2); hf.Color = 'w'; hold on

plot([Tp(1) Tp(end)],[r r],plotOpts{:},'Color',nicegray,'DisplayName',strcat("r"))
plot(Tp,Yp,plotOpts{:},'Color',niceblue,'DisplayName',strcat("y for P controller"))
plot(Ti,Yi,plotOpts{:},'Color',nicered,'DisplayName',strcat("y for PI controller"))

xlabel('Time [s]')
ylabel('Output')

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

hl = legend(); hl.Location = "best";

end