% ex_lqr_robot_linear.m
% determination of the linear model and infinite-horizon LQR gain matrix
% for two-link robot example below in LSC with linear simulations of the
% results
% [reference] Example 5.6 Steady State LQR for the Two Link Robot in LSC
% [reference] Example 4.14 Two Link Robot Arm Control with Multiple
% Integrators in LSC where the pole placement gain matrix is determined
% [reference] Example 2.10 Two Link Robot Arm in LSC where the nonlinear
% and linear model is described
% [course] Session 10 - Optimal Control (2)
close all; clear; clc

%--------------------------------------------------------------------------
% create reference signal and linear model
%--------------------------------------------------------------------------
% number of states, controls, and inputs
nx = 4; nu = 2; ny = 2;

% time grid
nt = 1e4;
T = linspace(0,5,1e4)';

% stationary state values
X0_stationary = [deg2rad(45),0,deg2rad(-30),0,0,0];

% initial states
X0 = [deg2rad(45),0,deg2rad(-30),0,0,0]-X0_stationary;

% input reference signal
r = zeros(nt,ny);
r(T>=0.1,1) = deg2rad(-45)-X0_stationary(1);
r(T>=3,2) = deg2rad(30)-X0_stationary(3);

% system matrices
A = [0, 1, 0, 0;
17.832, 0, -3.0024, 0;
0, 0, 0, 1;
-30.063, 0, 10.456, 0];
B = [0, 0; 1.2514, -2.4337; 0, 0; -2.4337, 6.5512];
C = [1, 0, 0, 0;
    0, 0, 1, 0];

% add integrator states
Ac = [A,zeros(nx,ny);-C,zeros(ny)];
Bc = [B; zeros(ny,nu)];
Br = [zeros(nx,ny);eye(ny)];
Cc = [C,zeros(ny)];

% calculate actual reference
Ractual = r'+Cc*X0_stationary';

%--------------------------------------------------------------------------
% pole placement gain matrix from Example 4.14 in LSC
%--------------------------------------------------------------------------
% desired eigenvalues
P = [-7+1i*7, -7-1i*7, -8.6+1i*5, -8.6-1i*5, -9.7+1i*2.6, -9.7-1i*2.6];

% closed-loop pole assignment using state feedback
Kc = place(Ac,Bc,P);

% closed-loop system
sys = ss(Ac-Bc*Kc,Br,Cc,[]);

% linear simulation (NOTE: for NONLINEAR sim, see ex_lqr_robot_simulink.slx)
Y = lsim(sys,r,T,X0);

% outputs from linear pole placement simulation
Ypp = Y'+Cc*X0_stationary';

%--------------------------------------------------------------------------
% infinite-horizon LQR gain matrix from Example 5.6 in LSC
%--------------------------------------------------------------------------
% state weighting matrix
Q = zeros(6);
Q(1,1) = 1;
Q(3,3) = 1;
Q(5,5) = 500;
Q(6,5) = 250;
Q(5,6) = 250;
Q(6,6) = 500;

% control weighting matrix
R = zeros(2);
R(1,1) = 5e-4;
R(2,2) = 10*5e-4;

% additional parameter to increase or decrease control penalty
% rho = 1/10; % <- try this
rho = 1;
% rho = 10; % <- try this
R = rho*R;

% linear-quadratic regulator design for state-space systems
[Klqr,S,CLP] = lqr(Ac,Bc,Q,R,[]);

% closed-loop system
sys = ss(Ac-Bc*Klqr,Br,Cc,[]);

% linear simulation (NOTE: for NONLINEAR sim, see ex_lqr_robot_simulink.slx)
Y = lsim(sys,r,T,X0);

% outputs from linear lqr simulation
Ylqr = Y'+Cc*X0_stationary';

% plot angles
plot_example(T,Ractual,Ypp,Ylqr)

% plot closed-loop poles with both approaches
plot_example_poles(P,CLP)

%--------------------------------------------------------------------------
% plotting code for angles
% (not the main content)
function plot_example(T,R,Ypp,Ylqr)

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
nicegreen = [109, 195, 80]/255;
nicegray = [170, 170, 170]/255;
nicepink = [239, 142, 219]/255;
LineWidth = 1;
MarkerSize = 12;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth,'MarkerSize',MarkerSize};

% initialize figure for plotting theta and r states
hf = figure; hf.Color = 'w'; hold on

plot(T,rad2deg(R(1,:)),'--',plotOpts{:},'Color',nicegray,'DisplayName','$r_1$')
plot(T,rad2deg(Ypp(1,:)),'-',plotOpts{:},'Color',niceblue,'DisplayName','$\theta_{1,PP}$')
plot(T,rad2deg(Ylqr(1,:)),'-',plotOpts{:},'Color',nicered,'DisplayName','$\theta_{1,LQR}$')

plot(T,rad2deg(R(2,:)),'--',plotOpts{:},'Color',nicegray,'DisplayName','$r_2$')
plot(T,rad2deg(Ypp(2,:)),'-',plotOpts{:},'Color',nicegreen,'DisplayName','$\theta_{2,PP}$')
plot(T,rad2deg(Ylqr(2,:)),'-',plotOpts{:},'Color',nicepink,'DisplayName','$\theta_{2,LQR}$')

xlabel('Time [sec]')
ylabel('Joint Angle [deg]')

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

hl = legend(); hl.Location = "best"; hl.FontSize = FontSize; hl.Interpreter = 'latex';

end

%--------------------------------------------------------------------------
% plotting code for poles
% (not the main content)
function plot_example_poles(P,CLP)

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
LineWidth = 1;
MarkerSize = 12;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth,'MarkerSize',MarkerSize};

% initialize figure for plotting theta and r states
hf = figure; hf.Color = 'w'; hold on

plot(real(P),imag(P),'*',plotOpts{:},'Color',niceblue,'DisplayName','poles with pole placement')
plot(real(CLP),imag(CLP),'o',plotOpts{:},'Color',nicered,'DisplayName','poles with lqr')

xlabel('Real')
ylabel('Imag')

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

hl = legend(); hl.Location = "best"; hl.FontSize = FontSize;

end