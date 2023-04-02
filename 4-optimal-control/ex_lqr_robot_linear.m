% ex_lqr_robot_linear.m
% determination of the linear model and infinite-horizon LQR gain matrix
% for Example 5.6 Steady State LQR for the Two Link Robot in LSC with
% linear simulations of the results
% *also see Example 4.14 Two Link Robot Arm Control with Multiple
% Integrators in LSC where the pole placement gain matrix is determined
% *also see Example 2.10 Two Link Robot Arm in LSC where the nonlinear and
% linear model is described
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

% initialize figure
hf = figure; hf.Color = 'w'; hold on
xlabel('Time [sec]')
ylabel('Joint Angle [rad]')

% plot reference
plot(T,r'+Cc*X0_stationary','k--')

%--------------------------------------------------------------------------
% pole placement gain matrix from Example 4.14 Two Link Robot Arm Control
% with Multiple Integrators in LSC
%--------------------------------------------------------------------------
% desired eigenvalues
P = [-7+1i*7, -7-1i*7, -8.6+1i*5, -8.6-1i*5, -9.7+1i*2.6, -9.7-1i*2.6];

% closed-loop pole assignment using state feedback
Kc = place(Ac,Bc,P);

% closed-loop system
sys = ss(Ac-Bc*Kc,Br,Cc,[]);

% linear simulation
% NOTE: for the NONLINEAR simulation, see ex_lqr_robot_simulink.slx
Y = lsim(sys,r,T,X0);

% plot outputs from pole placement simulation
plot(T,Y'+Cc*X0_stationary')

%--------------------------------------------------------------------------
% infinite-horizon LQR gain matrix from Example 5.6 Steady State LQR for
% the Two Link Robot in LSC
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

% linear simulation
% NOTE: for the NONLINEAR simulation, see ex_lqr_robot_simulink.slx
Y = lsim(sys,r,T,X0);

% plot outputs from lqr solution
plot(T,Y'+Cc*X0_stationary','linewidth',2)

% add legend legends
legend('r_1','r_2','\theta_{1,PP}','\theta_{2,PP}','\theta_{1,LQR}',...
    '\theta_{2,LQR}','location','best')

%--------------------------------------------------------------------------
% plot closed-loop poles with both approaches
%--------------------------------------------------------------------------
hf = figure; hf.Color = 'w'; hold on
xlabel('Real'); ylabel('Imag');
plot(real(P),imag(P),'*')
plot(real(CLP),imag(CLP),'o')
legend('poles with pole placement','poles with lqr')