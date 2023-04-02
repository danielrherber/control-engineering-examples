% ex_lqr_robot_eqs.m
% determination of the linear model and infinite-horizon LQR gain matrix
% for Example 5.6 Steady State LQR for the Two Link Robot in LSC, which is
% used in ex_lqr_robot_simulink.slx for the simulations
% *also see Example 4.14 Two Link Robot Arm Control with Multiple
% Integrators in LSC where the pole placement gain matrix is determined
% *also see Example 2.10 Two Link Robot Arm in LSC where the nonlinear and
% linear model is described
close all; clear; clc

% problem parameters
J1_ = 0.12;
J2_ = 0.15;
g_ = 9.81;
l1_ = 0.6;
l2_ = 0.8;
m1_ = 3;
m2_ = 2.5;

% linearization point
T10 = deg2rad(45);
T20 = deg2rad(-30);

% initialize the symbolic variables
syms m1 m2 l1 l2 J1 J2 g
syms t
syms T1 T2 DT1 DT2
syms tau1 tau2

% control and state vectors
u = [tau1;tau2];
x = [T1;DT1;T2;DT2];

% create the symbolic M matrix
M11 = (1/4*m1+m2)*l1^2 + 1/4*m2*l2^2 + J1 + J2 + m2*l1*l2*cos(T2);
M12 = 1/4*m2*l2^2 + J2 + 1/2*m2*l1*l2*cos(T2);
M22 = 1/4*m2*l2^2 + J2;
M = [M11 M12; M12 M22]; % combine

% create K matrix entries
% K1 = -1/2*m2*l1*l2*sin(T2)*DT2*(2*DT1 + DT2);
% K2 = 1/2*m2*l1*l2*sin(T2)*DT1^2;

% create G matrix entries
G1 = m1*g*l1/2*cos(T1) + m2*g*(l1*cos(T1) + l2/2*cos(T1+T2));
G2 = m2*g*l2/2*cos(T1+T2);

% linearization matrices in symbolic form
H = inv(M);

H11 = H(1,1);
H12 = H(1,2);
H21 = H(2,1);
H22 = H(2,2);

A21 = -diff(H11*G1 + H12*G2,T1);
A23 = -diff(H11*G1 + H12*G2,T2);

A41 = -diff(H21*G1 + H22*G2,T1);
A43 = -diff(H21*G1 + H22*G2,T2);

A_sym = [0 1 0 0; A21 0 A23 0; 0 0 0 1; A41 0 A43 0];
B_sym = [0 0; H11 H12; 0 0; H21 H22];

% substitute linearization point values and create matlab functions
A_sym = subs(A_sym,'T1',T10);
A_sym = subs(A_sym,'T2',T20);
Afun = matlabFunction(A_sym);

B_sym = subs(B_sym,'T1',T10);
B_sym = subs(B_sym,'T2',T20);
Bfun = matlabFunction(B_sym);

M = subs(M,'T1',T10);
M = subs(M,'T2',T20);
Mfun = matlabFunction(M);

G1 = subs(G1,'T1',T10);
G1 = subs(G1,'T2',T20);
G1fun = matlabFunction(G1);

G2 = subs(G2,'T1',T10);
G2 = subs(G2,'T2',T20);
G2fun = matlabFunction(G2);

% evaluate the matrices for the provided problem parameters
A_ = Afun(J1_,J2_,g_,l1_,l2_,m1_,m2_);
M_ = Mfun(J1_,J2_,l1_,l2_,m1_,m2_);
B_ = Bfun(J1_,J2_,l1_,l2_,m1_,m2_);
tau10 = G1fun(g_,l1_,l2_,m1_,m2_);
tau20 = G2fun(g_,l2_,m2_);

% stationary point values
u0 = [tau10; tau20];
x0 = [T10; 0; T20; 0];

% construct state weighting matrix
Q = zeros(6);
Q(1,1) = 1;
Q(3,3) = 1;
Q(5,5) = 500;
Q(5,6) = 250;
Q(6,5) = 250;
Q(6,6) = 500;

% construct control weighting matrix
% rho = 1/10; % <- try this
rho = 1;
% rho = 10; % <- try this
R = zeros(2,2);
R(1,1) = rho*5e-4*1;
R(2,2) = rho*5e-4*10;
C = [1 0 0 0 ; 0 0 1 0];

% combined linear system to be used with lqr and place
A_lin = [A_,zeros(4,2);-C,zeros(2)];
B_lin = [B_;zeros(2,2)];
C_lin = [C zeros(2,2)];

% create state-space model
sys = ss(A_lin,B_lin,C_lin,[]);

%--------------------------------------------------------------------------
% infinite-horizon linear-quadratic regulator design
[K1,S,CLP] = lqr(sys,Q,R);

% extract gain matrices
K = K1(:,1:4);
Ki = -K1(:,5:6);

%--------------------------------------------------------------------------
% % pole placement matrices from Example 4.14
% P = [-7+7i -7-7i -8.6+5i -8.6-5i -9.7+2.6i -9.7-2.6i ]*0.7;
% [Kpp,PREC] = place(A_lin,B_lin,P);
%
% % extract gain matrices
% K = Kpp(:,1:4);
% Ki = -Kpp(:,5:6);