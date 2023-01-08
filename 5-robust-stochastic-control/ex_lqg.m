close all; clear; clc;

% code from:
% https://www.mathworks.com/help/control/getstart/design-an-lqg-servo-controller.html

% create the state space system
A = [0 1 0;0 0 1;1 0 0];
B = [0.3 1;0 1;-0.3 0.9];
G = [-0.7 1.12; -1.17 1; .14 1.5];
C = [1.9 1.3 1];
D = [0.53 -0.61];
H = [-1.2 -0.89];
sys = ss(A,[B G],C,[D H]);

% construct the optimal state-feedback with integral control gain using the
% given cost function
nx = 3; % number of states
ny = 1; % number of outputs
Q = blkdiag(0.1*eye(nx),eye(ny));
R = [1 0;0 2];
[K,S,CLP] = lqi(ss(A,B,C,D),Q,R);

% construct Kalman state estimator using the given noise covariance data
Qn = [4 2;2 1];
Rn = 0.7;
[kest,L,P] = kalman(sys,Qn,Rn);

% connect the Kalman state estimator and the optimal state-feedback gain to
% form the LQG servo (integral) controller
trksys = lqgtrack(kest,K)

% compare open-loop and closed-loop eigenvalues
eig(A)
eig(trksys.A)

% alternative approaches
% https://www.mathworks.com/help/control/getstart/linear-quadratic-gaussian-lqg-design.html
% reg = lqg(sys,QXU,QWV)
% [K,S,e] = lqr(SYS,Q,R,N)
% [kalmf,L,P] = kalman(sys,Q,R,N)
% rlqg = lqgreg(kest,k)
% [K,S,e] = lqi(SYS,Q,R,N)
% C = lqgtrack(kest,k)