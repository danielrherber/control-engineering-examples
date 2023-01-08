close all; clear; clc

% system matrices
A = [-0.14 0.33 -0.33;
    0.1 -0.28 0;
    0 1.7 -0.77];
B1 = [0; 0; -1];
B3 = [-1 0 0; 0 -1 0; 0 0 -1];

% Bv = [1; 0; 0];
Bv = [0; 0; 1];
C = [2 0 0];
% C = [0 0 1];

% original system eigenvalues
eig(A)

% check controllability
Mc = ctrb(A,B1);
rank(Mc)

% desired eigenvalues
epsilon = 1e-4;
E = [-0.67 -0.67 -0.67] + [0 epsilon 2*epsilon];

% closed-loop pole assignment using state feedback
K1 = place(A,B1,E);
K3 = place(A,B3,E);

% create the closed-loop state-space models
sys1 = ss(A-B1*K1,Bv,C,[]);
sys3 = ss(A-B3*K3,Bv,C,[]);

% simulations options
% X0 = [0;0;0];
X0 = [1;1;1];
tfinal = 100;
T = linspace(0,tfinal,10000)';

% reference signal
r = 1*ones(size(T));
% r = 10*sin(T);

% simulate
[Y1,T1,X1] = lsim(sys1,r,T,X0);
[Y3,T3,X3] = lsim(sys3,r,T,X0);

% plot outputs
figure; hold on
plot(T1,Y1)
plot(T3,Y3)
plot(T,r)
legend('Y1','Y3','r')