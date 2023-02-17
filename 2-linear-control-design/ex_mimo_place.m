% ex_mimo_place.m
% illustration of eigenstructure assignment for MIMO systems using a
% modified system from Example 4.5 Continuous Control via the Controller
% Canonical Form in LSC using the Matlab place command
close all; clear; clc

% system matrices
A = [-0.14 0.33 -0.33;
    0.1 -0.28 0;
    0 1.7 -0.77];

B1 = [0; 0; -1]; % single input
B3 = [-1 0 0; 0 -1 0; 0 0 -1]; % multiple input

Bv = [0; 0; 1];
% Bv = [1; 0; 0]; % <- also try this

C = [2 0 0];
% C = [0 0 1]; % <- also try this

% original system eigenvalues
eig(A)

% check controllability
Mc = ctrb(A,B1);
rank(Mc)

% desired eigenvalues
epsilon = 1e-4;
tc = 1.5; E = [-1/tc -1/tc -1/tc] + [0 epsilon 2*epsilon];

% closed-loop pole assignment using state feedback
K1 = place(A,B1,E);
K3 = place(A,B3,E);

% create the closed-loop state-space models
sys1 = ss(A-B1*K1,Bv,C,[]);
sys3 = ss(A-B3*K3,Bv,C,[]);

% simulations options
X0 = [1;1;1];
tfinal = 100;
T = linspace(0,tfinal,10000)';

% disturbance signal
v = 1*ones(size(T));
% v = 10*sin(T); % <- also try this

% simulate
[Y1,T1,X1] = lsim(sys1,v,T,X0);
[Y3,T3,X3] = lsim(sys3,v,T,X0);

% plot outputs
hf = figure; hf.Color = 'w'; hold on
plot(T1,Y1)
plot(T3,Y3)
plot(T,v)
xlabel('time [sec]'); ylabel('output');
legend('Y with 1 input','Y with 3 inputs','disturbance')

% transfer functions between the output and disturbance
syms s
[N1,D1] = ss2tf(sys1.A,sys1.B,sys1.C,sys1.D);
[N3,D3] = ss2tf(sys3.A,sys3.B,sys3.C,sys3.D);