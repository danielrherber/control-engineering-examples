% ex_integral_control.m
% design and simulation of a full state feedback controller with
% output error integrators from Example 4.12 Integral Control of a Third
% Order System in LSC
close all; clear; clc

% augmented system matrices
A1 = [-0.14 0.33 -0.33 0;
    0.1 -0.28 0 0;
    0 1.7 -0.77 0;
    -2 0 0 0];
B1 = [0; 0; -0.025; 0];
Br1 = [0; 0; 0; 1];
Bv1 = [1; 0; 0; 0];
C1 = [2 0 0 0];

% original system matrices
A = [-0.14 0.33 -0.33;
    0.1 -0.28 0;
    0 1.7 -0.77];
B = [0; 0; -0.025];
Br = [0; 0; 0];
Bv = [1; 0; 0];
C = [2 0 0];

% check controllability
Mc = ctrb(A1,B1);
rank(Mc);

% desired eigenvalues
epsilon = 1e-3;
tc = 1.5; E1 = [-1/tc -1/tc -1/tc -1/tc] - [0 epsilon 2*epsilon 3*epsilon];

% closed-loop pole assignment using state feedback
K1 = place(A1,B1,E1); % with output error integrator
K = place(A,B,E1(1:3));

% create the closed-loop state-space models
sys1 = ss(A1-B1*K1,Br1,C1,[]); % with output error integrator
sys = ss(A-B*K,Br,C,[]);

% simulations options
X0 = [1 1 1 0]'; % integrator state initialized to 0
tfinal = 100;
T = linspace(0,tfinal,10000)';

% reference signals
% r = -3*ones(size(T)); % <- also try this
r = floor(sin(0.1*T));
% r = 0.1*T; % <- also try this
% r = sin(0.3*T); % <- also try this
% x = 1; % <- also try this
% r = 2*T.^x;  % <- also try this % note: error increases with x > 1

% simulate
Y1 = lsim(sys1,r,T,X0); % with output error integrator
Y = lsim(sys,r,T,X0(1:3));

% plot outputs and reference
hf = figure; hf.Color = 'w'; hold on
plot(T,[Y1,Y,r],'LineWidth',2)
xlabel('Time [sec]'); ylabel('Signals');
legend('Output with integrator','Output without integrator','Reference')
title('Output tracking without a disturbance')

% create the closed-loop state-space models for a non-zero disturbance
sys1v = ss(A1-B1*K1,[Br1,Bv1],C1,[]);
sysv = ss(A-B*K,[Br,Bv],C,[]);

% disturbance
% v = -3*ones(size(T)); % <- also try this
% v = 0.1*floor(sin(0.1*T)); % <- also try this
v = 0.1*sin(8*T);

% inputs (reference + disturbance)
u = [r,v];

% simulate
Y1v = lsim(sys1v,u,T,X0);
Yv = lsim(sysv,u,T,X0(1:3));

% plot outputs and inputs
hf = figure; hf.Color = 'w'; hold on
plot(T,[Y1v,Yv,r,v],'LineWidth',2)
xlabel('Time [sec]'); ylabel('Signals');
legend('Output with integrator','Output without integrator','Reference','Disturbance')
title('Output tracking with a disturbance')