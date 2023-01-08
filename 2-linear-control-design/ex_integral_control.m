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
E1 = [-0.67 -0.67 -0.67 -0.67] - [0 epsilon 2*epsilon 3*epsilon];

% closed-loop pole assignment using state feedback
K1 = place(A1,B1,E1);
K = place(A,B,E1(1:3));

% create the closed-loop state-space models
sys1 = ss(A1-B1*K1,Br1,C1,[]);
sys = ss(A-B*K,Br,C,[]);

% simulations options
X0 = [1 1 1 0]';
tfinal = 100;
T = linspace(0,tfinal,10000)';

% reference signals
% r = -3*ones(size(T));
r = floor(sin(0.1*T));
% r = 0.1*T;
% r = sin(0.3*T);
% x = 1;
% r = 2*T.^x; % note: error increases with x > 1

% simulate
Y1 = lsim(sys1,r,T,X0);
Y = lsim(sys,r,T,X0(1:3));

% plot outputs and reference
figure; hold on
plot(T,Y1)
plot(T,Y)
plot(T,r)
legend('Y1','Y','r')

% plot error
figure; hold on
plot(T,abs(r-Y1))
ha = gca; ha.YScale = 'log';

%% non-zero disturbance
% create the closed-loop state-space models
sys1v = ss(A1-B1*K1,[Br1,Bv1],C1,[]);
sysv = ss(A-B*K,[Br,Bv],C,[]);

% reference signals
r = 1*ones(size(T));

% disturbance
% v = -10*ones(size(T));
v = 0.1*floor(sin(0.1*T));
% v = 0.1*sin(0.1*T);

% inputs
u = [r,v];

% simulate
Y1v = lsim(sys1v,u,T,X0);
Yv = lsim(sysv,u,T,X0(1:3));

% plot outputs and inputs
figure; hold on
plot(T,Y1v)
plot(T,Yv)
plot(T,r)
plot(T,v)
legend('Y1','Y','r','v')

% bode with for reference and disturbance
figure
sys1v = ss(A1-B1*K1,Bv1,C1,[]);
bode(sys1v,'r')