close all; clear; clc

% system matrices
A = [-0.14 0.33 -0.33;
    0.1 -0.28 0;
    0 1.7 -0.77];
B = [0; 0; -0.025];
Br = [0; 0; 0];
Bv = [1; 0; 0];
C = [2 0 0];

% state-space model
P = ss(A,B,C,[]);
disp(stepinfo(P))

% PI controller tuned by Matlab
[C_PI,info] = pidtune(P,'PI');
T_PI = feedback(C_PI*P,1);
disp(info)
disp(stepinfo(T_PI))

% manual wc PI controller
wc = 1;
[C_PI_fast,info] = pidtune(P,'PI',wc);
T_PI_fast = feedback(C_PI_fast*P,1);
disp(info)
disp(stepinfo(T_PI_fast))

% PID controller tuned by Matlab
[C_PID,info] = pidtune(P,'PID');
T_PID = feedback(C_PID*P,1);
disp(info)
disp(stepinfo(T_PID))

% plot step responses
step(P,T_PI,T_PI_fast,T_PID)
legend('P','PI1','PI2','PID')

% PID tuner application
pidTuner(P,T_PID)