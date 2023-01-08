close all; clear; clc

% system matrices
A = [-0.14 0.33 -0.33; 0.1 -0.28 0; 0 1.7 -0.77];
B = [0; 0; -0.025];
C = [2 0 0];

% state-space model
sys = ss(A,B,C,[]);

% Rise time, settling time, and other step-response characteristics
S = stepinfo(sys)
k = dcgain(sys)

%% full state feedback
% desired eigenvalues
epsilon = [0 1e-4 -1e-4];
E = [-0.67 -0.67 -0.67]+epsilon;

% closed-loop pole assignment using state feedback
K = place(A,B,E);

% closed-loop system
sys1 = ss(A-B*K,B,C,[]);

S1 = stepinfo(sys1)
k1 = dcgain(sys1)

%% full state feedback with integrator
% augmented system matrices
Ai = [-0.14 0.33 -0.33 0;
    0.1 -0.28 0 0;
    0 1.7 -0.77 0;
    -2 0 0 0];
Bi = [0; 0; -0.025; 0];
Bri = [0; 0; 0; 1];
Bvi = [1; 0; 0; 0];
Ci = [2 0 0 0];

% desired eigenvalues
epsilon = 1e-3;
Ei = [-0.67 -0.67 -0.67 -0.67] - [0 epsilon 2*epsilon 3*epsilon];

% closed-loop pole assignment using state feedback
Ki = place(Ai,Bi,Ei);

% create the closed-loop state-space models
sysi = ss(Ai-Bi*Ki,Bri,Ci,[]);

Si = stepinfo(sysi)
ki = dcgain(sysi)

%% Gain margin, phase margin, and crossover frequencies
rng(4456434)
P = rss(4);
[Gm,Pm,Wcg,Wcp] = margin(P)

figure;
bode(P)

%% Sensitivity functions of plant-controller feedback loop
rng(242)
P = rss(6);
C = rss(3);

SF = loopsens(P,C)

figure;
bode(SF.Si)