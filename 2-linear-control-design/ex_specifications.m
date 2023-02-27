% ex_specifications.m
% illustration of the time and frequency domain specifications using Matlab
% functions stepinfo, dcgain, allmargin, and loopsens as well as Nyquist
% diagrams
close all; clear; clc

% system matrices
A = [-0.14 0.33 -0.33; 0.1 -0.28 0; 0 1.7 -0.77];
B = [0; 0; -0.025];
C = [2 0 0];

% state-space model
sys = ss(A,B,C,[]);

% rise time, settling time, and other step-response characteristics
S = stepinfo(sys)
k = dcgain(sys)

%--------------------------------------------------------------------------
% full state feedback
%--------------------------------------------------------------------------
% desired eigenvalues
epsilon = [0 1e-4 -1e-4];
E = [-0.67 -0.67 -0.67]+epsilon;

% closed-loop pole assignment using state feedback
K = place(A,B,E);

% closed-loop system
sys1 = ss(A-B*K,B,C,[]);

% rise time, settling time, and other step-response characteristics
S1 = stepinfo(sys1)
k1 = dcgain(sys1)

%--------------------------------------------------------------------------
% full state feedback with integrator
%--------------------------------------------------------------------------
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

% rise time, settling time, and other step-response characteristics
Si = stepinfo(sysi)
ki = dcgain(sysi)

%--------------------------------------------------------------------------
% gain margin, phase margin, and crossover frequencies
%--------------------------------------------------------------------------
rng(6848)
% rng(4456434) % <- also try this
P = rss(4); % generate a randomized continuous-time state-space model
Sm = allmargin(P)

% plot Bode frequency response
hf = figure; hf.Color = 'w';
bode(P)

% plot Nyquist frequency response
hf = figure; hf.Color = 'w';
nyquist(P)

%--------------------------------------------------------------------------
% sensitivity functions of plant-controller feedback loop
%--------------------------------------------------------------------------
% create a state-space model
rng(242)
P = rss(6);
C = rss(3);

% determine sensitivity functions of plant-controller feedback loop
SF = loopsens(P,C)

% plot sensitivity at the plant input
hf = figure; hf.Color = 'w';
bode(SF.Si)