% ex_Hinfinity.m
% Synthesize a H-infinity optimal controller controller using different
% target performance levels. The plant in this example is based on the
% augmented plant model used in Robust Control of an Active Suspension
% www.mathworks.com/help/robust/ref/lti.hinfsyn.html
close all; clear; clc

% load the plant
load hinfsynExData P

% get size of the plant
nP = size(P)

% this plant has five outputs and four inputs, where the last two outputs
% are measurement signals to provide to the controller, and the last input
% is a control signal. Compute an H-infinity optimal controller
ncont = 1;
nmeas = 2;
[K1,CL,gamma] = hinfsyn(P,nmeas,ncont);

% the resulting two-input, one-output controller has the same number of
% states as P
size(K1)

% the optimal performance level achieved by this controller is returned as
% gamma. This value is the H-infinity norm of the closed-loop system CL
% between W -> Z
gamma

% get original transfer function between W -> Z
P_original = P(1:(nP(1)-nmeas),1:(nP(2)-ncont));

% directly compute the H-infinity norm of both systems
Hinf_Pwz_original = mag2db(hinfnorm(P_original))
Hinf_CL = mag2db(hinfnorm(CL))

% plot singular values of the systems along with achieved gamma
hf = figure; hf.Color = 'w';
Gamma = ss(gamma);
sigma(P_original,CL,Gamma)
ylim([-60,10])
legend