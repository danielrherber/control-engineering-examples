close all; clear; clc

% Code from the first example in:
% https://www.mathworks.com/help/robust/ref/lti.hinfsyn.html

% Synthesize a controller using different target performance levels. The
% plant in this example is based on the augmented plant model used in
% Robust Control of an Active Suspension. Load the plant.
load hinfsynExData P
size(P)

% This plant has five outputs and four inputs, where the last two outputs
% are measurement signals to provide to the controller, and the last input
% is a control signal. Compute an H∞-optimal controller
ncont = 1;
nmeas = 2;
[K1,CL,gamma] = hinfsyn(P,nmeas,ncont);

% The resulting two-input, one-output controller has the same number of
% states as P
size(K1)

% The optimal performance level achieved by this controller is returned as
% gamma. This value is the H∞ norm of the closed-loop system CL
gamma

% You can examine the singular value plot of the closed-loop system to
% confirm that its largest singular value does not exceed gamma
sigma(CL,ss(gamma))
ylim([-120,20]);

% Directly compute the Hinf norm
P1 = P(end-nmeas+1:end,end-ncont+1:end);
ninf_P1 = mag2db(hinfnorm(P1))
ninf_CL = mag2db(hinfnorm(CL))

% Also plot original plant
figure
sigma(CL,ss(gamma),P1)
ylim([-120,30]);