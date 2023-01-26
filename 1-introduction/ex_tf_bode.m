% ex_tf_bode.m
% transfer function and bode plot of Example 2.1 RLC Circuit in LSC
close all; clear; clc

% numerator polynomial coefficients (e.g., 3s^2-4s+5 -> [3 -4 5])
num = [1];

% denominator polynomial coefficients (e.g., 7s^2+8s-9 -> [7 8 -9])
den = [1 1 1];

% transfer function object
sys = tf(num,den)

% Bode frequency response of dynamic systems
bodeplot(sys,'k')

% note the relationship between magnitude and decibels
title('dB = 20*log10(Mag)')

%--------------------------------------------------------------------------
% compute magnitude and phase at a specific frequency input
%--------------------------------------------------------------------------
% frequency of interest
w = 3;

% compute response magnitude and phase in degrees at w
[mag,phase] = bode(sys,w)

% transfer function
G = @(s) 1/(s^2 + s + 1);

% manually compute magnitude and phase shift at specific w
abs(G(1i*w))
rad2deg(angle(G(1i*w)))