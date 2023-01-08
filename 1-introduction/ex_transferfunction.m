close all; clear; clc

% numerator polynomial coefficients
NUM = [1];

% denominator polynomial coefficients
DEN = [1 1 1];

% transfer function
SYS = tf(NUM,DEN)

% Bode frequency response of dynamic systems
bode(SYS)

% transfer function
G = @(s) 1/(s^2 + s + 1);

% manually compute magnitude and phase shift at specific w
w = 3;
abs(G(i*w))
rad2deg(angle(G(i*w)))