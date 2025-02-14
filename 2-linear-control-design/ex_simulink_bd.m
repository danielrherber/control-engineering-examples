% ex_simulink_bd.m
% symbolic calculation of the equivalent transfer function and
% determination numerator and denominator coefficients
% [reference] See section Frequency-Domain Block Diagrams
% [course] Session 4 - Block Diagrams and Linear Control (1)
close all; clear; clc

% define transfer functions
syms s
G1 = 1/(s^3 + 4*s^2 + s + 1);
G2 = (s+1)/(10*s^2 + 5*s + 1);
G3 = (s+1)/(s^2+5*s+6);
G4 = 1/(s+1);
H1 = 1/(s+6);

% try different gains for H2
H2 = sym(0.01);
% H2 = sym(1);
% H2 = sym(10);

% numerator
N = G1*G2*(G3+G4);

% denominator
D = 1 - G1*G2*H1 + G1*G2*(G3+G4)*H2;

% overall transfer function
G = N/D;
G = simplify(G);

% determine numerator and denominator coefficients
[Nc,Dc] = numden(G);
coeffs(Nc,s,'All')
coeffs(Dc,s,'All')