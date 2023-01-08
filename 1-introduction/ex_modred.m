close all; clear; clc

% symbolic transfer function
syms s
num = (s+10)*(s-5)*(s^2 + 2*s + 5)*(s^2 - 0.5*s +5);
den = (s+4)*(s^2+4*s+8)*(s^2+0.2*s+100)*(s^2+5*s+2000);

% get coefficients
num2 = double(fliplr(coeffs(num)));
den2 = double(fliplr(coeffs(den)));

% transfer function coefficients to state-space matrices
[A,B,C,D] = tf2ss(num2,den2);

% state-space model
sys = ss(A,B,C,D);

% compute gramians
Wc  = gram(sys,'c');
Wo  = gram(sys,'o');

% Hankel singular values (direct)
G2 = sqrt(eig(Wc*Wo));

% Gramian-based balancing of state-space realizations
[SYSB,G,T,Ti] = balreal(sys);

% number of states in the original system
n = length(G);

% number of desired states in the reduced-order model
r = 4;

% 0: keep the state, 1:eliminate the state
elim = logical([zeros(r,1);ones(n-r,1)]);

% model simplification/reduction by state elimination
RSYS = modred(SYSB,elim,'truncate');

% Bode response of both systems
bode(sys,RSYS)