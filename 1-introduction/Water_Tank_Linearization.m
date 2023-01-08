close all; clear; clc

% initialize symbolic variables
syms x1 x2 u1 u2 A ka C0
syms x10 x20 u10 u20
syms x1_ x2_ x3_ x4_

% different variable vectors
vars = [x1;x2;u1;u2];
vars0 = [x10;x20;u10;u20];
vars_delta = [x1_;x2_;x3_;x4_];

% original nonlinear state 1 derivative function
Dx1 = 1/A*(ka*(u1+u2) - C0*sqrt(x1-x2));

% compute first-order taylor series
Dx1_taylor = taylor(Dx1,vars,vars0,'Order',2)

% stationary point (2.98), first equation
Usub = sqrt(x10-x20)*C0/ka;

% substitute in the stationary point
Dx1_taylor0 = subs(Dx1_taylor,u10 + u20,Usub);

% substitute in the new states
Dx1_taylor0 = subs(Dx1_taylor0,vars - vars0,vars_delta);

% simplify
Dx1_taylor0 = simplify(Dx1_taylor0,'steps',100);

% put in linear form
[A,b] = equationsToMatrix(Dx1_taylor0,vars_delta)

% so the differential equation using states vars_delta
Dx1_linear = A*vars_delta + b