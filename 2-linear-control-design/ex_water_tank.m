% ex_water_tank.m
% matrices for water tank example used when initializing the Simulink model
% in ex_water_tank_control.slx
% [reference] Example 4.13 Water Tank Process Control with Multiple
% Integrators in LSC
% [course] Session 5 - Linear Control (2)
close all; clear; clc

n = 4; % states
m = 2; % inputs
r = 2; % outputs
v = 3; % disturbances

% initialize symbolic variables
syms x1 x2 x3 x4 u1 u2 v1 v2 v3 A ka C0 Dv kh kt
syms x10 x20 x30 x40 u10 u20 v10 v20 v30
syms x1_ x2_ x3_ x4_ u1_ u2_ v1_ v2_ v3_

% different variable vectors
vars = [x1;x2;x3;x4;u1;u2;v1;v2;v3];
vars0 = [x10;x20;x30;x40;u10;u20;v10;v20;v30];
vars_delta = [x1_;x2_;x3_;x4_;u1_;u2_;v1_;v2_;v3_];

% original nonlinear state derivative functions
Dx1 = (ka*(u1+u2) - C0*sqrt(x1-x2))/A;
Dx2 = (C0*sqrt(x1 - x2) - Dv*sqrt(x2)*v1)/A;
Dx3 = ((v2-x3)*ka*u1 + (v3 - x3)*ka*u2)/(A*x1);
Dx4 = (x3 - x4)*C0*sqrt(x1 - x2)/(A*x2);

% compute first-order Taylor series
Dx1_taylor = taylor(Dx1,vars,vars0,'Order',2);
Dx2_taylor = taylor(Dx2,vars,vars0,'Order',2);
Dx3_taylor = taylor(Dx3,vars,vars0,'Order',2);
Dx4_taylor = taylor(Dx4,vars,vars0,'Order',2);

% stationary point (2.98), first equation
Usub = sqrt(x10-x20)*C0/ka;

% substitute in the stationary point
Dx1_taylor0 = subs(Dx1_taylor,u10 + u20,Usub);
Dx2_taylor0 = subs(Dx2_taylor,u10 + u20,Usub);
Dx3_taylor0 = subs(Dx3_taylor,u10 + u20,Usub);
Dx4_taylor0 = subs(Dx4_taylor,u10 + u20,Usub);

% substitute in the new states
Dx1_taylor0 = subs(Dx1_taylor0,vars - vars0,vars_delta);
Dx2_taylor0 = subs(Dx2_taylor0,vars - vars0,vars_delta);
Dx3_taylor0 = subs(Dx3_taylor0,vars - vars0,vars_delta);
Dx4_taylor0 = subs(Dx4_taylor0,vars - vars0,vars_delta);

% simplify
Dx1_taylor0 = simplify(Dx1_taylor0,'steps',100);
Dx2_taylor0 = simplify(Dx2_taylor0,'steps',100);
Dx3_taylor0 = simplify(Dx3_taylor0,'steps',100);
Dx4_taylor0 = simplify(Dx4_taylor0,'steps',100);

% put in linear form
[A1,b1] = equationsToMatrix(Dx1_taylor0,vars_delta);
[A2,b2] = equationsToMatrix(Dx2_taylor0,vars_delta);
[A3,b3] = equationsToMatrix(Dx3_taylor0,vars_delta);
[A4,b4] = equationsToMatrix(Dx4_taylor0,vars_delta);

% combine and extra submatrices
A_ = [A1;A2;A3;A4];
Al = A_(1:n,1:n);
Bl = A_(1:n,n+1:n+m);
Bvl = A_(1:n,n+m+1:end);
Cl = [0 kh 0 0; 0 0 0 kt];

% system parameters
% A = 0.785;
% Dv = 2.66;
% C0 = 0.056;
% ka = 0.004;
% kh = 2;
% kt = 0.1;
% x10 = 2.03;
% x20 = 2.03;
% x30 = 45;
% x40 = 45;

p = [0.785; 2.66; 0.056; 0.004; 2; 0.1];
u0 = [5; 5];
v0 = [0.0122; 60; 30];

% calculate stationary point state values
sol = solve(Dx1==0,Dx2==0,Dx3==0,Dx4==0,x1,x2,x3,x4);
sol = subs(sol,[A Dv C0 ka kh kt u1 u2 v1 v2 v3],[p' u0' v0']);
x0 = double([sol.x1; sol.x2; sol.x3; sol.x4]);

% substitute values
XX = {A Dv C0 ka kh kt x10 x20 x30 x40 u10 u20 v10 v20 v30};
XX_values = [p' x0' u0' v0'];
A = double(subs(Al,XX,XX_values));
B = double(subs(Bl,XX,XX_values));
Bv = double(subs(Bvl,XX,XX_values));
C = double(subs(Cl,XX,XX_values));

% eigenvalue assessment
P = [-0.095+0.02j -0.095-0.02j -0.08+0.06j -0.08-0.06j -0.05+0.085j -0.05-0.085j];
% P = P/2;

% A2 = [-0.0499 0.0499 0 0; 0.0499 -0.0667 0 0; 0 0 -0.0251 0; 0 0 0.0335 -0.0335]
% B2 = [0.00510 0.00510; 0 0; 0.0377 -0.0377; 0 0]
% C2 = [0 2 0 0; 0 0 0 0.1];
% A-A2
% B-B2

% matrices for augmented closed-loop system
Ai = [A, zeros(n,r); -C zeros(r,r)];
Bi = [B; zeros(r,m)];

% pole placement
[K1,PREC] = place(Ai,Bi,P);

% different gains
K = K1(:,1:n);
Ki = -K1(:,n+1:end);

% mat2str(K)
% mat2str(Ki)