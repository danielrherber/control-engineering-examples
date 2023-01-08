close all; clear; clc

% assumed parameters
g = 1; b = 1; m = 1;

% nominal thrust
T0 = 3;

% velocity for stationary condition
V0 = sqrt((T0-m*g)/b);

% user-defined thrust input
T = @(t) T0 + (t>=30)*2*T0 - (t>=60)*2.25*T0;
% T = @(t) T0 + sin(t);

% linear matrices
A = -2*b/m*V0; B = [1/m];

% nonlinear state derivative function
fnonlinear = @(t,v) -b/m*sign(v)*v^2 + T(t)/m - g;

% linear state derivative function
flinear = @(t,vdelta) A*vdelta + B*(T(t)-T0);

% time horizon bounds
TSPAN = [0 100];

% initial state
Y0 = 0.5;

% ode options (more accurate simulation)
OPTIONS = odeset('Reltol',1e-12);

% shift linear simulation initial value
Y0linear = Y0 - V0;

% nonlinear simulation
[Tnon,Ynon] = ode45(@(t,v) fnonlinear(t,v),TSPAN,Y0,OPTIONS);

% linear simulation
[Tlin,Ylin] = ode45(@(t,v) flinear(t,v),TSPAN,Y0linear,OPTIONS);

% plot
figure; hold on
plot(TSPAN,[V0 V0],'--')
plot(Tnon,Ynon)
plot(Tlin,Ylin+V0) % needs offset
legend('nonlinear','linear')