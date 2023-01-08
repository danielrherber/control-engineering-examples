close all; clear; clc

% NEEDS polish

b = 10;
k1 = 10;
k2 = 100;
M = 200;

f = @(x) (-b*x(2)*abs(x(2)) - k1*x(1) - k2*x(1)^3)/M;
g = @(x) 1/M;

% F = @(x,u) [x(2); f(x) + g(x)*u];
F = @(x,w) [x(2); f(x) + g(x)*(w - f(x))/g(x)];



% simulation options
TSPAN = [0 20];
X0 = [0.5, 0.5];
OPTIONS = odeset('RelTol',1e-15);

% simulate
[T,X] = ode45(@(t,x) F(x,control(x)),TSPAN,X0,OPTIONS);

figure; hold on
plot(T,X)

figure; hold on
plot(T,control(X'))

function u = control(x)

k1 = 1;
k2 = 1;
r1 = 1;
r2 = 0;

% u = -k1*x(1,:) - k2*x(2,:);
u = k1*(r1-x(1,:)) + k2*(r2 - x(2,:));

end