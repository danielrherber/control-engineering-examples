close all; clear; clc

% nonlinear function
a = 1;
b = 2;
f = @(t,x1,x2) a*cos(x1) + b*cos(x2 + 1) + exp(-t);

% bound on f
k = a + b + 1;
% k = 1;

% nonlinear state derivative function
F = @(t,x) [x(2); f(t,x(1),x(2)) + u(t,x(1),x(2),k)];

% simulation options
TSPAN = [0 20];
X0 = [0.5 0.5];
OPTIONS = odeset('RelTol',1e-8);

% simulate
[T,X] = ode45(@(t,x) F(t,x),TSPAN,X0,OPTIONS);

% plot states
figure; hold on
plot(T,X)

% plot phase-space and switching surface
figure; hold on
plot(X(:,1),X(:,2))
plot(X(:,1),-X(:,1))
xlabel('x1')
ylabel('x2')

% control
function U = u(t,x1,x2,k)

sigma = x1 + x2;
sign_sigma = sign(sigma);
approx_sign_sigma = 2./(1 + exp(-400*sigma)) - 1;

% no control
% U = 0;

% discontinuous controller
% U = -( abs(x2) + k + 1)*sign_sigma;

% continuous controller
U = -(abs(x2) + k + 1)*approx_sign_sigma;

% simple gain controller
% U = -k*approx_sign_sigma;

end

% approximate sign function plot
% figure; hold on
% Y = linspace(-1,1,1000);
% plot(Y,2./(1 + exp(-400*Y)) - 1)
% return