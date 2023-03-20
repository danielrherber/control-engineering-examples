% ex_sliding_mode.m
% simulation of a sliding mode controller on a 2d nonlinear affine control
% system
close all; clear; clc

% nonlinear function
a = 1;
b = 2;
f = @(t,x1,x2) a*cos(x1) + b*cos(x2 + 1) + exp(-t);

% bound on nonlinear f
k = a + b + 1;
% k = 1; % <- try this incorrect bound

% nonlinear state derivative function
F = @(t,x) [x(2); f(t,x(1),x(2)) + u(t,x(1),x(2),k)];

% simulation options
TSPAN = [0 10];
X0 = [0.5 0.5];
OPTIONS = odeset('RelTol',1e-6);

% simulate
[T,X] = ode45(@(t,x) F(t,x),TSPAN,X0,OPTIONS);

% plot states
hf = figure; hf.Color = 'w'; hold on
plot(T,X)
xlabel('Time [sec]'); ylabel('States');
legend('x_1','x_2')

% plot control
hf = figure; hf.Color = 'w'; hold on
plot(T,u(T,X(:,1),X(:,2),k))
xlabel('Time [sec]'); ylabel('Control');

% plot phase-space and switching surface
hf = figure; hf.Color = 'w'; hold on
plot(X(:,1),X(:,2))
plot(X(:,1),-X(:,1))
xlabel('x_1')
ylabel('x_2')
legend('Simulation trajectory','Switching surface')

%--------------------------------------------------------------------------
% control law
function U = u(t,x1,x2,k)

% distance from the switching surface
sigma = x1 + x2;

% sign of the distance from the switching surface
sign_sigma = sign(sigma);

% continuous approximation for the sign function
approx_sign_sigma = 2./(1 + exp(-400*sigma)) - 1;

% select the controller to be used (see below)
controller_option = 1;
switch controller_option
    case 0 % no control
        U = zeros(size(t));
    case 1 % discontinuous controller (will take some time to simulate)
        U = -( abs(x2) + k + 1).*sign_sigma;
    case 2 % continuous controller
        U = -(abs(x2) + k + 1).*approx_sign_sigma;
    case 3 % simple gain controller
        U = -k*approx_sign_sigma;
end

end

%--------------------------------------------------------------------------
% approximate sign function plot used above
% figure; hold on
% Y = linspace(-1,1,1000);
% plot(Y,2./(1 + exp(-400*Y)) - 1)
% return