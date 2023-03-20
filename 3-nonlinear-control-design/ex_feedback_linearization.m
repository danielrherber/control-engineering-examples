% ex_feedback_linearization.m
% simulation of a nonlinear control law based on feedback linearization for
% a nonlinear mass-damper-spring system from Chapter 9, Section 14 in
% Mechanical Engineer's Handbook
close all; clear; clc

% control gains
k1c = 1;
k2c = 1;

% constant state references
r1 = 1; % for x1
r2 = 0; % for x2

% model parameters
b = 10;
k1 = 10;
k2 = 100;
M = 200;

% companion form nonlinear functions
f = @(x) (-b*x(2,:).*abs(x(2,:)) - k1*x(1,:) - k2*x(1,:).^3)/M;
g = @(x) 1/M;

% state derivative function with feedback linearization
deriv = @(x,w) [x(2); f(x) + g(x)*(w - f(x))/g(x)];

% simulation options
TSPAN = [0 20];
X0 = [0.5, 0.5];
OPTIONS = odeset('RelTol',1e-15);

% simulate
[T,X] = ode45(@(t,x) deriv(x,control(x,k1c,k2c,r1,r2)),TSPAN,X0,OPTIONS);

% plot states
hf = figure; hf.Color = 'w'; hold on
plot([T(1) T(end)],[r1 r1])
plot([T(1) T(end)],[r2 r2])
plot(T,X)
xlabel('Time [sec]'); ylabel('States');
legend('r_1','r_2','x_1','x_2')

% plot control (u or F)
hf = figure; hf.Color = 'w'; hold on
plot(T,(control(X',k1c,k2c,r1,r2) - f(X'))./g(X'))
xlabel('Time [sec]'); ylabel('Control (u or F)')

%--------------------------------------------------------------------------
% P-type control function
function w = control(x,k1,k2,r1,r2)

% errors
e1 = r1 - x(1,:);
e2 = r2 - x(2,:);

% control law
w = k1*e1 + k2*e2;

end