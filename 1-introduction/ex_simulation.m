close all; clear; clc

% assumed parameters
R = 1; L = 1; C = 1;

% linear matrices
A = [-R/L -1/L; 1/C 0];
B = [1/L;0];

% linear state derivative function
f = @(t,x,u) A*x + B*u;

% number of time points in the simulation
N = 300;

% time horizon
t0 = 0; tfinal = 100;

% time vector
T = linspace(t0,tfinal,N);

% user-defined input
U = @(t) sin(t);

% initial state condition
Y0 = [1;0];

% assign initial state condition to first column
X(:,1) = Y0;

% go through each simulation time step
for k = 1:N-1

	% forward Euler integration time k to time k+1
    X(:,k+1) = X(:,k) + (T(k+1)-T(k))*f(T(k),X(:,k),U(T(k)));

end

% (not from the lecture)
% trapezoidal rule
X2(:,1) = Y0;

OPTIONS = optimoptions('fmincon','Display','Iter');

for k = 1:N-1

	% trapezoidal integration time k to time k+1
    X2(:,k+1) = fmincon(@(Xz) sum((Xz - X2(:,k) - (T(k+1)-T(k))/2*(f(T(k),X2(:,k),U(T(k))) + f(T(k+1),Xz,U(T(k+1))))).^2),...
        X2(:,k),[],[],[],[],[],[],[],OPTIONS);

end

options = odeset('RelTol',1e-12);

% nonlinear simulation
[TOUT,YOUT] = ode45(@(t,x) f(t,x,U(t)),[t0 tfinal],Y0,options);

% plot
hf = figure; hf.Color = 'w'; hold on
% plot(T,X)
plot(T,X2)
plot(TOUT,YOUT);
xlabel('t')
% legend('EF X1','EF X2','ODE45 X1','ODE45 X2')