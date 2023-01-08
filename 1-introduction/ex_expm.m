close all; clear; clc
hf= figure; hf.Color = 'w'; hold on

% state-space matrices
A = [0 1; -20 -12];
B = [0;1];
C = [1 1];
D = 0;

% simulation conditions
t0 = 0;
TF = 50;
Y0 = [1;-1];

% input function
u = @(t) sin(t);

% derivative function
f = @(t,y) A*y + B*u(t);

% simulation options
OPTIONS = odeset('reltol',1e-10);

% simulation
[Tsim,Ysim] = ode45(@(t,y) f(t,y),[t0 TF],Y0,OPTIONS);

% plot simulation results
plot(Tsim,Ysim,'g','linewidth',2)

% symbolic method
% x = L*x0 + R
syms t t0 tau s
L = expm(A*t);
L2 = ilaplace(inv(s*eye(2)-A));
U = sin(t);
R = int(expm(A*(t-tau))*B*U,tau,t0,t);

% convert to Matlab functions
Lsym = matlabFunction(L);
Rsym = matlabFunction(R);

% initial condition
x(:,1) = Y0;

% determine the state at the time points recorded in the simulation
for k = 1:length(Tsim)-1
    x(:,k+1) = Lsym(Tsim(k+1)-Tsim(k))*x(:,k) + Rsym(Tsim(k+1),Tsim(k));
end

% plot symbolic method results
plot(Tsim,x,'.')