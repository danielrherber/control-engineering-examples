close all; clear; clc
hf = figure; hf.Color = 'w'; hold on

% LTI matrices
A = [0,1;-20,-12];
B = [0;1];
C = [1 1];
D = [];

% sampling time
T = 0.5;

% number of samples
N = 50;

% discrete-time vector
TV = 0:T:(N-1)*T;

% zero-order hold on input
u = @(t) interp1(TV,sin(TV),t,'previous');

% initial conditions
Y0 = [1;-1];

% state derivative function
f = @(t,y) A*y + B*u(t);

% simulation options
options = odeset('RelTol',1e-12,'AbsTol',1e-15);

% simulation
[Tsim,Ysim] = ode45(@(t,y) f(t,y),[TV(1),TV(end)],Y0,options);

% plot simulation
plot(Tsim,Ysim,'k','linewidth',2)

% discrete-time matrices
[F,G] = c2d(A,B,T);

% compare
disp(F)
disp(expm(A*T))

% initial states
X = Y0;

% simulate the difference equation
for k = 0:N-2
    X(:,k+2) = F*X(:,k+1) + G*u(k*T);
end

% plot difference equation results
plot(TV,X,'.-r')