close all; clear; clc

% augmented system matrices
A = [-0.14 0.33 -0.33;
    0.1 -0.28 0;
    0 1.7 -0.77];
B = [0; 0; -0.025];
Bv = [1; 0; 0];
C = [2 0 0];

% simulation options
TSPAN = [0 100];
X0 = [0 0 0];
OPTIONS = odeset('RelTol',1e-13,'AbsTol',1e-10);

% initialize figures
hf = figure(1); hf.Color = 'w'; hold on
ylabel('y')
hf = figure(2); hf.Color = 'w'; hold on
ylabel('u')

%% parameters
Kp = 10;
Ti = 5;
r = 10;
v = -1;

%% P only
% simulate
[Tp,Xp] = ode45(@(t,x) derivP(t,x,r,Kp,A,B,Bv,C,v),TSPAN,X0,OPTIONS);

% output
figure(1)
plot(Tp,C*Xp')

% control
figure(2)
Yp = C*Xp';
Up = Kp*(r-Yp);
plot(Tp,Up)

%% PI
% initial integrator state
X0(end+1) = 0;

% simulate
[Ti,Xi] = ode45(@(t,x) derivPI(t,x,r,Kp,A,B,Bv,C,v,Ti),TSPAN,X0,OPTIONS);

% output
figure(1)
plot(Ti,C*Xi(:,1:end-1)')

% control
figure(2)
Yi = C*Xi(:,1:end-1)';
Ui = Kp*(r-Yi) + Kp/Ti*Xi(:,end);
plot(Ti,Ui)

% P controller derivative function
function Dx = derivP(t,x,r,Kp,A,B,Bv,C,v)

% output
y = C*x;

% error
e = r-y;

% control
u = Kp*e;

% state derivative function
Dx = A*x + B*u + Bv*v;

end

% PI controller derivative function
function Dx = derivPI(t,x,r,Kp,A,B,Bv,C,v,Ti)

% output
y = C*x(1:end-1);

% error
e = r-y;

% extract integral state
integral_error = x(end);

% control
u = Kp*e + Kp/Ti*integral_error;

% state derivative function
Dx = A*x(1:end-1) + B*u + Bv*v;
Dx(end+1) = r-y;

end