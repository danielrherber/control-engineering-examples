% ex_pi.m
% simulation of simple P and PI controllers on a third-order LTI system
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
ylabel('Output'); xlabel('Time [sec]'); legend();
hf = figure(2); hf.Color = 'w'; hold on
ylabel('Control'); xlabel('Time [sec]'); legend();

%--------------------------------------------------------------------------
% parameters (try changing these)
%--------------------------------------------------------------------------
Kp = 100; % proportional term gain
Ti = 5; % integral time constant
r = 10; % reference (constant)
v = -1; % disturbance (constant)

%--------------------------------------------------------------------------
% P controller
%--------------------------------------------------------------------------
% simulate
[Tp,Xp] = ode45(@(t,x) derivP(t,x,r,Kp,A,B,Bv,C,v),TSPAN,X0,OPTIONS);

% plot output
figure(1)
plot(Tp,C*Xp',"DisplayName","y for P controller")

% plot control
figure(2)
Yp = C*Xp';
Up = Kp*(r-Yp);
plot(Tp,Up,"DisplayName","u for P controller")

%--------------------------------------------------------------------------
% PI controller
%--------------------------------------------------------------------------
% initial integrator state
X0(end+1) = 0;

% simulate
[Ti,Xi] = ode45(@(t,x) derivPI(t,x,r,Kp,A,B,Bv,C,v,Ti),TSPAN,X0,OPTIONS);

% plot output
figure(1)
plot(Ti,C*Xi(:,1:end-1)',"DisplayName","y for PI controller")

% plot control
figure(2)
Yi = C*Xi(:,1:end-1)';
Ui = Kp*(r-Yi) + Kp/Ti*Xi(:,end);
plot(Ti,Ui,"DisplayName","u for PI controller")

%--------------------------------------------------------------------------
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

%--------------------------------------------------------------------------
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