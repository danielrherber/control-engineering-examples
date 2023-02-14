close all; clear; clc

% system matrices
A = [-0.14 0.33 -0.33; 0.1 -0.28 0; 0 1.7 -0.77];
B = [0; 0; -0.025];
C = [2 0 0];

% check controllability
Co = ctrb(A,B);
rank(Co)==length(A)

% desired eigenvalues
epsilon = [0 1e-4 -1e-4];
tc = 1.5;   E1p5 = [-1/tc -1/tc -1/tc]+epsilon;
tc = 1;     E1 = [-1/tc -1/tc -1/tc]+epsilon;
tc = 3;     E3 = [-1/tc -1/tc -1/tc]+epsilon;
tc = 0.125; E0p125 = [-1/tc -1/tc -1/tc]+epsilon;

% closed-loop pole assignment using state feedback
[K1p5,PREC1] = place(A,B,E1p5);
[K1,PREC2] = place(A,B,E1);
[K3,PREC3] = place(A,B,E3);
[K0p125,PREC4] = place(A,B,E0p125);

% control law
u = @(x,K,r) -K*x + r;

% feedback state derivative function
f = @(t,x,A,B,K,r) A*x + B*u(x,K,r);

% simulation options
TSPAN = [0 30];
X0 = [1;1;1];
OPTIONS = odeset('RelTol',1e-13);

% reference input
r = 0;

% run the simulations
[T,X] = ode45(@(t,x) f(t,x,A,B,0*K1p5,r),TSPAN,X0,OPTIONS);
[T1p5,X1p5] = ode45(@(t,x) f(t,x,A,B,K1p5,r),TSPAN,X0,OPTIONS);
[T1,X1] = ode45(@(t,x) f(t,x,A,B,K1,r),TSPAN,X0,OPTIONS);
[T3,X3] = ode45(@(t,x) f(t,x,A,B,K3,r),TSPAN,X0,OPTIONS);
[T0p125,X0p125] = ode45(@(t,x) f(t,x,A,B,K0p125,r),TSPAN,X0,OPTIONS);

% states
hf = figure; hf.Color = 'w'; hold on
plot(T,X,'k','DisplayName','original')
plot(T0p125,X0p125,'m','DisplayName',' 0.125 s')
plot(T1,X1,'g','DisplayName','1 s')
plot(T1p5,X1p5,'r','DisplayName','1.5 s')
plot(T3,X3,'b','DisplayName','3 s')
xlim([0 30]); ylim([-20 10])
xlabel('time [sec]'); ylabel('x(t)')
legend();

% control
hf = figure; hf.Color = 'w'; hold on
plot(T,0*K1p5*X' + r,'k','DisplayName','original')
plot(T0p125,K0p125*X0p125' + r,'m','DisplayName',' 0.125 s')
plot(T1,K1*X1' + r,'g','DisplayName','1 s')
plot(T1p5,K1p5*X1p5' + r,'r','DisplayName','1.5 s')
plot(T3,K3*X3' + r,'b','DisplayName','3 s')
xlim([0 30]); ylim([-200 200])
xlabel('time [sec]'); ylabel('u(t)')
legend();

% outputs
hf = figure; hf.Color = 'w'; hold on
plot(T,C*X','k','DisplayName','original')
plot(T0p125,C*X0p125','m','DisplayName',' 0.125 s')
plot(T1,C*X1','g','DisplayName','1 s')
plot(T1p5,C*X1p5','r','DisplayName','1.5 s')
plot(T3,C*X3','b','DisplayName','3 s')
xlim([0 30]); ylim([-6 3])
xlabel('time [sec]'); ylabel('y(t)')
legend();