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
E1 = [-0.67 -0.67 -0.67]+epsilon;
E2 = [-1 -1 -1]+epsilon;
E3 = [-0.33 -0.33 -0.33]+epsilon;
E4 = [-4 -4 -4]+epsilon;

% closed-loop pole assignment using state feedback
[K1,PREC1] = place(A,B,E1);
[K2,PREC2] = place(A,B,E2);
[K3,PREC3] = place(A,B,E3);
[K4,PREC4] = place(A,B,E4);

% control law
u = @(x,K,r) -K*x + r;

% feedback state derivative function
f = @(t,x,A,B,K,r) A*x + B*u(x,K,r);

% simulation options
TSPAN = [0 30];
x0 = [1;1;1];
OPTIONS = odeset('RelTol',1e-13);

% reference input
r = 0;

% run the simulations
[T0,X0] = ode45(@(t,x) f(t,x,A,B,0*K1,r),TSPAN,x0,OPTIONS);
[T1,X1] = ode45(@(t,x) f(t,x,A,B,K1,r),TSPAN,x0,OPTIONS);
[T2,X2] = ode45(@(t,x) f(t,x,A,B,K2,r),TSPAN,x0,OPTIONS);
[T3,X3] = ode45(@(t,x) f(t,x,A,B,K3,r),TSPAN,x0,OPTIONS);
[T4,X4] = ode45(@(t,x) f(t,x,A,B,K4,r),TSPAN,x0,OPTIONS);

% states
hf = figure; hf.Color = 'w'; hold on
plot(T0,X0,'k')
plot(T1,X1,'r')
plot(T2,X2,'g')
plot(T4,X4,'m')
xlim([0 30]); ylim([-20 10])
xlabel('time [sec]'); ylabel('x(t)')

% outputs
hf = figure; hf.Color = 'w'; hold on
plot(T0,C*X0','k')
plot(T1,C*X1','r')
plot(T2,C*X2','g')
plot(T3,C*X3','b')
plot(T4,C*X4','m')
xlim([0 30]); ylim([-6 3])
xlabel('time [sec]'); ylabel('y(t)')
legend('original','-0.67','-1','-0.33','-4')

% control
hf = figure; hf.Color = 'w'; hold on
plot(T0,0*K1*X0' + r,'k')
plot(T1,K1*X1' + r,'r')
plot(T2,K2*X2' + r,'g')
plot(T3,K3*X3' + r,'b')
plot(T4,K4*X4' + r,'m')
xlim([0 30]); ylim([-200 200])
xlabel('time [sec]'); ylabel('u(t)')
legend('original','-0.67','-1','-0.33','-4')