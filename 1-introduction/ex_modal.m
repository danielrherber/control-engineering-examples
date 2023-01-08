close all; clear; clc
hf = figure; hf.Color = 'w'; hold on

% LTI matrix
A = @(w0,zeta) [0 1; -w0^2 -2*zeta*w0];

% state derivative function
f = @(t,y,w0,zeta) A(w0,zeta)*y;

% initial condition
Y0 = [1 -1];

% time horizon
TSPAN = [0 15];

% test 1
w0 = 1; zeta = 2;
[T1,Y1] = ode45(@(t,y) f(t,y,w0,zeta),TSPAN,Y0);
plot(T1,Y1,'r','linewidth',2);

% test 2
w0 = 1; zeta = 0.2;
[T2,Y2] = ode45(@(t,y) f(t,y,w0,zeta),TSPAN,Y0);
plot(T2,Y2,'b','linewidth',2);

% test 3
w0 = 1; zeta = 0;
[T3,Y3] = ode45(@(t,y) f(t,y,w0,zeta),TSPAN,Y0);
plot(T3,Y3,'g','linewidth',2);

% test 4
w0 = 1; zeta = 1;
[T4,Y4] = ode45(@(t,y) f(t,y,w0,zeta),TSPAN,Y0);
plot(T4,Y4,'k','linewidth',2);

% test 5
w0 = 1; zeta = -0.01;
[T5,Y5] = ode45(@(t,y) f(t,y,w0,zeta),TSPAN,Y0);
plot(T5,Y5,'m','linewidth',2);

%% phase plot
hf = figure; hf.Color = 'w'; hold on
xlabel('x1'); ylabel('x2')
plot(Y1(:,1),Y1(:,2),'r','linewidth',2)
plot(Y2(:,1),Y2(:,2),'b','linewidth',2)
plot(Y3(:,1),Y3(:,2),'g','linewidth',2)
plot(Y4(:,1),Y4(:,2),'k','linewidth',2)
plot(Y5(:,1),Y5(:,2),'m','linewidth',2)