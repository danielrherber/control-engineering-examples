% ex_zoh.m
% simulation of an LTI system from Example 3.3 Transfer Function of a
% Discrete Time System in LSC using various methods, including zero-order
% hold
close all; clear; clc

% LTI matrices
A = [0,1;-20,-12];
B = [0;1];
C = [1 1];
D = [];

% initial conditions
X0 = [1;-1];

% sampling time
T = 0.5;
% T = 0.1; % <- also try this
% T = 1; % <- also try this
% T = 5; % <- also try this

% number of samples
N = 50;

% discrete-time vector
T_dt = 0:T:(N-1)*T;

% input
u = @(t) sin(t);

% zero-order hold on input
u_zoh = @(t) interp1(T_dt,u(T_dt),t,'previous');

% state derivative function
f = @(t,x) A*x + B*u_zoh(t);

% simulation options
options = odeset('RelTol',1e-10,'AbsTol',1e-10);

% simulation
[T_sim,X_sim] = ode45(@(t,x) f(t,x),[T_dt(1) T_dt(end)],X0,options);

% get discrete-time matrices from continuous-time system
[F,G] = c2d(A,B,T);

% direct formulas
F_alt = expm(A*T);
G_alt = integral(@(t) expm(A*(T-t))*B,0,T,'ArrayValued',true);

% assign initial state condition to first column
X_dt = zeros(2,length(T_dt));
X_dt(:,1) = X0;

% simulate the difference equation
for k = 0:N-2
    X_dt(:,k+2) = F*X_dt(:,k+1) + G*u(k*T);
end

% plot the simulation results (see function below)
plot_example(T_sim,X_sim,T_dt,X_dt)

%--------------------------------------------------------------------------
% plotting code (not the main content)
%--------------------------------------------------------------------------
function plot_example(T_sim,X_sim,T_dt,X_dt)

% colors
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
nicegreen = [109, 195, 80]/255;
% nicegray = [242, 242, 242]/255;
xmediumgray = [170, 170, 170]/255;

% initialize figure
hf = figure; hf.Color = 'w'; hold on
LineWidth = 1;

% plot simulation
plot(T_sim,X_sim(:,1),'Color',xmediumgray,'linewidth',LineWidth,'DisplayName',"CT State 1")
plot(T_sim,X_sim(:,2),'Color',nicegreen,'linewidth',LineWidth,'DisplayName',"CT State 2")

% plot difference equation results
plot(T_dt,X_dt(1,:),'.-','Color',nicered,'linewidth',LineWidth,'MarkerSize',16,...
    'DisplayName','DT State 1')
plot(T_dt,X_dt(2,:),'.-','Color',niceblue,'linewidth',LineWidth,'MarkerSize',16,...
    'DisplayName','DT State 2')

xlabel('Time [sec]')
ylabel('States')

legend();

end