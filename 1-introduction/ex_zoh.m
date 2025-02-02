% ex_zoh.m
% simulation of continuous-time and discrete-time models (zero-order hold)
% of an LTI system
% [reference] Example 3.3 Transfer Function of a Discrete Time System in LSC
% [reference] https://www.youtube.com/watch?v=PsBfTzsGsow
% [course] Session 2 - Analysis of State-Space Models (1)
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
% plotting code
% (not the main content)
function plot_example(T_sim,X_sim,T_dt,X_dt)

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
nicegreen = [109, 195, 80]/255;
nicegray = [170, 170, 170]/255;
LineWidth = 1;
MarkerSize = 12;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth,'MarkerSize',MarkerSize};

% initialize figure
hf = figure; hf.Color = 'w'; hold on

% plot simulation
plot(T_sim,X_sim(:,1),plotOpts{:},'Color',nicegray,'DisplayName',"CTM State 1")
plot(T_sim,X_sim(:,2),plotOpts{:},'Color',nicegreen,'DisplayName',"CTM State 2")

% plot difference equation results
plot(T_dt,X_dt(1,:),'.-',plotOpts{:},'Color',nicered,'DisplayName','DTM State 1')
plot(T_dt,X_dt(2,:),'.-',plotOpts{:},'Color',niceblue,'DisplayName','DTM State 2')

xlabel('Time [sec]')
ylabel('States')

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

hl = legend();

end