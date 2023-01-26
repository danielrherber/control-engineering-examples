% ex_simulation.m
% simulation of Example 2.1 RLC Circuit in LSC
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
% N = 100; % <- also try this
% N = 3000; % <- also try this

% time horizon
t0 = 0; tfinal = 100;

% time vector
T = linspace(t0,tfinal,N);

% user-defined input function
U = @(t) sin(t);

% initial state condition
X0 = [1;0];

% assign initial state condition to first column
X_ef = zeros(2,N);
X_ef(:,1) = X0;

% go through each simulation time step
for k = 1:N-1

    % current time step
    Hk = T(k+1)-T(k);

    % current state derivatives
    Fk = f(T(k),X_ef(:,k),U(T(k)));

	% forward Euler integration time k to time k+1
    X_ef(:,k+1) = X_ef(:,k) + Hk*Fk;

end

% simulation with matlab ode45
options = odeset('RelTol',1e-12); % simulation relative tolerance (accuracy)
[T_ode45,X_ode45] = ode45(@(t,x) f(t,x,U(t)),[t0 tfinal],X0,options);

% trapezoidal rule (not covered in the session, see function below)
X_tr = trapezoidal_sim(f,T,U,X0);

% plot the simulation results (see function below)
plot_example(T,X_ef,X_tr,T_ode45,X_ode45)

%--------------------------------------------------------------------------
% plotting code (not the main content)
%--------------------------------------------------------------------------
function plot_example(T,X_ef,X_tr,T_ode45,X_ode45)

% colors
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
% nicegreen = [109, 195, 80]/255;
% nicegray = [242, 242, 242]/255;
xmediumgray = [170, 170, 170]/255;

% initialize figure
hf = figure; hf.Color = 'w'; hold on
t = tiledlayout('flow','GridSize',[2 1],'TileSpacing','tight');
LineWidth = 1;

%--- state 1 results
nexttile(t,[1 1]); hold on

plot(T_ode45,X_ode45(:,1),'.-','Color',xmediumgray,'LineWidth',LineWidth,'DisplayName','ode45');
plot(T,X_ef(1,:),'.-','Color',niceblue,'LineWidth',LineWidth,'DisplayName','Euler Forward')
plot(T,X_tr(1,:),'.-','Color',nicered,'LineWidth',LineWidth,'DisplayName','Trapezoidal Rule')

xlabel('Time [sec]')
ylabel('State 1')

legend();

%--- state 2 results
nexttile(t,[1 1]); hold on

plot(T_ode45,X_ode45(:,2),'.-','Color',xmediumgray,'LineWidth',LineWidth,'DisplayName','ode45');
plot(T,X_ef(2,:),'.-','Color',niceblue,'LineWidth',LineWidth,'DisplayName','Euler Forward')
plot(T,X_tr(2,:),'.-','Color',nicered,'LineWidth',LineWidth,'DisplayName','Trapezoidal Rule')

xlabel('Time [sec]')
ylabel('State 2')

legend();

end

%--------------------------------------------------------------------------
% trapezoidal rule-based simulation (not covered in the session)
% https://en.wikipedia.org/wiki/Trapezoidal_rule
%--------------------------------------------------------------------------
function X = trapezoidal_sim(f,T,U,X0)

% assign initial state value
X(:,1) = X0;

% turn off display for fsolve
options = optimoptions('fsolve','Display','none');

% go through each simulation time step
for k = 1:length(T)-1

    % current time step
    H = T(k+1)-T(k);

    % current state derivatives
    Fk = f(T(k),X(:,k),U(T(k)));

	% trapezoidal integration time k to time k+1
    X(:,k+1) = fsolve(@(Xz) Xz - X(:,k) - H/2*(Fk + f(T(k+1),Xz,U(T(k+1)))),X(:,k),...
        options);

end

end