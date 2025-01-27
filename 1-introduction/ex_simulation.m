% ex_simulation.m
% simulation of an electrical RLC circuit with a CT LTI state-space model
% [reference] Example 2.1 RLC Circuit in LSC
% [course] Session 1 - Modeling for Control Design
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
% plotting code
% (not the main content)
function plot_example(T,X_ef,X_tr,T_ode45,X_ode45)

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
nicegray = [170, 170, 170]/255;
LineWidth = 1;
MarkerSize = 12;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth,'MarkerSize',MarkerSize};

% initialize figure
hf = figure; hf.Color = 'w'; hold on
t = tiledlayout('flow','GridSize',[2 1],'TileSpacing','tight');

%--- state 1 results
nexttile(t,[1 1]); hold on

plot(T_ode45,X_ode45(:,1),'.-',plotOpts{:},'Color',nicegray,'DisplayName','ode45');
plot(T,X_ef(1,:),'.-',plotOpts{:},'Color',niceblue,'DisplayName','Euler Forward');
plot(T,X_tr(1,:),'.-',plotOpts{:},'Color',nicered,'DisplayName','Trapezoidal Rule');

xlabel('Time [sec]')
ylabel('State 1')

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

legend();

%--- state 2 results
nexttile(t,[1 1]); hold on

plot(T_ode45,X_ode45(:,2),'.-',plotOpts{:},'Color',nicegray,'DisplayName','ode45');
plot(T,X_ef(2,:),'.-',plotOpts{:},'Color',niceblue,'DisplayName','Euler Forward');
plot(T,X_tr(2,:),'.-',plotOpts{:},'Color',nicered,'DisplayName','Trapezoidal Rule');

xlabel('Time [sec]')
ylabel('State 2')

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

legend();

end

%--------------------------------------------------------------------------
% trapezoidal rule-based simulation
% (not covered in the session and not an efficient implementation)
% https://en.wikipedia.org/wiki/Trapezoidal_rule
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