% ex_modal.m
% simulation of a 2nd-order oscillator in modal canonical form
% [reference] pp. 94-99 in LSC
% [reference] en.wikipedia.org/wiki/Harmonic_oscillator#Damped_harmonic_oscillator
% [reference] https://www.youtube.com/watch?v=caP8vKHtIY0
% [course] Session 2 - Analysis of State-Space Models (1)
close all; clear; clc

% LTI matrix
A = @(w0,zeta) [0 1; -w0^2 -2*zeta*w0];

% state derivative function
f = @(t,y,w0,zeta) A(w0,zeta)*y;

% initial condition
X0 = [1 0];
% X0 = [1 -1]; % <- also try this

% time horizon
TSPAN = [0 25];

% simulation options
OPTIONS = odeset('RelTol',1e-8,'AbsTol',1e-8);

%--------------------------------------------------------------------------
% different zeta (damping ratio) values
%--------------------------------------------------------------------------
w0 = 1;
ZETA = [2, 1, 0.4, 0.2, 0];

% initialize plot
hf = figure; hf.Color = 'w'; hold on

% go through each value of zeta
for idx = 1:length(ZETA)

    % extract current value
    zeta = ZETA(idx);

    % run simulation
    [T,X] = ode45(@(t,x) f(t,x,w0,ZETA(idx)),TSPAN,X0,OPTIONS);

    % add result to figure
    plot_example_add(T,X,zeta,"\zeta")

end

%--------------------------------------------------------------------------
% different w0 (natural frequency) values
%--------------------------------------------------------------------------
W0 = [0.2, 0.5, 0.8, 1, 2];
zeta = 0.4;

% initialize plot
hf = figure; hf.Color = 'w'; hold on

% go through each value of w0
for idx = 1:length(W0)

    % extract current value
    w0 = W0(idx);

    % run simulation
    [T,X] = ode45(@(t,x) f(t,x,w0,zeta),TSPAN,X0,OPTIONS);

    % add result to figure
    plot_example_add(T,X,w0,"\omega_0")

end

%--------------------------------------------------------------------------
% plotting code
% (not the main content)
function plot_example_add(T,X,value,type)

% colors and other parameters
LineWidth = 3;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth};

% plot state space
subplot(1,3,1); hold on;
ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;
axis equal; xlabel('State 1'); ylabel('State 2');
plot(X(:,1),X(:,2),plotOpts{:},'DisplayName',strcat(type," = ",string(value)))
hl = legend(); hl.Location = 'best';

% plot state 1 trajectory
subplot(1,3,2); hold on
ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;
xlabel('Time [sec]'); ylabel('State 1');
plot(T,X(:,1),plotOpts{:});

% plot state 2 trajectory
subplot(1,3,3); hold on
ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;
xlabel('Time [sec]'); ylabel('State 2');
plot(T,X(:,2),plotOpts{:});

end