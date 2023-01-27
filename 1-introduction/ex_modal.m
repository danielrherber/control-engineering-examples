% ex_modal.m
% simulation of a 2nd-order damped oscillator in modal canonical form
close all; clear; clc

% LTI matrix
A = @(w0,zeta) [0 1; -w0^2 -2*zeta*w0];

% state derivative function
f = @(t,y,w0,zeta) A(w0,zeta)*y;

% initial condition
X0 = [1 -1];
% X0 = [1 0]; % <- also try this

% time horizon
TSPAN = [0 15];

% simulation options
OPTIONS = odeset('RelTol',1e-8,'AbsTol',1e-8);

%--------------------------------------------------------------------------
% different zeta (damping) values
%--------------------------------------------------------------------------
w0 = 1;
ZETA = [2 1 0.4 0];

% initialize plot
[colors] = plot_example(length(ZETA));

% go through each value of zeta
for idx = 1:length(ZETA)

    % extract
    zeta = ZETA(idx);

    % run simulation
    [T,X] = ode45(@(t,x) f(t,x,w0,ZETA(idx)),TSPAN,X0,OPTIONS);

    % add result to figure
    plot_example_add(T,X,colors(idx,:),zeta,"\zeta")

end

%--------------------------------------------------------------------------
% different w0 (frequency) values
%--------------------------------------------------------------------------
W0 = [1/4, 1/2, 1, 2, 4];
zeta = 0.4;

% initialize plot
colors = plot_example(length(W0));

% go through each value of w0
for idx = 1:length(W0)

    % extract
    w0 = W0(idx);

    % run simulation
    [T,X] = ode45(@(t,x) f(t,x,w0,zeta),TSPAN,X0,OPTIONS);

    % add result to figure
    plot_example_add(T,X,colors(idx,:),w0,"\omega_0")

end

%--------------------------------------------------------------------------
% plotting code (not the main content)
%--------------------------------------------------------------------------
% initialize
function [colors] = plot_example(N)

hf = figure; hf.Color = 'w'; hold on
axis equal
colors = lines(N);

end
% add a single simulation
function plot_example_add(T,X,color,value,type)

% plot state space
subplot(1,3,1); hold on; legend();
axis equal; xlabel('State 1'); ylabel('State 2');
plot(X(:,1),X(:,2),'Color',color,'linewidth',2,...
    'DisplayName',strcat(type," = ",string(value)))

% plot state 1 trajectory
subplot(1,3,2); hold on
xlabel('Time [sec]'); ylabel('State 1');
plot(T,X(:,1),'Color',color,'linewidth',2);

% plot state 2 trajectory
subplot(1,3,3); hold on
xlabel('Time [sec]'); ylabel('State 2');
plot(T,X(:,2),'Color',color,'linewidth',2);

end