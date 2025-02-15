% ex_full_state_feedback.m
% full-state feedback controller design using the place command
% [reference] Example 4.5 Continuous Control via the Controller Canonical
% Form in LSC
% [course] Session 4 - Block Diagrams and Linear Control (1)
close all; clear; clc

% system matrices
A = [-0.14 0.33 -0.33; 0.1 -0.28 0; 0 1.7 -0.77];
B = [0; 0; -0.025];
C = [2 0 0];

% check controllability
Co = ctrb(A,B);
disp(rank(Co)==length(A))

% desired time constants
TC = [0.125,1,1.5,3];

% desired minor eigenvalue offsets for numerical stability
epsilon = [0 1e-4 -1e-4];

% control law
u = @(x,K) -K*x;

% state derivative function
f = @(t,x,A,B,K) A*x + B*u(x,K);

% simulation options
TSPAN = [0 30];
X0 = [1;1;1];
OPTIONS = odeset('RelTol',1e-13);

% go through each time constant
for k = 1:length(TC)

    % extract
    tc = TC(k);

    % desired eigenvalues
    E = [-1/tc -1/tc -1/tc]+epsilon;

    % closed-loop pole assignment using state feedback
    [K{k},PREC] = place(A,B,E);

    % run the simulation
    [T{k},X{k}] = ode45(@(t,x) f(t,x,A,B,K{k}),TSPAN,X0,OPTIONS);

end

% simulate zero gain (uncontrolled) case
K{k+1} = 0*K{k};
[T{k+1},X{k+1}] = ode45(@(t,x) f(t,x,A,B,K{k+1}),TSPAN,X0,OPTIONS);

% plot results
plot_example_add(T,X,K,TC,C)

%--------------------------------------------------------------------------
% plotting code
% (not the main content)
function plot_example_add(T,X,K,TC,C)

% colors and other parameters
LineWidth = 1;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth};

% zero gain (uncontrolled) case
TC(end+1) = nan;

% initialize figure
hf = figure; hf.Color = 'w'; hold on

% plot state 1
subplot(2,2,1); hold on;
ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;
xlabel('Time [s]'); ylabel('State 1');
for k = 1:length(T)
    plot(T{k},X{k}(:,1),plotOpts{:})
end
ylim([-2 1])

% plot state 3
subplot(2,2,2); hold on;
ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;
xlabel('Time [s]'); ylabel('State 2');
for k = 1:length(T)
    plot(T{k},X{k}(:,2),plotOpts{:})
end
ylim([-0.5 1])

% plot state 3
subplot(2,2,3); hold on;
ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;
xlabel('Time [s]'); ylabel('State 3');
for k = 1:length(T)
    plot(T{k},X{k}(:,3),plotOpts{:})
end
ylim([-6 6])

% plot output
subplot(2,2,4); hold on;
ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;
xlabel('Time [s]'); ylabel('Output');
for k = 1:length(T)
    plot(T{k},sum(C.*X{k},2),plotOpts{:},'DisplayName',strcat("$t_c = ",string(TC(k)),"$"))
end
ylim([-2 2])
hl = legend(); hl.Location = 'best'; hl.Interpreter = 'latex';

% initialize figure
hf = figure; hf.Color = 'w'; hold on

% plot control
ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;
xlabel('Time [s]'); ylabel('Control');
for k = 1:length(T)
    plot(T{k},sum(-K{k}.*X{k},2),plotOpts{:},'DisplayName',strcat("$t_c = ",string(TC(k)),"$"))
end
ylim([-200 200])
hl = legend(); hl.Location = 'best'; hl.Interpreter = 'latex';

end