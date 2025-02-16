% ex_multi_input_place.m
% illustration of eigenstructure assignment for multi-input systems using
% the Matlab place command
% [reference] Example 4.5 Continuous Control via the Controller Canonical
% Form in LSC
% [course] Session 5 - Linear Control (2)
close all; clear; clc

% system matrices
A = [-0.14 0.33 -0.33;
    0.1 -0.28 0;
    0 1.7 -0.77];

% (control) input matrix
B1 = [0; 0; -1]; % single input
B3 = [-1 0 0; 0 -1 0; 0 0 -1]; % multiple input

% disturbance input matrix
Bv = [0; 0; 1];
% Bv = [1; 0; 0]; % <- also try this

% output matrix
C = [2 0 0];
% C = [0 0 1]; % <- also try this

% original system eigenvalues
eig(A)

% check controllability
Mc = ctrb(A,B1);
rank(Mc)

% desired eigenvalues
epsilon = 1e-4;
tc = 1.5; E = [-1/tc -1/tc -1/tc] + [0 epsilon 2*epsilon];

% closed-loop pole assignment using state feedback
K1 = place(A,B1,E);
K3 = place(A,B3,E);

% create the closed-loop state-space models
sys1 = ss(A-B1*K1,Bv,C,[]);
sys3 = ss(A-B3*K3,Bv,C,[]);

% simulations options
X0 = [1;1;1];
tfinal = 100;
T = linspace(0,tfinal,10000)';

% disturbance signal
V = 1*ones(size(T));
% V = 10*sin(T); % <- also try this

% simulate
[Y1,T1,X1] = lsim(sys1,V,T,X0);
[Y3,T3,X3] = lsim(sys3,V,T,X0);

% transfer functions between the output and disturbance (note zeros in TFs)
[N1,D1] = ss2tf(sys1.A,sys1.B,sys1.C,sys1.D)
[N3,D3] = ss2tf(sys3.A,sys3.B,sys3.C,sys3.D)

% plot the simulation results (see function below)
plot_example(T,Y1,Y3,V)

%--------------------------------------------------------------------------
% plotting code
% (not the main content)
function plot_example(T,Y1,Y3,V)

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

plot(T,Y1,plotOpts{:},'Color',niceblue,'DisplayName','Y with 1 input')
plot(T,Y3,plotOpts{:},'Color',nicered,'DisplayName','Y with 3 inputs')
plot(T,V,'--',plotOpts{:},'Color',nicegray,'DisplayName','disturbance');

xlabel('Time [sec]')
ylabel('Output')

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

legend();

end