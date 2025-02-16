% ex_integral_control.m
% design and simulation of a full-state feedback controller with output
% error integrators
% [reference] Example 4.12 Integral Control of a Third Order System in LSC
% [course] Session 5 - Linear Control (2)
close all; clear; clc

% simulations options
X0 = [1 1 1 0]'; % integrator state initialized to 0
tfinal = 100;
T = linspace(0,tfinal,10000)';

% reference signal
% R = -3*ones(size(T)); % <- also try this
R = floor(sin(0.1*T))+1.5;
% R = 0.1*T; % <- also try this
% R = sin(0.3*T); % <- also try this
% x = 2; R = 0.1*T.^x;  % <- also try this % note: error increases with x > 1

% disturbance signal
% V = -3*ones(size(T)); % <- also try this
V = 0.1*floor(sin(0.2*T));
% V = 0.1*sin(8*T); % <- also try this

% original system matrices
A = [-0.14 0.33 -0.33;
    0.1 -0.28 0;
    0 1.7 -0.77];
B = [0; 0; -0.025];
Br = [0; 0; 0];
Bv = [1; 0; 0];
C = [2 0 0];

% number of states, inputs, and outputs/references
nx = size(A,1);
nm = size(B,2);
nr = size(C,1);

% augmented system matrices
A1 = [[A;-C],zeros(nx+nr,nr)];
B1 = [B;zeros(nr,nm)];
Br1 = [zeros(nx,nr);eye(nr)];
Bv1 = [Bv;zeros(nr,nm)];
C1 = [C,zeros(nr,nr)];

% check controllability
Mc = ctrb(A1,B1);
rank(Mc);

% desired eigenvalues
epsilon = 1e-3;
tc = 1.5; E1 = [-1/tc -1/tc -1/tc -1/tc] - [0 epsilon 2*epsilon 3*epsilon];

% closed-loop pole assignment using state feedback
K1 = place(A1,B1,E1); % with output error integrator
K = place(A,B,E1(1:3));

% create the closed-loop state-space models
sys1 = ss(A1-B1*K1,Br1,C1,[]); % with output error integrator
sys = ss(A-B*K,Br,C,[]);

% simulate
Y1 = lsim(sys1,R,T,X0); % with output error integrator
Y = lsim(sys,R,T,X0(1:3));

% plot the simulation results (see function below)
plot_example(T,Y1,Y,R,[],1)

% create the closed-loop state-space models for a non-zero disturbance
sys1v = ss(A1-B1*K1,[Br1,Bv1],C1,[]);
sysv = ss(A-B*K,[Br,Bv],C,[]);

% inputs (reference + disturbance)
U = [R,V];

% simulate
Y1v = lsim(sys1v,U,T,X0);
Yv = lsim(sysv,U,T,X0(1:3));

% plot the simulation results (see function below)
plot_example(T,Y1v,Yv,R,V,2)

%--------------------------------------------------------------------------
% plotting code
% (not the main content)
function plot_example(T,Y1,Y,R,V,mycase)

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

if mycase==2
    plot(T,V,'-',plotOpts{:},'Color',nicegreen,'DisplayName','Disturbance');
end

plot(T,R,'--',plotOpts{:},'Color',nicegray,'DisplayName','Reference');
plot(T,Y1,plotOpts{:},'Color',niceblue,'DisplayName','Output with integrator')
plot(T,Y,plotOpts{:},'Color',nicered,'DisplayName','Output without integrator')

xlabel('Time [sec]')
ylabel('Signals')

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

hl = legend(); hl.Location = "best";

switch mycase
    case 1
        title('Output Tracking Without a Disturbance')
    case 2
        title('Output Tracking With a Disturbance')
end

end