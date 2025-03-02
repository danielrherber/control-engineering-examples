% ex_pidtuner.m
% pidtune/pidTuner demonstrations for tuning (designing) PID controllers
% [reference] Section 8.1 The PID Regulator Family and Design Methods (pp.
% 278–290) in CE
% [course] Session 6 - Linear Control (3)
close all; clear; clc

% system matrices
A = [-0.14 0.33 -0.33;
    0.1 -0.28 0;
    0 1.7 -0.77];
B = [0; 0; -0.025];
Br = [0; 0; 0];
Bv = [1; 0; 0];
C = [2 0 0];

% LTI state-space model
P = ss(A,B,C,[]);
disp(stepinfo(P))

% PI controller tuned by Matlab
[C_PI,info] = pidtune(P,'PI');
T_PI = feedback(C_PI*P,1);
disp(info)
disp(stepinfo(T_PI))

% manual wc PI controller
wc = 2;
[C_PI_fast,info] = pidtune(P,'PI',wc);
T_PI_fast = feedback(C_PI_fast*P,1);
disp(info)
disp(stepinfo(T_PI_fast))

% PID controller tuned by Matlab
[C_PID,info] = pidtune(P,'PID');
T_PID = feedback(C_PID*P,1);
disp(info)
disp(stepinfo(T_PID))

% plot step responses
step(P,T_PI,T_PI_fast,T_PID,linspace(0,100,1e5))
legend('P','PI1','PI2','PID')

% customize the plot (see function below)
plot_example

% PID tuner application
pidTuner(P,T_PID)

%--------------------------------------------------------------------------
% plotting code
% (not the main content)
function plot_example

% colors and other parameters
LineWidth = 1;
FontSize = 12;

% get figure
hf = gcf; hf.Color = 'w';

% get axis
ha = gca;

ha1 = ha.Children(1);
ha1.XColor = 'k'; ha1.YColor = 'k'; ha1.LineWidth = 1; ha1.FontSize = FontSize;
ha1.Children(1).LineWidth = LineWidth;
ha1.Children(2).LineWidth = LineWidth;
ha1.Children(3).LineWidth = LineWidth;
ha1.Children(4).LineWidth = LineWidth;

ha2 = ha.Children(2); ha2.Color = 'k'; ha2.FontSize = FontSize;
ha3 = ha.Children(3); ha3.Color = 'k'; ha3.FontSize = FontSize;
ha4 = ha.Children(4); ha4.Color = 'k'; ha4.FontSize = FontSize;

end