% ex_Hinfinity.m
% synthesize a H-infinity optimal controller controller using different
% target performance levels. The plant in this example is based on the
% augmented plant model used in Robust Control of an Active Suspension
% [reference] www.mathworks.com/help/robust/ref/lti.hinfsyn.html
% [course] Session 13 - Control under Uncertainty (2)
close all; clear; clc

% load the example plant
mdl = 'robust/HinfinityControllerSynthesisExample';
openExample(mdl)
load hinfsynExData P

% get size of the plant
nP = size(P)

% this plant has five outputs and four inputs, where the last two outputs
% are measurement signals to provide to the controller, and the last input
% is a control signal. Compute an H-infinity optimal controller
ncont = 1;
nmeas = 2;
[K1,CL,gamma] = hinfsyn(P,nmeas,ncont);

% the resulting two-input, one-output controller has the same number of
% states as P
size(K1)

% the optimal performance level achieved by this controller is returned as
% gamma. This value is the H-infinity norm of the closed-loop system
% between V -> Ys
gamma

% get original transfer function between W -> Z
P_original = P(1:(nP(1)-nmeas),1:(nP(2)-ncont));

% directly compute the H-infinity norm of both systems
Hinf_Pwz_original = mag2db(hinfnorm(P_original))
Hinf_CL = mag2db(hinfnorm(CL))

% plot singular values of the systems along with achieved gamma
hf = figure; hf.Color = 'w';
Gamma = ss(gamma);
plot_example(P_original,CL,Gamma)

%--------------------------------------------------------------------------
% plotting code
% (not the main content)
function plot_example(P_original,CL,Gamma)

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
nicegray = [170, 170, 170]/255;
LineWidth = 1;
FontSize = 12;

% initialize figure
hf = figure; hf.Color = 'w'; hold on

sigma(P_original,CL,Gamma)

ylim([-60,10])

ha = gca; ha.AxesStyle.RulerColor = 'k';
ha.AxesStyle.GridColor = 'k';
ha.AxesStyle.FontSize = FontSize;

ha.YLabel.Color = 'k';
ha.YLabel.FontSize = FontSize;

ha.XLabel.Color = 'k';
ha.XLabel.FontSize = FontSize;

ha.Title.Color = 'k';
ha.Title.FontSize = FontSize;

ha.Responses(1).Color = niceblue;
ha.Responses(1).LineWidth = LineWidth;

ha.Responses(2).Color = nicered;
ha.Responses(2).LineWidth = LineWidth;

ha.Responses(3).Color = nicegray;
ha.Responses(3).LineWidth = LineWidth;

hl = legend(); hl.Location = "best"; hl.FontSize = FontSize;

end