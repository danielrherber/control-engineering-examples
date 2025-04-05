% ex_pi_gain_optimization.m
% optimization of the Kp and Ki gains in a PI controller using fmincon with
% the objective of minimize maximum control effort subject to overshoot and
% settling time constraints
% [course] Session 10 - Optimal Control (2)
close all; clear; clc

% create the plant model
rng(3535)
P = rss(5);
P.D = 0; % no feedforward matrix

% use pidtune for a reasonable initial controller
C0 = pidtune(P,'PI');

% upper bounds on overshoot and settling time
limits.Overshoot = 5; % percentage
limits.SettlingTime = 4; % seconds

% multiplication factor for gain value range to consider during optimization
m = 10;

% bounds on the Kp gain
KP0 = C0.Kp;
lb(1) = KP0-m*abs(KP0);
ub(1) = KP0+m*abs(KP0);

% bounds on the Ki gain
KI0 = C0.Ki;
lb(2) = KI0-m*abs(KI0);
ub(2) = KI0+m*abs(KI0);

% initial guess is the pidtune controller
x0 = [KP0;KI0];

% options for fmincon
options= optimoptions('fmincon','Display','iter',...
    'FiniteDifferenceType','central','ConstraintTolerance',1e-3);

% find the optimal gains
gains = fmincon(@(x) objective(x,P),x0,[],[],[],[],lb,ub,...
    @(x) nonlcon(x,P,limits),options);

% analyze original and optimized closed-loop systems
[sys0,S0,t0,y0,u0,umax0] = analysis(x0,P);
[sys,S,t,y,u,umax] = analysis(gains,P);

% plot output
plot_example(t0,y0,t,y,"Output")

% plot control
plot_example(t0,u0,t,u,"Control")

%--------------------------------------------------------------------------
% given the gains and plant, create the closed-loop system and compute the
% step response and maximum control effort
function [sys,S,t,y,u,umax] = analysis(gains,P)

% extract gains
Kp = gains(1);
Ki = gains(2);

% construct PI controller
C = pid(Kp,Ki);

% closed-loop system with controller
sys = feedback(P*C,1);

% calculate step response
T = linspace(0,20,1e5);
[y,t] = step(sys,T);

% calculate step info
S = stepinfo(y,t);

% compute control of the PI controller
u = Kp*(1-y) + Ki*cumtrapz(t,1-y);

% maximum control effort
umax = max(abs(u));

end

%--------------------------------------------------------------------------
% calculate the objective function value
function f = objective(gains,P)

% compute various values given gains and plant
[~,~,~,~,~,umax] = analysis(gains,P);

% objective value
f = umax;

end

%--------------------------------------------------------------------------
% calculate the nonlinear constraints in standard form
function [g,h] = nonlcon(gains,P,limits)

% compute various values given gains and plant
[~,S,~,~,~,~] = analysis(gains,P);

% inequality constraints
g(1) = S.SettlingTime - limits.SettlingTime;
g(2) = S.Overshoot - limits.Overshoot;

% equality constraints
h = [];

end

%--------------------------------------------------------------------------
% plotting code for output and control
% (not the main content)
function plot_example(t1,x1,t2,x2,mycase)

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
LineWidth = 1;
MarkerSize = 12;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth,'MarkerSize',MarkerSize};

% initialize figure
hf = figure; hf.Color = 'w'; hold on;

plot(t1,x1,plotOpts{:},'Color',niceblue)
plot(t2,x2,plotOpts{:},'Color',nicered)

xlabel('Time [s]')
ylabel(mycase)
legend('pidtune','optimized','Location','best')

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

hl = legend(); hl.Location = "best"; hl.FontSize = FontSize;

end