% ex_modred.m
% model reduction using a balanced realization
% [reference] pp. 164-165 in A Course in Robust Control Theory
% [course] Session 3 - Analysis of State-Space Models (2)
close all; clear; clc

% symbolic transfer function
syms s
num = (s+10)*(s-5)*(s^2 + 2*s + 5)*(s^2 - 0.5*s +5);
den = (s+4)*(s^2+4*s+8)*(s^2+0.2*s+100)*(s^2+5*s+2000);

% get coefficients
num2 = double(fliplr(coeffs(num)));
den2 = double(fliplr(coeffs(den)));

% transfer function coefficients to state-space matrices
[A,B,C,D] = tf2ss(num2,den2);

% state-space model
sys = ss(A,B,C,D);

% compute gramians
Wc  = gram(sys,'c');
Wo  = gram(sys,'o');

% Hankel singular values (direct)
G2 = sqrt(eig(Wc*Wo));

% Gramian-based balancing of state-space realizations
[SYSB,G,T,Ti] = balreal(sys);

% number of states in the original system
n = length(G);

% number of desired states in the reduced-order model
r = 4;
% r = 2; % <- also try this

% 0: keep the state, 1:eliminate the state
elim = logical([zeros(r,1);ones(n-r,1)]);

% model simplification/reduction by state elimination
RSYS = xelim(SYSB,elim,'truncate');
% RSYS = xelim(SYSB,elim,'MatchDC') % <- also try this

% Bode response of both systems
bode(sys,RSYS)

% format the plot (see function below)
plot_example

%--------------------------------------------------------------------------
% plotting code
% (not the main content)
function plot_example

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
LineWidth = 1;
FontSize = 12;

% figure
hf = gcf; hf.Color = 'w';

% axis handles
ha = gca; ha1 = ha.Children(1); ha2 = ha.Children(2);

% magnitude plot
ha1.LineWidth = LineWidth;
ha1.XColor = 'k';
ha1.YColor = 'k';
ha1.FontSize = FontSize;
ha1.Children(1).LineWidth = LineWidth;
ha1.Children(1).Color = nicered;
ha1.Children(4).LineWidth = LineWidth;
ha1.Children(4).Color = niceblue;

% phase plot
ha2.LineWidth = LineWidth;
ha2.XColor = 'k';
ha2.YColor = 'k';
ha2.FontSize = FontSize;
ha2.Children(1).LineWidth = LineWidth;
ha2.Children(1).Color = nicered;
ha2.Children(4).LineWidth = LineWidth;
ha2.Children(4).Color = niceblue;

% frequency x label
ha.Children(4).FontSize = FontSize;

end