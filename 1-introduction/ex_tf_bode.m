% ex_tf_bode.m
% transfer function and bode plot of an electrical RLC circuit
% [reference] Example 2.1 RLC Circuit in LSC
% [course] Session 1 - Modeling for Control Design
close all; clear; clc

% numerator polynomial coefficients (e.g., 3s^2-4s+5 -> [3 -4 5])
num = [1];

% denominator polynomial coefficients (e.g., 7s^2+8s-9 -> [7 8 -9])
den = [1 1 1];

% transfer function object
sys = tf(num,den)

% Bode frequency response of dynamic systems
bodeplot(sys,'k')

% format the plot (see function below)
plot_example

% note the relationship between magnitude and decibels
title('dB = 20*log10(Mag)')

%--------------------------------------------------------------------------
% compute magnitude and phase at a specific frequency input
%--------------------------------------------------------------------------
% frequency of interest
w = 3;

% compute response magnitude and phase in degrees at w
[mag,phase] = bode(sys,w)

% transfer function
G = @(s) 1/(s^2 + s + 1);

% manually compute magnitude and phase shift at specific w
abs(G(1i*w))
rad2deg(angle(G(1i*w)))

%--------------------------------------------------------------------------
% plotting code
% (not the main content)
function plot_example

% colors and other parameters
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

% phase plot
ha2.LineWidth = LineWidth;
ha2.XColor = 'k';
ha2.YColor = 'k';
ha2.FontSize = FontSize;
ha2.Children(1).LineWidth = LineWidth;
ha2.Children(1).Color = nicered;

% frequency x label
ha.Children(4).FontSize = FontSize;

end