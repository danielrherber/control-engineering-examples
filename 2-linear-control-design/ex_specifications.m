% ex_specifications.m
% illustration of the time- and frequency-domain specifications using
% Matlab functions stepinfo, dcgain, allmargin, and loopsens as well as
% Nyquist diagrams
% [course] Session 6 - Linear Control (3)
close all; clear; clc

%--------------------------------------------------------------------------
% original system
%--------------------------------------------------------------------------
% system matrices
A = [-0.14 0.33 -0.33; 0.1 -0.28 0; 0 1.7 -0.77];
B = [0; 0; -0.025];
C = [2 0 0];

% LTI state-space model
sys = ss(A,B,C,[]);

% rise time, settling time, and other step-response characteristics
S = stepinfo(sys)
k = dcgain(sys)

%--------------------------------------------------------------------------
% full-state feedback
%--------------------------------------------------------------------------
% desired eigenvalues
epsilon = [0 1e-4 -1e-4];
E = [-0.67 -0.67 -0.67]+epsilon;

% closed-loop pole assignment using state feedback
K = place(A,B,E);

% closed-loop system
sys1 = ss(A-B*K,B,C,[]);

% rise time, settling time, and other step-response characteristics
S1 = stepinfo(sys1)
k1 = dcgain(sys1)

%--------------------------------------------------------------------------
% full-state feedback with integrator
%--------------------------------------------------------------------------
% augmented system matrices
Ai = [-0.14 0.33 -0.33 0;
    0.1 -0.28 0 0;
    0 1.7 -0.77 0;
    -2 0 0 0];
Bi = [0; 0; -0.025; 0];
Bri = [0; 0; 0; 1];
Bvi = [1; 0; 0; 0];
Ci = [2 0 0 0];

% desired eigenvalues
epsilon = 1e-3;
Ei = [-0.67 -0.67 -0.67 -0.67] - [0 epsilon 2*epsilon 3*epsilon];

% closed-loop pole assignment using state feedback
Ki = place(Ai,Bi,Ei);

% create the closed-loop state-space models
sysi = ss(Ai-Bi*Ki,Bri,Ci,[]);

% rise time, settling time, and other step-response characteristics
Si = stepinfo(sysi)
ki = dcgain(sysi)

%--------------------------------------------------------------------------
% gain margin, phase margin, and crossover frequencies
%--------------------------------------------------------------------------
% generate a randomized continuous-time LTI state-space model
rng(23525)
% rng(2652526) % <- also try
% rng(973724) % <- also try
P = rss(4);

% calculate various margins
Sm = allmargin(P);
Sm.Stable

% plot Bode frequency response
figure;
bode(P)
plot_example_bode1(P,Sm) % see function below

% plot Nyquist frequency response
figure;
nyquist(P)
plot_example_nyquist % see function below

%--------------------------------------------------------------------------
% sensitivity functions of plant-controller feedback loop
%--------------------------------------------------------------------------
% create a LTI state-space model
rng(242)
P = rss(6);
C = rss(3);

% determine sensitivity functions of plant-controller feedback loop
SF = loopsens(P,C)

% plot sensitivity at the plant input
figure
bode(SF.Si)
plot_example_bode2 % see function below

%--------------------------------------------------------------------------
% plotting code
% (not the main content)
function plot_example_bode1(P,Sm)

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
nicegreen = [109, 195, 80]/255;
LineWidth = 1;
MarkerSize = 12;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth,'MarkerSize',MarkerSize};

% figure
hf = gcf; hf.Color = 'w'; hold on

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

% plot points for gain margins
if ~isempty(Sm.GMFrequency)
[mag,phase,w] = bode(P,Sm.GMFrequency);
mag = mag2db(mag);
plot(ha1,w,squeeze(mag(:,:,:)),'.',plotOpts{:},'Color',nicegreen)
plot(ha2,w,squeeze(phase(:,:,:)),'.',plotOpts{:},'Color',nicegreen)
end

% plot points for phase margins
if ~isempty(Sm.PMFrequency)
[mag,phase,w] = bode(P,Sm.PMFrequency);
mag = mag2db(mag);
plot(ha1,w,squeeze(mag(:,:,:)),'.',plotOpts{:},'Color',niceblue)
plot(ha2,w,squeeze(phase(:,:,:)),'.',plotOpts{:},'Color',niceblue)
end

end

%--------------------------------------------------------------------------
% plotting code
% (not the main content)
function plot_example_nyquist

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
LineWidth = 1;
FontSize = 12;

% figure
hf = gcf; hf.Color = 'w'; hold on

grid on

% axis handles
ha = gca;

% customize main axis
ha1 = ha.Children(1);
ha1.LineWidth = LineWidth;
ha1.XColor = 'k';
ha1.YColor = 'k';
ha1.FontSize = FontSize;
ha1.Children(1).LineWidth = LineWidth;
ha1.Children(1).Color = nicered;
ha1.Children(2).FaceColor = nicered;
ha1.Children(2).EdgeColor = nicered;
ha1.Children(3).FaceColor = nicered;
ha1.Children(3).EdgeColor = nicered;
ha1.Children(6).MarkerEdgeColor = niceblue;

% customize axis labels and title
ha2 = ha.Children(2);
ha2.Color = 'k';
ha2.FontSize = FontSize;

ha3 = ha.Children(3);
ha3.Color = 'k';
ha3.FontSize = FontSize;

ha4 = ha.Children(4);
ha4.Color = 'k';
ha4.FontSize = FontSize;

end

%--------------------------------------------------------------------------
% plotting code
% (not the main content)
function plot_example_bode2

% colors and other parameters
nicered = [225, 86, 86]/255;
LineWidth = 1;
MarkerSize = 12;
FontSize = 12;

% figure
hf = gcf; hf.Color = 'w'; hold on

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