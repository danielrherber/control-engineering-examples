% ex_modal_interactive.m
% simulation of a 2nd-order damped oscillator in modal canonical form
% interactive sliders available in the figure for the key parameters
close all; clear; clc

% initial parameter values
w0 = 1;
zeta = 1;
X0 = [1;-1];

% initialize the figure (you should interact with the sliders
initialize_figure(zeta,w0,X0);

%--------------------------------------------------------------------------
% figure code (not the main content)
%--------------------------------------------------------------------------
% initialize figure
function initialize_figure(zeta,w0,X0)

% axis limit parameter
lim = 3;

% create figure
hp = figure;
hp.Color = 'w';
hp.Position = [100 100 840 420];

% create state trajectories axis
haxes(1) = axes;
haxes(1).Units = 'Pixel';
haxes(1).Position = [50 210 375 200];
haxes(1).YLabel.String = 'States';
haxes(1).XLabel.String = 'Time [sec]';
haxes(1).LineWidth = 1;
ylim([-lim+0.5 lim-0.5])
hold on

% create state-space plot axis
haxes(2) = axes;
haxes(2).Units = 'Pixel';
haxes(2).Position = [500 100 300 300];
haxes(2).YLabel.String = 'State 2';
haxes(2).XLabel.String = 'State 1';
haxes(2).LineWidth = 1;
xlim([-lim lim])
ylim([-lim lim])
grid on
hold on

% create zeta slider
h_zeta = uicontrol(hp,'style','slider','units','pixel','position',[20 20 300 20]);
h_zeta.String = 'zeta';
h_zeta.Value = zeta;
h_zeta.Max = 2;
h_zeta.Min = -0.1;

% create w0 slider
h_w0 = uicontrol(hp,'style','slider','units','pixel','position',[20 60 300 20],...
    'String','w0');
h_w0.String = 'w0';
h_w0.Value = w0;
h_w0.Max = 5;
h_w0.Min = 0.1;

% create x1(0) slider
h_x10 = uicontrol(hp,'style','slider','units','pixel','position',[20 100 300 20]);
h_x10.String = 'x10';
h_x10.Value = X0(1);
h_x10.Max = 2;
h_x10.Min = -2;

% create x2(0) slider
h_x20 = uicontrol(hp,'style','slider','units','pixel','position',[20 140 300 20]);
h_x20.String = 'x20';
h_x20.Value = X0(2);
h_x20.Max = 2;
h_x20.Min = -2;

% combine slider handles
hall = [h_zeta,h_w0,h_x10,h_x20];

% add listeners
addlistener(h_zeta,'ContinuousValueChange',@(hObject,event) changed_figure(hObject,event,hall,haxes));
addlistener(h_w0,'ContinuousValueChange',@(hObject,event) changed_figure(hObject,event,hall,haxes));
addlistener(h_x10,'ContinuousValueChange',@(hObject,event) changed_figure(hObject,event,hall,haxes));
addlistener(h_x20,'ContinuousValueChange',@(hObject,event) changed_figure(hObject,event,hall,haxes));

% run the simulation
[T,X] = run_simulation(w0,zeta,X0);

% update the figure with the new results
update_figure(X,T,zeta,w0,X0,haxes)

end

% do stuff when the slide values change
function changed_figure(hObject,event,hall,haxes)

% get all slider values
zeta = get(hall(1),'Value');
w0 = get(hall(2),'Value');
X0(1) = get(hall(3),'Value');
X0(2) = get(hall(4),'Value');

% run a new simulation
[T,X] = run_simulation(w0,zeta,X0);

% update the figure with the results
update_figure(X,T,zeta,w0,X0,haxes)

end

% update the figure's content
function update_figure(X,T,zeta,w0,X0,haxes)

% delete all current textboxes
delete(findall(gcf,'Type','TextBox'));

% create zeta value textbox
str = sprintf('%1.3f',zeta);
annotation('textbox','units','pixel','position',[320 20 300 20],...
    'String',str,'FitBoxToText','on','LineStyle','none','VerticalAlignment','middle');

% create w0 value textbox
str = sprintf('%1.3f',w0);
annotation('textbox','units','pixel','position',[320 60 300 20],...
    'String',str,'FitBoxToText','on','LineStyle','none','VerticalAlignment','middle');

% create x10 value textbox
str = sprintf('%1.3f',X0(1));
annotation('textbox','units','pixel','position',[320 100 300 20],...
    'String',str,'FitBoxToText','on','LineStyle','none','VerticalAlignment','middle');

% create x20 value textbox
str = sprintf('%1.3f',X0(2));
annotation('textbox','units','pixel','position',[320 140 300 20],...
    'String',str,'FitBoxToText','on','LineStyle','none','VerticalAlignment','middle');

% create zeta string textbox
annotation('textbox','units','pixel','position',[370 20 300 20],...
    'String','$\zeta$','FitBoxToText','on','Interpreter','latex',...
    'LineStyle','none','VerticalAlignment','middle','FontSize',16);

% create w0 string textbox
annotation('textbox','units','pixel','position',[370 60 300 20],...
    'String','$\omega_0$','FitBoxToText','on','Interpreter','latex',...
    'LineStyle','none','VerticalAlignment','middle','FontSize',16);

% create x10 string textbox
annotation('textbox','units','pixel','position',[370 100 300 20],...
    'String','$x_1(0)$','FitBoxToText','on','Interpreter','latex',...
    'LineStyle','none','VerticalAlignment','middle','FontSize',16);

% create x20 string textbox
annotation('textbox','units','pixel','position',[370 140 300 20],...
    'String','$x_2(0)$','FitBoxToText','on','Interpreter','latex',...
    'LineStyle','none','VerticalAlignment','middle','FontSize',16);

% colors
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
nicegreen = [109, 195, 80]/255;
% nicegray = [242, 242, 242]/255;
xmediumgray = [170, 170, 170]/255;

% plot state trajectories
delete(haxes(1).Children)
plot(haxes(1),T,X(:,1),'LineWidth',1.5,'Color',niceblue)
plot(haxes(1),T,X(:,2),'LineWidth',1.5,'Color',nicered)

% plot state-space trajectory
delete(haxes(2).Children)
plot(haxes(2),X(:,1),X(:,2),'LineWidth',1.5,'Color',nicegreen)
plot(0,0,'.','Color',xmediumgray,'MarkerSize',16)

% update graphics
drawnow;

end

% run the simulation
function [T,X] = run_simulation(w0,zeta,X0)

% state matrix
A = [0 1; -w0^2 -2*zeta*w0];

% time horizon
TSPAN = linspace(0,15,1e3);

% simulation options
OPTIONS = odeset('RelTol',1e-6,'AbsTol',1e-6);

% run simulation
[T,X] = ode45(@(t,x) A*x,TSPAN,X0,OPTIONS);

end