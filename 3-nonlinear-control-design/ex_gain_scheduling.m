% ex_gain_scheduling.m
% illustrates gain scheduling for the design of an array of PID controllers
% for a nonlinear continuous stirred tank reactor (CSTR) model in Simulink
% that operates over a wide range of operating points
% [reference] https://www.mathworks.com/help/control/ug/designing-a-family-of-pid-controllers-for-multiple-operating-points.html
% [reference] https://www.mathworks.com/help/control/ug/gain-scheduled-control-of-a-chemical-reactor.html
% [reference] https://www.mathworks.com/help/slcontrol/ug/implement-gain-scheduled-pid-controllers.html
% [course] Session 7 - Nonlinear Control (1)
close all; clear; clc

% open the nonlinear continuous stirred tank reactor (CSTR) plant model
mdl = 'scdcstrctrlplant';
openExample(mdl)

%--------------------------------------------------------------------------
% linearize the model at different operating points
%--------------------------------------------------------------------------
% specify the operating regions defined by the output concentration C
C = [2 3 4 5 6 7 8 9];

% create an array of default operating point specifications
op = operspec(mdl,numel(C));

% initialize the operating point specifications by specifying that the
% output concentration is a known value, and specifying the output
% concentration value C(k)
for k = 1:numel(C)
	op(k).Outputs.Known = true;
	op(k).Outputs.y = C(k);
end

% compute the equilibrium operating points corresponding to the values of C
opoint = findop(mdl,op,findopOptions('DisplayReport','off'));

% linearize the plant at these operating points
Plants = linearize(mdl,opoint);

% since the CSTR plant is nonlinear, the plant models with high and low
% output concentrations are stable, while the others are not
isstable(Plants,'elem')'

%--------------------------------------------------------------------------
% design the PID controller for each value of C with 1 rad/sec wc
%--------------------------------------------------------------------------
% design multiple PID controllers in batch using the pidtune function
% The following command generates an array of PID controllers in parallel
% form. The desired open-loop crossover frequency is at 1 rad/sec and the
% phase margin is the default value of 60 degrees
Controllers = pidtune(Plants,'pidf',pidtuneOptions('Crossover',1));

% display the controller for C = 4
Controllers(:,:,4)

% construct the closed-loop systems
clsys = feedback(Plants*Controllers,1);

% plot closed-loop step responses
hf = figure; hold on
for k = 1:length(C)
    sys = clsys(:,:,k);  % select a system from the LTI array
    sys.Name = ['C=',num2str(C(k))];
    sys.InputName = 'Reference';
    stepplot(sys,20); % plot step response
end
plot_example_step(hf,1) % see function below

%--------------------------------------------------------------------------
% design the PID controller for each value of C with 10 rad/sec wc
%--------------------------------------------------------------------------
% design updated controllers for the unstable plant models
Controllers = pidtune(Plants,'pidf',10);

% display the controller for C = 4
Controllers(:,:,4)

% construct the closed-loop systems
clsys = feedback(Plants*Controllers,1);

% plot closed-loop step responses
hf = figure; hold on
for k = 1:length(C)
    sys = clsys(:,:,k);  % select a system from the LTI array
    set(sys,'Name',['C=',num2str(C(k))],'InputName','Reference');
    stepplot(sys,20); % plot step response
end
plot_example_step(hf,1) % see function below

% for comparison, examine the response when you use the same controller at
% all operating points (namely, the C = 2 controller)
clsys_flat = feedback(Plants*Controllers(:,:,1),1);

% plot closed-loop step responses
hf = figure; hold on
stepplot(clsys,clsys_flat,20)
plot_example_step(hf,2) % see function below

% close the model
bdclose(mdl)

% plot the gains as a function of the scheduling parameter C
plot_example_gains(C,Controllers)

% open a model that simulates the behavior across the operating points
openExample('PIDGainSchedCSTRExampleModel')

%--------------------------------------------------------------------------
% plotting code
% (not the main content)
function plot_example_step(hf,mycase)

% colors and other parameters
LineWidth = 1;
FontSize = 12;

% legend
switch mycase
    case 1
        legend('show','location','southeast')
    case 2
        legend('C-Dependent Controllers','Single Controller')
end

% customize plot
hf.Color = 'w';
ha = gca;
ha.XLabel.FontSize = FontSize;
ha.XLabel.Color = 'k';
ha.YLabel.FontSize = FontSize;
ha.YLabel.Color = 'k';
ha.Title.FontSize = FontSize;
ha.Title.Color = 'k';
ha.Children(1).XColor = 'k';
ha.Children(1).YColor = 'k';
ha.Children(1).LineWidth = 1;
ha.Children(1).FontSize = FontSize;

% change line thickness
for k = 1:length(ha.Children(1).Children(:))
    ha.Children(1).Children(k).LineWidth = LineWidth;
end

end

%--------------------------------------------------------------------------
function plot_example_gains(C,Controllers)

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
nicegreen = [109, 195, 80]/255;
LineWidth = 1;
MarkerSize = 12;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth,'MarkerSize',MarkerSize};

% initialize figure
hf = figure; hf.Color = 'w'; hold on

% plot lines
plot(C,Controllers.Kp,'-',plotOpts{:},'Color',nicered,'DisplayName','Kp');
plot(C,Controllers.Ki,'-',plotOpts{:},'Color',niceblue,'DisplayName','Ki');
plot(C,Controllers.Kd,'-',plotOpts{:},'Color',nicegreen,'DisplayName','Kd');

% axis and legend
xlabel('Concentration')
ylabel('Gain')
ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;
hl = legend(); hl.Location = "best";

end