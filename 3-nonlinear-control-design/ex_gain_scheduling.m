% ex_gain_scheduling.M
% this example illustrates gain scheduling for the design of an array of
% PID controllers for a nonlinear continuous stirred tank reactor (CSTR)
% model in Simulink that operates over a wide range of operating points
% heavily based on the following URLs:
% https://www.mathworks.com/help/control/ug/designing-a-family-of-pid-controllers-for-multiple-operating-points.html
% https://www.mathworks.com/help/control/ug/gain-scheduled-control-of-a-chemical-reactor.html
% https://www.mathworks.com/help/slcontrol/ug/implement-gain-scheduled-pid-controllers.html
close all; clear; clc

% open the nonlinear continuous stirred tank reactor (CSTR) plant model
mdl = 'scdcstrctrlplant';
open_system(mdl)

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
hf = figure; hf.Color = 'w'; hold on
for k = 1:length(C)
    sys = clsys(:,:,k);  % select a system from the LTI array
    sys.Name = ['C=',num2str(C(k))];
    sys.InputName = 'Reference';
    stepplot(sys,20); % plot step response
end
legend('show','location','southeast')

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
hf = figure; hf.Color = 'w'; hold on
for k = 1:length(C)
    sys = clsys(:,:,k);  % select a system from the LTI array
    set(sys,'Name',['C=',num2str(C(k))],'InputName','Reference');
    stepplot(sys,20); % plot step response
end
legend('show','location','southeast')

% for comparison, examine the response when you use the same controller at
% all operating points (namely, the C = 2 controller)
clsys_flat = feedback(Plants*Controllers(:,:,1),1);

% plot closed-loop step responses
hf = figure; hf.Color = 'w'; hold on
stepplot(clsys,clsys_flat,20)
legend('C-dependent Controllers','Single Controller')

% close the model
bdclose(mdl)

% plot the gains as a function of the scheduling parameter C
hf = figure; hf.Color = 'w'; hold on
plot(C,[Controllers.Kp,Controllers.Ki,Controllers.Kd])
xlabel('C'); ylabel('Gain'); legend('Kp','Ki','Kd')

% open a model that simulates the behavior across the operating points
open_system('PIDGainSchedCSTRExampleModel')