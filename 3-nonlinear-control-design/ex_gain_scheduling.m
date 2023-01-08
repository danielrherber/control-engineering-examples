% https://www.mathworks.com/help/control/ug/designing-a-family-of-pid-controllers-for-multiple-operating-points.html
close all; clear; clc

% Open the continuous stirred tank reactor (CSTR) plant model
mdl = 'scdcstrctrlplant';
open_system(mdl)

% Specify the operating regions
% output concentration C
C = [2 3 4 5 6 7 8 9];

% Create an array of default operating point specifications
op = operspec(mdl,numel(C));

% Initialize the operating point specifications by specifying that the
% output concentration is a known value, and specifying the output
% concentration value
for ct = 1:numel(C)
	op(ct).Outputs.Known = true;
	op(ct).Outputs.y = C(ct);
end

% Compute the equilibrium operating points corresponding to the values of C
opoint = findop(mdl,op,findopOptions('DisplayReport','off'));

% Linearize the plant at these operating points
Plants = linearize(mdl,opoint);

% Since the CSTR plant is nonlinear, the linear models display different
% characteristics. For example, plant models with high and low conversion
% rates are stable, while the others are not
isstable(Plants,'elem')'

% To design multiple PID controllers in batch, use the pidtune function.
% The following command generates an array of PID controllers in parallel
% form. The desired open-loop crossover frequency is at 1 rad/sec and the
% phase margin is the default value of 60 degrees
Controllers = pidtune(Plants,'pidf',pidtuneOptions('Crossover',1));

% Display the controller for C = 4
Controllers(:,:,4)

% To analyze the closed-loop responses for step setpoint tracking, first
% construct the closed-loop systems
clsys = feedback(Plants*Controllers,1);

% Plot closed-loop responses
figure
hold on
for ct = 1:length(C)
    % Select a system from the LTI array
    sys = clsys(:,:,ct);
    sys.Name = ['C=',num2str(C(ct))];
    sys.InputName = 'Reference';
    % Plot step response
    stepplot(sys,20);
end
legend('show','location','southeast')

% Design updated controllers for the unstable plant models
Controllers = pidtune(Plants,'pidf',10);

% Display the controller for C = 4
Controllers(:,:,4)

% Construct the closed-loop systems, and plot the closed-loop step
% responses for the new controllers
clsys = feedback(Plants*Controllers,1);
figure
hold on
for ct = 1:length(C)
    % Select a system from the LTI array
    sys = clsys(:,:,ct);
    set(sys,'Name',['C=',num2str(C(ct))],'InputName','Reference');
    % Plot step response
    stepplot(sys,20);
end
legend('show','location','southeast')

% For comparison, examine the response when you use the same controller at
% all operating points. Create another set of closed-loop systems, where
% each one uses the C = 2 controller, and plot their responses
clsys_flat = feedback(Plants*Controllers(:,:,1),1);
figure
stepplot(clsys,clsys_flat,20)
legend('C-dependent Controllers','Single Controller')

% Close the model
bdclose(mdl)

% Plot the gains as a function of the scheduling parameter
hf = figure; hf.Color = 'w'; hold on
plot(C,Controllers.Kp)
plot(C,Controllers.Ki)
plot(C,Controllers.Kd)
legend('Kp','Ki','Kd')

% other links
% https://www.mathworks.com/help/control/ug/gain-scheduled-control-of-a-chemical-reactor.html