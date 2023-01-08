function single_shooting_ex_class

close all; clc

% NOTE: using Z to denote the optimization variables

% problem parameters
auxdata.a = 1; auxdata.m0 = 1; auxdata.Dm = 0.0749; auxdata.T = 0.1405;

% fixed initial states, final states, and final time
r0 = 1; T0 = 0; vr0 = 0; vT = 1;
auxdata.vrf = 0;
tf = 3.32;
auxdata.X0 = [r0;T0;vr0;vT];

% shooting parameters
auxdata.nt = 10; % number of time points
auxdata.t = linspace(0,tf,auxdata.nt)'; % time mesh/grid
auxdata.nu = 1; % number of controls

% discretized variable indices in x = [u];
auxdata.phii = 1:(auxdata.nt);

% initial guess for U (all zeros)
Z0 = zeros(auxdata.nt*auxdata.nu,1);

% fmincon options
options = optimoptions(@fmincon,'display','iter','MaxFunEvals',5e3,...
    'UseParallel',false);

% solve the optimization problem
Z = fmincon(@(Z) objective(Z,auxdata),Z0,[],[],[],[],[],[],...
    @(Z) constraints(Z,auxdata),options);

% extract controls from Z
U = Z(auxdata.phii);

% obtain the optimal solution (on a more dense time grid)
[To,Xo] = ode45(@(t,x) derivative(t,x,U,auxdata),[0 tf],auxdata.X0,...
    odeset('RelTol',1e-13)); % run a simulation
r = Xo(:,1); T = Xo(:,2); vr = Xo(:,3); vT = Xo(:,4); % extract states

% plot theta and r states
hf = figure; hf.Color = 'w'; clf;
polarplot(T,r,'linewidth',2,'color','r');

% plot control
hf = figure; hf.Color = 'w'; hold on;
plot(To,rad2deg(interp1(auxdata.t,U,To,'spline')),'r','linewidth',2)
plot(auxdata.t,rad2deg(U),'.k','markersize',16)
legend('interpolated U(t)','U')

end

% compute the objective function
function J = objective(Z,auxdata)

% extract controls from Z
U = Z(auxdata.phii);

% simulate
X = run_simulation(U,auxdata);

% extract (needed) states
r = X(:,1);

% compute Lagrange term (none in this problem)
Jl = 0;

% compute Mayer term
Jm = -r(end);

% combine
J = Jl + Jm;

end

% compute the nonlinear constraints
function [g,h] = constraints(Z,auxdata)

% extract controls from Z
U = Z(auxdata.phii);

% simulate
X = run_simulation(U,auxdata);

% extract (needed) states
r = X(:,1); T = X(:,2); vr = X(:,3); vT = X(:,4);

% boundary (final state) constraints
h1 = vr(end) - auxdata.vrf;
h2 = vT(end) - sqrt(auxdata.a/r(end));

% path constraints
% none

% combine constraints
g = [];
h = [h1;h2];

end

% run the simulation
function X = run_simulation(U,auxdata)

% simulation options
options = odeset('RelTol',1e-13);

% run a simulation
[~,X] = ode45(@(t,x) derivative(t,x,U,auxdata),auxdata.t,auxdata.X0,options);

end

% nonlinear state derivative function
function Dx = derivative(t,x,U,auxdata)

% extract states
r = x(1); T = x(2); vr = x(3); vT = x(4);

% compute control using spline interpolation
phi = interp1(auxdata.t,U,t,'spline');

% compute derivatives
Dr = vr;
DT = vT/r;
Dvr = vT^2/r - auxdata.a/r^2 + auxdata.T/(auxdata.m0 - auxdata.Dm*t)*sin(phi);
DvT = -vr*vT/r + auxdata.T/(auxdata.m0 - auxdata.Dm*t)*cos(phi);

% combine
Dx = [Dr; DT; Dvr; DvT];

end

% how to run the simulation only when optimization variables are updated:
% https://www.mathworks.com/help/optim/ug/objective-and-nonlinear-constraints-in-the-same-function.html