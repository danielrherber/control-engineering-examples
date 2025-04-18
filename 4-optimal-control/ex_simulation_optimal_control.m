% ex_simulation_optimal_control.m
% example of simulation-based optimal control using fmincon and ode45 for
% the open-loop control design of a constant-thrust rocket from a given
% initial circular orbit to the largest possible circular orbit
% [reference] pp. 66-69 in Applied Optimal Control
% [reference] Moyer and Pinkham (1964), doi: 10.1016/b978-1-4831-9812-5.50008-4
% [reference] https://github.com/danielrherber/dt-qp-project/tree/master/examples/nonlin/transfer-max-radius
% [course] Session 10 - Optimal Control (2)
close all; clear; clc

% problem parameters
auxdata.a = 1; % gravitational constant of the attracting center
auxdata.m0 = 1; % initial mass
auxdata.Dm = 0.0749; % constant mass loss rate
auxdata.T = 0.1405; % rocket thrust
r0 = 1; T0 = 0; vr0 = 0; vT = 1; % initial states
auxdata.X0 = [r0;T0;vr0;vT]; % initial state vector
auxdata.vrf = 0;  % final vr
tf = 3.32; % final time

% mesh or time grid
mesh_opt = 1;
switch mesh_opt
    %----------------------------------------------------------------------
    case 1 % equidistant time grid with N+1 = 9
        auxdata.nt = 9; % number of time points
        auxdata.t = linspace(0,tf,auxdata.nt)'; % equidistant time grid
    %----------------------------------------------------------------------
    case 2 % equidistant time grid with N+1 = 21
        auxdata.nt = 21; % number of time points
        auxdata.t = linspace(0,tf,auxdata.nt)'; % equidistant time grid
    %----------------------------------------------------------------------
    case 3
        d = 0.3;
        auxdata.t = unique([linspace(0,tf/2-d,5),...
            linspace(tf/2-d,tf/2+d,9),linspace(tf/2+d,tf,5)]'); % irregular time grid
        auxdata.nt = length(auxdata.t); % number of time points
    %----------------------------------------------------------------------
end

% number of controls
auxdata.nu = 1;

% discretized variable indices in P = [u1]
auxdata.phi_indices = 1:(auxdata.nt);

% initial guess for U
P0 = linspace(0,2*pi,auxdata.nt)'; % linearly increasing (good guess)
% P0 = zeros(auxdata.nt*auxdata.nu,1); % all zeros (poor guess) <- try this

% fmincon options
options = optimoptions(@fmincon,'display','iter','MaxFunEvals',1e4);

% solve the optimization problem
P = fmincon(@(P) objective(P,auxdata),P0,[],[],[],[],[],[],...
    @(P) constraints(P,auxdata),options);

% extract controls from P
U = P(auxdata.phi_indices);

% obtain the optimal solution (on a more dense time grid)
[To,Xo] = ode45(@(t,x) derivative(t,x,U,auxdata),linspace(0,tf,1e4),...
    auxdata.X0,odeset('RelTol',1e-13));

% extract states from the final simulation
r = Xo(:,1); T = Xo(:,2); vr = Xo(:,3); vT = Xo(:,4);

% plot results
plot_example(r,T,To,U,auxdata)

%--------------------------------------------------------------------------
% compute the objective function
function J = objective(P,auxdata)

% extract controls from P
U = P(auxdata.phi_indices);

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

%--------------------------------------------------------------------------
% compute the nonlinear constraints
function [g,h] = constraints(P,auxdata)

% extract controls from P
U = P(auxdata.phi_indices);

% simulate
X = run_simulation(U,auxdata);

% extract states from the simulation
r = X(:,1); T = X(:,2); vr = X(:,3); vT = X(:,4);

% boundary (final state) constraints
h1 = vr(end) - auxdata.vrf;
h2 = vT(end) - sqrt(auxdata.a/r(end));

% path constraints
g1 = 0 - U; % U >= 0
g2 = U - 2*pi; % U <= 2*pi

% combine constraints
g = [g1;g2];
h = [h1;h2];

end

%--------------------------------------------------------------------------
% run the simulation
function X = run_simulation(U,auxdata)

[~,X] = ode45(@(t,x) derivative(t,x,U,auxdata),auxdata.t,auxdata.X0,...
    odeset('RelTol',1e-13));

end

%--------------------------------------------------------------------------
% nonlinear state derivative function
function Dx = derivative(t,x,U,auxdata)

% extract states
r = x(1); T = x(2); vr = x(3); vT = x(4);

% compute continuous control signal using spline interpolation
phi = interp1(auxdata.t,U,t,'spline');

% compute state derivatives
Dr = vr;
DT = vT/r;
Dvr = vT^2/r - auxdata.a/r^2 + auxdata.T/(auxdata.m0 - auxdata.Dm*t)*sin(phi);
DvT = -vr*vT/r + auxdata.T/(auxdata.m0 - auxdata.Dm*t)*cos(phi);

% combine
Dx = [Dr; DT; Dvr; DvT];

end

%--------------------------------------------------------------------------
% plotting code for states and control
% (not the main content)
function plot_example(r,T,To,U,auxdata)

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
LineWidth = 1;
MarkerSize = 12;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth,'MarkerSize',MarkerSize};

% initialize figure for plotting theta and r states
hf = figure; hf.Color = 'w'; clf;

polarplot(T,r,'linewidth',2*LineWidth,'color',nicered);

ha = gca; ha.ThetaColor = 'k'; ha.RColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

% initialize figure for plotting control
hf = figure; hf.Color = 'w'; hold on;

plot(To,rad2deg(interp1(auxdata.t,U,To,'spline')),plotOpts{:},'Color',niceblue)
plot(auxdata.t,rad2deg(U),'.k',plotOpts{:})

xlabel('Time')
ylabel('Control [deg]')
legend('Interpolated u(t)','U','Location','best')

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

hl = legend(); hl.Location = "best"; hl.FontSize = FontSize;

end