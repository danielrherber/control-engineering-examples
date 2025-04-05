% ex_direct_transcription.m
% example of a single-step direct transcription method using the
% trapezoidal rule for the defect constraints and fmincon to solve the
% finite-dimensional optimization problem, which is the open-loop control
% design of a constant-thrust rocket from a given initial circular orbit
% to the largest possible circular orbit
% [reference] pp. 66-69 in Applied Optimal Control
% [reference] Moyer and Pinkham (1964), doi: 10.1016/b978-1-4831-9812-5.50008-4
% [reference] https://github.com/danielrherber/dt-qp-project/tree/master/examples/nonlin/transfer-max-radius
% [course] Session 11 - Optimal Control (3)
close all; clear; clc

% problem parameters
auxdata.a = 1; % gravitational constant of the attracting center
auxdata.m0 = 1; % initial mass
auxdata.Dm = 0.0749; % constant mass loss rate
auxdata.T = 0.1405; % rocket thrust
r0 = 1; T0 = 0; vr0 = 0; vT = 1; % initial states
X0 = [r0;T0;vr0;vT];
auxdata.X0 = X0; % initial state vector
auxdata.vrf = 0;  % final vr
tf = 3.32; % final time

% mesh or time grid
mesh_opt = 1;
switch mesh_opt
    %----------------------------------------------------------------------
    case 1 % equidistant time grid with N+1 = 21
        auxdata.nt = 21; % number of time points
        auxdata.t = linspace(0,tf,auxdata.nt)'; % equidistant time grid
    %----------------------------------------------------------------------
    case 2 % equidistant time grid with N+1 = 101
        auxdata.nt = 101; % number of time points
        auxdata.t = linspace(0,tf,auxdata.nt)'; % equidistant time grid
    %----------------------------------------------------------------------
    case 3
        d = 0.5; n_add = 0;
        auxdata.t = unique([linspace(0,tf/2-d,5+n_add),...
            linspace(tf/2-d,tf/2+d,9+n_add),linspace(tf/2+d,tf,5+n_add)]'); % irregular time grid
        auxdata.nt = length(auxdata.t); % number of time points
    %----------------------------------------------------------------------
end

% step sizes
auxdata.h = diff(auxdata.t);

% number of controls and states
auxdata.nu = 1;
auxdata.nx = 4;

% discretized variable indices in P = [x1,x2,x3,x4,u1] = [r,T,vr,vT,phi];
nt = auxdata.nt;
auxdata.r_ind = (0*nt)+1:(1*nt);
auxdata.T_ind = (1*nt)+1:(2*nt);
auxdata.vr_ind = (2*nt)+1:(3*nt);
auxdata.vT_ind = (3*nt)+1:(4*nt);
auxdata.phi_ind = (4*nt)+1:(5*nt);

% initial guess for all optimization variables (all zeros)
guess_opt = 2;
switch guess_opt
    %----------------------------------------------------------------------
    case 1 % all ones (bad guess)
        P0 = ones(nt*(auxdata.nu+auxdata.nx),1);
    %----------------------------------------------------------------------
    case 2 % linear (good guess)
        P0 = ones(nt*(auxdata.nu+auxdata.nx),1);
        P0(auxdata.r_ind) = linspace(X0(1),X0(1),nt)';
        P0(auxdata.T_ind) = linspace(X0(2),X0(2),nt)';
        P0(auxdata.vr_ind) = linspace(X0(3),X0(3),nt)';
        P0(auxdata.vT_ind) = linspace(X0(4),X0(4),nt)';
        P0(auxdata.phi_ind) = linspace(0,2*pi,nt)';
    %----------------------------------------------------------------------
end

% fmincon options
options = optimoptions(@fmincon,'display','iter','MaxFunEvals',5e5);

% solve the optimization problem
P = fmincon(@(P) objective(P,auxdata),P0,[],[],[],[],[],[],...
    @(P) constraints(P,auxdata),options);

% extract controls from P
U = P(auxdata.phi_ind);

% obtain the optimal solution (on a more dense time grid)
[To,Xo] = ode45(@(t,x) derivative_simulation(t,x,U,auxdata),[0 tf],auxdata.X0,...
    odeset('RelTol',1e-13));

% plot results
plot_example(P,To,Xo,auxdata)

%--------------------------------------------------------------------------
% compute the objective function
function J = objective(P,auxdata)

% extract states and controls from P
r = P(auxdata.r_ind);
T = P(auxdata.T_ind);
vr = P(auxdata.vr_ind);
vT = P(auxdata.vT_ind);
U = P(auxdata.phi_ind);

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

% extract states and controls from P
r = P(auxdata.r_ind);
T = P(auxdata.T_ind);
vr = P(auxdata.vr_ind);
vT = P(auxdata.vT_ind);
U = P(auxdata.phi_ind);

% discretized state matrices
X = [r(:),T(:),vr(:),vT(:)];

% add defect constraints for each time segment
zeta = zeros(auxdata.nt-1,auxdata.nx); % initialize
for k = 1:auxdata.nt-1

    % various quantities on the segment boundaries
    tk = auxdata.t(k);
    tk1 = auxdata.t(k+1);
    Xk = X(k,:);
    Xk1 = X(k+1,:);
    Uk = U(k,:);
    Uk1 = U(k+1,:);

    % state derivatives
    fk = derivative(tk,Xk,Uk,auxdata)';
    fk1 = derivative(tk1,Xk1,Uk1,auxdata)';

    % defect constraints
    zeta(k,:) = Xk1 - Xk - auxdata.h(k)/2*( fk + fk1 );

end

% boundary (initial state) constraints
h3 = X(1,:) - auxdata.X0';

% boundary (final state) constraints
h1 = vr(end) - auxdata.vrf;
h2 = vT(end) - sqrt(auxdata.a/r(end));

% path constraints
g1 = 0 - U; % U >= 0
g2 = U - 2*pi; % U <= 2*pi

% combine constraints
g = [g1(:),g2(:)];
h = [h1(:);h2(:);h3(:);zeta(:)];

end

%--------------------------------------------------------------------------
% nonlinear state derivative function
function Dx = derivative(t,x,U,auxdata)

% extract states
r = x(1); T = x(2); vr = x(3); vT = x(4);

% control
phi = U;

% compute derivatives
Dr = vr;
DT = vT/r;
Dvr = vT^2/r - auxdata.a/r^2 + auxdata.T/(auxdata.m0 - auxdata.Dm*t)*sin(phi);
DvT = -vr*vT/r + auxdata.T/(auxdata.m0 - auxdata.Dm*t)*cos(phi);

% combine
Dx = [Dr; DT; Dvr; DvT];

end

%--------------------------------------------------------------------------
% nonlinear state derivative function (spline for control)
function Dx = derivative_simulation(t,x,U,auxdata)

% extract states
r = x(1); T = x(2); vr = x(3); vT = x(4);

% compute control using linear interpolation
phi = interp1(auxdata.t,U,t,'linear');

% compute derivatives
Dr = vr;
DT = vT/r;
Dvr = vT^2/r - auxdata.a/r^2 + auxdata.T/(auxdata.m0 - auxdata.Dm*t)*sin(phi);
DvT = -vr*vT/r + auxdata.T/(auxdata.m0 - auxdata.Dm*t)*cos(phi);

% combine
Dx = [Dr; DT; Dvr; DvT];

end

%--------------------------------------------------------------------------
% plotting code for states, control, and state errors
% (not the main content)
function plot_example(P,To,Xo,auxdata)

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
LineWidth = 1;
MarkerSize = 12;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth,'MarkerSize',MarkerSize};

% extract states and controls from P
r = P(auxdata.r_ind);
T = P(auxdata.T_ind);
vr = P(auxdata.vr_ind);
vT = P(auxdata.vT_ind);
U = P(auxdata.phi_ind);

% extract states
r_ = Xo(:,1); T_ = Xo(:,2); vr_ = Xo(:,3); vT_ = Xo(:,4);

%--------------------------------------------------------------------------
% initialize figure for plotting theta and r states
hf = figure; hf.Color = 'w'; clf;

polarplot(T_,r_,plotOpts{:},'color',niceblue,'DisplayName','ode45'); hold on
polarplot(T,r,'.',plotOpts{:},'color',nicered,'DisplayName','DT');

ha = gca; ha.ThetaColor = 'k'; ha.RColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

hl = legend(); hl.Location = "best"; hl.FontSize = FontSize;

%--------------------------------------------------------------------------
% initialize figure for plotting control
hf = figure; hf.Color = 'w'; hold on;

plot(To,rad2deg(interp1(auxdata.t,U,To,'spline')),plotOpts{:},'Color',niceblue)
plot(auxdata.t,rad2deg(U),'.k',plotOpts{:})

xlabel('Time')
ylabel('Control [deg]')
legend('Interpolated u(t)','U','Location','best')

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

hl = legend(); hl.Location = "best"; hl.FontSize = FontSize;

%--------------------------------------------------------------------------
% initialize figure for plotting state errors
hf = figure; hf.Color = 'w'; hold on;

% obtain the optimal solution (original time grid)
[~,Xo2] = ode45(@(t,x) derivative_simulation(t,x,U,auxdata),auxdata.t,auxdata.X0,...
    odeset('RelTol',1e-13)); % run a simulation

plot(auxdata.t,abs(Xo2(:,1)-r)+eps(1),plotOpts{:},'DisplayName','r error');
plot(auxdata.t,abs(Xo2(:,2)-T)+eps(1),plotOpts{:},'DisplayName','T error');
plot(auxdata.t,abs(Xo2(:,3)-vr)+eps(1),plotOpts{:},'DisplayName','vr error');
plot(auxdata.t,abs(Xo2(:,4)-vT)+eps(1),plotOpts{:},'DisplayName','vT error');

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;
ha.YScale = 'log';

xlabel('Time')
ylabel('State Errors')

ylim([1e-10 1e0])

hl = legend(); hl.Location = "best"; hl.FontSize = FontSize;

end