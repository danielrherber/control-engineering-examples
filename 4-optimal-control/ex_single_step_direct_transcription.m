% ex_single_step_direct_transcription.m
% example of a single-step direct transcription method using the
% trapezoidal rule for the defect constraints and fmincon to solve the
% finite-dimensional optimization problem
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

% time grid or mesh
mesh_opt = 1;
switch mesh_opt
    case 1 % equidistant time grid with N = 10
        auxdata.nt = 21; % number of time points
        auxdata.t = linspace(0,tf,auxdata.nt)'; % equidistant time grid
    case 2 % equidistant time grid with N = 21
        auxdata.nt = 101; % number of time points
        auxdata.t = linspace(0,tf,auxdata.nt)'; % equidistant time grid
    case 3
        d = 0.3; n_add = 0;
        auxdata.t = unique([linspace(0,tf/2-d,7+n_add),...
            linspace(tf/2-d,tf/2+d,11+n_add),linspace(tf/2+d,tf,7+n_add)]'); % irregular time grid
        auxdata.nt = length(auxdata.t); % number of time points
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
    case 1 % all ones (bad guess)
        P0 = ones(nt*(auxdata.nu+auxdata.nx),1);
    case 2 % linear (good guess)
        P0 = ones(nt*(auxdata.nu+auxdata.nx),1);
        P0(auxdata.r_ind) = linspace(X0(1),X0(1),nt)';
        P0(auxdata.T_ind) = linspace(X0(2),X0(2),nt)';
        P0(auxdata.vr_ind) = linspace(X0(3),X0(3),nt)';
        P0(auxdata.vT_ind) = linspace(X0(4),X0(4),nt)';
        P0(auxdata.phi_ind) = linspace(0,2*pi,nt)';
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
plot_helper(P,To,Xo,auxdata)

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
% plot function
function plot_helper(P,To,Xo,auxdata)

% extract states and controls from P
r = P(auxdata.r_ind);
T = P(auxdata.T_ind);
vr = P(auxdata.vr_ind);
vT = P(auxdata.vT_ind);
U = P(auxdata.phi_ind);

% extract states
r_ = Xo(:,1); T_ = Xo(:,2); vr_ = Xo(:,3); vT_ = Xo(:,4);

% plot theta and r states
hf = figure; hf.Color = 'w'; clf;

polarplot(T_,r_,'linewidth',2,'color','r','DisplayName','ode45'); hold on
polarplot(T,r,'.','markersize',16,'color','g','DisplayName','DT');

legend('Location','best')

% plot control
hf = figure; hf.Color = 'w'; hold on;

plot(To,rad2deg(interp1(auxdata.t,U,To,'linear')),'r','linewidth',2)
plot(auxdata.t,rad2deg(U),'.k','markersize',16)

xlabel('Time')
ylabel('Control')
legend('Interpolated u(t)','U','Location','best')

% obtain the optimal solution (original time grid)
[~,Xo2] = ode45(@(t,x) derivative_simulation(t,x,U,auxdata),auxdata.t,auxdata.X0,...
    odeset('RelTol',1e-13)); % run a simulation

% plot state error
hf = figure; hf.Color = 'w'; hold on;

plot(auxdata.t,abs(Xo2-[r,T,vr,vT]),'linewidth',2);

ha = gca; ha.YScale = 'log';
xlabel('Time')
ylabel('State Errors')

end