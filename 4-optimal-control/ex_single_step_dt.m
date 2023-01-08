function single_step_dt_ex

close all; clc

% NOTE: using P to denote the optimization variables

% problem parameters
auxdata.a = 1; auxdata.m0 = 1; auxdata.Dm = 0.0749; auxdata.T = 0.1405;

% fixed initial states, final states, and final time
r0 = 1; T0 = 0; vr0 = 0; vT = 1;
auxdata.vrf = 0;
tf = 3.32;
auxdata.X0 = [r0;T0;vr0;vT];

% shooting parameters
auxdata.nt = 30; % number of time points
auxdata.t = linspace(0,tf,auxdata.nt)'; % time mesh/grid
auxdata.h = diff(auxdata.t);
auxdata.nu = 1; % number of controls
auxdata.nx = 4; % number of states

% discretized variable indices in p = [x1,x2,x3,x4,u1] = [r,T,vr,vT,phi];
nt = auxdata.nt;
auxdata.r_ind = (0*nt)+1:(1*nt);
auxdata.T_ind = (1*nt)+1:(2*nt);
auxdata.vr_ind = (2*nt)+1:(3*nt);
auxdata.vT_ind = (3*nt)+1:(4*nt);
auxdata.phi_ind = (4*nt)+1:(5*nt);

% initial guess for all optimization variables (all zeros)
p0 = ones(nt*(auxdata.nu+auxdata.nx),1);

% fmincon options
options = optimoptions(@fmincon,'display','iter','MaxFunEvals',5e5,...
    'UseParallel',false,'OptimalityTolerance',1e-4);

% solve the optimization problem
P = fmincon(@(P) objective(P,auxdata),p0,[],[],[],[],[],[],...
    @(P) constraints(P,auxdata),options);

% extract states and controls from P
r = P(auxdata.r_ind);
T = P(auxdata.T_ind);
vr = P(auxdata.vr_ind);
vT = P(auxdata.vT_ind);
U = P(auxdata.phi_ind);

% obtain the optimal solution (on a more dense time grid)
[To_sim,Xo] = ode45(@(t,x) derivative_simulation(t,x,U,auxdata),[0 tf],auxdata.X0,...
    odeset('RelTol',1e-13)); % run a simulation
ro = Xo(:,1); To = Xo(:,2); vro = Xo(:,3); vTo = Xo(:,4); % extract states

% plot theta and r states
hf = figure; hf.Color = 'w'; clf;
polarplot(T,r,'linewidth',2,'color','g'); hold on
polarplot(To,ro,'linewidth',2,'color','r');

% plot control
hf = figure; hf.Color = 'w'; hold on;
plot(To_sim,rad2deg(interp1(auxdata.t,U,To_sim,'spline')),'r','linewidth',2)
plot(auxdata.t,rad2deg(U),'.k','markersize',16)
legend('interpolated U(t)','U')

% obtain the optimal solution (original time grid)
[~,Xo2] = ode45(@(t,x) derivative_simulation(t,x,U,auxdata),auxdata.t,auxdata.X0,...
    odeset('RelTol',1e-13)); % run a simulation

% plot state error
hf = figure; hf.Color = 'w'; hold on;
plot(auxdata.t,abs(Xo2-[r,T,vr,vT]));
ha = gca; ha.YScale = 'log';
ylabel('state errors')

end

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
g1 = U - pi; % phi <= pi, g(p) <= 0
g2 = -U - pi; % phi >= -pi

% combine constraints
g = [g1(:),g2(:)];
h = [h1(:);h2(:);h3(:);zeta(:)];

end

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

% nonlinear state derivative function (spline for control)
function Dx = derivative_simulation(t,x,U,auxdata)

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