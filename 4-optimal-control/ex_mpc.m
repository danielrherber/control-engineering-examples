% ex_mpc.m
% example of linear-quadratic model predictive control applied for the
% lateral control of a car with nonlinear simulation comparisons
% [reference] Based on Lateral Control of a Car - MPC from Lecture 14 of
% EE392m - Spring 2005 slides by Dimitry Gorinevsky
% [course] Session 11 - Optimal Control (3)
close all; clear; clc

% problem parameters
N = 20; % number of points in the prediction horizon
R = 1; % control effort penalty
V = 22.352; % speed, 50 mph to m/s
Ts = 200/1000; % sample time, seconds
Y = 2; % initial lateral error, m
A = deg2rad(0); % initial angle, deg to rad
M = deg2rad(1); % maximum angle change, deg/s to rad/s

% initial control value (while first problem is being solved)
U(1) = 0;

% initial states
Xk = [Y;A];

% discretization points
T = 0:Ts:12;

% nonlinear simulation options
ode_options = odeset('RelTol',1e-13,'AbsTol',1e-14);

% go through all discretization points
for k = 1:length(T)-1

    % current time horizon
    TSPAN = [T(k),T(k+1)];

    % nonlinear simulation over current time horizon with previous control
    [Tsim{k},Xsim{k}] = ode45(@(t,y) derivative(t,y,V,U(k)),TSPAN,Xk,ode_options);
    disp("Plant applying previous control")

    % solve QP for next control value
    [u,sol{k}] = solveQP(N,R,V,Ts,Xk(1),Xk(2),M,U(k),k);
    disp(strcat("Solved QP with next u = ",string(rad2deg(u))))

    % end of simulation is next initial states
    Xk = Xsim{k}(end,:)';

    % set new optimal control value as the next one to be applied
    U(k+1) = u;

    % display end of the current time horizon
    disp(strcat("Time is now ", string(TSPAN(end))))
    disp("----------")

end

% plot actual control and some individual MPC QP solutions
plot_example_control(sol,T,U)

% plot actual output and some individual MPC QP solutions
plot_example_output(Tsim,Xsim,sol)

%--------------------------------------------------------------------------
% nonlinear state derivative function
function xd = derivative(t,x,V,u)

% extract states
y = x(1);
a = x(2);

% output time derivatives
xd(1,1) = V*sin(a);
xd(2,1) = u;

end

%--------------------------------------------------------------------------
% construct and solve the quadratic program (QP) for the next control value
function [uout,sol] = solveQP(N,R,V,Ts,Y,A,M,U0,int)

% optimization variable ordering is [y,a,u]
% number of states, controls, and optimization variables
nx = 2;
nu = 1;
np = (nx+nu)*(N+1);

% hessian
H = zeros(np); % initialize
for k = 1:N+1
    H(k,k) = 1; % for y(k)^2
    H(2*(N+1)+k,2*(N+1)+k) = R; % for R*u(k)^2
end

% initialize equality constraints
Aeq = zeros(nx*N,np);
beq = zeros(nx*N,1);

% state 1 (y) defect equality constraints
for k = 1:N
    Aeq(k,0*(N+1)+k) = 1; % y(k)
    Aeq(k,1*(N+1)+k) = V*Ts; % a(k)
    Aeq(k,2*(N+1)+k) = 0.5*V*Ts^2; % u(k)
    Aeq(k,0*(N+1)+k+1) = -1; % y(k+1)
end

% state 2 (a) defect equality constraints
for k = 1:N
    % Aeq(N+k,0*(N+1)+k) = 0; % y(k), not needed
    Aeq(N+k,1*(N+1)+k) = 1; % a(k)
    Aeq(N+k,2*(N+1)+k) = Ts; % u(k)
    Aeq(N+k,1*(N+1)+k+1) = -1; % a(k+1)
end

% initial values equality boundary constraints
Aeq(end+1,0*(N+1)+1) = 1; % y(0)
beq(end+1,1) = Y;
Aeq(end+1,1*(N+1)+1) = 1; % a(0)
beq(end+1,1) = A;
Aeq(end+1,2*(N+1)+1) = 1; % u(0)
beq(end+1,1) = U0;

% initial "no" simple bound path constraints on the optimization variables
LB = -inf(np,1);
UB = inf(np,1);

% steering rate simple bound path constraints
LB(2*(N+1)+1:3*(N+1)) = -M;
UB(2*(N+1)+1:3*(N+1)) = M;

% [try this] no negative lateral displacement, can give infeasible problems
% LB(0*(N+1)+1:1*(N+1)) = 0;

% optimization algorithm options
OPTIONS = optimoptions('quadprog');
OPTIONS.Display = "none";
OPTIONS.OptimalityTolerance = 1e-10;
OPTIONS.StepTolerance = 1e-10;
OPTIONS.ConstraintTolerance = 1e-10;

% remove final control value as it is not needed (would be 0)
H(end,:) = [];
H(:,end) = [];
Aeq(:,end) = [];
LB(end) = [];
UB(end) = [];

% solve the quadratic program
X = quadprog(H,[],[],[],Aeq,beq,LB,UB,[],OPTIONS);

% extract the optimal solution
Y_ = X(0*(N+1)+1:1*(N+1));
A_ = X(1*(N+1)+1:2*(N+1));
U_ = X(2*(N+1)+1:3*(N+1)-1);

% store the solution
sol.T = (int-1)*Ts + (0:N)*Ts;
sol.Y = Y_;
sol.A = A_;
sol.U = U_;

% final control value to use
uout = U_(2); % u(1)

end

%--------------------------------------------------------------------------
% plotting code for control
% (not the main content)
function plot_example_control(sol,T,Uall)

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
nicegray = [170, 170, 170]/255;
LineWidth = 1;
MarkerSize = 18;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth,'MarkerSize',MarkerSize};

% initialize figure
hf = figure; hf.Color = 'w'; hold on

stairs(T,rad2deg(Uall),plotOpts{:},'Color','k')

% plot 1st MPC solution
try
ind = 1;
plot(sol{ind}.T(1:end-1),rad2deg(sol{ind}.U),'.',plotOpts{:},'Color',nicegray)
plot(sol{ind}.T(2),rad2deg(sol{ind}.U(2)),'.',plotOpts{:},'Color',nicered)
plot(sol{ind}.T(2:3),[rad2deg(sol{ind}.U(2)),rad2deg(sol{ind}.U(2))],'-',plotOpts{:},'Color',nicered)
end

% plot 17th MPC solution
try
ind = 17;
plot(sol{ind}.T(2),rad2deg(sol{ind}.U(2)),'.',plotOpts{:},'Color',niceblue)
plot(sol{ind}.T(2:3),[rad2deg(sol{ind}.U(2)),rad2deg(sol{ind}.U(2))],'-',plotOpts{:},'Color',niceblue)
end

xlabel('Time [sec]')
ylabel('Steering Rate [deg/s]')

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

end

%--------------------------------------------------------------------------
% plotting code for output
% (not the main content)
function plot_example_output(Tsim,Xsim,sol)

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
nicegreen = [109, 195, 80]/255;
LineWidth = 1;
MarkerSize = 18;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth,'MarkerSize',MarkerSize};

% initialize figure
hf = figure; hf.Color = 'w'; hold on

% plot 1st MPC solution
try
ind = 1;
plot(sol{ind}.T,sol{ind}.Y,'.',plotOpts{:},'Color',nicered)
end

% plot 17th MPC solution
try
ind = 17;
plot(sol{ind}.T,sol{ind}.Y,'.',plotOpts{:},'Color',niceblue)
end

% plot nonlinear simulation result
Tall = vertcat(Tsim{:});
Xall = vertcat(Xsim{:});
plot(Tall,Xall(:,1),plotOpts{:},'Color','k')

xlim([min(Tall) max(Tall)])

xlabel('Time [sec]')
ylabel('Lateral Displacement [m]')

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

% go through and plot each MPC solution
for k = 1:length(sol)

    hp = plot(sol{k}.T,sol{k}.Y,'.',plotOpts{:},'Color',nicegreen);

    % pause for some time
    pause(0.2)

    % delete old line
    delete(hp)

end

end