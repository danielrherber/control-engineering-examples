% ex_observer.m
% illustration of eigenvalue assignment for a full-order and reduced-order
% observer using the Matlab place command
% [reference] Example 4.20 Reduced Order Observer for a Third Order System
% in LSC
% [course] Session 5 - Linear Control (2)
close all; clear; clc

% system matrices
A = [-0.14 0.33 -0.33;
    0.1 -0.28 0;
    0 1.7 -0.77];
B = [0; 0; -0.025];
C = [2 0 0];

% number of states
n = length(A);

% simulation options
TFINAL = 10;
T = linspace(0,TFINAL,10000)';
U = ones(size(T));

%--------------------------------------------------------------------------
% full-order observer
%--------------------------------------------------------------------------
% desired eigenvalues
E1 = [-3.5 + 1i*3.5 -3.5 - 1i*3.5 -3.5];

% gain matrix for full-order observer
L1 = place(A',C',E1)';

% closed-loop system matrices
Anew = [A zeros(n); L1*C A-L1*C];
Bnew = [B;B];
Cnew = blkdiag(C,eye(n));
sysNew = ss(Anew,Bnew,Cnew,[]);

% initial states
X0 = [0 0 0 -0.03 0.01 0.02]';

% simulate
[Y,T,X] = lsim(sysNew,U,T,X0);

% plot the simulation results (see function below)
plot_example(T,X(:,1:3),Y(:,2:4),1)

%--------------------------------------------------------------------------
% reduced-order observer
%--------------------------------------------------------------------------
% reduce-order observer matrices
C1 = 2;
A11 = A(1,1);
A12 = A(1,2:3);
A21 = A(2:3,1);
A22 = A(2:3,2:3);
B1 = B(1,:);
B2 = B(2:3,:);

% desired eigenvalues
E2 = [-3.5 + 1i*3.5 -3.5 - 1i*3.5];

% gain matrix for reduced-order observer
L2 = place(A22',(C1*A12)',E2)';

% number of states and outputs
r = size(C1,1);

% closed-loop matrices
A31 = A21;
A32 = L2*C1*A12;
A33 = A22 - L2*C1*A12;
Aa = [A zeros(n,n-r)];
Ab = [A31,A32,A33];
Anew = [Aa;Ab];
Bnew = [B;B2];
Cnew = [C zeros(1,n-r); zeros(n-r,n) eye(n-r)];
sysNew = ss(Anew,Bnew,Cnew,[]);

% initial states
X0 = [0 0 0 0.01 0.02]';

% simulate
[Y,T,X] = lsim(sysNew,U,T,X0);

% plot the simulation results (see function below)
plot_example(T,X(:,1:3),Y(:,2:3),2)

%--------------------------------------------------------------------------
% plotting code
% (not the main content)
function plot_example(T,X,Y,mycase)

% colors and other parameters
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
LineWidth = 1;
MarkerSize = 12;
FontSize = 12;
plotOpts = {'LineWidth',LineWidth,'MarkerSize',MarkerSize};

% initialize figure
hf = figure; hf.Color = 'w'; hold on

plot(T,X,plotOpts{:},'Color',nicered,'DisplayName','Actual States')
plot(T,Y,plotOpts{:},'Color',niceblue,'DisplayName','Estimated States')

xlabel('Time [s]')
ylabel('States')

ha = gca; ha.XColor = 'k'; ha.YColor = 'k'; ha.LineWidth = 1; ha.FontSize = FontSize;

hl = legend(); hl.Location = "best";

switch mycase
    case 1
        title('Full-Order Observer')
    case 2
        title('Reduced-Order Observer')
end

end