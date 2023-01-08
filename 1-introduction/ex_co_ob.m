close all; clear; clc

% sample LTI system
A = [2 3 2 1; -2 -3 0 0; -2 -2 -4 0; -2 -2 -2 -5];
B = [1;-2;2;-1];
C = [7 6 4 2];
D = 0;

% create state-space model
sys = ss(A,B,C,D)

% controllability gramian
Wc = gram(sys,'c')

% observability gramian
Wo = gram(sys,'o')

% controllability matrix (direct)
Mc = [B A*B A^2*B A^3*B]
rank(Mc)

% controllability matrix (matlab)
Mc2 = ctrb(sys)

% controllable/reachable subspace decomposition
[ABAR,BBAR,CBAR,T,K] = ctrbf(sys.A,sys.B,sys.C)

% observability matrix (matlab)
Mo = obsv(sys)
rank(Mo)

% observable subspace decomposition
[ABAR,BBAR,CBAR,T,K] = obsvf(sys.A,sys.B,sys.C)