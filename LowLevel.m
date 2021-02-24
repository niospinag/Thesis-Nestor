% point stabilization + Multiple shooting + obstacle avoidance
clear all
close all
clc

% CORREGIR MAKE STEP IN SYSTEM




% CasADi v3.4.5
addpath('C:\Users\nesto\OneDrive\Escritorio\casadi-windows-matlabR2016a-v3.5.5')
import casadi.*


n_x = 3;
n_u = 2;


%  sys = System2(A,B,C,D)
sys = System;

rand('seed',1234);


%% simulate and plot system response
sys = sys.reset(zeros(3,1));

u_max = 20;

u0 = zeros(2, 1);
for k = 1:100
    if (k < 50)

        u0 = random_u(u0, u_max);
    else
        u0 = zeros(2,1);
    end
    sys = sys.make_step(u0);
    
    end
subplot(2,1,1);
plot(sys.time_, sys.y_)

subplot(2,1,2);
plot(sys.time_, sys.u_)

%% initialize
T_ini = 4;
N = 40;

L = T_ini + N;

T = 150;

n_u = sys.n_u;
n_y = sys.n_y;



U_L = [];
Y_L = [];

u0 = zeros(2,1);

for k = 1:T
    x0 = randn(3,1);
    sys = sys.reset(x0);
    for kk = 1:L
        u0 = random_u(u0, u_max);
        sys = sys.make_step(u0);
    end
    U_L = [U_L reshape(sys.u_,[],1)];
    Y_L = [Y_L reshape(sys.y_,[],1)];
end


U_Tini = U_L(1:n_u*T_ini,:);  U_N = U_L(n_u*T_ini+1:end,:);
Y_Tini = Y_L(1:n_y*T_ini,:);  Y_N = Y_L(n_y*T_ini+1:end,:);


%% trajectories
% Sampled trajectories

n_traj = 8;
n_rows = 2;

% fig, ax = plt.subplots(2, n_traj//n_rows, figsize=(12,6), sharex=True, sharey=True)
subplot(2,n_traj/n_rows, 1)


for k = 1:n_traj
    i = mod(k,n_rows);
    j = k/n_rows; % n_rows, k//n_rows
    subplot(2,n_traj/n_rows,k);
    plot(Y_L);
    plot(reshape(Y_L(:,k),[],n_y));
end   
%%///////////////////////////////////////////////////////////////////////////
%%  DEEPC
%variables a optimizar
g = MX.sym('g',T);
u_N = MX.sym('u_N',n_u,N);
y_N = MX.sym('y_N',n_y,N);

u_Tini = MX.sym('u_Tini',n_u ,T_ini);
y_Tini = MX.sym('y_Tini',n_y ,T_ini);


% 
% opt_x_dpc = struct_symMX([entry('g', shape=(T)), entry('u_N', shape=(n_u), repeat=N),
%     entry('y_N', shape=(n_y), repeat=N)
% ])
% 
% opt_p_dpc = struct_symMX([
%     entry('u_Tini', shape=(n_u), repeat=T_ini),
%     entry('y_Tini', shape=(n_y), repeat=T_ini),
% ])
% 
% opt_x_num_dpc = opt_x_dpc(0)
% opt_p_num_dpc = opt_p_dpc(0)

%% Create the objective:
obj = 0;
for k = 1:N
    obj = obj + sum(y_N(:,k)'*y_N(:,k))+0.1*sum(u_N(:,k)'*u_N(:,k));
end
    
% Create the constraints
A = [U_Tini; U_N; Y_Tini; Y_N];
b = vertcat(reshape(u_Tini,n_u*T_ini,1), reshape(u_N,n_u*N,1), reshape(y_Tini,n_y*T_ini,1), reshape(y_N,n_y*N,1));


cons = A*g-b;

% make the decision variables one column vector
OPT_variables = vertcat(g, reshape(u_N,n_u*N,1), reshape(y_N,n_y*N,1));
OPT_param = vertcat(reshape(u_Tini,n_u*T_ini,1), reshape(y_Tini,n_y*T_ini,1));
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', cons, 'p', OPT_param);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

args = struct;

% inequality constraints (state constraints)
args.lbg = 0;  % lower bound of the states x and y
args.ubg = 0;   % upper bound of the states x and y 


% input constraints
args.lbx(1:T,1) = -inf; args.lbx(T+1:T+n_u*N,1)   = -u_max; args.lbx(T+n_u*N+1:T+n_u*N+n_y*N,1) = -inf;
args.ubx(1:T,1) =  inf; args.ubx(T+1:T+n_u*N,1)   = u_max;  args.ubx(T+n_u*N+1:T+n_u*N+n_y*N,1) = inf;






%% open loop comparison

rand('seed',12);
sys.reset(zeros(3,1));

%% Excitement
n_exc = 20;
u0 = zeros(2,1);
for k = 1:n_exc
    u0 = random_u(u0, u_max);
    sys.make_step(u0);
end

shape_Y = size(sys.y_,2);   
shape_U = size(sys.u_,2);   
y_Tini = sys.y_(:,shape_Y-T_ini+1:end);
u_Tini = sys.u_(:,shape_U-T_ini+1:end);

% 
% opt_p_num['y_Tini'] = vertsplit(y_Tini)
% opt_p_num['u_Tini'] = vertsplit(u_Tini)
% 
% opt_p_num_dpc['y_Tini'] = vertsplit(y_Tini)
% opt_p_num_dpc['u_Tini'] = vertsplit(u_Tini)

% %% DeePC
args.p   = [reshape(u_Tini,n_u*T_ini,1); reshape(y_Tini,n_y*T_ini,1)]; % set the values of the parameters vector
% args.x0 = reshape(u0',2*N,1); % initial value of the optimization variables

sol = solver( 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);    
% r = S_dpc(p=opt_p_num_dpc, lbg=0, ubg=0, lbx=lbx_dpc, ubx=ubx_dpc)
% % Extract solution
% opt_x_num_dpc.master = r['x'] 
% u_N_dpc = horzcat(opt_x_num_dpc['u_N']).full().T
% y_N_dpc = horzcat(opt_x_num_dpc['y_N']).full().T
u_N_dpc = reshape(full(sol.x(T+1:T+n_u*N))',2,[]);
y_N_dpc = reshape(full(sol.x(T+n_u*N+1:end))',3,[]);

t = 1:N;
subplot(2,1,1)
plot(u_N_dpc')

subplot(2,1,2)
plot(y_N_dpc')

