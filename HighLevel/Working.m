%UNIVERSIDAD NACIONAL DE COLOMBIA
% Multi Vehicle automated drivring
%Autor: Nestor Ospina
clear
close all
clc

% %---------laptop asus
% addpath(genpath('C:\gurobi901\win64\matlab'))%GUROBI
% addpath(genpath('C:\Users\nesto\OneDrive\Documentos\YALMIP-master'))%yalmip

yalmip('clear')
%% PROGRAM
% Model data
nx = 1; % Number of agents
nu = 1; % Number of inputs
% MPC data
Q = 1 * eye(1);
R = 10 * eye(1);
N = 3; %horizon
T = 0.3; %[s]
Ds = 15; %Safety distance [m]
Dl = 35; %lateral distance
V_max = 80;
A_max = 30;
L = 6; %number of lanes
Mmax = L - 1;
mmin = -L + 1;
% p_max = 1;

%------------desired states-----------
Zd = sdpvar(1, 1); %carril deseado
Vd = sdpvar(1, 1); %velocidad deseada
% DS = sdpvar(1, 1); %velocidad deseada

% -------------local vehicle---------------
v = sdpvar(ones(1, N + 1), ones(1, N + 1)); %velocidad del vehiculo actual
a = sdpvar(ones(1, N), ones(1, N)); %aceleracion actual del vehiculo
z = intvar(ones(1, N + 1), ones(1, N + 1)); %carril actual
ll = binvar(ones(1, N), ones(1, N)); %paso izquierda
lr = binvar(ones(1, N), ones(1, N)); %paso derecha
% -------------- neighboor ---------------

v_12 = sdpvar(1, 1); v_13 = sdpvar(1, 1); %velocidad del otro vehculo
z_12 = sdpvar(1, 1); z_13 = sdpvar(1, 1); %carril del vehiculo j
v_14 = sdpvar(1, 1); v_15 = sdpvar(1, 1); %velocidad del otro vehculo
z_14 = sdpvar(1, 1); z_15 = sdpvar(1, 1); %carril del vehiculo j



% ------ distance between two vehicles ------
dis12 = sdpvar(ones(1, N + 1), ones(1, N + 1)); %distancia entre vehiculo 1 y 2
dz_12 = intvar(ones(1, N + 1), ones(1, N + 1)); %z2-z1

dis13 = sdpvar(ones(1, N + 1), ones(1, N + 1)); %distancia entre vehiculo 1 y 2
dz_13 = intvar(ones(1, N + 1), ones(1, N + 1)); %z2-z1

dis14 = sdpvar(ones(1, N + 1), ones(1, N + 1)); %distancia entre vehiculo 1 y 2
dz_14 = intvar(ones(1, N + 1), ones(1, N + 1)); %z2-z1

dis15 = sdpvar(ones(1, N + 1), ones(1, N + 1)); %distancia entre vehiculo 1 y 2
dz_15 = intvar(ones(1, N + 1), ones(1, N + 1)); %z2-z1



%% neihboorhod variables

% ##############vehiculo 2 ##############
a12 = binvar(ones(1, N), ones(1, N));
b12 = binvar(ones(1, N), ones(1, N));
ab12 = binvar(ones(1, N), ones(1, N));
n12 = binvar(ones(1, N), ones(1, N));
th12 = binvar(ones(1, N), ones(1, N));
f12 = sdpvar(ones(1, N), ones(1, N));
g12 = sdpvar(ones(1, N), ones(1, N));
h12 = sdpvar(ones(1, N), ones(1, N));

k12 = binvar(ones(1, N), ones(1, N));
del12 = binvar(ones(1, N), ones(1, N));
r1_12 = binvar(ones(1, N), ones(1, N));
kk12 = binvar(ones(1, N), ones(1, N));
dell12 = binvar(ones(1, N), ones(1, N));
r2_12 = binvar(ones(1, N), ones(1, N));
xl_12 = binvar(ones(1, N), ones(1, N));
xr_12 = binvar(ones(1, N), ones(1, N));
xlr_12 = binvar(ones(1, N), ones(1, N));
xrr_12 = binvar(ones(1, N), ones(1, N));
xa_12 = sdpvar(ones(1, N), ones(1, N));
xb_12 = sdpvar(ones(1, N), ones(1, N));
xc_12 = sdpvar(ones(1, N), ones(1, N));
xd_12 = sdpvar(ones(1, N), ones(1, N));

d12 = binvar(ones(1, N), ones(1, N));
u12 = binvar(ones(1, N), ones(1, N));
v12 = binvar(ones(1, N), ones(1, N));
x12 = binvar(ones(1, N), ones(1, N));
rd12 = binvar(ones(1, N), ones(1, N));
ps12 = binvar(ones(1, N), ones(1, N));
p12 = intvar(ones(1, N), ones(1, N));
s12 = intvar(ones(1, N), ones(1, N));
% ##############vehiculo 3 ##############
a13 = binvar(ones(1, N), ones(1, N));
b13 = binvar(ones(1, N), ones(1, N));
ab13 = binvar(ones(1, N), ones(1, N));
n13 = binvar(ones(1, N), ones(1, N));
th13 = binvar(ones(1, N), ones(1, N));
f13 = sdpvar(ones(1, N), ones(1, N));
g13 = sdpvar(ones(1, N), ones(1, N));
h13 = sdpvar(ones(1, N), ones(1, N));

k13 = binvar(ones(1, N), ones(1, N));
del13 = binvar(ones(1, N), ones(1, N));
r1_13 = binvar(ones(1, N), ones(1, N));
kk13 = binvar(ones(1, N), ones(1, N));
dell13 = binvar(ones(1, N), ones(1, N));
r2_13 = binvar(ones(1, N), ones(1, N));
xl_13 = binvar(ones(1, N), ones(1, N));
xr_13 = binvar(ones(1, N), ones(1, N));
xlr_13 = binvar(ones(1, N), ones(1, N));
xrr_13 = binvar(ones(1, N), ones(1, N));
xa_13 = sdpvar(ones(1, N), ones(1, N));
xb_13 = sdpvar(ones(1, N), ones(1, N));
xc_13 = sdpvar(ones(1, N), ones(1, N));
xd_13 = sdpvar(ones(1, N), ones(1, N));

d13 = binvar(ones(1, N), ones(1, N));
u13 = binvar(ones(1, N), ones(1, N));
v13 = binvar(ones(1, N), ones(1, N));
x13 = binvar(ones(1, N), ones(1, N));
rd13 = binvar(ones(1, N), ones(1, N));
ps13 = binvar(ones(1, N), ones(1, N));
p13 = intvar(ones(1, N), ones(1, N));
s13 = intvar(ones(1, N), ones(1, N));

% ##############vehiculo 4 ##############
a14 = binvar(ones(1, N), ones(1, N));
b14 = binvar(ones(1, N), ones(1, N));
ab14 = binvar(ones(1, N), ones(1, N));
n14 = binvar(ones(1, N), ones(1, N));
th14 = binvar(ones(1, N), ones(1, N));
f14 = sdpvar(ones(1, N), ones(1, N));
g14 = sdpvar(ones(1, N), ones(1, N));
h14 = sdpvar(ones(1, N), ones(1, N));

k14 = binvar(ones(1, N), ones(1, N));
del14 = binvar(ones(1, N), ones(1, N));
r1_14 = binvar(ones(1, N), ones(1, N));
kk14 = binvar(ones(1, N), ones(1, N));
dell14 = binvar(ones(1, N), ones(1, N));
r2_14 = binvar(ones(1, N), ones(1, N));
xl_14 = binvar(ones(1, N), ones(1, N));
xr_14 = binvar(ones(1, N), ones(1, N));
xlr_14 = binvar(ones(1, N), ones(1, N));
xrr_14 = binvar(ones(1, N), ones(1, N));
xa_14 = sdpvar(ones(1, N), ones(1, N));
xb_14 = sdpvar(ones(1, N), ones(1, N));
xc_14 = sdpvar(ones(1, N), ones(1, N));
xd_14 = sdpvar(ones(1, N), ones(1, N));

d14 = binvar(ones(1, N), ones(1, N));
u14 = binvar(ones(1, N), ones(1, N));
v14 = binvar(ones(1, N), ones(1, N));
x14 = binvar(ones(1, N), ones(1, N));
rd14 = binvar(ones(1, N), ones(1, N));
ps14 = binvar(ones(1, N), ones(1, N));
p14 = intvar(ones(1, N), ones(1, N));
s14 = intvar(ones(1, N), ones(1, N));


% ##############vehiculo 5 ##############
a15 = binvar(ones(1, N), ones(1, N));
b15 = binvar(ones(1, N), ones(1, N));
ab15 = binvar(ones(1, N), ones(1, N));
n15 = binvar(ones(1, N), ones(1, N));
th15 = binvar(ones(1, N), ones(1, N));
f15 = sdpvar(ones(1, N), ones(1, N));
g15 = sdpvar(ones(1, N), ones(1, N));
h15 = sdpvar(ones(1, N), ones(1, N));

k15 = binvar(ones(1, N), ones(1, N));
del15 = binvar(ones(1, N), ones(1, N));
r1_15 = binvar(ones(1, N), ones(1, N));
kk15 = binvar(ones(1, N), ones(1, N));
dell15 = binvar(ones(1, N), ones(1, N));
r2_15 = binvar(ones(1, N), ones(1, N));
xl_15 = binvar(ones(1, N), ones(1, N));
xr_15 = binvar(ones(1, N), ones(1, N));
xlr_15 = binvar(ones(1, N), ones(1, N));
xrr_15 = binvar(ones(1, N), ones(1, N));
xa_15 = sdpvar(ones(1, N), ones(1, N));
xb_15 = sdpvar(ones(1, N), ones(1, N));
xc_15 = sdpvar(ones(1, N), ones(1, N));
xd_15 = sdpvar(ones(1, N), ones(1, N));

d15 = binvar(ones(1, N), ones(1, N));
u15 = binvar(ones(1, N), ones(1, N));
v15 = binvar(ones(1, N), ones(1, N));
x15 = binvar(ones(1, N), ones(1, N));
rd15 = binvar(ones(1, N), ones(1, N));
ps15 = binvar(ones(1, N), ones(1, N));
p15 = intvar(ones(1, N), ones(1, N));
s15 = intvar(ones(1, N), ones(1, N));

%% making the optimizer longitudinal
constraints = [];
objective = 0;

for k = 1:N
    objective = objective + (v{k + 1} - Vd)' * Q * (v{k + 1} - Vd); % calculate obj
    % Feasible region
    constraints = [constraints, 0 <= v{k + 1} <= V_max, %no exceda las velocidades
                                -A_max <= a{k} <= A_max];

    constraints = [constraints, v{k + 1} == v{k} + T * a{k}]; %velocidad futura

    %################################ vehiculo 2 ################################
    constraints = [constraints, dis12{k + 1} == dis12{k} + T * (v_12 - v{k})];
    %................................... (12)...............................
    constraints = log_min(constraints, n12{k}, dz_12{k}, 0);
    constraints = log_may(constraints, th12{k}, dz_12{k}, 0);
    constraints = log_and(constraints, a12{k}, n12{k}, th12{k});
    %................................... (13)...............................
    constraints = log_may(constraints, b12{k}, dis12{k}, 0);
    %................................... (18)...............................
    constraints = log_and(constraints, ab12{k}, a12{k}, b12{k});
    %................................... (21)...............................
    constraints = log_imp(constraints, f12{k}, dis12{k}, ab12{k});
    %................................... (22)...............................
    constraints = log_imp(constraints, g12{k}, Ds, a12{k});
    %................................... (23)...............................
    constraints = log_imp(constraints, h12{k}, dis12{k}, a12{k});
    %................................... (24)...............................
    constraints = [constraints, -2 * f12{k} + g12{k} + h12{k} <= 0];

    
    % ################################ vehiculo 3 ################################ 
    constraints = [constraints, dis13{k + 1} == dis13{k} + T * (v_13 - v{k})];
    %................................... (13)...............................
    constraints = log_min(constraints, n13{k}, dz_13{k}, 0);
    constraints = log_may(constraints, th13{k}, dz_13{k}, 0);
    constraints = log_and(constraints, a13{k}, n13{k}, th13{k});
    %................................... (13)...............................
    constraints = log_may(constraints, b13{k}, dis13{k}, 0);
    %................................... (18)...............................
    constraints = log_and(constraints, ab13{k}, a13{k}, b13{k});
    %................................... (21)...............................
    constraints = log_imp(constraints, f13{k}, dis13{k}, ab13{k});
    %................................... (22)...............................
    constraints = log_imp(constraints, g13{k}, Ds, a13{k});
    %................................... (23)...............................
    constraints = log_imp(constraints, h13{k}, dis13{k}, a13{k});
    %................................... (24)...............................
    constraints = [constraints, -2 * f13{k} + g13{k} + h13{k} <= 0];
    

    % ################################ vehiculo 4 ################################ 
    constraints = [constraints, dis14{k + 1} == dis14{k} + T * (v_14 - v{k})];
    %................................... (14)...............................
    constraints = log_min(constraints, n14{k}, dz_14{k}, 0);
    constraints = log_may(constraints, th14{k}, dz_14{k}, 0);
    constraints = log_and(constraints, a14{k}, n14{k}, th14{k});
    %................................... (14)...............................
    constraints = log_may(constraints, b14{k}, dis14{k}, 0);
    %................................... (18)...............................
    constraints = log_and(constraints, ab14{k}, a14{k}, b14{k});
    %................................... (21)...............................
    constraints = log_imp(constraints, f14{k}, dis14{k}, ab14{k});
    %................................... (22)...............................
    constraints = log_imp(constraints, g14{k}, Ds, a14{k});
    %................................... (23)...............................
    constraints = log_imp(constraints, h14{k}, dis14{k}, a14{k});
    %................................... (24)...............................
    constraints = [constraints, -2 * f14{k} + g14{k} + h14{k} <= 0];
    
    
    
    % ################################ vehiculo 5 ################################ 
    constraints = [constraints, dis15{k + 1} == dis15{k} + T * (v_15 - v{k})];
    %................................... (15)...............................
    constraints = log_min(constraints, n15{k}, dz_15{k}, 0);
    constraints = log_may(constraints, th15{k}, dz_15{k}, 0);
    constraints = log_and(constraints, a15{k}, n15{k}, th15{k});
    %................................... (15)...............................
    constraints = log_may(constraints, b15{k}, dis15{k}, 0);
    %................................... (18)...............................
    constraints = log_and(constraints, ab15{k}, a15{k}, b15{k});
    %................................... (21)...............................
    constraints = log_imp(constraints, f15{k}, dis15{k}, ab15{k});
    %................................... (22)...............................
    constraints = log_imp(constraints, g15{k}, Ds, a15{k});
    %................................... (23)...............................
    constraints = log_imp(constraints, h15{k}, dis15{k}, a15{k});
    %................................... (24)...............................
    constraints = [constraints, -2 * f15{k} + g15{k} + h15{k} <= 0];
    
    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables

end

parameters_in = {Vd, v{1}, ...
                [dz_12{:}], v_12, dis12{1},...
                [dz_13{:}], v_13, dis13{1},...
                [dz_14{:}], v_14, dis14{1},...
                [dz_15{:}], v_15, dis15{1}}; 

solutions_out = {[a{:}], [v{:}], [dis12{:}],  [dis13{:}],[dis14{:}],[dis15{:}]}; 
% controller = optimizer(constraints, objective,sdpsettings('verbose',1),[x{1};r],u{1});
control_front = optimizer(constraints, objective, sdpsettings('solver', 'gurobi'), parameters_in, solutions_out);

%% making the optimizer lateral
constraints = [];
objective = 0;

for k = 1:N
    objective = objective + (z{k + 1} - Zd)' * R * (z{k + 1} - Zd); % calculate obj

    % Feasible region
    constraints = [constraints, z{k} - lr{k} <= z{k + 1},
                                                z{k + 1} <= z{k} + ll{k},
                                                1 <= z{k} <= L];
    constraints = [constraints, ll{k} + lr{k} <= 1];

    %  ################################ vehiculo 2 ################################ 
    constraints = [constraints, 1 <= z_12 <= L]; %tome valores posibles
    constraints = [constraints, mmin <= z_12 - z{k + 1} <= Mmax];
    %................................... (15.a)...............................
    constraints = log_min(constraints, k12{k}, z_12-z{k} , 1);
    constraints = log_may(constraints, del12{k}, z_12-z{k} , 1);
    constraints = log_and(constraints, r1_12{k}, k12{k} , del12{k} );
    %................................... (15.b)...............................
    constraints = log_min(constraints, kk12{k}, z_12-z{k} , -1);
    constraints = log_may(constraints, dell12{k}, z_12-z{k} , -1);
    constraints = log_and(constraints, r2_12{k}, kk12{k} , dell12{k} );
    %................................... (17)...............................
    constraints = log_min(constraints, u12{k}, dis12{k} , Dl);
    constraints = log_may(constraints, v12{k}, dis12{k} , -Dl);
    constraints = log_and(constraints, x12{k}, u12{k} , v12{k} );
    %................................... (30)...............................
    constraints = log_and(constraints, xl_12{k}, x12{k} , ll{k});
    constraints = log_and(constraints, xr_12{k}, x12{k} , lr{k});
    %................................... (31)...............................
    constraints = log_and(constraints, xlr_12{k}, xl_12{k} , r1_12{k});
    constraints = log_and(constraints, xrr_12{k}, xr_12{k} , r2_12{k});

    %................................... (33)...............................
    constraints = log_imp(constraints, xa_12{k}, z{k+1} , xlr_12{k});
    constraints = log_imp(constraints, xb_12{k}, z{k} ,   xlr_12{k});
    constraints = log_imp(constraints, xc_12{k}, z{k+1} , xrr_12{k});
    constraints = log_imp(constraints, xd_12{k}, z{k} ,   xrr_12{k});
    %................................... (32)...............................
    constraints = [constraints, xa_12{k} - xb_12{k} + xc_12{k} - xd_12{k} <= 0];
    constraints = [constraints, 0 <= xa_12{k} - xb_12{k} + xc_12{k} - xd_12{k} ];
    
    
    
    %  ################################ vehiculo 3 ################################ 
    constraints = [constraints, 1 <= z_13 <= L]; %tome valores posibles
    constraints = [constraints, mmin <= z_13 - z{k + 1} <= Mmax];
    %................................... (15.a)...............................
    constraints = log_min(constraints, k13{k}, z_13-z{k} , 1);
    constraints = log_may(constraints, del13{k}, z_13-z{k} , 1);
    constraints = log_and(constraints, r1_13{k}, k13{k} , del13{k} );
    %................................... (15.b)...............................
    constraints = log_min(constraints, kk13{k}, z_13-z{k} , -1);
    constraints = log_may(constraints, dell13{k}, z_13-z{k} , -1);
    constraints = log_and(constraints, r2_13{k}, kk13{k} , dell13{k} );
    %................................... (17)...............................
    constraints = log_min(constraints, u13{k}, dis13{k} , Dl);
    constraints = log_may(constraints, v13{k}, dis13{k} , -Dl);
    constraints = log_and(constraints, x13{k}, u13{k} , v13{k} );
    %................................... (30)...............................
    constraints = log_and(constraints, xl_13{k}, x13{k} , ll{k});
    constraints = log_and(constraints, xr_13{k}, x13{k} , lr{k});
    %................................... (31)...............................
    constraints = log_and(constraints, xlr_13{k}, xl_13{k} , r1_13{k});
    constraints = log_and(constraints, xrr_13{k}, xr_13{k} , r2_13{k});

    %................................... (33)...............................
    constraints = log_imp(constraints, xa_13{k}, z{k+1} , xlr_13{k});
    constraints = log_imp(constraints, xb_13{k}, z{k} ,   xlr_13{k});
    constraints = log_imp(constraints, xc_13{k}, z{k+1} , xrr_13{k});
    constraints = log_imp(constraints, xd_13{k}, z{k} ,   xrr_13{k});
    %................................... (32)...............................
    constraints = [constraints, xa_13{k} - xb_13{k} + xc_13{k} - xd_13{k} <= 0];
    constraints = [constraints, 0 <= xa_13{k} - xb_13{k} + xc_13{k} - xd_13{k} ];
    
    
    
    %  ################################ vehiculo 4 ################################ 
    constraints = [constraints, 1 <= z_14 <= L]; %tome valores posibles
    constraints = [constraints, mmin <= z_14 - z{k + 1} <= Mmax];
    %................................... (15.a)...............................
    constraints = log_min(constraints, k14{k}, z_14-z{k} , 1);
    constraints = log_may(constraints, del14{k}, z_14-z{k} , 1);
    constraints = log_and(constraints, r1_14{k}, k14{k} , del14{k} );
    %................................... (15.b)...............................
    constraints = log_min(constraints, kk14{k}, z_14-z{k} , -1);
    constraints = log_may(constraints, dell14{k}, z_14-z{k} , -1);
    constraints = log_and(constraints, r2_14{k}, kk14{k} , dell14{k} );
    %................................... (17)...............................
    constraints = log_min(constraints, u14{k}, dis14{k} , Dl);
    constraints = log_may(constraints, v14{k}, dis14{k} , -Dl);
    constraints = log_and(constraints, x14{k}, u14{k} , v14{k} );
    %................................... (30)...............................
    constraints = log_and(constraints, xl_14{k}, x14{k} , ll{k});
    constraints = log_and(constraints, xr_14{k}, x14{k} , lr{k});
    %................................... (31)...............................
    constraints = log_and(constraints, xlr_14{k}, xl_14{k} , r1_14{k});
    constraints = log_and(constraints, xrr_14{k}, xr_14{k} , r2_14{k});

    %................................... (33)...............................
    constraints = log_imp(constraints, xa_14{k}, z{k+1} , xlr_14{k});
    constraints = log_imp(constraints, xb_14{k}, z{k} ,   xlr_14{k});
    constraints = log_imp(constraints, xc_14{k}, z{k+1} , xrr_14{k});
    constraints = log_imp(constraints, xd_14{k}, z{k} ,   xrr_14{k});
    %................................... (32)...............................
    constraints = [constraints, xa_14{k} - xb_14{k} + xc_14{k} - xd_14{k} <= 0];
    constraints = [constraints, 0 <= xa_14{k} - xb_14{k} + xc_14{k} - xd_14{k} ];
    
    
    
    %  ################################ vehiculo 5 ################################ 
    constraints = [constraints, 1 <= z_15 <= L]; %tome valores posibles
    constraints = [constraints, mmin <= z_15 - z{k + 1} <= Mmax];
    %................................... (15.a)...............................
    constraints = log_min(constraints, k15{k}, z_15-z{k} , 1);
    constraints = log_may(constraints, del15{k}, z_15-z{k} , 1);
    constraints = log_and(constraints, r1_15{k}, k15{k} , del15{k} );
    %................................... (15.b)...............................
    constraints = log_min(constraints, kk15{k}, z_15-z{k} , -1);
    constraints = log_may(constraints, dell15{k}, z_15-z{k} , -1);
    constraints = log_and(constraints, r2_15{k}, kk15{k} , dell15{k} );
    %................................... (17)...............................
    constraints = log_min(constraints, u15{k}, dis15{k} , Dl);
    constraints = log_may(constraints, v15{k}, dis15{k} , -Dl);
    constraints = log_and(constraints, x15{k}, u15{k} , v15{k} );
    %................................... (30)...............................
    constraints = log_and(constraints, xl_15{k}, x15{k} , ll{k});
    constraints = log_and(constraints, xr_15{k}, x15{k} , lr{k});
    %................................... (31)...............................
    constraints = log_and(constraints, xlr_15{k}, xl_15{k} , r1_15{k});
    constraints = log_and(constraints, xrr_15{k}, xr_15{k} , r2_15{k});

    %................................... (33)...............................
    constraints = log_imp(constraints, xa_15{k}, z{k+1} , xlr_15{k});
    constraints = log_imp(constraints, xb_15{k}, z{k} ,   xlr_15{k});
    constraints = log_imp(constraints, xc_15{k}, z{k+1} , xrr_15{k});
    constraints = log_imp(constraints, xd_15{k}, z{k} ,   xrr_15{k});
    %................................... (32)...............................
    constraints = [constraints, xa_15{k} - xb_15{k} + xc_15{k} - xd_15{k} <= 0];
    constraints = [constraints, 0 <= xa_15{k} - xb_15{k} + xc_15{k} - xd_15{k} ];
    
    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables

end
% constraints = [constraints, [dis12{1} <= 100000]];
  

    parameters_in = {Zd, z{1}, z_12, [dis12{:}],...
                               z_13, [dis13{:}],...
                               z_14, [dis14{:}],...
                               z_15, [dis15{:}]}; 

    solutions_out = {[z{:}], [ll{:}]}; %, [lr{:}]};
                
    % controller = optimizer(constraints, objective,sdpsettings('verbose',1),[x{1};r],u{1});
    control_lat = optimizer(constraints, objective, sdpsettings('solver', 'gurobi'), parameters_in, solutions_out);

    %% Building variables

%define las condiciones iniciales que deben tener las variables
%logicas

%.....................vehiculo 1..........................

% ..........historial de las predicciones
hist_vp1 = [];
hist_zp1 = [];

hist_vp2 = [];
hist_zp2 = [];

hist_vp3 = [];
hist_zp3 = [];

hist_vp4 = [];
hist_zp4 = [];

hist_vp5 = [];
hist_zp5 = [];

hist_vp6 = [];
hist_zp6 = [];


%  history of the logical variables
hist_r1 = [];
hist_d1 = [];
hist_x1 = [];
hist_rd1 = [];
hist_ps1 = [];
hist_p1 = [];
hist_s1 = [];
hist_del1 = [];

hist_r2 = [];
hist_d2 = [];
hist_x2 = [];
hist_rd2 = [];
hist_ps2 = [];
hist_p2 = [];
hist_v2 = [];
hist_ul2 = [];
%  change lane variables
hist_ll1 = [];
hist_lr1 = [];
hist_ll2 = [];
hist_lr2 = [];
hist_ll3 = [];
hist_lr3 = [];
hist_ll4 = [];
hist_lr4 = [];
hist_ll5 = [];
hist_lr5 = [];
hist_ll6 = [];
hist_lr6 = [];

hist_dz = [];
hist_dis1 = [];
hist_dis2 = [];

% auxiliary variables
AA = [];
BB = [];
CC = [];
DD = [];
EE = [];
FF = [];
GG  = [];
HH  = [];
II  = [];
JJ = [];
KK = [];

%------condiciones iniciales----------
vel =  [25; 20; 20; 20; 30; 10]; % velociodad inicial
Vdes = [30; 50; 40; 60; 20; 35]; %velocidad deseada

zel =  [3; 4; 2; 5; 1; 3]; %carril inicial
Zdes = [5; 1; 5; 1; 5; 1]; %carril deseado

acel = zeros(6,1);
%---distancia inicial de cada agente
pos = [0 -20 -40 -60 -50 -80]';

% hold on
vhist = vel;
zhist = zel;
ahist = acel;
hist_pos = pos;
mpciter = 0;



%% Optimization

zel2 = zel; %same dimentions

F = 80;
nv = 6; %numero de vehiculos sin el agente no cooperativo
vphist = nan(F, N+1, nv);
zphist = nan(F, N+1, nv);
dif_z1 = ones(1, N+1).*(zel-zel(1));
dif_z2 = ones(1, N+1).*(zel-zel(2));
dif_z3 = ones(1, N+1).*(zel-zel(3));
dif_z4 = ones(1, N+1).*(zel-zel(4));
dif_z5 = ones(1, N+1).*(zel-zel(5));
dif_z6 = ones(1, N+1).*(zel-zel(6));


tic
for i = 1:F
%     ######################  VEHICULO 1 #######################
% neightbors
%NH = [2 3 4 5];
zel= zel2;
NH = closest(pos, zel, 1);

    %.........................      solver Frontal       ............................

    inputs1 = {Vdes(1), vel(1), ...
        dif_z1(NH(1),:),vel(NH(1)), (-pos(1)+pos(NH(1))),...
        dif_z1(NH(2),:),vel(NH(2)), (-pos(1)+pos(NH(2))),...
        dif_z1(NH(3),:),vel(NH(3)), (-pos(1)+pos(NH(3))),...
        dif_z1(NH(4),:),vel(NH(4)), (-pos(1)+pos(NH(4)))};
    [solutions1, diagnostics] = control_front{inputs1};
        
    A = solutions1{1}; acel(1) = A(:, 1);
    V = solutions1{2}; vphist(i,:,1) = V;
    d_12 = solutions1{3}; d_12(1) = -pos(1)+pos(NH(1));
    d_13 = solutions1{4}; d_13(1) = -pos(1)+pos(NH(2));
    d_14 = solutions1{5}; d_14(1) = -pos(1)+pos(NH(3));
    d_15 = solutions1{6}; d_15(1) = -pos(1)+pos(NH(4));
    
    if diagnostics == 1
    error('control_front failed 1');
    end
%     hist_dis1 = [hist_dis1; d_12];
     %.........................      solver lateral       ............................

    inputs2 = {Zdes(1), zel(1), zel(NH(1)), d_12,...
                                zel(NH(2)), d_13,...
                                zel(NH(3)), d_14,...
                                zel(NH(4)), d_15}; 
    [solutions2, diagnostics] = control_lat{inputs2};
    Z = solutions2{1}; zel2(1) = Z(:, 2); zphist(i,:,1) = Z;
%     I = solutions2{2}; hist_ll1 = [hist_ll1; I];
%     J = solutions2{3}; hist_lr1 = [hist_lr1; J];     
    
    if diagnostics == 1
    error('control_lat failed 1');
    end

    
%     ######################  VEHICULO 2 #######################
%NH = [1 3 4 5];
zel= zel2;
NH = closest(pos, zel, 2);
    %.........................      solver Frontal       ............................

    inputs1 = {Vdes(2), vel(2),...
        dif_z2(NH(1),:), vel(NH(1)), (-pos(2)+pos(NH(1))),...
        dif_z2(NH(2),:), vel(NH(2)), (-pos(2)+pos(NH(2))),...
        dif_z2(NH(3),:), vel(NH(3)), (-pos(2)+pos(NH(3))),...
        dif_z2(NH(4),:), vel(NH(4)), (-pos(2)+pos(NH(4)))}; 
    [solutions1, diagnostics] = control_front{inputs1};
        
    A = solutions1{1}; acel(2) = A(:, 1);
    V = solutions1{2}; vphist(i,:,2) = V;
    d_12 = solutions1{3}; d_12(1) = -pos(2)+pos(NH(1));
    d_23 = solutions1{4}; d_23(1) = -pos(2)+pos(NH(2));
    d_24 = solutions1{5}; d_24(1) = -pos(2)+pos(NH(3));
    d_25 = solutions1{6}; d_25(1) = -pos(2)+pos(NH(4));
    
    if diagnostics == 1
    error('control_front failed 2');
    end
    
  
     %.........................      solver lateral       ............................

    inputs2 = {Zdes(2), zel(2), zel(NH(1)), d_12,...
                                zel(NH(2)), d_23,...
                                zel(NH(3)), d_24,...
                                zel(NH(4)), d_25}; 
    [solutions2, diagnostics] = control_lat{inputs2};
    
    Z = solutions2{1}; zel2(2) = Z(:, 2); zphist(i,:,2) = Z;
%     II2 = solutions2{2}; hist_ll2 = [hist_ll2; II2];
%     JJ2 = solutions2{3}; hist_lr2 = [hist_lr2; JJ2];
    
    if diagnostics == 1
    error('control_lat failed 2');
    end

%     ######################  VEHICULO 3 #######################
%NH = [1 2 4 5];
zel= zel2;
NH = closest(pos, zel, 3);
    %.........................      solver Frontal       ............................

    inputs1 = {Vdes(3), vel(3),...
        dif_z3(NH(1),:), vel(NH(1)), (-pos(3)+pos(NH(1))),...
        dif_z3(NH(2),:), vel(NH(2)), (-pos(3)+pos(NH(2))),...
        dif_z3(NH(3),:), vel(NH(3)), (-pos(3)+pos(NH(3))),...
        dif_z3(NH(4),:), vel(NH(4)), (-pos(3)+pos(NH(4)))}; 
    [solutions1, diagnostics] = control_front{inputs1};
        
    A = solutions1{1}; acel(3) = A(:, 1);
    V = solutions1{2}; vphist(i,:,3) = V;
    d_31 = solutions1{3}; d_31(1) = -pos(3)+pos(NH(1));
    d_32 = solutions1{4}; d_32(1) = -pos(3)+pos(NH(2));
    d_34 = solutions1{5}; d_34(1) = -pos(3)+pos(NH(3));
    d_35 = solutions1{6}; d_35(1) = -pos(3)+pos(NH(4));
    if diagnostics == 1
    error('control_front failed 3');
    end
    
     %.........................      solver lateral       ............................

    inputs2 = {Zdes(3), zel(3), zel(NH(1)), d_31,...
                                zel(NH(2)), d_32,...
                                zel(NH(3)), d_34,...
                                zel(NH(4)), d_35}; 
    [solutions2, diagnostics] = control_lat{inputs2};
    
    Z = solutions2{1}; zel2(3) = Z(:, 2); zphist(i,:,3) = Z;
%     II2 = solutions2{2}; hist_ll3 = [hist_ll3; II2];
%     JJ2 = solutions2{3}; hist_lr3 = [hist_lr3; JJ2]; 
    
    if diagnostics == 1
    error('control_lat failed 3');
    end

%     ######################  VEHICULO 4 #######################
%NH = [2 3 5 6];
zel= zel2;
NH = closest(pos, zel, 4);
    %.........................      solver Frontal       ............................

    inputs1 = {Vdes(4), vel(4),...
        dif_z4(NH(1),:), vel(NH(1)), (-pos(4)+pos(NH(1))),...
        dif_z4(NH(2),:), vel(NH(2)), (-pos(4)+pos(NH(2))),...
        dif_z4(NH(3),:), vel(NH(3)), (-pos(4)+pos(NH(3))),...
        dif_z4(NH(4),:), vel(NH(4)), (-pos(4)+pos(NH(4)))}; 
    [solutions1, diagnostics] = control_front{inputs1};
        
    A = solutions1{1}; acel(4) = A(:, 1);
    V = solutions1{2}; vphist(i,:,4) = V;
    d_41 = solutions1{3}; d_41(1) = -pos(4)+pos(NH(1));
    d_42 = solutions1{4}; d_42(1) = -pos(4)+pos(NH(2));
    d_43 = solutions1{5}; d_43(1) = -pos(4)+pos(NH(3));
    d_45 = solutions1{6}; d_45(1) = -pos(4)+pos(NH(3));
    
    if diagnostics == 1
    error('control_front failed 4');
    end
    

     %.........................      solver lateral       ............................

    inputs2 = {Zdes(4), zel(4), zel(NH(1)), d_41,...
                                zel(NH(2)), d_42,...
                                zel(NH(3)), d_43,...
                                zel(NH(4)), d_45}; 
    [solutions2, diagnostics] = control_lat{inputs2};
    
    Z = solutions2{1}; zel2(4) = Z(:, 2); zphist(i,:,4) = Z;

    
    if diagnostics == 1
    error('control_lat failed 4');
    end

    
    %     ######################  VEHICULO 5 #######################
%NH = [2 3 4 6];
zel= zel2;
NH = closest(pos, zel, 5);
    %.........................      solver Frontal       ............................

    inputs1 = {Vdes(5), vel(5),...
        dif_z5(NH(1),:), vel(NH(1)), (-pos(5)+pos(NH(1))),...
        dif_z5(NH(2),:), vel(NH(2)), (-pos(5)+pos(NH(2))),...
        dif_z5(NH(3),:), vel(NH(3)), (-pos(5)+pos(NH(3))),...
        dif_z5(NH(4),:), vel(NH(4)), (-pos(5)+pos(NH(4)))}; 
    [solutions1, diagnostics] = control_front{inputs1};
        
    A = solutions1{1}; acel(5) = A(:, 1);
    V = solutions1{2}; vphist(i,:,5) = V;
    d_51 = solutions1{3}; d_51(1) = -pos(5)+pos(NH(1));
    d_52 = solutions1{4}; d_52(1) = -pos(5)+pos(NH(2));
    d_54 = solutions1{5}; d_54(1) = -pos(5)+pos(NH(3));
    d_56 = solutions1{6}; d_56(1) = -pos(5)+pos(NH(4));
    
    if diagnostics == 1
    error('control_front failed 5');
    end
    
     %.........................      solver lateral       ............................

    inputs2 = {Zdes(5), zel(5), zel(NH(1)), d_51,...
                                zel(NH(2)), d_52,...
                                zel(NH(3)), d_54,...
                                zel(NH(4)), d_56}; 
    [solutions2, diagnostics] = control_lat{inputs2};
    
    Z = solutions2{1}; zel2(5) = Z(:, 2); zphist(i,:,5) = Z;
%     II2 = solutions2{2}; hist_ll3 = [hist_ll3; II2];
%     JJ2 = solutions2{3}; hist_lr3 = [hist_lr3; JJ2]; 
    
    if diagnostics == 1
    error('control_lat failed 5');
    end

%     ######################  VEHICULO 6 #######################
%NH = [2 3 4 5];
zel= zel2;
NH = closest(pos, zel, 6);
    %.........................      solver Frontal       ............................

    inputs1 = {Vdes(6), vel(6),...
        dif_z6(NH(1),:), vel(NH(1)), (-pos(6)+pos(NH(1))),...
        dif_z6(NH(2),:), vel(NH(2)), (-pos(6)+pos(NH(2))),...
        dif_z6(NH(3),:), vel(NH(3)), (-pos(6)+pos(NH(3))),...
        dif_z6(NH(4),:), vel(NH(4)), (-pos(6)+pos(NH(4)))}; 
    [solutions1, diagnostics] = control_front{inputs1};
        
    A = solutions1{1}; acel(6) = A(:, 1);
    V = solutions1{2}; vphist(i,:,6) = V;
    d_61 = solutions1{3}; d_61(1) = -pos(6)+pos(NH(1));
    d_62 = solutions1{4}; d_62(1) = -pos(6)+pos(NH(2));
    d_63 = solutions1{5}; d_63(1) = -pos(6)+pos(NH(3));
    d_64 = solutions1{6}; d_64(1) = -pos(6)+pos(NH(4));
    
    if diagnostics == 1
    error('control_front failed 6');
    end
    

     %.........................      solver lateral       ............................

    inputs2 = {Zdes(6), zel(6), zel(NH(1)), d_61,...
                                zel(NH(2)), d_62,...
                                zel(NH(3)), d_63,...
                                zel(NH(4)), d_64}; 
    [solutions2, diagnostics] = control_lat{inputs2};
    
    Z = solutions2{1}; zel2(6) = Z(:, 2); zphist(i,:,6) = Z;

    
    if diagnostics == 1
    error('control_lat failed 6');
    end

    
    

    %----------------------------------------------------------------------
        zel= zel2;

    dif_z1 = reshape(zphist(i,:,:)-zphist(i,:,1) , [N+1, nv])';
    dif_z2 = reshape(zphist(i,:,:)-zphist(i,:,2) , [N+1, nv])';
    dif_z3 = reshape(zphist(i,:,:)-zphist(i,:,3) , [N+1, nv])';
    dif_z4 = reshape(zphist(i,:,:)-zphist(i,:,4) , [N+1, nv])';
    dif_z5 = reshape(zphist(i,:,:)-zphist(i,:,5) , [N+1, nv])';
    dif_z6 = reshape(zphist(i,:,:)-zphist(i,:,6) , [N+1, nv])';
    
    pos = pos + T*vel;
    

    vel = vel + T * acel;
    vhist = [vhist vel];
    zhist = [zhist zel];
    ahist = [ahist acel];
    hist_pos = [hist_pos pos];
%     hist_dz = [hist_dz; dif_z12];


    mpciter;
    mpciter = mpciter + 1;
end
toc

disp("it's done")

%% plot
dhist = [-hist_pos(1,:)+hist_pos(2,:); -hist_pos(1,:)+hist_pos(3,:); -hist_pos(1,:)+hist_pos(4,:)];


Draw_object(vhist, zhist, vphist, zphist, dhist, T, 0)
% save('myFile5.mat','vhist','zhist','vphist','zphist','dhist','T')
