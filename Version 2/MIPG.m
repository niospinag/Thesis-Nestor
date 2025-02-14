% UNIVERSIDAD NACIONAL DE COLOMBIA
% Multi Vehicle automated drivring
% Autor: Nestor Ospina
clear
close all
clc

% %---------laptop asus
addpath(genpath('C:\gurobi950\win64\matlab'))%GUROBI 
addpath(genpath('C:\Users\nesto\OneDrive\Documentos\YALMIP-master'))%yalmip

yalmip('clear')
%% PROGRAM
% MPC data
Q = 1 * eye(1);
R  = 10 * eye(1);
N = 5; % horizon 5
T = 0.3; % [s]
Ds = 15; % Safety distance [m]
Dl = 20; % lateral distance
V_max = 80;
A_max = 30;
L = 6; % number of lanes
Mmax = L - 1;
mmin = -L + 1;

%------------desired states-----------
Zd = sdpvar(1, 1); % carril deseado
Vd = sdpvar(1, 1); % velocidad deseada

% -------------local vehicle---------------
v = sdpvar(ones(1, N + 1), ones(1, N + 1)); % velocidad del vehiculo actual
v_ref = sdpvar(1,1); % vel ref
a = sdpvar(ones(1, N), ones(1, N)); % aceleracion actual del vehiculo
z = intvar(ones(1, N + 1), ones(1, N + 1)); % carril actual
z_ref = intvar(1,1); %lane ref

% -------------- neighboor ---------------
v_2 = sdpvar(1, 1); v_3 = sdpvar(1, 1); % velocidad del otro vehculo
z_2 = sdpvar(1, 1); z_3 = sdpvar(1, 1); % carril del vehiculo j

% ------ distance between two vehicles ------
dis12 = sdpvar(ones(1, N + 1), ones(1, N + 1)); % distancia entre vehiculo 1 y 2
dis12_ref = sdpvar(1,1); % ref distance

dis13 = sdpvar(ones(1, N + 1), ones(1, N + 1)); % distancia entre vehiculo 1 y 3
dis13_ref = sdpvar(1,1); % ref distance


%% neihborhood variables

%  FRONTAL VARIABLES
a12 = binvar(ones(1, N), ones(1, N));
b12 = binvar(ones(1, N), ones(1, N));
ab12 = binvar(ones(1, N), ones(1, N));
n12 = binvar(ones(1, N), ones(1, N));
th12 = binvar(ones(1, N), ones(1, N));
f12 = sdpvar(ones(1, N), ones(1, N));
g12 = sdpvar(ones(1, N), ones(1, N));

a13 = binvar(ones(1, N), ones(1, N));
b13 = binvar(ones(1, N), ones(1, N));
ab13 = binvar(ones(1, N), ones(1, N));
n13 = binvar(ones(1, N), ones(1, N));
th13 = binvar(ones(1, N), ones(1, N));
f13 = sdpvar(ones(1, N), ones(1, N));
g13 = sdpvar(ones(1, N), ones(1, N));

%  LATERAL VARIABLES
k12  = binvar(ones(1, N), ones(1, N));
del12 = binvar(ones(1, N), ones(1, N));
r1_12 = binvar(ones(1, N), ones(1, N));
kk12 = binvar(ones(1, N), ones(1, N));
dell12 = binvar(ones(1, N), ones(1, N));
r2_12 = binvar(ones(1, N), ones(1, N));
u12 = binvar(ones(1, N), ones(1, N));
v12 = binvar(ones(1, N), ones(1, N));
x12 = binvar(ones(1, N), ones(1, N));
xr1_12 = binvar(ones(1, N), ones(1, N));
xr2_12 = binvar(ones(1, N), ones(1, N));
xa_12 = sdpvar(ones(1, N), ones(1, N));
xb_12 = sdpvar(ones(1, N), ones(1, N));
xc_12 = sdpvar(ones(1, N), ones(1, N));
xd_12 = sdpvar(ones(1, N), ones(1, N));

k13  = binvar(ones(1, N), ones(1, N));
del13 = binvar(ones(1, N), ones(1, N));
r1_13 = binvar(ones(1, N), ones(1, N));
kk13 = binvar(ones(1, N), ones(1, N));
dell13 = binvar(ones(1, N), ones(1, N));
r2_13 = binvar(ones(1, N), ones(1, N));
u13 = binvar(ones(1, N), ones(1, N));
v13 = binvar(ones(1, N), ones(1, N));
x13 = binvar(ones(1, N), ones(1, N));
xr1_13 = binvar(ones(1, N), ones(1, N));
xr2_13 = binvar(ones(1, N), ones(1, N));
xa_13 = sdpvar(ones(1, N), ones(1, N));
xb_13 = sdpvar(ones(1, N), ones(1, N));
xc_13 = sdpvar(ones(1, N), ones(1, N));
xd_13 = sdpvar(ones(1, N), ones(1, N));


%% making the optimizer longitudinal
constraints = [];

% constraints = [constraints,  diff([p_z z{1}]) == 0];
objective = 0;
constraints = [constraints, v{1} == v_ref ];
constraints = [constraints, z{1} == z_ref ];
constraints = [constraints, dis12{1} == dis12_ref ];
constraints = [constraints, dis13{1} == dis13_ref ];
for k = 1:N
    objective = objective + (v{k + 1} - Vd)'*Q*(v{k + 1} - Vd) + (z{k + 1}-Zd)'*R*(z{k + 1}-Zd); % calculate obj
    % Feasible region
    constraints = [constraints, 0 <= v{k + 1} <= V_max, % no exceda las velocidades
        -A_max <= a{k} <= A_max];

    constraints = [constraints, v{k + 1} == v{k} + T * a{k}]; % velocidad futura

    % ---------------------------- vehiculo 2 -----------------------------
    % ------------------ si dz=0  -------------------->>>    dij >= Ds----------------

    constraints = [constraints, dis12{k + 1} == dis12{k} + T * (v_2 - v{k})];

    %................................... (12)...............................
    constraints = log_min(constraints, n12{k}, z_2-z{k+1}, 0);
    constraints = log_may(constraints, th12{k}, z_2-z{k+1}, 0);
    constraints = log_and(constraints, a12{k}, n12{k}, th12{k});
    %................................... (13)...............................
    constraints = log_may(constraints, b12{k}, dis12{k}, 0);
    %................................... (18)...............................
    constraints = log_and(constraints, ab12{k}, a12{k}, b12{k});
    %................................... (21)...............................
    constraints = log_imp(constraints, f12{k}, dis12{k}, ab12{k});
    %................................... (22)...............................
    constraints = log_imp(constraints, g12{k}, Ds, ab12{k});
    %................................... (23)...............................
    constraints = [constraints, g12{k} - f12{k} <= 0];

    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables

    % Feasible region
    constraints = [constraints,       1 <= z_2 <= L, %tome valores posibles
                                      1 <= z{k+1} <= L,
                                      z{k} - 1 <= z{k + 1},
                                      z{k + 1} <= z{k} + 1];

    %................................... (15.a)...............................
    constraints = log_min(constraints, k12{k}, z_2-z{k} , 1);
    constraints = log_may(constraints, del12{k}, z_2-z{k} , 1);
    constraints = log_and(constraints, r1_12{k}, k12{k} , del12{k} );
    %................................... (15.b)...............................
    constraints = log_min(constraints, kk12{k}, z_2-z{k} , -1);
    constraints = log_may(constraints, dell12{k}, z_2-z{k} , -1);
    constraints = log_and(constraints, r2_12{k}, kk12{k} , dell12{k} );
    %................................... (17)...............................
    constraints = log_min(constraints, u12{k}, dis12{k} , Dl);
    constraints = log_may(constraints, v12{k}, dis12{k} , -Dl);
    constraints = log_and(constraints, x12 {k}, u12{k} , v12{k} );
    %................................... (30)...............................
    constraints = log_and(constraints, xr1_12{k}, x12{k} , r1_12{k});
    constraints = log_and(constraints, xr2_12{k}, x12{k} , r2_12{k});

    %................................... (33)...............................
    constraints = log_imp(constraints, xa_12{k}, z{k+1} , xr1_12{k});
    constraints = log_imp(constraints, xb_12{k}, z{k} ,   xr1_12{k});
    constraints = log_imp(constraints, xc_12{k}, z{k+1} , xr2_12{k});
    constraints = log_imp(constraints, xd_12{k}, z{k} ,   xr2_12{k});
    %................................... (32)...............................
    constraints = [constraints, xa_12{k} - xb_12{k} - xc_12{k} + xd_12{k} <= 0];
    


        % ---------------------------- vehiculo 3 -----------------------------
    % ------------------ si dz=0  -------------------->>>    dij >= Ds----------------

    constraints = [constraints, dis13{k + 1} == dis13{k} + T * (v_3 - v{k})];

    %................................... (12)...............................
    constraints = log_min(constraints, n13{k}, z_3-z{k+1}, 0);
    constraints = log_may(constraints, th13{k}, z_3-z{k+1}, 0);
    constraints = log_and(constraints, a13{k}, n13{k}, th13{k});
    %................................... (13)...............................
    constraints = log_may(constraints, b13{k}, dis13{k}, 0);
    %................................... (18)...............................
    constraints = log_and(constraints, ab13{k}, a13{k}, b13{k});
    %................................... (21)...............................
    constraints = log_imp(constraints, f13{k}, dis13{k}, ab13{k});
    %................................... (22)...............................
    constraints = log_imp(constraints, g13{k}, Ds, ab13{k});
    %................................... (24)...............................
    constraints = [constraints, g13{k} - f13{k} <= 0];

    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables

    % Feasible region
    constraints = [constraints,       1 <= z_3 <= L];
    %................................... (15.a)...............................
    constraints = log_min(constraints, k13{k}, z_3-z{k} , 1);
    constraints = log_may(constraints, del13{k}, z_3-z{k} , 1);
    constraints = log_and(constraints, r1_13{k}, k13{k} , del13{k} );
    %................................... (15.b)...............................
    constraints = log_min(constraints, kk13{k}, z_3-z{k} , -1);
    constraints = log_may(constraints, dell13{k}, z_3-z{k} , -1);
    constraints = log_and(constraints, r2_13{k}, kk13{k} , dell13{k} );
    %................................... (17)...............................
    constraints = log_min(constraints, u13{k}, dis13{k} , Dl);
    constraints = log_may(constraints, v13{k}, dis13{k} , -Dl);
    constraints = log_and(constraints, x13 {k}, u13{k} , v13{k} );
    %................................... (30)...............................
    constraints = log_and(constraints, xr1_13{k}, x13{k} , r1_13{k});
    constraints = log_and(constraints, xr2_13{k}, x13{k} , r2_13{k});

    %................................... (33)...............................
    constraints = log_imp(constraints, xa_13{k}, z{k+1} , xr1_13{k});
    constraints = log_imp(constraints, xb_13{k}, z{k} ,   xr1_13{k});
    constraints = log_imp(constraints, xc_13{k}, z{k+1} , xr2_13{k});
    constraints = log_imp(constraints, xd_13{k}, z{k} ,   xr2_13{k});
    %................................... (32)...............................
    constraints = [constraints, xa_13{k} - xb_13{k} - xc_13{k} + xd_13{k} <= 0];


end

parameters_in = {Vd, v_ref, Zd, z_ref, v_2, dis12_ref, z_2, v_3, dis13_ref, z_3};

solutions_out = {[a{:}], [v{:}], [z{:}], [dis12{:}] ,[a12{:}], [b12{:}], [ab12{:}], [f12{:}] ,[g12{:}] };

control_front = optimizer(constraints, objective, sdpsettings('solver', 'gurobi'), parameters_in, solutions_out);


%% Building variables



%.....................vehiculo 1..........................

% ..........historial de las predicciones
hist_vp1 = [];
hist_zp1 = [];

hist_vp2 = [];
hist_zp2 = [];

hist_vp3 = [];
hist_zp3 = [];

%------condiciones iniciales----------
vel = [20; 10; 20]; % velociodad inicial
Vdes = [30; 80; 60]; %velocidad deseada

zel = [1; 2; 3]; %carril inicial
Zdes = [5; 5; 5]; %carril deseado

acel = [0 0 0]';
%---distancia inicial de cada agente
d1i = [-15; -30];

% hold on
vhist = vel;
zhist = zel;
ahist = acel;
dhist = d1i;
mpciter = 0;

hist_dz = [];
hist_dis1 = [];
hist_dis2 = [];
hist_dis3 = [];

hist_a12 = [];
hist_b12 = [];
hist_ab12 = [];
hist_f12 = [];
hist_g12 = [];

%% Optimization

zel2 = zel; %same dimentions
time = 20;
tic
sim_tim = 20;
nv= length(vel);
% LR2 = [1];
% LR1 = [1];
dif_z = ones(1,N+1)*[zel(2)-zel(1)];
for i = 1:30
    %     ######################  VEHICULO 1 #######################

    %.........................      solver Frontal       ............................

    inputs1 = {Vdes(1), vel(1), Zdes(1), zel(1),     vel(2), d1i(1),  zel(2), vel(3), d1i(2),  zel(3)};
    [solutions1, diagnostics] = control_front{inputs1};

    A = solutions1{1};      acel(1) = A(:, 1);
    V = solutions1{2};      hist_vp1 = [hist_vp1; V];
    Z = solutions1{3};      zel2(1) = Z(:, 2);          hist_zp1 = [hist_zp1; Z];
    dis = solutions1{4}; 
    hist_a12 = [hist_a12; solutions1{5}];
    hist_b12 = [hist_b12; solutions1{6}];
    hist_ab12 = [hist_ab12; solutions1{7}];
    hist_f12 = [hist_f12; solutions1{8}];
    hist_g12 = [hist_g12; solutions1{9}];


    if diagnostics == 1
        error('control_front failed 1');
    end


    zel= zel2;
    %     ######################  VEHICULO 2 #######################

    %.........................      solver Frontal       ............................

    inputs2 = {Vdes(2), vel(2), Zdes(2), zel(2),     vel(3), -d1i(1)+d1i(2),  zel(3), vel(1), -d1i(1),  zel(1)}; 
    [solutions2, diagnostics] = control_front{inputs2};

    A = solutions2{1}; acel(2) = A(:, 1);
    V = solutions2{2}; hist_vp2 = [hist_vp2; V];
    Z = solutions2{3}; zel2(2) = Z(:, 2); hist_zp2 = [hist_zp2; Z];
    dis = solutions2{4};  


    if diagnostics == 1
        error('control_front failed 2');
    end
   
    zel= zel2;
    %     ######################  VEHICULO 3 #######################

    %.........................      solver Frontal       ............................

    inputs3 = {Vdes(3), vel(3), Zdes(3), zel(3),     vel(1), -d1i(2),  zel(1), vel(2), -d1i(2)+d1i(1),  zel(2)}; 
    [solutions3, diagnostics] = control_front{inputs3};

    A = solutions3{1}; acel(3) = A(:, 1);
    V = solutions3{2}; hist_vp3 = [hist_vp3; V];
    Z = solutions3{3}; zel2(3) = Z(:, 2); hist_zp3 = [hist_zp3; Z];
    dis = solutions3{4};  


    if diagnostics == 1
        error('control_front failed 3');
    end
   

    %----------------------------------------------------------------------
    zel= zel2;

    d1i = d1i + T * (vel(2:nv) - ones((nv - 1), 1) * vel(1));

    vel = vel + T * acel;
    vhist = [vhist vel];
    zhist = [zhist zel];
    ahist = [ahist acel];
    dhist = [dhist d1i];


    mpciter;
    mpciter = mpciter + 1;
end
toc

disp("it's done")

%% plot

vphist = cat(3, hist_vp1, hist_vp2, hist_vp3);
zphist = cat(3, hist_zp1, hist_zp2, hist_zp3 );

Draw_object(vhist, zhist, vphist, zphist, dhist, T, 0.1)
% save('myFile5.mat','vhist','zhist','vphist','zphist','dhist','T')
