% UNIVERSIDAD NACIONAL DE COLOMBIA
% Multi Vehicle automated drivring
% Autor: Nestor Ospina
clear
close all
clc

% %---------laptop asus
addpath(genpath('/opt/gurobi951/linux64/matlab'))%GUROBI 
addpath(genpath('~/YALMIP-master'))%yalmip

yalmip('clear')

%% INITIAL CONDITIONS
%------condiciones iniciales----------
vel =  [10; 20; 10; 20; 10; 10; 20; 10; 10; 20; 10; 20]; % velociodad inicial
Vdes = [30; 13; 20; 10; 10; 20; 30; 30; 20; 50;  15;  15]; %velocidad deseada

zel =  [6; 1; 3; 5; 2; 4; 6; 4; 3; 5; 2; 1]; %carril inicial
Zdes = [3; 1; 2; 1; 1; 1; 3; 3; 2; 5; 1; 2]; %carril deseado

% zel =  [6; 1; 3; 5; 2; 4; 6; 4; 3; 5; 2; 1]; %carril inicial
% Zdes = [3; 3; 3; 3; 3; 3; 3; 3; 3; 3; 3; 3]; %carril deseado

%--- Initial distances 
pos =  [0 -40 -10 -10 -30 -30 -30 10 10 10 20 -20]';

N = 10; % horizon 

nv = size(vel,1);
acel = zeros(nv,1);


%% PROGRAM
% MPC data
k_step = 50;
Q = 2 * eye(1); % vel 
R = 25 * eye(1); % z

T = 0.2; % [s]
Ds = 10; % Safety distance [m]
Dl = 20; % lateral distance
V_max = 80; 
A_max = 70;
L = 7; % number of lanes


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
v_2 = sdpvar(1, 1); v_3 = sdpvar(1, 1); v_4 = sdpvar(1, 1); v_5 = sdpvar(1, 1); % velocidad del otro vehculo
z_2 = sdpvar(1, 1); z_3 = sdpvar(1, 1); 
z_4 = sdpvar(1, 1); z_5 = sdpvar(1, 1); % carril del vehiculo j

% ------ distance between two vehicles ------
dis12 = sdpvar(ones(1, N + 1), ones(1, N + 1)); % distancia entre vehiculo 1 y 2
dis12_ref = sdpvar(1,1); % ref distance

dis13 = sdpvar(ones(1, N + 1), ones(1, N + 1)); % distancia entre vehiculo 1 y 3
dis13_ref = sdpvar(1,1); % ref distance

dis14 = sdpvar(ones(1, N + 1), ones(1, N + 1)); % distancia entre vehiculo 1 y 4
dis14_ref = sdpvar(1,1); % ref distance

dis15 = sdpvar(ones(1, N + 1), ones(1, N + 1)); % distancia entre vehiculo 1 y 4
dis15_ref = sdpvar(1,1); % ref distance

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

a14 = binvar(ones(1, N), ones(1, N));
b14 = binvar(ones(1, N), ones(1, N));
ab14 = binvar(ones(1, N), ones(1, N));
n14 = binvar(ones(1, N), ones(1, N));
th14 = binvar(ones(1, N), ones(1, N));
f14 = sdpvar(ones(1, N), ones(1, N));
g14 = sdpvar(ones(1, N), ones(1, N));

a15 = binvar(ones(1, N), ones(1, N));
b15 = binvar(ones(1, N), ones(1, N));
ab15 = binvar(ones(1, N), ones(1, N));
n15 = binvar(ones(1, N), ones(1, N));
th15 = binvar(ones(1, N), ones(1, N));
f15 = sdpvar(ones(1, N), ones(1, N));
g15 = sdpvar(ones(1, N), ones(1, N));

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

k14  = binvar(ones(1, N), ones(1, N));
del14 = binvar(ones(1, N), ones(1, N));
r1_14 = binvar(ones(1, N), ones(1, N));
kk14 = binvar(ones(1, N), ones(1, N));
dell14 = binvar(ones(1, N), ones(1, N));
r2_14 = binvar(ones(1, N), ones(1, N));
u14 = binvar(ones(1, N), ones(1, N));
v14 = binvar(ones(1, N), ones(1, N));
x14 = binvar(ones(1, N), ones(1, N));
xr1_14 = binvar(ones(1, N), ones(1, N));
xr2_14 = binvar(ones(1, N), ones(1, N));
xa_14 = sdpvar(ones(1, N), ones(1, N));
xb_14 = sdpvar(ones(1, N), ones(1, N));
xc_14 = sdpvar(ones(1, N), ones(1, N));
xd_14 = sdpvar(ones(1, N), ones(1, N));

k15  = binvar(ones(1, N), ones(1, N));
del15 = binvar(ones(1, N), ones(1, N));
r1_15 = binvar(ones(1, N), ones(1, N));
kk15 = binvar(ones(1, N), ones(1, N));
dell15 = binvar(ones(1, N), ones(1, N));
r2_15 = binvar(ones(1, N), ones(1, N));
u15 = binvar(ones(1, N), ones(1, N));
v15 = binvar(ones(1, N), ones(1, N));
x15 = binvar(ones(1, N), ones(1, N));
xr1_15 = binvar(ones(1, N), ones(1, N));
xr2_15 = binvar(ones(1, N), ones(1, N));
xa_15 = sdpvar(ones(1, N), ones(1, N));
xb_15 = sdpvar(ones(1, N), ones(1, N));
xc_15 = sdpvar(ones(1, N), ones(1, N));
xd_15 = sdpvar(ones(1, N), ones(1, N));

%% making the optimizer 
constraints = [];

objective = 0;
% coupling constraints
constraints = [constraints, v{1} == v_ref ];
constraints = [constraints, z{1} == z_ref ];

constraints = [constraints, dis12{1} == dis12_ref ];
constraints = [constraints, dis13{1} == dis13_ref ];
constraints = [constraints, dis14{1} == dis14_ref ];
constraints = [constraints, dis15{1} == dis15_ref ];

for k = 1:N
    objective = objective + (v{k + 1} - Vd)'*Q*(v{k + 1} - Vd) + (z{k + 1}-Zd)'*R*(z{k + 1}-Zd); % obj function
    % Feasible region
    constraints = [constraints, 0 <= v{k + 1} <= V_max, % speed limits
                                  1 <= z{k+1} <= L,
                                -A_max <= a{k} <= A_max, % take possible values
                                  z{k} - 1 <= z{k + 1},
                                  z{k + 1} <= z{k} + 1];

    constraints = [constraints, v{k + 1} == v{k} + T * a{k}]; % speed dynamic



    % ---------------------------- vehiculo 2 -----------------------------
    % ------------------ si dz=0  -------------------->>>    dij >= Ds----------------

    constraints = [constraints, dis12{k + 1} == dis12{k} + T * (v_2 - v{k})]; %positin dynamic

    %................................... (12)...............................
    constraints = log_min(constraints, n12{k}, z_2-z{k+1}, 0);
    constraints = log_may(constraints, th12{k}, z_2-z{k+1}, 0);
    constraints = log_and(constraints, a12{k}, n12{k}, th12{k});
    %................................... (13)...............................
    constraints = log_may(constraints, b12{k}, dis12{k+1}, 0);
    %................................... (18)...............................
    constraints = log_and(constraints, ab12{k}, a12{k}, b12{k});
    %................................... (21)...............................
    constraints = log_imp(constraints, f12{k}, dis12{k+1}, ab12{k});
    %................................... (22)...............................
    constraints = log_imp(constraints, g12{k}, Ds, ab12{k});
    %................................... (23)...............................
    constraints = [constraints, g12{k} - f12{k} <= 0];

    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables

    % Feasible region
    constraints = [constraints,       1 <= z_2 <= L];

    %................................... (15.a)...............................
    constraints = log_min(constraints, k12{k}, z_2-z{k} , 1);
    constraints = log_may(constraints, del12{k}, z_2-z{k} , 1);
    constraints = log_and(constraints, r1_12{k}, k12{k} , del12{k} );
    %................................... (15.b)...............................
    constraints = log_min(constraints, kk12{k}, z_2-z{k} , -1);
    constraints = log_may(constraints, dell12{k}, z_2-z{k} , -1);
    constraints = log_and(constraints, r2_12{k}, kk12{k} , dell12{k} );
    %................................... (17)...............................
    constraints = log_min(constraints, u12{k}, dis12{k+1} , Dl);
    constraints = log_may(constraints, v12{k}, dis12{k+1} , -Dl);
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
    constraints = log_may(constraints, b13{k}, dis13{k+1}, 0);
    %................................... (18)...............................
    constraints = log_and(constraints, ab13{k}, a13{k}, b13{k});
    %................................... (21)...............................
    constraints = log_imp(constraints, f13{k}, dis13{k+1}, ab13{k});
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
    constraints = log_min(constraints, u13{k}, dis13{k+1} , Dl);
    constraints = log_may(constraints, v13{k}, dis13{k+1} , -Dl);
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

        % ---------------------------- vehiculo 4 -----------------------------
    % ------------------ si dz=0  -------------------->>>    dij >= Ds----------------

    constraints = [constraints, dis14{k + 1} == dis14{k} + T * (v_4 - v{k})];

    %................................... (12)...............................
    constraints = log_min(constraints, n14{k}, z_4-z{k+1}, 0);
    constraints = log_may(constraints, th14{k}, z_4-z{k+1}, 0);
    constraints = log_and(constraints, a14{k}, n14{k}, th14{k});
    %................................... (14)...............................
    constraints = log_may(constraints, b14{k}, dis14{k+1}, 0);
    %................................... (18)...............................
    constraints = log_and(constraints, ab14{k}, a14{k}, b14{k});
    %................................... (21)...............................
    constraints = log_imp(constraints, f14{k}, dis14{k+1}, ab14{k});
    %................................... (22)...............................
    constraints = log_imp(constraints, g14{k}, Ds, ab14{k});
    %................................... (24)...............................
    constraints = [constraints, g14{k} - f14{k} <= 0];

    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables

    % Feasible region
    constraints = [constraints,       1 <= z_4 <= L];
    %................................... (15.a)...............................
    constraints = log_min(constraints, k14{k}, z_4-z{k} , 1);
    constraints = log_may(constraints, del14{k}, z_4-z{k} , 1);
    constraints = log_and(constraints, r1_14{k}, k14{k} , del14{k} );
    %................................... (15.b)...............................
    constraints = log_min(constraints, kk14{k}, z_4-z{k} , -1);
    constraints = log_may(constraints, dell14{k}, z_4-z{k} , -1);
    constraints = log_and(constraints, r2_14{k}, kk14{k} , dell14{k} );
    %................................... (17)...............................
    constraints = log_min(constraints, u14{k}, dis14{k+1} , Dl);
    constraints = log_may(constraints, v14{k}, dis14{k+1} , -Dl);
    constraints = log_and(constraints, x14 {k}, u14{k} , v14{k} );
    %................................... (30)...............................
    constraints = log_and(constraints, xr1_14{k}, x14{k} , r1_14{k});
    constraints = log_and(constraints, xr2_14{k}, x14{k} , r2_14{k});

    %................................... (33)...............................
    constraints = log_imp(constraints, xa_14{k}, z{k+1} , xr1_14{k});
    constraints = log_imp(constraints, xb_14{k}, z{k} ,   xr1_14{k});
    constraints = log_imp(constraints, xc_14{k}, z{k+1} , xr2_14{k});
    constraints = log_imp(constraints, xd_14{k}, z{k} ,   xr2_14{k});
    %................................... (32)...............................
    constraints = [constraints, xa_14{k} - xb_14{k} - xc_14{k} + xd_14{k} <= 0];


    % ---------------------------- vehiculo 5 -----------------------------
    % ------------------ si dz=0  -------------------->>>    dij >= Ds----------------

    constraints = [constraints, dis15{k + 1} == dis15{k} + T * (v_5 - v{k})];

    %................................... (12)...............................
    constraints = log_min(constraints, n15{k}, z_5-z{k+1}, 0);
    constraints = log_may(constraints, th15{k}, z_5-z{k+1}, 0);
    constraints = log_and(constraints, a15{k}, n15{k}, th15{k});
    %................................... (15)...............................
    constraints = log_may(constraints, b15{k}, dis15{k+1}, 0);
    %................................... (18)...............................
    constraints = log_and(constraints, ab15{k}, a15{k}, b15{k});
    %................................... (21)...............................
    constraints = log_imp(constraints, f15{k}, dis15{k+1}, ab15{k});
    %................................... (22)...............................
    constraints = log_imp(constraints, g15{k}, Ds, ab15{k});
    %................................... (25)...............................
    constraints = [constraints, g15{k} - f15{k} <= 0];

    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables
    % It is EXTREMELY important to add as many
    % constraints as possible to the binary variables

    % Feasible region
    constraints = [constraints,       1 <= z_5 <= L];
    %................................... (15.a)...............................
    constraints = log_min(constraints, k15{k}, z_5-z{k} , 1);
    constraints = log_may(constraints, del15{k}, z_5-z{k} , 1);
    constraints = log_and(constraints, r1_15{k}, k15{k} , del15{k} );
    %................................... (15.b)...............................
    constraints = log_min(constraints, kk15{k}, z_5-z{k} , -1);
    constraints = log_may(constraints, dell15{k}, z_5-z{k} , -1);
    constraints = log_and(constraints, r2_15{k}, kk15{k} , dell15{k} );
    %................................... (17)...............................
    constraints = log_min(constraints, u15{k}, dis15{k+1} , Dl);
    constraints = log_may(constraints, v15{k}, dis15{k+1} , -Dl);
    constraints = log_and(constraints, x15 {k}, u15{k} , v15{k} );
    %................................... (30)...............................
    constraints = log_and(constraints, xr1_15{k}, x15{k} , r1_15{k});
    constraints = log_and(constraints, xr2_15{k}, x15{k} , r2_15{k});

    %................................... (33)...............................
    constraints = log_imp(constraints, xa_15{k}, z{k+1} , xr1_15{k});
    constraints = log_imp(constraints, xb_15{k}, z{k} ,   xr1_15{k});
    constraints = log_imp(constraints, xc_15{k}, z{k+1} , xr2_15{k});
    constraints = log_imp(constraints, xd_15{k}, z{k} ,   xr2_15{k});
    %................................... (32)...............................
    constraints = [constraints, xa_15{k} - xb_15{k} - xc_15{k} + xd_15{k} <= 0];






end

parameters_in = {Vd, v_ref, Zd, z_ref, v_2, dis12_ref, z_2, v_3, dis13_ref, z_3, v_4, dis14_ref, z_4, v_5, dis15_ref, z_5};

solutions_out = {[a{:}], [v{:}], [z{:}]}; %, [dis12{:}] ,[a12{:}], [b12{:}], [ab12{:}], [f12{:}] ,[g12{:}] };

control_front = optimizer(constraints, objective, sdpsettings('solver', 'gurobi'), parameters_in, solutions_out);


%% Historical variables

ktime = NaN(k_step,1);
time_hist = NaN(nv,k_step);
hist_pos = NaN(nv,k_step+1);
elap_time = NaN(nv,1);
vhist = NaN(nv,k_step+1);
zhist = NaN(nv,k_step+1);


hist_pos(:,1) = pos;
vhist(:,1) = vel;
zhist(:,1) = zel;

mpciter = 0;

%% Optimization

zel2 = zel; %same dimentions
time = 20;
Start = tic;
sim_tim = 20;
dif_z = ones(1,N+1)* (zel(2) - zel(1));
vphist = NaN(k_step,N+1, nv);
zphist = NaN(k_step,N+1, nv);

zphist(1,:,:)= zel' .* ones(N+1, nv);

for i = 1:k_step
    k_t = tic;
    for agent = 1:nv
        %     ######################  VEHICULO agent #######################
        tic
        agent;
        NH = closest(pos, zel, agent);
        inputs = {Vdes(agent), vel(agent), Zdes(agent), zel(agent), ...
            vel(NH(1)), -pos(agent)+pos(NH(1)),  zel(NH(1)), ...
            vel(NH(2)), -pos(agent)+pos(NH(2)),  zel(NH(2)), ...
            vel(NH(3)), -pos(agent)+pos(NH(3)),  zel(NH(3)), ...
            vel(NH(4)), -pos(agent)+pos(NH(4)),  zel(NH(4))};
        [solutions, diagnostics] = control_front{inputs};
        elap_time(agent) = toc;

        A = solutions{1}; acel(agent) = A(:, 1);
        V = solutions{2}; 
        Z = solutions{3}; zel2(agent) = Z(:, 2);
        vphist(i,:,agent) = V;
        zphist(i,:,agent) = Z;

        if diagnostics == 1
            error_desc =sprintf('control of agent %d in iteration %d', agent, i);
            error(error_desc);
        end
        zel= zel2;

%     if (norm(zel(agent)-Zdes(agent)) == 0) && (norm(vel(agent)-Vdes(agent)) == 0)
%         agent
%         disp("objetivo cumplido")
%         
%     end
    end

    % zel= zel2;

    %----------------------------------------------------------------------

    pos = pos + T*vel;
    vel = vel + T * acel;    

    time_hist(:,i) = elap_time;
    hist_pos(:,i+1) = pos;
    vhist(:,i+1) = vel;
    zhist(:,i+1) = zel;
    
    % ahist = [ahist acel];

    mpciter = mpciter + 1;

    ktime(i) = toc(k_t);
    
end
End = toc(Start)
% ktime(k_step+1) = End;
disp("it's done")

%% plot

shift_x = -20; 
shift_y = -70;
scale_x = 0.5;
scale_y = 25;

name_gif = "MPC"
Draw_object(vhist*scale_x , zhist*scale_y + shift_y, vphist*scale_x, zphist*scale_y + shift_y, hist_pos*scale_x + shift_x, T, name_gif, 0)
save('myData.mat','vhist','zhist','vphist','zphist','hist_pos','End', "ktime", "time_hist")