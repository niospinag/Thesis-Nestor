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

% horizon = [20 20 20 20 20 20 20 20 20 ]
% N = 10; % horizon 
T = 0.3;
% 
% for vh=5:12
vh=5;
    vel1  = vel(1:vh);
    Vdes1 = Vdes(1:vh);
    zel1  = zel(1:vh);
    Zdes1 = Zdes(1:vh);
    pos1  = pos(1:vh);
% for N=4:10
    N=10
    [vhist,zhist,vphist,zphist,hist_pos,End, ktime, time_hist] = func_controller(vel1, Vdes1, zel1, Zdes1, pos1, N);
    filename  = sprintf('datasave/data_%dv_%dN', size(vel1,1), N)
    save(filename,'vhist','zhist','vphist','zphist','hist_pos','End', "ktime", "time_hist")
% end

% end
% save('myData.mat','vhist','zhist','vphist','zphist','hist_pos','End', "ktime", "time_hist")

%% plot

shift_x = -20; 
shift_y = -70;
scale_x = 0.5;
scale_y = 25;
name_gif = sprintf('data_%dv_%dN', size(vel,1), N);
Draw_object(vhist*scale_x , zhist*scale_y + shift_y, vphist*scale_x, zphist*scale_y + shift_y, hist_pos*scale_x + shift_x, T, name_gif, 0)