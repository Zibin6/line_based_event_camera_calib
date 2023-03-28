% DEMO : Algorithm testing
%
%
% This code follows the algorithm given by 
% [1] "Line-based Event Camera Calibration"
%
% 
%
% National University of Defense Technology, China

clc;clear;
addpath(genpath("../func/"));

%% Planar_teat

num=50;%number of line endpoints
[P_p,P_n,p,pt,K,R,t] = gendata_simulate(num);


%% Planar_teat
[K_e0 R_e0 t_e0] =DLT_planar(P_p, p);

[K_e0, RT_optim, k_e0] = Optim_Lines_All( P_p, p, K_e0, [0,0], R_e0 ,t_e0);

[eR0 et0 efx0 efy0 ecx0 ecy0 ek10 ek20] = err_both(K,R,t,K_e0,RT_optim(1:3,1:3),RT_optim(1:3,4),k_e0)
 
%% nonPlanar_teat

[K1 R1 t1] =DLT_nonplanar(p,P_n);

[ K_optim, RT_optim, coe_optim] = Optim_Lines_All( P_n, p, K1, [0,0], R1,t1);

[eR et efx efy ecx ecy ek1 ek2 ]= err_both(K,R,t,K_optim,RT_optim(1:3,1:3),RT_optim(1:3,4),coe_optim)
