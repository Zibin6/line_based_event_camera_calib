% Real-world test using a box
%
%
% This code follows the algorithm given by 
% [1] "LECalib: Line-based Event Camera Calibration"
%
% 
%
% This work was developed at National University of Defense Technology, 
% Hunan Provincial Key Laboratory of Image Measurement and Vision Navigation.

clc;clear;
close all
addpath(genpath("../func/"));
load("box_data.mat")

n_PtperLine=2;
undist_max_iter = 32;
undist_eps = 1e-10;
dist_type = 'Brown';


[K_e0,R_e0,t_e0] =DLT_nonplanar(p(:,:,1),P_p);

[coe0] = GetCoeByLines_Brown(p(:,:,1), P_p, K_e0,R_e0,t_e0, 1, 1,0,0, 0);

xita=0.000001*[sum(imudata(:,2)),sum(imudata(:,3)),sum(imudata(:,4))];

Rx=[1 0 0;0 cos(xita(1)) -sin(xita(1)); 0 sin(xita(1)) cos(xita(1))];
Ry=[cos(xita(2)) 0 sin(xita(2)); 0  1 0; -sin(xita(2)) 0 cos(xita(2))];
Rz=[cos(xita(3)) -sin(xita(2)) 0; sin(xita(2)) cos(xita(2)) 0;0 0 1];
rotation = Rz*Ry*Rx;

R_e1=rotation*R_e0;
t_e1=rotation*t_e0;

R_e(:,:,1)=R_e0;R_e(:,:,2)=R_e1;
t_e(:,:,1)=t_e0;t_e(:,:,2)=t_e1;
coe(:,:,1)=coe0;coe(:,:,2)=coe0;

n_PtperLine=2;
undist_max_iter = 32;
undist_eps = 1e-10;
dist_type = 'Brown';

[K_final, R_final,t_final, coe_final] =Optim_Lines_All_planar_eventcamera( P_p, p, n_PtperLine, K_e0, coe, R_e ,t_e, 1, 1, 0, 0, 0, undist_max_iter, undist_eps);


fx=K_final(1,1)
fy=K_final(2,2)
cx=K_final(1,3)
cy=K_final(2,3)
Distortion=coe_final

