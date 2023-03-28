% Algorithm test: Median error w.r.t varying number of lines
%
%
% This code follows the algorithm given by 
% [1] "Line-based Event Camera Calibration"
%
% This work was developed at National University of Defense Technology, 
% Hunan Provincial Key Laboratory of Image Measurement and Vision Navigation.

clc;clear;
close all
addpath(genpath("./func/"));
k=[0.1,0.1];
noise= 1;
num= [12,20,40,80,120,160,200];
line_num=[6,10,20,40,60,80,100];
nls=[6,10,20,40,60,80,100];
nls2=[6,10,20,40,60,80,100];
A=zeros(size(num));

%......
name= {'planar-R','nonplanar-R','planar-T','nonplanar-T','planar-fx','nonplanar-fx','planar-fy','nonplanar-fy',...
    'planar-cx','nonplanar-cx','planar-cy','nonplanar-cy','planar-k1','nonplanar-k1','planar-k2','nonplanar-k2'};

marker= {'+','s','d','*','+','s','d','*','+','s','d','*','+','s','d','*'};
color= {'r','c','k','m','r','c','k','m','r','c','k','m','r','c','k','m'};
markerfacecolor={'r','c','k','m','r','c','k','m','r','c','k','m','r','c','k','m'};

linestyle= {'-','-','-','-','-','-','-','-','-','-','-','-','-','-','-','-'};

method_list= struct('name', name, 'median_r', A, 'median_t', A,'median_fx', A,'median_fy', ...
A, 'median_cx', A,'median_cy', A, 'median_k1', A,'median_k2', A,...
    'marker', marker, 'color', color, 'markerfacecolor', markerfacecolor, 'linestyle', linestyle);
%.....

for i= 1:length(num)
    number= num(i);

    for j=1:500
    % planar test    

    [P_p,P_n,p,pt,K,R,t] = gendata_both(k,noise,number);

    %planar test  

    [K_e0 R_e0 t_e0] =DLT_planar(P_p, p);

    [K_e0, RT_optim, k_e0] = Optim_Lines_All( P_p, p, K_e0, [0,0], R_e0 ,t_e0);

    [eR0(j) et0(j) efx0(j) efy0(j) ecx0(j) ecy0(j) ek10(j) ek20(j) ] = err_both(K,R,t,K_e0,RT_optim(1:3,1:3),RT_optim(1:3,4),k_e0);

    % nonPlanar test

    [K1 R1 t1]=DLT_nonplanar(p,P_n);

    [K_optim, RT_optim, coe_optim] = Optim_Lines_All( P_n, p, K1, [0,0], R1,t1);

    [eR(j) et(j) efx(j) efy(j) ecx(j) ecy(j)  ek1(j) ek2(j) ]= err_both(K,R,t,K_optim,RT_optim(1:3,1:3),RT_optim(1:3,4),coe_optim);


    end


    p_median_r(i)= median(eR0);
    p_median_t(i)= median(et0)*100;
    p_median_fx(i)= median(efx0)*100;
    p_median_fy(i)= median(efy0)*100;
    p_median_cx(i)= median(ecx0)*100;
    p_median_cy(i)= median(ecy0)*100;
    p_median_k1(i)= median(ek10);
    p_median_k2(i)= median(ek20);

    np_median_r(i)= median(eR);
    np_median_t(i)= median(et)*100;
    np_median_fx(i)= median(efx)*100;
    np_median_fy(i)= median(efy)*100;
    np_median_cx(i)= median(ecx)*100;
    np_median_cy(i)= median(ecy)*100;
    np_median_k1(i)= median(ek1);
    np_median_k2(i)= median(ek2);

end

        
    

subplot(1,3,1)

yyaxis left;
plot(nls,p_median_r,'marker',method_list(1).marker,...
        'color',method_list(1).color,...
        'markerfacecolor',method_list(1).markerfacecolor,...
        'displayname',method_list(1).name, ...
        'LineWidth',2,'MarkerSize',8,'LineStyle',method_list(1).linestyle);hold on;
plot(nls2, np_median_r,'marker',method_list(2).marker,...
        'color',method_list(2).color,...
        'markerfacecolor',method_list(2).markerfacecolor,...
        'displayname',method_list(2).name, ...
        'LineWidth',2,'MarkerSize',8,'LineStyle',method_list(2).linestyle);
ylabel('Rotation Error (degrees)','FontSize',18);
ylim([0 5]);

yyaxis right;
plot(nls,p_median_t,'marker',method_list(3).marker,...
        'color',method_list(3).color,...
        'markerfacecolor',method_list(3).markerfacecolor,...
        'displayname',method_list(3).name, ...
        'LineWidth',2,'MarkerSize',8,'LineStyle',method_list(3).linestyle);hold on;
plot(nls2, np_median_t,'marker',method_list(4).marker,...
        'color',method_list(4).color,...
        'markerfacecolor',method_list(4).markerfacecolor,...
        'displayname',method_list(4).name, ...
        'LineWidth',2,'MarkerSize',8,'LineStyle',method_list(4).linestyle);hold on;
ylabel('Translation Error (%)','FontSize',18);
ylim([0 5]);
xlim(nls([1 end]));

set(gca,'xtick',nls,'FontSize',14);
title('Rotation and Translation error','FontSize',18,'FontName','Time New Roman');
xlabel('Number of Lines','FontSize',18);
% ylabel('Rotation Error (degrees)','FontSize',12);
legend('planar-R', 'nonplanar-R','planar-T', 'nonplanar-T');

subplot(1,3,2)
yyaxis left;
plot(nls,0.5*(p_median_fx+p_median_fy),'marker',method_list(5).marker,...
        'color',method_list(5).color,...
        'markerfacecolor',method_list(5).markerfacecolor,...
        'displayname',method_list(5).name, ...
        'LineWidth',2,'MarkerSize',8,'LineStyle',method_list(5).linestyle);hold on;
plot(nls,0.5*(np_median_fx+np_median_fy),'marker',method_list(6).marker,...
        'color',method_list(6).color,...
        'markerfacecolor',method_list(6).markerfacecolor,...
        'displayname',method_list(6).name, ...
        'LineWidth',2,'MarkerSize',8,'LineStyle',method_list(6).linestyle);hold on;
ylabel('Focal length Error (%)','FontSize',18);
ylim([0 2]);

yyaxis right;
plot(nls,0.5*(p_median_cx+p_median_cy),'marker',method_list(7).marker,...
        'color',method_list(7).color,...
        'markerfacecolor',method_list(7).markerfacecolor,...
        'displayname',method_list(7).name, ...
        'LineWidth',2,'MarkerSize',8,'LineStyle',method_list(7).linestyle);hold on;
plot(nls,0.5*(np_median_cx+np_median_cy),'marker',method_list(8).marker,...
        'color',method_list(8).color,...
        'markerfacecolor',method_list(8).markerfacecolor,...
        'displayname',method_list(8).name, ...
        'LineWidth',2,'MarkerSize',8,'LineStyle',method_list(8).linestyle);
ylabel('Principal point Error (%)','FontSize',18);
ylim([0 2]);
xlim(nls([1 end]));

set(gca,'xtick',nls,'FontSize',14);
title('Intrinsic Parameter error','FontSize',18,'FontName','Time New Roman');
xlabel('Number of Lines','FontSize',18);

legend('planar-f', 'nonplanar-f','planar-c', 'nonplanar-c');

subplot(1,3,3)
yyaxis left;
plot(nls,p_median_k1,'marker',method_list(13).marker,...
        'color',method_list(13).color,...
        'markerfacecolor',method_list(13).markerfacecolor,...
        'displayname',method_list(13).name, ...
        'LineWidth',2,'MarkerSize',8,'LineStyle',method_list(13).linestyle);hold on;
plot(nls, np_median_k1,'marker',method_list(14).marker,...
        'color',method_list(14).color,...
        'markerfacecolor',method_list(14).markerfacecolor,...
        'displayname',method_list(14).name, ...
        'LineWidth',2,'MarkerSize',8,'LineStyle',method_list(14).linestyle);hold on;
ylabel('k1 Error','FontSize',18);
ylim([0 0.05]);

yyaxis right;
plot(nls,p_median_k2,'marker',method_list(15).marker,...
        'color',method_list(15).color,...
        'markerfacecolor',method_list(15).markerfacecolor,...
        'displayname',method_list(15).name, ...
        'LineWidth',2,'MarkerSize',8,'LineStyle',method_list(15).linestyle);hold on;
plot(nls, np_median_k2,'marker',method_list(16).marker,...
        'color',method_list(16).color,...
        'markerfacecolor',method_list(16).markerfacecolor,...
        'displayname',method_list(16).name, ...
        'LineWidth',2,'MarkerSize',8,'LineStyle',method_list(16).linestyle);
ylabel('k2 Error','FontSize',18);
ylim([0 0.05]);
xlim(nls([1 end]));

set(gca,'xtick',nls,'FontSize',14);
title('Distortion error','FontSize',18,'FontName','Time New Roman');
xlabel('Number of Lines','FontSize',18);

legend('planar-k1', 'nonplanar-k1','planar-k2', 'nonplanar-k2');

