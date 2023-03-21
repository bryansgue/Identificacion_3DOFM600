%% VALIDATION FO PARAMETERS DORNE DYNAMIC %%

%% clear variables
clc, clear all, close all;

%% LOAD VALUES FROM MATRICES
% LOAD VALUES FROM MATRICES
%load('Test4.mat')
% load('Datos23-Apr-2021 09_10_53_4.mat')
% load('Datos23-Apr-2021 09_53_19_5.mat')
% load('Datos27-Apr-2021 10_01_57_10.mat')
% load('Datos20-Apr-2021 17_13_25_6.mat') % pulsos
load('Datos23-Apr-2021 09_58_57_6.mat') % Pulsos
load("A_B_values.mat")
% Configuracion de vector
clear tf;
n = 20;
t = t(1,1:end-n);
 
%% REFERENCE SIGNALS UAV
ul_ref = vf_ref(1,1:end-n);
um_ref = vl_ref(1,1:end-n);
un_ref = ve_ref(1,1:end-n);
w_ref = w_ref(1,1:end-n);

%% REFERENCE SIGNALS ARM
q1p_ref=q1p_ref(1:end-n);
q2p_ref=q2p_ref(1:end-n);
q3p_ref=q3p_ref(1:end-n);

%% REAL SIGNALS UAV
ul = double(xup(1,1:length(ul_ref)));
um = double(yup(1,1:length(um_ref)));
un = double(ve(1,1:length(un_ref)));
w = double(w(1,1:length(w_ref)));

%% REAL SIGNALS ARM
q1p_real=q1p(1:length(t));
q2p_real=q2p(1:length(t));
q3p_real=q3p(1:length(t));

%% REFERENCE SIGNALS VECTOR
v_ref = [ul_ref; um_ref; un_ref; w_ref; q1p_ref; q2p_ref; q3p_ref;];

%% REAL SYSTEM VELICITIES VECTOR
v_real = [ul; um; un; w; q1p_real; q2p_real; q3p_real];

%% REAL SYSTEM ACCELERATIONS VECTOR
ulp_real = [0, diff(ul)/ts];
ump_real = [0 , diff(um)/ts];
unp_real = [0 , diff(un)/ts];
wp_real= [0 , diff(w)/ts];
q1pp_real = [0 , diff(q1p_real)/ts];
q2pp_real = [0 , diff(q2p_real)/ts];
q3pp_real = [0 , diff(q3p_real)/ts];

vp_real = [ulp_real; ump_real; unp_real; wp_real; q1pp_real; q2pp_real; q3pp_real];



%% Definition of System Matrcies
Ac = A % From data saved

Bc = B % From data saved
%% SIMULATION DYNAMICS
v_estimate = v_real(:,1);
for k=1:length(t)
    vp = (Ac*v_estimate(:,k)+Bc*v_ref(:,k));
    v_estimate(:, k+1) = v_estimate(:, k) + vp*ts;
    %v_estimate(:, k+1) = A*v_estimate(:,k)+B*vref(:,k);
end



%% Parameters fancy plots
% define plot properties
lw = 2; % linewidth 1
lwV = 2; % linewidth 2
fontsizeLabel = 9; %11
fontsizeLegend = 9;
fontsizeTicks = 9;
fontsizeTitel = 9;
sizeX = 900; % size figure
sizeY = 300; % size figure

% color propreties
C1 = [246 170 141]/255;
C2 = [51 187 238]/255;
C3 = [0 153 136]/255;
C4 = [238 119 51]/255;
C5 = [204 51 17]/255;
C6 = [238 51 119]/255;
C7 = [187 187 187]/255;
C8 = [80 80 80]/255;
C9 = [140 140 140]/255;
C10 = [0 128 255]/255;
C11 = [234 52 89]/255;
C12 = [39 124 252]/255;
C13 = [40 122 125]/255;
%C14 = [86 215 219]/255;
C14 = [252 94 158]/255;
C15 = [244 171 39]/255;
C16 = [100 121 162]/255;
C17 = [255 0 0]/255;


figure('Position', [15 15 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');

subplot(2,2,1)
plot(t(1:length(ul_ref)),ul_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
%plot(uv(1,:),uv(2,:),'-','Color',C11,'LineWidth',lw);
plot(t,ul,'-','Color',C11,'LineWidth',lw);
plot(t,v_estimate(1,1:length(t)),'--','Color',C12,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\mu_{lref}$','$\mu_{l}$','$\mu_{lm}$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;


subplot(2,2,2)
plot(t(1:length(um_ref)),um_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,um,'-','Color',C13,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
plot(t,v_estimate(2,1:length(t)),'--','Color',C14,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\mu_{mref}$','$\mu_{m}$','$\mu_{mm}$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(2,2,3)
plot(t(1:length(un_ref)),un_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,un,'-','Color',C2,'LineWidth',lw);
plot(t,v_estimate(3,1:length(t)),'--','Color',C15,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(c)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\mu_{nref}$','$\mu_{n}$','$\mu_{nm}$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(2,2,4)
plot(t(1:length(w_ref)),w_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,w,'-','Color',C16,'LineWidth',lw);
plot(t,v_estimate(4,1:length(t)),'--','Color',C17,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(d)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\omega_{ref}$','$\omega$','$\omega_{m}$'},'interpreter','latex','fontsize',fontsizeLegend)
%print -dpng Model_dmd_validation
print -depsc Model_dmd_validation

%% Fig de los brazos
figure('Position', [10 10 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');

subplot(3,1,1)
plot(t(1:length(q1p_ref)),q1p_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t(1:length(q1p_real)),q1p_real,'-','Color',C11,'LineWidth',lw);
plot(t,v_estimate(5,1:length(t)),'--','Color',C12,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$q1p_{ref}$','$q1p_{real}$','$q1p_{est}$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;


subplot(3,1,2)
plot(t(1:length(q2p_ref)),q2p_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t(1:length(q2p_real)),q2p_real,'-','Color',C13,'LineWidth',lw);
plot(t,v_estimate(6,1:length(t)),'--','Color',C14,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$q2p_{ref}$','$q2p_{real}$','$q2p_{est}$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(3,1,3)
plot(t(1:length(q3p_ref)),q3p_ref,'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot(t(1:length(q3p_real)),q3p_real,'-','Color',C2,'LineWidth',lw);
plot(t,v_estimate(7,1:length(t)),'--','Color',C15,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(c)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$q3p_{ref}$','$q3p_{real}$','$q3p_{est}$'},'interpreter','latex','fontsize',fontsizeLegend)



