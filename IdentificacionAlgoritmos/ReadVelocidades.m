%% CARGAR LOS DATOS DE LAS VELOCIDADES DE REFERNCIA Y VELOCIDADES REALIES DEL ROBOR MANIPULRO AERE

% load('Datos23-Apr-2021 09_10_53_4.mat')
% load('Datos23-Apr-2021 09_53_19_5.mat')
% load('Datos27-Apr-2021 10_01_57_10.mat')
% load('Datos20-Apr-2021 17_13_25_6.mat') % pulsos
load('Datos23-Apr-2021 09_58_57_6.mat') % Pulsos

n=length(t);
n=1000;
t=t(1:n);
%% Vel reales
vf_real=xup(1:n);
vl_real=yup(1:n);
ve_real=ve(1:n);
w_real=w(1:n);
q1p_real=q1p(1:n);
q2p_real=q2p(1:n);
q3p_real=q3p(1:n);

%% Velocidades Referencia
vf_ref=vf_ref(1:n);
vl_ref=vl_ref(1:n);
ve_ref=ve_ref(1:n);
w_ref=w_ref(1:n);
q1p_ref=q1p_ref(1:n);
q2p_ref=q2p_ref(1:n);
q3p_ref=q3p_ref(1:n);

%% sustitucion de variables posicion
xu_real=double(xu(1:n));
yu_real=double(yu(1:n));
zu_real=double(zu(1:n));
psi_real=double(psi(1:n));
q1_real=double(q1(1:n));
q2_real=double(q2(1:n));
q3_real=double(q3(1:n));
%%
 clearvars -except t xu_real yu_real zu_real psi_real q1_real q2_real q3_real...
             vf_real vl_real ve_real w_real q1p_real q2p_real q3p_real...
             vf_ref vl_ref ve_ref w_ref q1p_ref q2p_ref q3p_ref

%%
figure(1)
subplot(4,1,1);plot(t,vf_ref(1:length(t)));hold on;plot(t,vf_real(1:length(t)));legend('vf ref','vf');grid on
subplot(4,1,2);plot(t,vl_ref(1:length(t)));hold on;plot(t,vl_real(1:length(t)));legend('vl_{ref}','vl');grid on
subplot(4,1,3);plot(t,ve_ref(1:length(t)));hold on;plot(t,ve_real(1:length(t)));legend('ve_{ref}''ve');grid on
subplot(4,1,4);plot(t,w_ref(1:length(t)));hold on;plot(t,w_real(1:length(t)));legend('\omega_{ref}','\omega');grid on

figure(2)
subplot(3,1,1),plot(t,q1p_ref(1:length(t)));hold on,plot(t,q1p_real(1:length(t)));legend('q1p_{ref}','q1p');grid on
subplot(3,1,2),plot(t,q2p_ref(1:length(t)));hold on,plot(t,q2p_real(1:length(t)));legend('q2p_{ref}','q1p');grid on
subplot(3,1,3),plot(t,q3p_ref(1:length(t)));hold on,plot(t,q3p_real(1:length(t)));legend('q3p_{ref}','q1p');grid on