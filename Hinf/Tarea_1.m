clc;
clear;
% modelo
load('modelo_lin.mat')
%% PID
% LATERAL
%Entradas: Rudder, Aileron
%Salidas: psi(guiñado), phi(alabeo), r, p
G_rpsi = tf(latmod(5,2)); % Rudder con psi
G_rr = tf(latmod(3,2)); % Rudder con r
G_aphi = tf(latmod(4,1)); % Aileron con phi
G_ap = tf(latmod(2,1)); % Aileron con p


% LONGITUDINAL
%Entradas: Elevator
%Salidas: tetha(cabeceo), q
G_et = tf(longmod(4,1)); % Elevator con tetha
G_eq = tf(longmod(3,1)); % Elevator con q



%% H_inf
% LATERAL
%psi
close all
M1_psi = 2; %Pasabajo - perturbación
A1_psi = 0.001;
Wb1_psi = 1*2*pi;

M3_psi = 0.001; %Pasaalto - ruido
A3_psi = 1;
Wb3_psi = 3*2*pi;

W1_psi= tf([1/M1_psi Wb1_psi],[1 A1_psi*Wb1_psi]);
W2_psi =1e6*tf(1,1);
W3_psi = tf([1/M3_psi Wb3_psi],[1 A3_psi*Wb3_psi]);

ncont_psi = 1;
nmeas_psi = 1;
[K_psi,sys_CL_psi,gam1_psi,info_psi] = hinfsyn(G_rpsi,nmeas_psi,ncont_psi);

L_psi  = G_rpsi*K_psi; 
sysnom_CL_psi = feedback(L_psi,1);
figure (1)
step(sysnom_CL_psi);
% Comprobacion W1
S_psi = feedback(1,L_psi);
figure (2)
bodemag(S_psi,'r');
hold on
bodemag(1/W1_psi,'g');
% Comprobacion W2
S_psi = feedback(1,L_psi);
figure(3)
bodemag(S_psi*K_psi,'r');
hold on
bodemag(1/W2_psi,'g');
% Comprobacion W3
T_psi = feedback(L_psi,1);
figure (4)
bodemag(T_psi,'r');
hold on
bodemag(1/W3_psi,'g');

%phi
close all
M1_phi = 3; %Pasabajo - perturbación
A1_phi = 0.5;
Wb1_phi = 1*2*pi;

M3_phi = 0.001; %Pasaalto - ruido
A3_phi = 0.1;
Wb3_phi = 4*2*pi;

W1_phi= tf([1/M1_phi Wb1_phi],[1 A1_phi*Wb1_phi]);
W2_phi =1000*tf(1,1);
W3_phi = tf([1/M3_phi Wb3_phi],[1 A3_phi*Wb3_phi]);

ncont_phi = 1;
nmeas_phi = 1;
[K_phi,sys_CL_phi,gam1_phi,info_phi] = hinfsyn(G_aphi,nmeas_phi,ncont_phi);

L_phi  = G_aphi*K_phi; 
sysnom_CL_phi = feedback(L_phi,1);
figure (1)
step(sysnom_CL_phi);
% Comprobacion W1
S_phi = feedback(1,L_phi);
figure (2)
bodemag(S_phi,'r');
hold on
bodemag(1/W1_phi,'g');
% Comprobacion W2
S_phi = feedback(1,L_phi);
figure(3)
bodemag(S_phi*K_phi,'r');
hold on
bodemag(1/W2_phi,'g');
% Comprobacion W3
T_phi = feedback(L_phi,1);
figure (4)
bodemag(T_phi,'r');
hold on
bodemag(1/W3_phi,'g');


% LONGITUDINAL
close all;
M1_t =2; %Pasabajo - perturbación
A1_t = 0.5;
Wb1_t = 1*2*pi;

M3_t = 0.0001; %Pasaalto - ruido
A3_t = 2;
Wb3_t = 4*2*pi;

W1_t= tf([1/M1_t Wb1_t],[1 A1_t*Wb1_t]);
W2_t =10*tf(1,1);
W3_t = tf([1/M3_t Wb3_t],[1 A3_t*Wb3_t]);

ncont_t = 1;
nmeas_t = 1;
[K_t,sys_CL_t,gam1_t,info_t] = hinfsyn(G_et,nmeas_t,ncont_t);

L_t  = G_et*K_t; 
sysnom_CL_t = feedback(L_t,1);
figure (1)
step(sysnom_CL_t);
% Comprobacion W1
S_t = feedback(1,L_t);
figure (2)
bodemag(S_t,'r');
hold on
bodemag(1/W1_t,'g');
% Comprobacion W2
S_t = feedback(1,L_t);
figure(3)
bodemag(S_t*K_t,'r');
hold on
bodemag(1/W2_t,'g');
% Comprobacion W3
T_t = feedback(L_t,1);
figure (4)
bodemag(T_t,'r');
hold on
bodemag(1/W3_t,'g');

