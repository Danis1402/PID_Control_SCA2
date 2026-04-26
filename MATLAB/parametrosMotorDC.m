clear all
close all
clc
%% Parámetros del motor
Km=0.06;
Kt=0.06;
R=4.7;
L= 0;
J=7.95e-6;
b=40.923e-6;
%% Función de transferencia del motor DC
num = [Kt];
den = [L*J, L*b + R*J, R*b + Kt*Km];
